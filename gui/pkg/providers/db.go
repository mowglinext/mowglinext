package providers

import (
	"errors"
	"fmt"
	"git.mills.io/prologic/bitcask"
	"github.com/mowglinext/mowglinext/pkg/types"
	"io"
	"log"
	"os"
	"path/filepath"
	"runtime"
	"strings"
	"time"
)

type DBProvider struct {
	db *bitcask.Bitcask
}

var EnvFallbacks = map[string]string{
	"system.api.addr":             "API_ADDR",
	"system.api.webDirectory":     "WEB_DIR",
	"system.map.enabled":          "MAP_TILE_ENABLED",
	"system.map.tileServer":       "MAP_TILE_SERVER",
	"system.map.tileUri":          "MAP_TILE_URI",
	"system.homekit.enabled":      "HOMEKIT_ENABLED",
	"system.mqtt.enabled":         "MQTT_ENABLED",
	"system.mqtt.prefix":          "MQTT_PREFIX",
	"system.mqtt.host":            "MQTT_HOST",
	"system.mower.configFile":     "MOWER_CONFIG_FILE",
	"system.mower.yamlConfigFile": "MOWER_YAML_CONFIG_FILE",
	"system.mower.runtimeEnvFile": "MOWER_RUNTIME_ENV_FILE",
	"system.ros.masterUri":        "ROS_MASTER_URI",
	"system.ros.nodeName":         "ROS_NODE_NAME",
	"system.ros.nodeHost":         "ROS_NODE_HOST",
	"system.ros.foxgloveUrl":      "FOXGLOVE_URL",
	"system.homekit.pincode":      "HOMEKIT_PINCODE",
}
var Defaults = map[string]string{
	"system.api.addr":             ":4006",
	"system.api.webDirectory":     "/app/web",
	"system.map.enabled":          "false",
	"system.map.tileServer":       "http://localhost:5000",
	"system.map.tileUri":          "/tiles/vt/lyrs=s,h&x={x}&y={y}&z={z}",
	"system.homekit.enabled":      "false",
	"system.homekit.pincode":      "00102003",
	"system.mqtt.enabled":         "false",
	"system.mqtt.host":            ":1883",
	"system.mqtt.prefix":          "/gui",
	"system.mower.configFile":     "/config/mower_config.sh",
	"system.mower.yamlConfigFile": "/config/mowgli_robot.yaml",
	"system.mower.runtimeEnvFile": "/runtime_config/.env",
	"system.ros.masterUri":        "http://localhost:11311",
	"system.ros.nodeName":         "mowglinext",
	"system.ros.nodeHost":         "localhost",
}

func (d *DBProvider) Set(key string, value []byte) error {
	return d.db.Put([]byte(key), value)
}

func (d *DBProvider) Get(key string) ([]byte, error) {
	value, err := d.db.Get([]byte(key))
	if err != nil || value == nil || len(value) == 0 {
		if !errors.Is(err, bitcask.ErrKeyNotFound) {
			return nil, err
		}
		if EnvFallbacks[key] != "" && os.Getenv(EnvFallbacks[key]) != "" {
			return []byte(os.Getenv(EnvFallbacks[key])), nil
		}
		if Defaults[key] != "" {
			return []byte(Defaults[key]), nil
		}
		return nil, fmt.Errorf("config %w", types.KeyNotFoundError{Key: key})
	}
	return value, nil
}

func (d *DBProvider) Delete(key string) error {
	return d.db.Delete([]byte(key))
}

func (d *DBProvider) KeysWithSuffix(suffix string) ([]string, error) {
	var keys []string
	err := d.db.Scan([]byte(suffix), func(key []byte) error {
		keys = append(keys, string(key))
		return nil
	})
	if err != nil {
		return nil, err
	}
	return keys, nil
}

func (d *DBProvider) GetWithEnvFallback(key string, env string, def string) string {
	value, err := d.Get(key)
	if err != nil || value == nil || len(value) == 0 {
		if os.Getenv(env) == "" {
			return def
		} else {
			return os.Getenv(env)
		}
	}
	return string(value)
}

func NewDBProvider() *DBProvider {
	dbPath := os.Getenv("DB_PATH")
	db, err := bitcask.Open(dbPath)
	if err == nil {
		return &DBProvider{db: db}
	}
	if !isRecoverableBitcaskCorruption(err) {
		panic(err)
	}

	log.Printf("bitcask database open failed; attempting recovery: %v", err)
	backupPath, backupErr := backupDatabase(dbPath)
	if backupErr != nil {
		panic(fmt.Errorf("backing up corrupted bitcask database: %w", backupErr))
	}
	log.Printf("bitcask database backup created at %s", backupPath)

	// bitcask v1.0.2 retains its lock when opening a corrupted data file fails.
	// Release the discarded instance's file handles, then replace only that lock.
	runtime.GC()
	if removeErr := os.Remove(filepath.Join(dbPath, "lock")); removeErr != nil && !os.IsNotExist(removeErr) {
		panic(fmt.Errorf("removing failed bitcask lock before recovery: %w", removeErr))
	}

	db, err = bitcask.Open(dbPath, bitcask.WithAutoRecovery(true))
	if err != nil {
		panic(fmt.Errorf("recovering bitcask database backed up at %s: %w", backupPath, err))
	}
	log.Printf("bitcask database recovery completed using backup %s", backupPath)
	return &DBProvider{db: db}
}

func isRecoverableBitcaskCorruption(err error) bool {
	message := err.Error()
	return strings.Contains(message, "key/value size is invalid") ||
		strings.Contains(message, "data is truncated")
}

func backupDatabase(dbPath string) (string, error) {
	info, err := os.Stat(dbPath)
	if err != nil {
		return "", err
	}
	if !info.IsDir() {
		return "", fmt.Errorf("database path is not a directory: %s", dbPath)
	}

	backupPath, err := os.MkdirTemp(filepath.Dir(dbPath), fmt.Sprintf("%s.corrupt-%s-", filepath.Base(dbPath), time.Now().UTC().Format("20060102T150405.000000000Z")))
	if err != nil {
		return "", err
	}

	err = filepath.Walk(dbPath, func(path string, info os.FileInfo, walkErr error) error {
		if walkErr != nil {
			return walkErr
		}
		relPath, err := filepath.Rel(dbPath, path)
		if err != nil {
			return err
		}
		if relPath == "." || relPath == "lock" {
			return nil
		}
		destination := filepath.Join(backupPath, relPath)
		if info.IsDir() {
			return os.MkdirAll(destination, info.Mode())
		}
		if !info.Mode().IsRegular() {
			return fmt.Errorf("unsupported database file type: %s", path)
		}
		return copyFile(path, destination, info.Mode())
	})
	if err != nil {
		return "", err
	}
	return backupPath, nil
}

func copyFile(source string, destination string, mode os.FileMode) error {
	input, err := os.Open(source)
	if err != nil {
		return err
	}
	defer input.Close()

	output, err := os.OpenFile(destination, os.O_WRONLY|os.O_CREATE|os.O_EXCL, mode)
	if err != nil {
		return err
	}
	_, copyErr := io.Copy(output, input)
	closeErr := output.Close()
	if copyErr != nil {
		return copyErr
	}
	return closeErr
}
