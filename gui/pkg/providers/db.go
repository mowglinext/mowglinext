package providers

import (
	"errors"
	"git.mills.io/prologic/bitcask"
	"golang.org/x/xerrors"
	"os"
)

type DBProvider struct {
	db *bitcask.Bitcask
}

var EnvFallbacks = map[string]string{
	"system.api.addr":         "API_ADDR",
	"system.api.webDirectory": "WEB_DIR",
	"system.map.enabled":      "MAP_TILE_ENABLED",
	"system.map.tileServer":   "MAP_TILE_SERVER",
	"system.map.tileUri":      "MAP_TILE_URI",
	"system.homekit.enabled":  "HOMEKIT_ENABLED",
	"system.mqtt.enabled":     "MQTT_ENABLED",
	"system.mqtt.prefix":      "MQTT_PREFIX",
	"system.mqtt.host":        "MQTT_HOST",
	"system.mower.configFile":     "MOWER_CONFIG_FILE",
	"system.mower.yamlConfigFile": "MOWER_YAML_CONFIG_FILE",
	"system.mower.runtimeEnvFile": "MOWER_RUNTIME_ENV_FILE",
	"system.ros.masterUri":     "ROS_MASTER_URI",
	"system.ros.nodeName":      "ROS_NODE_NAME",
	"system.ros.nodeHost":      "ROS_NODE_HOST",
	"system.ros.foxgloveUrl":   "FOXGLOVE_URL",
	"system.homekit.pincode":  "HOMEKIT_PINCODE",
}
var Defaults = map[string]string{
	"system.api.addr":         ":4006",
	"system.api.webDirectory": "/app/web",
	"system.map.enabled":      "false",
	"system.map.tileServer":   "http://localhost:5000",
	"system.map.tileUri":      "/tiles/vt/lyrs=s,h&x={x}&y={y}&z={z}",
	"system.homekit.enabled":  "false",
	"system.homekit.pincode":  "00102003",
	"system.mqtt.enabled":     "false",
	"system.mqtt.host":        ":1883",
	"system.mqtt.prefix":      "/gui",
	"system.mower.configFile":     "/config/mower_config.sh",
	"system.mower.yamlConfigFile": "/config/mowgli_robot.yaml",
	"system.mower.runtimeEnvFile": "/runtime_config/.env",
	"system.ros.masterUri":    "http://localhost:11311",
	"system.ros.nodeName":     "mowglinext",
	"system.ros.nodeHost":     "localhost",
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
		return nil, xerrors.Errorf("config key %s not found", key)
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
	var err error
	d := &DBProvider{}
	d.db, err = bitcask.Open(os.Getenv("DB_PATH"))
	if err != nil {
		panic(err)
	}
	return d
}
