package updater

import (
	"context"
	"crypto/sha256"
	"encoding/hex"
	"encoding/json"
	"errors"
	"fmt"
	"io"
	"net/http"
	"os"
	"os/exec"
	"path/filepath"
	"sort"
	"strconv"
	"strings"
	"time"

	"github.com/mowglinext/mowglinext/pkg/updates"
)

type HostConfig struct {
	Directory     string   `json:"directory"`
	Project       string   `json:"project"`
	StateDir      string   `json:"state_dir"`
	Trusted       []string `json:"trusted_repositories"`
	Platform      string   `json:"platform"`
	InitialSource Source   `json:"initial_source"`
}
type DockerBackend struct{ Config HostConfig }

func (b DockerBackend) Lock() (func(), error) {
	return processLock(filepath.Join(b.Config.Directory, ".deployment.lock"))
}

type serviceConfig struct {
	Image         string `json:"image"`
	ContainerName string `json:"container_name"`
}
type composeConfig struct {
	Services map[string]serviceConfig `json:"services"`
}
type containerInfo struct {
	ID     string `json:"Id"`
	Image  string `json:"Image"`
	Config struct {
		Labels map[string]string `json:"Labels"`
	} `json:"Config"`
	State struct {
		Running bool `json:"Running"`
		Health  *struct {
			Status string `json:"Status"`
		} `json:"Health"`
	} `json:"State"`
	Mounts []struct {
		Type        string `json:"Type"`
		Source      string `json:"Source"`
		Destination string `json:"Destination"`
		RW          bool   `json:"RW"`
	} `json:"Mounts"`
}

func command(ctx context.Context, name string, args ...string) ([]byte, error) {
	c := exec.CommandContext(ctx, name, args...)
	b, e := c.CombinedOutput()
	if e != nil {
		return nil, fmt.Errorf("%s: %w: %.2000s", name, e, b)
	}
	return b, nil
}
func (b DockerBackend) compose(ctx context.Context, args ...string) ([]byte, error) {
	return b.composeWithOverride(ctx, true, args...)
}
func (b DockerBackend) composeWithOverride(ctx context.Context, override bool, args ...string) ([]byte, error) {
	prefix := []string{"compose", "--project-name", b.Config.Project, "--project-directory", filepath.Dir(b.Config.Directory), "--env-file", filepath.Join(b.Config.Directory, ".env"), "-f", filepath.Join(b.Config.Directory, "docker-compose.yaml")}
	if _, err := os.Stat(b.override()); err == nil && override {
		prefix = append(prefix, "-f", b.override())
	}
	return command(ctx, "docker", append(prefix, args...)...)
}
func (b DockerBackend) override() string {
	return filepath.Join(b.Config.Directory, "update-images.json")
}
func (b DockerBackend) model(ctx context.Context) (composeConfig, []byte, error) {
	var c composeConfig
	data, err := b.compose(ctx, "config", "--format", "json")
	if err == nil {
		err = json.Unmarshal(data, &c)
	}
	return c, data, err
}
func (b DockerBackend) inspect(ctx context.Context, name string) (containerInfo, error) {
	var list []containerInfo
	data, err := command(ctx, "docker", "inspect", name)
	if err == nil {
		err = json.Unmarshal(data, &list)
	}
	if err != nil {
		return containerInfo{}, err
	}
	if len(list) != 1 {
		return containerInfo{}, errors.New("container missing")
	}
	return list[0], nil
}
func (b DockerBackend) Inventory(ctx context.Context) (string, map[string]string, error) {
	c, data, err := b.model(ctx)
	if err != nil {
		return "", nil, err
	}
	images := map[string]string{}
	keys := []string{}
	for s := range c.Services {
		keys = append(keys, s)
	}
	sort.Strings(keys)
	for _, s := range keys {
		if _, ok := Services[s]; !ok {
			continue
		}
		ci, e := b.inspect(ctx, c.Services[s].ContainerName)
		if e != nil {
			return "", nil, e
		}
		if ci.Config.Labels["com.docker.compose.project"] != b.Config.Project {
			return "", nil, errors.New("container belongs to another Compose project")
		}
		images[s] = ci.Image
		data = append(data, []byte(ci.ID+ci.Image)...)
	}
	if len(images) < 2 || images["mowgli"] == "" || images["gui"] == "" {
		return "", nil, errors.New("unsupported Compose layout")
	}
	return updates.Hash(data), images, nil
}
func (b DockerBackend) PlanImages(ctx context.Context, d Deployment) (map[string]string, error) {
	if err := d.Validate(b.Config.Trusted); err != nil {
		return nil, err
	}
	ready, err := b.readiness(ctx)
	if err != nil {
		return nil, err
	}
	if ready.FirmwareProtocol != d.FirmwareProtocol {
		return nil, errors.New("target requires a different mainboard firmware protocol")
	}
	c, _, err := b.model(ctx)
	if err != nil {
		return nil, err
	}
	result := map[string]string{}
	for service, name := range Services {
		sc, exists := c.Services[service]
		if !exists {
			continue
		}
		ci, e := b.inspect(ctx, sc.ContainerName)
		if e != nil {
			return nil, e
		}
		// Legacy images cannot promise a maintenance gate on reboot or rollback.
		if (service == "mowgli" || service == "gui") && ci.Config.Labels["garden.mowgli.maintenance-api"] != "1" {
			return nil, errors.New("run the installer upgrade first: installed GUI/ROS2 lacks update maintenance support")
		}
		if service == "lidar" {
			// Recovery overrides may use bare local image IDs. The generated base
			// Compose file retains the operator's actual LiDAR model selection.
			baseData, e := b.composeWithOverride(ctx, false, "config", "--format", "json")
			if e != nil {
				return nil, e
			}
			var base composeConfig
			if e = json.Unmarshal(baseData, &base); e != nil {
				return nil, e
			}
			sc = base.Services[service]
			for _, candidate := range []string{"lidar-ldlidar", "lidar-rplidar", "lidar-stl27l"} {
				if strings.Contains(sc.Image, "/"+candidate+":") || strings.Contains(sc.Image, "/"+candidate+"@") {
					name = candidate
				}
			}
		}
		image, ok := d.Images[name]
		if !ok {
			return nil, fmt.Errorf("deployment does not cover %s", service)
		}
		platform, ok := image.Platforms[b.Config.Platform]
		if !ok {
			return nil, fmt.Errorf("no %s image for %s", b.Config.Platform, service)
		}
		result[service] = image.Repository + "@" + platform.Manifest
	}
	return result, nil
}
func (b DockerBackend) Pull(ctx context.Context, images map[string]string) error {
	free, err := freeBytes(b.Config.StateDir)
	if err != nil {
		return err
	}
	if free < 2*1024*1024*1024 {
		return errors.New("less than 2 GiB free; free storage before downloading updates")
	}
	for _, image := range images {
		if _, err := command(ctx, "docker", "pull", "--platform", b.Config.Platform, image); err != nil {
			return err
		}
	}
	return nil
}

type Readiness struct {
	Ready            bool   `json:"ready"`
	Maintenance      bool   `json:"maintenance"`
	FirmwareProtocol int    `json:"firmware_protocol"`
	Reason           string `json:"reason"`
	GPSFresh         bool   `json:"gps_fresh"`
	LidarFresh       bool   `json:"lidar_fresh"`
}

func (b DockerBackend) readiness(ctx context.Context) (Readiness, error) {
	c := &http.Client{Timeout: 5 * time.Second}
	data, e := updates.Read(ctx, c, "http://127.0.0.1:4006/api/system/update-readiness", "")
	var r Readiness
	if e == nil {
		e = json.Unmarshal(data, &r)
	}
	return r, e
}
func (b DockerBackend) MaintenanceSet() (bool, error) {
	_, err := os.Stat(filepath.Join(b.Config.StateDir, "maintenance"))
	if os.IsNotExist(err) {
		return false, nil
	}
	return err == nil, err
}
func (b DockerBackend) Maintenance(ctx context.Context, enable bool) error {
	marker := filepath.Join(b.Config.StateDir, "maintenance")
	if !enable {
		if err := os.Remove(marker); err != nil && !os.IsNotExist(err) {
			return err
		}
		return nil
	}
	r, err := b.readiness(ctx)
	if err != nil {
		return err
	}
	if !r.Ready {
		return fmt.Errorf("mower not ready: %s", r.Reason)
	}
	// Never let a second updater recreate a container during our transaction.
	data, err := command(ctx, "docker", "ps", "--format", "{{.Image}}")
	if err != nil {
		return err
	}
	if strings.Contains(strings.ToLower(string(data)), "watchtower") {
		return errors.New("Watchtower is running; migrate or exclude this stack before installation")
	}
	if err = AtomicWrite(marker, []byte("Container update in progress\n"), 0644); err != nil {
		return err
	}
	// Require the GUI to acknowledge the persisted gate before stopping writers.
	for i := 0; i < 20; i++ {
		r, e := b.readiness(ctx)
		if e == nil && r.Maintenance && r.Ready {
			return nil
		}
		select {
		case <-ctx.Done():
			return ctx.Err()
		case <-time.After(250 * time.Millisecond):
		}
	}
	return errors.New("maintenance gate was not acknowledged; marker retained")
}

type backupRecord struct {
	Sources   []string `json:"sources"`
	Checksums []string `json:"checksums"`
	Override  []byte   `json:"override,omitempty"`
	Compose   []byte   `json:"compose"`
	Env       []byte   `json:"env"`
}

func (b DockerBackend) Backup(ctx context.Context, id string) (string, error) {
	if !idPattern.MatchString(id) {
		return "", errors.New("invalid job ID")
	}
	path := filepath.Join(b.Config.StateDir, "backups", id)
	if err := os.MkdirAll(path, 0700); err != nil {
		return "", err
	}
	c, _, err := b.model(ctx)
	if err != nil {
		return "", err
	}
	record := backupRecord{}
	record.Override, _ = os.ReadFile(b.override())
	record.Compose, err = os.ReadFile(filepath.Join(b.Config.Directory, "docker-compose.yaml"))
	if err != nil {
		return "", err
	}
	record.Env, err = os.ReadFile(filepath.Join(b.Config.Directory, ".env"))
	if err != nil {
		return "", err
	}
	sources := map[string]bool{}
	for s := range Services {
		sc, ok := c.Services[s]
		if !ok {
			continue
		}
		ci, e := b.inspect(ctx, sc.ContainerName)
		if e != nil {
			return "", e
		}
		// Tag old IDs before stopping containers so legacy pruning cannot discard
		// the only recovery copy of a locally patched image.
		if _, e = command(ctx, "docker", "tag", ci.Image, "mowgli-rollback:"+id+"-"+s); e != nil {
			return "", e
		}
		for _, mount := range ci.Mounts {
			if mount.RW && (mount.Destination == "/db" || mount.Destination == "/mowgli_config" || mount.Destination == "/ros2_ws/maps" || mount.Destination == "/ros2_ws/config") {
				sources[mount.Source] = true
			}
		}
	}
	for source := range sources {
		record.Sources = append(record.Sources, source)
	}
	sort.Strings(record.Sources)
	var size uint64
	for _, source := range record.Sources {
		data, e := command(ctx, "du", "-sb", "--", source)
		if e != nil {
			return "", e
		}
		fields := strings.Fields(string(data))
		if len(fields) == 0 {
			return "", errors.New("cannot measure backup size")
		}
		n, e := strconv.ParseUint(fields[0], 10, 64)
		if e != nil {
			return "", e
		}
		size += n
	}
	free, err := freeBytes(path)
	if err != nil {
		return "", err
	}
	if free < size*2+256*1024*1024 {
		return "", errors.New("insufficient space for a backup and failed-deployment data")
	}
	if _, err = b.compose(ctx, "stop", "gui", "mowgli"); err != nil {
		return "", err
	}
	// Persist the backup inventory only after every archive completed. Failure
	// leaves maintenance active rather than attempting an incomplete restore.
	for i, source := range record.Sources {
		if _, err = command(ctx, "tar", "--numeric-owner", "-cpf", filepath.Join(path, fmt.Sprintf("%d.tar", i)), "-C", source, "."); err != nil {
			return "", err
		}
		archive := filepath.Join(path, fmt.Sprintf("%d.tar", i))
		hash, e := archiveHash(archive, true)
		if e != nil {
			return "", e
		}
		record.Checksums = append(record.Checksums, hash)
	}
	if err = AtomicJSON(filepath.Join(path, "backup.json"), record); err != nil {
		return "", err
	}
	return path, nil
}
func (b DockerBackend) Apply(ctx context.Context, images map[string]string) error {
	c, _, err := b.model(ctx)
	if err != nil {
		return err
	}
	override := map[string]any{}
	for s, image := range images {
		if _, ok := Services[s]; !ok {
			return errors.New("unsupported service")
		}
		if _, ok := c.Services[s]; !ok {
			return errors.New("service layout changed")
		}
		override[s] = map[string]string{"image": image}
	}
	data, err := json.MarshalIndent(map[string]any{"services": override}, "", "  ")
	if err != nil {
		return err
	}
	if err = AtomicWrite(b.override(), data, 0644); err != nil {
		return err
	}
	// Consumers are stopped before dependencies are recreated. No orphan/volume
	// removal and no generated configuration reset occurs here.
	if _, err = b.compose(ctx, "stop", "gui", "mowgli"); err != nil {
		return err
	}
	for _, s := range []string{"gps", "lidar", "mowgli", "gui"} {
		if _, ok := images[s]; !ok {
			continue
		}
		if _, err = b.compose(ctx, "up", "-d", "--no-deps", "--pull", "never", s); err != nil {
			return err
		}
	}
	return nil
}
func (b DockerBackend) Verify(ctx context.Context, images map[string]string, d *Deployment) error {
	deadline := time.NewTimer(3 * time.Minute)
	defer deadline.Stop()
	stable := 0
	for {
		ok := true
		c, _, err := b.model(ctx)
		if err != nil {
			ok = false
		}
		for s, image := range images {
			sc, exists := c.Services[s]
			if !exists {
				ok = false
				continue
			}
			ci, e := b.inspect(ctx, sc.ContainerName)
			if e != nil || !ci.State.Running || (ci.State.Health != nil && ci.State.Health.Status != "healthy") {
				ok = false
				continue
			}
			ids, e := command(ctx, "docker", "image", "inspect", "--format", "{{.Id}}", image)
			if e != nil || strings.TrimSpace(string(ids)) != ci.Image {
				ok = false
			}
		}
		ready, e := b.readiness(ctx)
		if e != nil || !ready.Ready {
			ok = false
		}
		if _, e := os.Stat(filepath.Join(b.Config.StateDir, "maintenance")); e == nil && !ready.Maintenance {
			ok = false
		}
		if d != nil && ready.FirmwareProtocol != d.FirmwareProtocol {
			ok = false
		}
		if _, exists := images["gps"]; exists && !ready.GPSFresh {
			ok = false
		}
		if _, exists := images["lidar"]; exists && !ready.LidarFresh {
			ok = false
		}
		if ok {
			stable++
		} else {
			stable = 0
		}
		if stable >= 3 {
			return nil
		}
		select {
		case <-ctx.Done():
			return ctx.Err()
		case <-deadline.C:
			return errors.New("application health or image verification timed out")
		case <-time.After(2 * time.Second):
		}
	}
}
func (b DockerBackend) Restore(ctx context.Context, path string) error {
	root := filepath.Join(b.Config.StateDir, "backups") + string(os.PathSeparator)
	if !strings.HasPrefix(filepath.Clean(path)+string(os.PathSeparator), root) {
		return errors.New("backup outside updater state directory")
	}
	data, err := os.ReadFile(filepath.Join(path, "backup.json"))
	if err != nil {
		return err
	}
	var r backupRecord
	if err = json.Unmarshal(data, &r); err != nil {
		return err
	}
	if len(r.Checksums) != len(r.Sources) {
		return errors.New("backup checksum inventory missing")
	}
	for i, expected := range r.Checksums {
		actual, e := archiveHash(filepath.Join(path, fmt.Sprintf("%d.tar", i)), false)
		if e != nil {
			return e
		}
		if actual != expected {
			return errors.New("backup archive checksum mismatch; current data retained")
		}
	}
	if _, err = b.compose(ctx, "stop", "gui", "mowgli"); err != nil {
		return err
	}
	for i, source := range r.Sources {
		// Preserve the failed data instead of recursively deleting it. Refuse a
		// symlink or an unexpected source, and restore into the same mounted dir.
		info, e := os.Lstat(source)
		if e != nil || !info.IsDir() || info.Mode()&os.ModeSymlink != 0 {
			return errors.New("backup source is not a directory")
		}
		entries, e := os.ReadDir(source)
		if e != nil {
			return e
		}
		failed := filepath.Join(path, fmt.Sprintf("failed-%d-%d", i, time.Now().UnixNano()))
		if e = os.Mkdir(failed, 0700); e != nil {
			return e
		}
		for _, entry := range entries {
			if _, e = command(ctx, "mv", "--", filepath.Join(source, entry.Name()), failed); e != nil {
				return e
			}
		}
		if _, e = command(ctx, "tar", "--numeric-owner", "-xpf", filepath.Join(path, fmt.Sprintf("%d.tar", i)), "-C", source); e != nil {
			return e
		}
	}
	if len(r.Override) > 0 {
		if err = AtomicWrite(b.override(), r.Override, 0644); err != nil {
			return err
		}
	} else if err = os.Remove(b.override()); err != nil && !os.IsNotExist(err) {
		return err
	}
	if len(r.Compose) > 0 {
		if err = AtomicWrite(filepath.Join(b.Config.Directory, "docker-compose.yaml"), r.Compose, 0644); err != nil {
			return err
		}
	}
	if len(r.Env) > 0 {
		if err = AtomicWrite(filepath.Join(b.Config.Directory, ".env"), r.Env, 0600); err != nil {
			return err
		}
	}
	return nil
}

func archiveHash(path string, sync bool) (string, error) {
	f, err := os.Open(path)
	if err != nil {
		return "", err
	}
	defer f.Close()
	if sync {
		if err = f.Sync(); err != nil {
			return "", err
		}
	}
	h := sha256.New()
	if _, err = io.Copy(h, f); err != nil {
		return "", err
	}
	return hex.EncodeToString(h.Sum(nil)), nil
}
