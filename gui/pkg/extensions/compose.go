package extensions

import (
	"bytes"
	"context"
	"crypto/rand"
	"crypto/sha256"
	"encoding/hex"
	"encoding/json"
	"errors"
	"io"
	"net"
	"os"
	"path/filepath"
	"reflect"
	"regexp"
	"runtime"
	"sort"
	"strconv"
	"strings"
	"time"
)

const ownerLabel = "garden.mowgli.extensions.id"
const releaseLabel = "garden.mowgli.extensions.release"

var resourceName = regexp.MustCompile(`^[a-z0-9][a-z0-9_-]{0,99}$`)
var pinnedImage = regexp.MustCompile(`^[a-zA-Z0-9][a-zA-Z0-9./_:-]*@sha256:[a-f0-9]{64}$`)
var digestHex = regexp.MustCompile(`^[a-f0-9]{64}$`)

type composeSpec struct {
	Name     string                    `json:"name,omitempty"`
	Services map[string]composeService `json:"services"`
	Networks map[string]composeNetwork `json:"networks,omitempty"`
	Volumes  map[string]composeVolume  `json:"volumes,omitempty"`
}
type composeService struct {
	Image         string                     `json:"image"`
	ContainerName string                     `json:"container_name,omitempty"`
	User          string                     `json:"user,omitempty"`
	ReadOnly      bool                       `json:"read_only"`
	CapDrop       []string                   `json:"cap_drop"`
	CapAdd        []string                   `json:"cap_add,omitempty"`
	SecurityOpt   []string                   `json:"security_opt"`
	Restart       string                     `json:"restart,omitempty"`
	Init          bool                       `json:"init,omitempty"`
	Command       []string                   `json:"command,omitempty"`
	Entrypoint    []string                   `json:"entrypoint,omitempty"`
	Environment   map[string]string          `json:"environment,omitempty"`
	Labels        map[string]string          `json:"labels,omitempty"`
	Volumes       []composeMount             `json:"volumes,omitempty"`
	Networks      map[string]json.RawMessage `json:"networks,omitempty"`
	DNS           []string                   `json:"dns,omitempty"`
	ExtraHosts    []string                   `json:"extra_hosts,omitempty"`
	Tmpfs         []string                   `json:"tmpfs,omitempty"`
	Healthcheck   *composeHealth             `json:"healthcheck,omitempty"`
}
type composeHealth struct {
	Test        []string `json:"test,omitempty"`
	Interval    string   `json:"interval,omitempty"`
	Timeout     string   `json:"timeout,omitempty"`
	StartPeriod string   `json:"start_period,omitempty"`
	Retries     int      `json:"retries,omitempty"`
	Disable     bool     `json:"disable,omitempty"`
}
type composeMount struct {
	Type     string `json:"type"`
	Source   string `json:"source"`
	Target   string `json:"target"`
	ReadOnly bool   `json:"read_only,omitempty"`
	Bind     *struct {
		CreateHostPath bool `json:"create_host_path"`
	} `json:"bind,omitempty"`
	Volume *struct {
		NoCopy bool `json:"nocopy,omitempty"`
	} `json:"volume,omitempty"`
}
type composeNetwork struct {
	Name     string `json:"name"`
	Driver   string `json:"driver,omitempty"`
	Internal bool   `json:"internal,omitempty"`
	IPAM     *struct {
		Config []struct {
			Subnet  string `json:"subnet"`
			Gateway string `json:"gateway,omitempty"`
		} `json:"config,omitempty"`
	} `json:"ipam,omitempty"`
}
type composeVolume struct {
	Name     string `json:"name"`
	External bool   `json:"external"`
}

func ValidateCompose(bundle Bundle, binding Binding) error {
	_, err := validatedCompose(bundle, binding)
	return err
}
func validatedCompose(bundle Bundle, binding Binding) (composeSpec, error) {
	var s composeSpec
	if err := validateBundle(bundle); err != nil {
		return s, err
	}
	if err := ValidateBinding(binding); err != nil {
		return s, err
	}
	if bundle.Manifest.Runtime.Kind != "compose" {
		return s, errors.New("managed runtime must be compose")
	}
	legacyConnect := binding.Project == "mowgli-connect" && bundle.Manifest.ID == "mowgli-connect" && bundle.Manifest.Profile == "connect"
	if !resourceName.MatchString(binding.Project) || strings.HasPrefix(binding.Project, "mowgli-") && !strings.Contains(binding.Project, "-ext-") && !legacyConnect {
		return s, errors.New("invalid or reserved extension project")
	}
	if len(bundle.Compose) == 0 || len(bundle.Compose) > 1<<20 {
		return s, errors.New("compose must be self-contained normalized JSON")
	}
	if err := strictJSON(bundle.Compose, &s, 1<<20); err != nil {
		return s, err
	}
	canonical, err := json.Marshal(s)
	if err != nil {
		return s, errors.New("invalid compose serialization")
	}
	if bytes.Contains(canonical, []byte("$")) {
		return s, errors.New("Compose dollar literals and interpolation are unsupported")
	}
	if s.Name != "" && s.Name != binding.Project {
		return s, errors.New("compose project differs from binding")
	}
	if len(s.Networks) == 0 {
		return s, errors.New("explicit isolated bridge network required")
	}
	if len(s.Services) == 0 {
		return s, errors.New("compose requires services")
	}
	if bundle.Manifest.Profile != "isolated" && bundle.Manifest.Profile != "connect" {
		return s, errors.New("unsupported extension isolation profile")
	}
	hasGrant := func(grant string) bool {
		for _, g := range bundle.Manifest.Grants {
			if g == grant {
				return true
			}
		}
		return false
	}
	if bundle.Manifest.Profile == "connect" {
		for _, grant := range []string{"network:gui-api", "network:relay-egress", "sandbox:connect-nft-bootstrap"} {
			if !hasGrant(grant) {
				return s, errors.New("connect requires explicit approved network and bootstrap grants")
			}
		}
	}
	for _, network := range s.Networks {
		if bundle.Manifest.Profile == "isolated" && !network.Internal && !hasGrant("network:egress") {
			return s, errors.New("outbound bridge requires network:egress grant")
		}
	}
	for key, v := range s.Volumes {
		if !resourceName.MatchString(key) || !resourceName.MatchString(v.Name) || !v.External || v.Name == "mowgli_maps" {
			return s, errors.New("data volumes require explicit external physical names")
		}
	}
	for key, n := range s.Networks {
		if !resourceName.MatchString(key) || !resourceName.MatchString(n.Name) || n.Driver != "" && n.Driver != "bridge" {
			return s, errors.New("only explicit private bridge networks are supported")
		}
		if n.IPAM != nil && len(n.IPAM.Config) > 0 {
			return s, errors.New("custom network address pools require a separately reviewed profile")
		}
		if n.Name != binding.Project+"_"+key {
			return s, errors.New("network must belong to the isolated extension project")
		}
	}
	for key, v := range s.Services {
		if len(v.Networks) != 1 {
			return s, errors.New("service requires one explicit isolated bridge")
		}
		if !resourceName.MatchString(key) || !pinnedImage.MatchString(v.Image) {
			return s, errors.New("service requires an immutable sha256 image")
		}
		if v.ContainerName != "" && !resourceName.MatchString(v.ContainerName) {
			return s, errors.New("invalid container name")
		}
		if !v.ReadOnly || !sameStrings(v.CapDrop, []string{"ALL"}) || !sameStrings(v.SecurityOpt, []string{"no-new-privileges:true"}) {
			return s, errors.New("read-only rootfs, cap_drop ALL and no-new-privileges are required")
		}
		if v.Command != nil && len(v.Command) == 0 || v.Entrypoint != nil && len(v.Entrypoint) == 0 {
			return s, errors.New("empty execution overrides are unsupported")
		}
		if bundle.Manifest.Profile == "connect" {
			if v.Entrypoint != nil || v.Command != nil && !sameOrdered(v.Command, []string{"run"}) {
				return s, errors.New("connect bootstrap entrypoint must be preserved")
			}
			if !sameStrings(v.CapAdd, []string{"NET_ADMIN", "SETUID", "SETGID", "KILL"}) || len(v.Networks) != 1 || len(s.Networks) != 1 || v.User != "" && v.User != "0" && v.User != "0:0" {
				return s, errors.New("connect requires its root bootstrap and isolated bridge capabilities")
			}
		} else if len(v.CapAdd) > 0 {
			return s, errors.New("isolated services cannot add capabilities")
		}
		for net, options := range v.Networks {
			if _, ok := s.Networks[net]; !ok {
				return s, errors.New("service references unknown network")
			}
			if string(options) != "null" && string(options) != "{}" {
				return s, errors.New("network attachment options are not supported")
			}
		}
		if v.Restart != "" && v.Restart != "no" && v.Restart != "unless-stopped" {
			return s, errors.New("unsupported restart policy")
		}
		if v.Healthcheck != nil {
			if _, e := expectedHealth(nil, v.Healthcheck); e != nil {
				return s, e
			}
			if v.Healthcheck.Retries < 0 {
				return s, errors.New("invalid healthcheck retries")
			}
		}
		for _, dns := range v.DNS {
			ip := net.ParseIP(dns)
			if ip == nil || !ip.IsGlobalUnicast() {
				return s, errors.New("DNS servers require unicast IP addresses")
			}
		}
		if len(v.Volumes) > 0 && !hasGrant("storage:extension-data") {
			return s, errors.New("mounts require storage:extension-data grant")
		}
		if len(v.ExtraHosts) > 0 && !hasGrant("network:gui-api") {
			return s, errors.New("host-gateway requires network:gui-api grant")
		}
		for _, host := range v.ExtraHosts {
			if host != "host.docker.internal:host-gateway" {
				return s, errors.New("only explicit host-gateway alias is supported")
			}
		}
		for _, tmp := range v.Tmpfs {
			if tmp != "/tmp" && tmp != "/run" {
				return s, errors.New("unsupported tmpfs mount")
			}
		}
		for label := range v.Labels {
			if strings.HasPrefix(label, "garden.mowgli.update.") || strings.HasPrefix(label, "com.docker.compose.") || strings.HasPrefix(label, "garden.mowgli.extensions.") {
				return s, errors.New("reserved management labels are prohibited")
			}
		}
		targets := map[string]bool{}
		for _, m := range v.Volumes {
			if !filepath.IsAbs(m.Target) || filepath.Clean(m.Target) != m.Target || m.Target == "/" || targets[m.Target] || m.Bind != nil && m.Bind.CreateHostPath {
				return s, errors.New("invalid or overlapping mount")
			}
			for target := range targets {
				if strings.HasPrefix(target, m.Target+"/") || strings.HasPrefix(m.Target, target+"/") {
					return s, errors.New("overlapping mounts")
				}
			}
			targets[m.Target] = true
			switch m.Type {
			case "bind":
				if m.Source == "" || !filepath.IsAbs(m.Source) || filepath.Clean(m.Source) != m.Source || !safeBindingPath(m.Source) || m.Source != binding.ConfigDir && m.Source != binding.DataDir || m.Source == binding.ConfigDir && !m.ReadOnly {
					return s, errors.New("bind mounts must use approved config read-only or data directories")
				}
				if m.Volume != nil {
					return s, errors.New("invalid bind options")
				}
				if err := noSymlink(m.Source); err != nil {
					return s, err
				}
				if info, e := os.Stat(m.Source); e == nil && !info.IsDir() {
					return s, errors.New("bind source must be a directory")
				}
			case "volume":
				if _, ok := s.Volumes[m.Source]; !ok || m.Bind != nil {
					return s, errors.New("unknown external data volume")
				}
			default:
				return s, errors.New("unsupported mount type")
			}
		}
	}
	return s, nil
}
func safeBindingPath(p string) bool {
	for _, prefix := range []string{"/dev", "/proc", "/sys", "/run", "/var/run", "/var/lib/docker", "/ros2_ws", "/config", "/mowgli_config"} {
		if p == prefix || strings.HasPrefix(p, prefix+"/") {
			return false
		}
	}
	if p == "/etc" || strings.HasPrefix(p, "/etc/") && !strings.HasPrefix(p, "/etc/mowgli-extensions/") {
		return false
	}
	if p == "/" || p == "/etc" || p == "/var" || p == "/var/lib" || p == "/home" || p == "/tmp" {
		return false
	}
	return true
}
func sameStrings(a, b []string) bool {
	a = append([]string{}, a...)
	b = append([]string{}, b...)
	sort.Strings(a)
	sort.Strings(b)
	return reflect.DeepEqual(a, b)
}
func noSymlink(p string) error {
	for cur := filepath.Clean(p); cur != "/" && cur != "."; cur = filepath.Dir(cur) {
		f, err := os.Lstat(cur)
		if os.IsNotExist(err) {
			continue
		}
		if err != nil || f.Mode()&os.ModeSymlink != 0 {
			return errors.New("extension paths must not traverse symlinks")
		}
	}
	return nil
}
func releaseKey(r Release) (string, error) {
	key := strings.TrimPrefix(r.Digest, "sha256:")
	if !digestHex.MatchString(key) {
		return "", errors.New("invalid release digest")
	}
	return key, nil
}
func (b *RuntimeBackend) artifact(r Release, s composeSpec, write bool) (string, error) {
	if r.Digest != BundleDigest(r.Bundle) {
		return "", errors.New("release digest differs from immutable bundle")
	}
	key, err := releaseKey(r)
	if err != nil {
		return "", err
	}
	dir := filepath.Join(b.Root, "releases", key)
	path := filepath.Join(dir, "compose.json")
	services := make(map[string]composeService, len(s.Services))
	for name, v := range s.Services {
		v.Labels = cloneLabels(v.Labels)
		v.Labels[ownerLabel] = r.Bundle.Manifest.ID
		v.Labels[releaseLabel] = r.Digest
		v.Volumes = append([]composeMount{}, v.Volumes...)
		for i, mount := range v.Volumes {
			if mount.Type == "bind" && mount.Bind == nil {
				v.Volumes[i].Bind = &struct {
					CreateHostPath bool `json:"create_host_path"`
				}{false}
			}
		}
		services[name] = v
	}
	s.Services = services
	s.Name = r.Binding.Project
	raw, err := json.Marshal(s)
	if err != nil {
		return "", errors.New("compose serialization failed")
	}
	// Directory descriptors reject symlinks in every ancestor. The retained
	// descriptor anchors both reads and publication against path replacement.
	d, err := openStoreDir(dir, write)
	if errors.Is(err, os.ErrNotExist) && !write {
		return path, nil
	}
	if err != nil {
		return "", errors.New("unsafe extension release directory")
	}
	defer d.Close()
	read := func() ([]byte, error) {
		f, e := openStoreFile(d, "compose.json")
		if e != nil {
			return nil, e
		}
		defer f.Close()
		info, e := f.Stat()
		if e != nil || info.Mode().Perm() != 0600 {
			return nil, errors.New("unsafe immutable compose permissions")
		}
		return io.ReadAll(io.LimitReader(f, 2<<20))
	}
	existing, err := read()
	if err == nil {
		if !bytes.Equal(existing, raw) {
			return "", errors.New("immutable compose artifact differs")
		}
		return path, nil
	}
	if !errors.Is(err, os.ErrNotExist) {
		return "", errors.New("cannot read immutable compose artifact")
	}
	if !write {
		return path, nil
	}
	anchor, err := os.OpenRoot("/proc/self/fd/" + strconv.FormatUint(uint64(d.Fd()), 10))
	if err != nil {
		return "", errors.New("cannot anchor release directory")
	}
	defer anchor.Close()
	var nonce [16]byte
	if _, err = rand.Read(nonce[:]); err != nil {
		return "", errors.New("cannot create compose artifact")
	}
	tmp := ".compose-" + hex.EncodeToString(nonce[:])
	f, err := anchor.OpenFile(tmp, os.O_WRONLY|os.O_CREATE|os.O_EXCL, 0600)
	if err != nil {
		return "", errors.New("cannot create compose artifact")
	}
	defer anchor.Remove(tmp)
	if _, err = f.Write(raw); err == nil {
		err = f.Sync()
	}
	closeErr := f.Close()
	if err != nil || closeErr != nil {
		return "", errors.New("cannot persist compose artifact")
	}
	// Link publishes once without replacing another writer's immutable file.
	if err = anchor.Link(tmp, "compose.json"); err != nil {
		existing, e := read()
		if e != nil || !bytes.Equal(existing, raw) {
			return "", errors.New("immutable compose artifact collision")
		}
	}
	if err = anchor.Remove(tmp); err != nil {
		return "", errors.New("cannot finalize compose artifact")
	}
	if d.Sync() != nil {
		return "", errors.New("cannot sync release directory")
	}
	return path, nil
}
func cloneLabels(in map[string]string) map[string]string {
	out := map[string]string{}
	for k, v := range in {
		out[k] = v
	}
	return out
}
func (b *RuntimeBackend) mutate(ctx context.Context, r Release, args ...string) error {
	s, err := validatedCompose(r.Bundle, r.Binding)
	if err != nil {
		return err
	}
	if len(args) == 0 || args[0] != "pull" {
		if !b.registered(r) {
			return errors.New("mutation requires durable extension registration")
		}
	}
	if err = b.checkCompose(ctx, r, s, false); err != nil {
		return err
	}
	path, err := b.artifact(r, s, true)
	if err != nil {
		return err
	}
	if args[0] == "up" {
		if err = b.checkImages(ctx, r, s); err != nil {
			return err
		}
	}
	cmd := []string{"compose", "--project-name", r.Binding.Project, "--project-directory", filepath.Dir(path), "-f", path}
	cmd = append(cmd, args...)
	_, err = b.run(ctx, "docker", cmd...)
	if err == nil && args[0] == "pull" {
		err = b.checkImages(ctx, r, s)
	}
	return err
}

type dockerHealth struct {
	Test        []string
	Interval    int64
	Timeout     int64
	StartPeriod int64
	Retries     int
}
type dockerConfig struct {
	Image       string
	Labels      map[string]string
	Env         []string
	User        string
	Cmd         []string
	Entrypoint  []string
	Healthcheck *dockerHealth
	WorkingDir  string
	Volumes     map[string]json.RawMessage
}
type dockerContainer struct {
	ID     string       `json:"Id"`
	Name   string       `json:"Name"`
	Image  string       `json:"Image"`
	Config dockerConfig `json:"Config"`
	State  struct {
		Running   bool
		StartedAt string
		Health    *struct{ Status string }
	} `json:"State"`
	HostConfig struct {
		Privileged      bool
		ReadonlyRootfs  bool
		NetworkMode     string
		PidMode         string
		IpcMode         string
		UTSMode         string
		UsernsMode      string
		Runtime         string
		CapAdd          []string
		CapDrop         []string
		SecurityOpt     []string
		Devices         []json.RawMessage
		DeviceRequests  []json.RawMessage
		Binds           []string
		VolumesFrom     []string
		Links           []string
		GroupAdd        []string
		DNS             []string
		ExtraHosts      []string
		Sysctls         map[string]string
		PortBindings    map[string]json.RawMessage
		PublishAllPorts bool
		Tmpfs           map[string]string
		RestartPolicy   struct{ Name string }
		Init            *bool
	} `json:"HostConfig"`
	Mounts []struct {
		Type        string
		Name        string
		Source      string
		Destination string
		RW          bool
	} `json:"Mounts"`
	NetworkSettings struct{ Networks map[string]json.RawMessage } `json:"NetworkSettings"`
}
type dockerImage struct {
	ID           string `json:"Id"`
	OS           string `json:"Os"`
	Architecture string
	Variant      string
	Config       dockerConfig
}

// Inventory crosses project boundaries, so its template must never include full
// Config or label maps, network details, or other sensitive metadata.
const containerInventoryFormat = `{"Id":{{json .Id}},"Name":{{json .Name}},"Image":{{json .Image}},"Labels":{ {{if .Config.Labels}}"com.docker.compose.project":{{json (index .Config.Labels "com.docker.compose.project")}},"com.docker.compose.service":{{json (index .Config.Labels "com.docker.compose.service")}},"garden.mowgli.extensions.id":{{json (index .Config.Labels "garden.mowgli.extensions.id")}},"garden.mowgli.extensions.release":{{json (index .Config.Labels "garden.mowgli.extensions.release")}}{{end}}},"Mounts":[{{range $i, $m := .Mounts}}{{if $i}},{{end}}{"Type":{{json $m.Type}},"Name":{{json (index $m "Name")}},"Source":{{json $m.Source}},"Destination":{{json $m.Destination}},"RW":{{json $m.RW}}}{{end}}],"Networks":[{{$first := true}}{{range $name, $_ := .NetworkSettings.Networks}}{{if not $first}},{{end}}{{$first = false}}{{json $name}}{{end}}]}`

type dockerContainerInventory struct {
	ID     string `json:"Id"`
	Name   string
	Image  string
	Labels map[string]string
	Mounts []struct {
		Type        string
		Name        string
		Source      string
		Destination string
		RW          bool
	}
	Networks []string
}

func (b *RuntimeBackend) containers(ctx context.Context, r Release) ([]dockerContainer, error) {
	raw, err := b.run(ctx, "docker", "ps", "-aq", "--no-trunc")
	if err != nil {
		return nil, err
	}
	ids := strings.Fields(string(raw))
	if len(ids) == 0 {
		return nil, nil
	}
	requested := make(map[string]bool, len(ids))
	for _, id := range ids {
		if !regexp.MustCompile(`^[a-f0-9]{12,64}$`).MatchString(id) || requested[id] {
			return nil, errors.New("invalid container inventory")
		}
		requested[id] = true
	}
	raw, err = b.run(ctx, "docker", append([]string{"inspect", "--type", "container", "--format", containerInventoryFormat}, ids...)...)
	if err != nil {
		return nil, err
	}
	list := make([]dockerContainer, 0, len(ids))
	owned := []string{}
	positions := map[string]int{}
	for _, line := range bytes.Split(bytes.TrimSpace(raw), []byte("\n")) {
		var basic dockerContainerInventory
		decoder := json.NewDecoder(bytes.NewReader(line))
		decoder.DisallowUnknownFields()
		if decoder.Decode(&basic) != nil || decoder.Decode(new(any)) != io.EOF || !requested[basic.ID] {
			return nil, errors.New("invalid container inventory response")
		}
		delete(requested, basic.ID)
		c := dockerContainer{ID: basic.ID, Name: basic.Name, Image: basic.Image, Mounts: basic.Mounts}
		c.Config.Labels = basic.Labels
		c.NetworkSettings.Networks = map[string]json.RawMessage{}
		for _, name := range basic.Networks {
			c.NetworkSettings.Networks[name] = json.RawMessage(`{}`)
		}
		positions[c.ID] = len(list)
		list = append(list, c)
		if r.Binding.Project != "" && c.Config.Labels["com.docker.compose.project"] == r.Binding.Project {
			owned = append(owned, c.ID)
		}
	}
	if len(requested) != 0 {
		return nil, errors.New("incomplete container inventory response")
	}
	if len(owned) == 0 {
		return list, nil
	}
	// Exact local adoption checks need image defaults, Env and isolation details.
	// Request these only for IDs discovered in this release's exact project; never
	// persist them or include them in observations or command error messages.
	raw, err = b.run(ctx, "docker", append([]string{"inspect", "--type", "container"}, owned...)...)
	if err != nil {
		return nil, err
	}
	var detailed []dockerContainer
	if json.Unmarshal(raw, &detailed) != nil || len(detailed) != len(owned) {
		return nil, errors.New("invalid container inspect response")
	}
	requested = make(map[string]bool, len(owned))
	for _, id := range owned {
		requested[id] = true
	}
	for _, c := range detailed {
		if !requested[c.ID] || c.Config.Labels["com.docker.compose.project"] != r.Binding.Project {
			return nil, errors.New("container inspect identity differs from requested project")
		}
		delete(requested, c.ID)
		list[positions[c.ID]] = c
	}
	return list, nil
}
func (b *RuntimeBackend) image(ctx context.Context, image string) (dockerImage, error) {
	raw, err := b.run(ctx, "docker", "image", "inspect", image)
	if err != nil {
		return dockerImage{}, err
	}
	var list []dockerImage
	if json.Unmarshal(raw, &list) != nil || len(list) != 1 || !strings.HasPrefix(list[0].ID, "sha256:") || !digestHex.MatchString(strings.TrimPrefix(list[0].ID, "sha256:")) {
		return dockerImage{}, errors.New("invalid pinned image identity")
	}
	if list[0].OS != runtime.GOOS || list[0].Architecture != runtime.GOARCH ||
		(list[0].Variant != "" && !(runtime.GOARCH == "arm64" && list[0].Variant == "v8")) {
		return dockerImage{}, errors.New("image platform differs from the local runtime")
	}
	return list[0], nil
}
func (b *RuntimeBackend) checkCompose(ctx context.Context, r Release, s composeSpec, adopt bool, inventory ...[]dockerContainer) error {
	for _, path := range []string{r.Binding.ConfigDir, r.Binding.DataDir} {
		if path == b.Root || strings.HasPrefix(b.Root, path+"/") || strings.HasPrefix(path, b.Root+"/") && (!strings.HasPrefix(path, filepath.Join(b.Root, r.Bundle.Manifest.ID)+"/") || strings.HasPrefix(path, filepath.Join(b.Root, "releases")+"/")) {
			return errors.New("extension storage overlaps manager metadata")
		}
	}
	if _, err := b.artifact(r, s, false); err != nil {
		return err
	}
	var list []dockerContainer
	if len(inventory) != 0 {
		list = inventory[0]
	} else {
		var err error
		list, err = b.containers(ctx, r)
		if err != nil {
			return err
		}
	}
	candidates := []Release{}
	if adopt {
		candidates = append(candidates, r)
	}
	// Observation verifies this exact release against one fresh snapshot. A
	// mutation may also authorize the persisted previous/journal releases.
	if len(inventory) == 0 {
		candidates = append(candidates, b.registeredReleases(r)...)
	}
	seen := map[string]bool{}
	for _, c := range list {
		project := c.Config.Labels["com.docker.compose.project"]
		service := c.Config.Labels["com.docker.compose.service"]
		if project != r.Binding.Project {
			for _, network := range s.Networks {
				if _, attached := c.NetworkSettings.Networks[network.Name]; attached {
					return errors.New("extension network is shared with another project")
				}
			}
			for _, service := range s.Services {
				for _, mount := range service.Volumes {
					for _, actual := range c.Mounts {
						if mount.Type == "volume" && actual.Type == "volume" && actual.Name == s.Volumes[mount.Source].Name || mount.Type == "bind" && actual.Type == "bind" && (actual.Source == mount.Source || strings.HasPrefix(actual.Source, mount.Source+"/") || strings.HasPrefix(mount.Source, actual.Source+"/")) {
							return errors.New("extension storage is shared with another project")
						}
					}
				}
			}
			for key, v := range s.Services {
				name := v.ContainerName
				if name == "" {
					name = r.Binding.Project + "-" + key + "-1"
				}
				if strings.TrimPrefix(c.Name, "/") == name {
					return errors.New("extension container name collision")
				}
			}
			continue
		}
		target, ok := s.Services[service]
		if !ok || seen[service] {
			return errors.New("extension project has unexpected or duplicate service")
		}
		seen[service] = true
		owner := c.Config.Labels[ownerLabel]
		if owner == "" && c.Config.Labels[releaseLabel] != "" {
			return errors.New("incomplete extension ownership labels")
		}
		if owner != "" && owner != r.Bundle.Manifest.ID {
			return errors.New("extension project has another owner")
		}

		// Project labels discover containers, while the registry authorizes them.
		matched := false
		for _, candidate := range candidates {
			if owner != "" && c.Config.Labels[releaseLabel] != candidate.Digest {
				continue
			}
			oldSpec, e := validatedCompose(candidate.Bundle, candidate.Binding)
			if e != nil {
				continue
			}
			target, ok = oldSpec.Services[service]
			if !ok {
				continue
			}
			if e = b.matchContainer(ctx, c, target, oldSpec, candidate); e == nil {
				matched = true
				break
			}
		}
		if !matched {
			return errors.New("existing container does not match an authorized release")
		}

	}
	if adopt && len(inventory) == 0 && len(seen) != len(s.Services) {
		return errors.New("adoption requires the complete exact service set")
	}
	return b.checkResources(ctx, r, s, adopt)
}
func (b *RuntimeBackend) matchContainer(ctx context.Context, c dockerContainer, v composeService, s composeSpec, r Release) error {
	image, err := b.image(ctx, v.Image)
	if err != nil {
		return err
	}
	if err = validateImage(r, v, image); err != nil {
		return err
	}
	if c.Image != image.ID {
		return errors.New("container differs from pinned image")
	}
	h := c.HostConfig
	if h.Privileged || !h.ReadonlyRootfs || h.PidMode != "" || h.IpcMode != "" && h.IpcMode != "private" && h.IpcMode != "shareable" || h.UTSMode != "" || h.UsernsMode != "" || h.Runtime != "" && h.Runtime != "runc" || len(h.Devices) > 0 || len(h.DeviceRequests) > 0 || len(h.VolumesFrom) > 0 || len(h.Links) > 0 || len(h.GroupAdd) > 0 || !sameStrings(h.DNS, v.DNS) || len(h.Sysctls) > 0 || len(h.PortBindings) > 0 || h.PublishAllPorts || !sameStrings(h.CapDrop, v.CapDrop) || !sameStrings(h.CapAdd, v.CapAdd) || !sameStrings(normalizeSecurity(h.SecurityOpt), normalizeSecurity(v.SecurityOpt)) || !sameStrings(h.ExtraHosts, v.ExtraHosts) {
		return errors.New("container isolation differs from approved compose")
	}
	if h.RestartPolicy.Name != v.Restart && !(h.RestartPolicy.Name == "no" && v.Restart == "") || h.Init != nil && *h.Init != v.Init || h.Init == nil && v.Init {
		return errors.New("container process policy differs")
	}
	expectedNetworks := []string{}
	for name := range v.Networks {
		expectedNetworks = append(expectedNetworks, s.Networks[name].Name)
	}
	actualNetworks := []string{}
	for name := range c.NetworkSettings.Networks {
		actualNetworks = append(actualNetworks, name)
	}
	if !sameStrings(actualNetworks, expectedNetworks) || len(expectedNetworks) != 1 || h.NetworkMode != expectedNetworks[0] {
		return errors.New("container network identity differs")
	}
	tmpfs := []string{}
	for target, options := range h.Tmpfs {
		if options != "" {
			return errors.New("unsupported actual tmpfs options")
		}
		tmpfs = append(tmpfs, target)
	}
	if !sameStrings(tmpfs, v.Tmpfs) {
		return errors.New("container tmpfs differs")
	}
	want := image.Config
	if v.User != "" {
		want.User = v.User
	}
	if v.Command != nil {
		want.Cmd = v.Command
	}
	if v.Entrypoint != nil {
		want.Entrypoint = v.Entrypoint
	}
	if c.Config.User != want.User || !sameOrdered(c.Config.Cmd, want.Cmd) || !sameOrdered(c.Config.Entrypoint, want.Entrypoint) || c.Config.WorkingDir != want.WorkingDir {
		return errors.New("container execution configuration differs")
	}
	env := envMap(image.Config.Env)
	for k, v := range v.Environment {
		env[k] = v
	}
	if len(envMap(c.Config.Env)) != len(c.Config.Env) || !reflect.DeepEqual(envMap(c.Config.Env), env) {
		return errors.New("container environment differs")
	}
	health, err := expectedHealth(image.Config.Healthcheck, v.Healthcheck)
	if err != nil {
		return err
	}
	if !reflect.DeepEqual(c.Config.Healthcheck, health) {
		return errors.New("container healthcheck differs")
	}
	expectedLabels := cloneLabels(image.Config.Labels)
	for k, value := range v.Labels {
		expectedLabels[k] = value
	}
	for k, value := range expectedLabels {
		if actual, ok := c.Config.Labels[k]; !ok || actual != value {
			return errors.New("container labels differ")
		}
	}
	for k, value := range c.Config.Labels {
		if strings.HasPrefix(k, "garden.mowgli.update.") {
			return errors.New("container has conflicting update ownership")
		}
		if strings.HasPrefix(k, "com.docker.compose.") || k == ownerLabel || k == releaseLabel {
			continue
		}
		if expected, ok := expectedLabels[k]; !ok || expected != value {
			return errors.New("container has unapproved labels")
		}
	}
	persistentMounts := 0
	for _, mount := range c.Mounts {
		if mount.Type != "tmpfs" {
			persistentMounts++
		}
	}
	if persistentMounts != len(v.Volumes) {
		return errors.New("container data mount identity differs")
	}
	for _, mount := range v.Volumes {
		found := false
		for _, actual := range c.Mounts {
			source := mount.Source
			if mount.Type == "volume" {
				source = s.Volumes[mount.Source].Name
			}
			if actual.Destination == mount.Target && actual.Type == mount.Type && actual.RW == !mount.ReadOnly && (mount.Type == "volume" && actual.Name == source || mount.Type == "bind" && actual.Source == source) {
				found = true
			}
		}
		if !found {
			return errors.New("container data mount identity differs")
		}
	}
	expectedName := v.ContainerName
	if expectedName == "" {
		expectedName = r.Binding.Project + "-" + c.Config.Labels["com.docker.compose.service"] + "-1"
	}
	if strings.TrimPrefix(c.Name, "/") != expectedName {
		return errors.New("container name differs")
	}
	if oneoff := c.Config.Labels["com.docker.compose.oneoff"]; oneoff != "" && !strings.EqualFold(oneoff, "false") {
		return errors.New("one-off container cannot be adopted as a service")
	}
	if replica := c.Config.Labels["com.docker.compose.container-number"]; replica != "" && replica != "1" {
		return errors.New("scaled service identity is unsupported")
	}
	return nil
}
func sameOrdered(a, b []string) bool { return strings.Join(a, "\x00") == strings.Join(b, "\x00") }
func normalizeSecurity(in []string) []string {
	out := append([]string{}, in...)
	for i, s := range out {
		if s == "no-new-privileges" {
			out[i] = "no-new-privileges:true"
		}
	}
	return out
}
func envMap(in []string) map[string]string {
	out := map[string]string{}
	for _, item := range in {
		key, value, ok := strings.Cut(item, "=")
		if ok {
			out[key] = value
		}
	}
	return out
}
func expectedHealth(base *dockerHealth, override *composeHealth) (*dockerHealth, error) {
	if override == nil {
		return base, nil
	}
	out := dockerHealth{}
	if base != nil {
		out = *base
	}
	if override.Disable {
		return &dockerHealth{Test: []string{"NONE"}}, nil
	}
	if override.Test != nil {
		out.Test = override.Test
	}
	for _, field := range []struct {
		value  string
		target *int64
	}{{override.Interval, &out.Interval}, {override.Timeout, &out.Timeout}, {override.StartPeriod, &out.StartPeriod}} {
		value, target := field.value, field.target
		if value != "" {
			d, e := time.ParseDuration(value)
			if e != nil || d <= 0 {
				return nil, errors.New("invalid healthcheck duration")
			}
			*target = int64(d)
		}
	}
	if override.Retries != 0 {
		out.Retries = override.Retries
	}
	return &out, nil
}
func (b *RuntimeBackend) checkResources(ctx context.Context, r Release, s composeSpec, adopt bool) error {
	state, err := (Store{Root: b.Root}).Load()
	if err != nil {
		return errors.New("extension ownership metadata is unavailable")
	}
	claims := map[string]bool{}
	for _, name := range VolumeClaims(r) {
		claims[name] = true
	}
	for id, entry := range state.Entries {
		if id == r.Bundle.Manifest.ID {
			continue
		}
		for _, release := range []*Release{&entry.Active, entry.Previous} {
			if release != nil {
				for _, name := range VolumeClaims(*release) {
					if claims[name] {
						return errors.New("external volume belongs to another registered extension")
					}
				}
			}
		}
	}
	for id, retained := range state.Retained {
		if id == r.Bundle.Manifest.ID {
			continue
		}
		for _, name := range retained.VolumeNames {
			if claims[name] {
				return errors.New("external volume belongs to retained extension data")
			}
		}
	}
	raw, err := b.run(ctx, "docker", "network", "ls", "--format", "{{json .}}")
	if err != nil {
		return err
	}
	for _, line := range strings.Split(strings.TrimSpace(string(raw)), "\n") {
		if line == "" {
			continue
		}
		var network struct{ Name string }
		if json.Unmarshal([]byte(line), &network) != nil {
			return errors.New("invalid network inventory")
		}
		for key, want := range s.Networks {
			if network.Name != want.Name {
				continue
			}
			if !adopt && len(b.registeredReleases(r)) == 0 {
				return errors.New("existing extension network requires adoption")
			}
			out, e := b.run(ctx, "docker", "network", "inspect", network.Name)
			if e != nil {
				return e
			}
			var actual []struct {
				Labels   map[string]string
				Driver   string
				Internal bool
				Options  map[string]string
				IPAM     struct {
					Driver string
					Config []struct {
						Subnet  string
						Gateway string
					}
				}
			}
			if json.Unmarshal(out, &actual) != nil || len(actual) != 1 {
				return errors.New("invalid network identity")
			}
			n := actual[0]
			if n.Driver != "bridge" || n.Internal != want.Internal || len(n.Options) > 0 || n.IPAM.Driver != "" && n.IPAM.Driver != "default" || n.Labels["com.docker.compose.project"] != r.Binding.Project || n.Labels["com.docker.compose.network"] != key {
				return errors.New("extension network collision")
			}
			for label := range n.Labels {
				if strings.HasPrefix(label, "garden.mowgli.update.") {
					return errors.New("extension network has core updater ownership")
				}
			}
			if want.IPAM != nil && len(want.IPAM.Config) > 0 {
				if len(n.IPAM.Config) != len(want.IPAM.Config) {
					return errors.New("extension network subnet identity differs")
				}
				for i, config := range want.IPAM.Config {
					if n.IPAM.Config[i].Subnet != config.Subnet || config.Gateway != "" && n.IPAM.Config[i].Gateway != config.Gateway {
						return errors.New("extension network address identity differs")
					}
				}
			}
		}
	}
	for _, v := range s.Volumes {
		raw, e := b.run(ctx, "docker", "volume", "inspect", v.Name)
		if e != nil {
			return errors.New("external data volume is missing or inaccessible")
		}
		var list []struct {
			Name    string
			Driver  string
			Options map[string]string
			Labels  map[string]string
		}
		if json.Unmarshal(raw, &list) != nil || len(list) != 1 || list[0].Name != v.Name || list[0].Driver != "local" || len(list[0].Options) > 0 {
			return errors.New("external data volume identity is unsafe")
		}
		for label := range list[0].Labels {
			if strings.HasPrefix(label, "garden.mowgli.update.") {
				return errors.New("external data volume has core updater ownership")
			}
		}
		if project := list[0].Labels["com.docker.compose.project"]; project != "" && project != r.Binding.Project {
			return errors.New("external data volume belongs to another project")
		}
	}
	return nil
}
func (b *RuntimeBackend) observeCompose(ctx context.Context, r Release) (Observation, error) {
	o := Observation{Process: "unknown", Application: "unknown", Data: "unknown", Services: map[string]string{}}
	s, err := validatedCompose(r.Bundle, r.Binding)
	if err != nil {
		return o, err
	}
	list, err := b.containers(ctx, r)
	if err != nil {
		return o, err
	}
	if err = b.checkCompose(ctx, r, s, true, list); err != nil {
		return o, err
	}
	identities := []string{}
	allRunning := true
	allHealthy := true
	running := 0
	anyUnhealthy, anyStarting := false, false
	found := 0
	for _, c := range list {
		if c.Config.Labels["com.docker.compose.project"] != r.Binding.Project {
			continue
		}
		service := c.Config.Labels["com.docker.compose.service"]
		if _, ok := s.Services[service]; !ok {
			return o, errors.New("observation service changed")
		}
		if _, duplicate := o.Services[service]; duplicate {
			return o, errors.New("duplicate observed service")
		}
		if c.Config.Labels[releaseLabel] != "" && c.Config.Labels[releaseLabel] != r.Digest {
			return o, errors.New("observation belongs to another release")
		}
		found++
		state := "stopped"
		if c.State.Running {
			state = "running"
			running++
		} else {
			allRunning = false
		}
		o.Services[service] = state
		if c.State.Running {
			started, e := time.Parse(time.RFC3339Nano, c.State.StartedAt)
			if e != nil || started.IsZero() {
				return o, errors.New("running container process instance is unavailable")
			}
		}
		identities = append(identities, c.ID+"@"+c.State.StartedAt)
		hasHealth := c.Config.Healthcheck != nil && len(c.Config.Healthcheck.Test) > 0 && c.Config.Healthcheck.Test[0] != "NONE" && c.State.Health != nil
		if hasHealth && c.State.Health.Status == "unhealthy" {
			anyUnhealthy = true
		}
		if hasHealth && c.State.Health.Status == "starting" {
			anyStarting = true
		}
		if c.Config.Healthcheck == nil || len(c.Config.Healthcheck.Test) == 0 || c.Config.Healthcheck.Test[0] == "NONE" || c.State.Health == nil || c.State.Health.Status != "healthy" {
			allHealthy = false
		}
	}
	if running == 0 {
		o.Process = "stopped"
	} else if found == len(s.Services) && allRunning {
		o.Process = "running"
	} else {
		o.Process = "partial"
	}
	if found == len(s.Services) && allRunning && r.Bundle.Manifest.Health == "application" {
		switch {
		case anyUnhealthy:
			o.Application = "unhealthy"
		case anyStarting:
			o.Application = "starting"
		case allHealthy:
			o.Application = "healthy"
		}
	}
	sort.Strings(identities)
	sum := sha256.Sum256([]byte(strings.Join(identities, "\n")))
	if found > 0 {
		o.Identity = r.Digest + ":" + hex.EncodeToString(sum[:])
	}
	return o, nil
}

// Registration is authorization, while project labels by themselves are only
// discovery. The manager holds its host lock across this read and mutation.
func (b *RuntimeBackend) registeredReleases(r Release) []Release {
	state, err := (Store{Root: b.Root}).Load()
	if err != nil {
		return nil
	}
	var out []Release
	add := func(e *Entry) {
		if e == nil {
			return
		}
		for _, other := range []*Release{&e.Active, e.Previous} {
			if other != nil && other.Binding == r.Binding && other.Bundle.Manifest.ID == r.Bundle.Manifest.ID {
				out = append(out, *other)
			}
		}
	}
	if e, ok := state.Entries[r.Bundle.Manifest.ID]; ok {
		add(&e)
	}
	if j := state.Journal; j != nil && j.ID == r.Bundle.Manifest.ID {
		add(j.Before)
		add(j.Target)
	}
	return out
}
func (b *RuntimeBackend) registered(r Release) bool {
	for _, other := range b.registeredReleases(r) {
		if other.Digest == r.Digest {
			return true
		}
	}
	return false
}

// StorageIdentity captures only persistent storage layout. It deliberately
// excludes image, environment, and credentials so upgrades can retain data.
func StorageIdentity(r Release) (string, error) {
	var projection struct {
		Services map[string]struct {
			Volumes []composeMount `json:"volumes"`
		} `json:"services"`
		Volumes map[string]composeVolume `json:"volumes"`
	}
	if r.Bundle.Manifest.Runtime.Kind == "compose" {
		if json.Unmarshal(r.Bundle.Compose, &projection) != nil || projection.Services == nil {
			return "", errors.New("invalid compose storage layout")
		}
	}
	type storageMount struct {
		Type     string `json:"type"`
		Source   string `json:"source"`
		Target   string `json:"target"`
		ReadOnly bool   `json:"read_only"`
	}
	layout := struct {
		Volumes  map[string]composeVolume  `json:"volumes"`
		Services map[string][]storageMount `json:"services"`
	}{Volumes: projection.Volumes, Services: map[string][]storageMount{}}
	for name, service := range projection.Services {
		mounts := []storageMount{}
		for _, v := range service.Volumes {
			if v.Type != "bind" && v.Type != "volume" || v.Source == "" || !filepath.IsAbs(v.Target) {
				return "", errors.New("invalid compose storage mount")
			}
			mounts = append(mounts, storageMount{v.Type, v.Source, v.Target, v.ReadOnly})
		}
		if len(mounts) > 0 {
			sort.Slice(mounts, func(i, j int) bool { return mounts[i].Target < mounts[j].Target })
			layout.Services[name] = mounts
		}
	}
	raw, err := json.Marshal(layout)
	if err != nil {
		return "", errors.New("invalid storage identity")
	}
	sum := sha256.Sum256(raw)
	return "sha256:" + hex.EncodeToString(sum[:]), nil
}
func VolumeClaims(r Release) []string {
	var projection struct {
		Volumes map[string]composeVolume `json:"volumes"`
	}
	if json.Unmarshal(r.Bundle.Compose, &projection) != nil {
		return nil
	}
	out := []string{}
	for _, v := range projection.Volumes {
		if v.Name != "" {
			out = append(out, v.Name)
		}
	}
	sort.Strings(out)
	return out
}

// NormalizeCompose removes only Compose's known representation defaults. It
// leaves every authority-bearing field present for the closed policy validator.
// Callers must still approve immutable images and explicit external volumes.
func NormalizeCompose(raw []byte) ([]byte, error) {
	if len(raw) > 1<<20 {
		return nil, errors.New("compose exceeds size limit")
	}
	d := json.NewDecoder(bytes.NewReader(raw))
	d.UseNumber()
	value, err := composeValue(d, 0)
	if err != nil {
		return nil, err
	}
	if _, err = d.Token(); err != io.EOF {
		return nil, errors.New("trailing compose data")
	}
	root, ok := value.(map[string]any)
	if !ok {
		return nil, errors.New("compose must be an object")
	}
	services, ok := root["services"].(map[string]any)
	if !ok {
		return nil, errors.New("compose services must be an object")
	}
	for _, value := range services {
		service, ok := value.(map[string]any)
		if !ok {
			return nil, errors.New("compose service must be an object")
		}
		for _, key := range []string{"command", "entrypoint"} {
			if v, present := service[key]; present && v == nil {
				delete(service, key)
			}
		}
		if networks, ok := service["networks"].(map[string]any); ok {
			for name, attachment := range networks {
				if attachment == nil {
					networks[name] = map[string]any{}
				}
			}
		}
		if hosts, ok := service["extra_hosts"].([]any); ok {
			for i, host := range hosts {
				if host == "host.docker.internal=host-gateway" {
					hosts[i] = "host.docker.internal:host-gateway"
				}
			}
		}
	}
	out, err := json.Marshal(root)
	if err != nil {
		return nil, errors.New("invalid compose JSON")
	}
	var spec composeSpec
	if err = strictJSON(out, &spec, 1<<20); err != nil {
		return nil, errors.New("unsupported normalized compose configuration")
	}
	return out, nil
}
func composeValue(d *json.Decoder, depth int) (any, error) {
	if depth > 64 {
		return nil, errors.New("compose nesting too deep")
	}
	token, err := d.Token()
	if err != nil {
		return nil, errors.New("invalid compose JSON")
	}
	delim, ok := token.(json.Delim)
	if !ok {
		return token, nil
	}
	switch delim {
	case '{':
		out := map[string]any{}
		for d.More() {
			keyToken, e := d.Token()
			if e != nil {
				return nil, errors.New("invalid compose object")
			}
			key, ok := keyToken.(string)
			if !ok {
				return nil, errors.New("invalid compose object key")
			}
			if _, duplicate := out[key]; duplicate {
				return nil, errors.New("duplicate compose key")
			}
			value, e := composeValue(d, depth+1)
			if e != nil {
				return nil, e
			}
			out[key] = value
		}
		if _, err = d.Token(); err != nil {
			return nil, err
		}
		return out, nil
	case '[':
		out := []any{}
		for d.More() {
			value, e := composeValue(d, depth+1)
			if e != nil {
				return nil, e
			}
			out = append(out, value)
		}
		if _, err = d.Token(); err != nil {
			return nil, err
		}
		return out, nil
	default:
		return nil, errors.New("unexpected compose delimiter")
	}
}

func (b *RuntimeBackend) checkImages(ctx context.Context, r Release, s composeSpec) error {
	for _, service := range s.Services {
		image, err := b.image(ctx, service.Image)
		if err != nil {
			return err
		}
		if err = validateImage(r, service, image); err != nil {
			return err
		}
	}
	return nil
}
func validateImage(r Release, service composeService, image dockerImage) error {
	for label := range image.Config.Labels {
		if strings.HasPrefix(label, "garden.mowgli.update.") || strings.HasPrefix(label, "garden.mowgli.extensions.") || strings.HasPrefix(label, "com.docker.compose.") {
			return errors.New("image contains conflicting runtime ownership labels")
		}
	}
	for target := range image.Config.Volumes {
		found := false
		for _, mount := range service.Volumes {
			if mount.Target == target {
				found = true
			}
		}
		if !found {
			return errors.New("image would create an unapproved anonymous data volume")
		}
	}
	if r.Bundle.Manifest.Profile == "connect" {
		if !sameOrdered(image.Config.Entrypoint, []string{"/usr/local/bin/entrypoint.sh"}) || !sameOrdered(image.Config.Cmd, []string{"run"}) || image.Config.User != "" && image.Config.User != "0" && image.Config.User != "0:0" {
			return errors.New("connect image does not preserve the reviewed root bootstrap")
		}
	}
	return nil
}
