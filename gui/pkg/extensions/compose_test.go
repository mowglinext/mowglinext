package extensions

import (
	"bytes"
	"context"
	"encoding/json"
	"errors"
	"os"
	"os/exec"
	"path/filepath"
	"reflect"
	"runtime"
	"strconv"
	"strings"
	"testing"
	"text/template"
	"time"
)

type adapterRunner struct {
	calls          [][]string
	containers     []dockerContainer
	image          dockerImage
	network        string
	networkInspect string
	volume         string
	fail           string
	inventoryReads [][]byte
	compose        func([]string) error
}

func (f *adapterRunner) Run(ctx context.Context, name string, args ...string) ([]byte, error) {
	if _, ok := ctx.Deadline(); !ok {
		return nil, errors.New("deadline missing")
	}
	call := append([]string{name}, args...)
	f.calls = append(f.calls, call)
	key := strings.Join(call, " ")
	if f.fail != "" && strings.Contains(key, f.fail) {
		return nil, errors.New("SECRET command output")
	}
	switch {
	case len(args) > 0 && args[0] == "ps":
		ids := []string{}
		for _, c := range f.containers {
			ids = append(ids, c.ID)
		}
		return []byte(strings.Join(ids, "\n")), nil
	case len(args) > 0 && args[0] == "inspect":
		if len(args) < 4 || args[1] != "--type" || args[2] != "container" {
			return nil, errors.New("unexpected inspect arguments")
		}
		ids := args[3:]
		format := ""
		if ids[0] == "--format" {
			if len(ids) < 3 {
				return nil, errors.New("missing inspect projection or IDs")
			}
			format, ids = ids[1], ids[2:]
		}
		selected := []dockerContainer{}
		for _, id := range ids {
			found := false
			for _, c := range f.containers {
				if c.ID == id {
					selected = append(selected, c)
					found = true
					break
				}
			}
			if !found {
				return nil, errors.New("container not found")
			}
		}
		if format == "" {
			return json.Marshal(selected)
		}
		projection, err := template.New("inspect").Option("missingkey=error").Funcs(template.FuncMap{"json": func(v any) (string, error) {
			raw, err := json.Marshal(v)
			return string(raw), err
		}}).Parse(format)
		if err != nil {
			return nil, err
		}
		var out bytes.Buffer
		for _, c := range selected {
			// Docker inspect templates expose the decoded inspect object.
			raw, _ := json.Marshal(c)
			var object map[string]any
			if err := json.Unmarshal(raw, &object); err != nil {
				return nil, err
			}
			if mounts, ok := object["Mounts"].([]any); ok {
				for _, mount := range mounts {
					value := mount.(map[string]any)
					if value["Type"] == "bind" {
						delete(value, "Name") // real Engine bind mounts omit this optional key
					}
				}
			}
			if err := projection.Execute(&out, object); err != nil {
				return nil, err
			}
			out.WriteByte('\n')
		}
		f.inventoryReads = append(f.inventoryReads, append([]byte(nil), out.Bytes()...))
		return out.Bytes(), nil
	case len(args) > 1 && args[0] == "image" && args[1] == "inspect":
		return json.Marshal([]dockerImage{f.image})
	case len(args) > 1 && args[0] == "network" && args[1] == "ls":
		return []byte(f.network), nil
	case len(args) > 1 && args[0] == "network" && args[1] == "inspect":
		return []byte(f.networkInspect), nil
	case len(args) > 1 && args[0] == "volume" && args[1] == "inspect":
		return []byte(f.volume), nil
	case len(args) > 0 && args[0] == "compose":
		if f.compose != nil {
			return nil, f.compose(args)
		}
		return nil, nil
	}
	return nil, errors.New("unexpected command")
}

func TestInventoryProjectionAllowsUnnamedBindMounts(t *testing.T) {
	r := adapterRelease(t)
	other := adapterContainer(t, r)
	other.Config.Labels["com.docker.compose.project"] = "other-project"
	b, _ := adapterBackend(t, r, other)
	values, err := b.containers(context.Background(), r)
	if err != nil || len(values) != 1 || len(values[0].Mounts) != 2 {
		t.Fatalf("unnamed bind mount broke bounded inventory: %+v %v", values, err)
	}
	if values[0].Mounts[0].Name != "" || values[0].Mounts[0].Source != other.Mounts[0].Source {
		t.Fatal("bind identity was lost")
	}
}

func TestObservationUsesOneFreshInventoryAndImageCheck(t *testing.T) {
	r := adapterRelease(t)
	b, f := adapterBackend(t, r, adapterContainer(t, r))
	for _, running := range []bool{true, false} {
		f.containers[0].State.Running = running
		f.calls = nil
		observed, err := b.Observe(context.Background(), r)
		if err != nil || (observed.Process == "running") != running {
			t.Fatal(observed, err)
		}
		inventories, images := 0, 0
		for _, call := range f.calls {
			if len(call) > 1 && call[1] == "ps" {
				inventories++
			}
			if len(call) > 2 && call[1] == "image" && call[2] == "inspect" {
				images++
			}
		}
		if inventories != 1 || images != 1 {
			t.Fatalf("redundant observation work: inventories=%d image checks=%d", inventories, images)
		}
	}
}
func adapterRelease(t *testing.T) Release {
	t.Helper()
	r := schemaTestRelease()
	r.Bundle.Manifest.Grants = append(r.Bundle.Manifest.Grants, "network:egress")
	r.Binding.ConfigDir = filepath.Join(t.TempDir(), "config")
	r.Binding.DataDir = filepath.Join(t.TempDir(), "data")
	s := composeSpec{Services: map[string]composeService{"agent": {Image: "example/agent@sha256:" + strings.Repeat("a", 64), ReadOnly: true, CapDrop: []string{"ALL"}, SecurityOpt: []string{"no-new-privileges:true"}, Networks: map[string]json.RawMessage{"private": json.RawMessage(`{}`)}, Volumes: []composeMount{{Type: "bind", Source: r.Binding.ConfigDir, Target: "/config", ReadOnly: true}, {Type: "bind", Source: r.Binding.DataDir, Target: "/data"}}}}, Networks: map[string]composeNetwork{"private": {Name: r.Binding.Project + "_private", Driver: "bridge"}}}
	r.Bundle.Compose, _ = json.Marshal(s)
	r.Digest = BundleDigest(r.Bundle)
	return r
}
func adapterContainer(t *testing.T, r Release) dockerContainer {
	t.Helper()
	s, err := validatedCompose(r.Bundle, r.Binding)
	if err != nil {
		t.Fatal(err)
	}
	v := s.Services["agent"]
	var c dockerContainer
	c.ID = strings.Repeat("1", 64)
	c.Name = "/" + r.Binding.Project + "-agent-1"
	c.Image = "sha256:" + strings.Repeat("b", 64)
	c.Config.Image = v.Image
	c.Config.Labels = map[string]string{"com.docker.compose.project": r.Binding.Project, "com.docker.compose.service": "agent"}
	c.State.Running = true
	c.State.StartedAt = "2026-11-10T10:00:00.123456789Z"
	c.HostConfig.ReadonlyRootfs = true
	c.HostConfig.CapDrop = v.CapDrop
	c.HostConfig.CapAdd = v.CapAdd
	c.HostConfig.SecurityOpt = v.SecurityOpt
	c.HostConfig.NetworkMode = r.Binding.Project + "_private"
	c.NetworkSettings.Networks = map[string]json.RawMessage{r.Binding.Project + "_private": json.RawMessage(`{}`)}
	for _, m := range v.Volumes {
		c.Mounts = append(c.Mounts, struct {
			Type        string
			Name        string
			Source      string
			Destination string
			RW          bool
		}{Type: m.Type, Source: m.Source, Destination: m.Target, RW: !m.ReadOnly})
	}
	return c
}
func adapterBackend(t *testing.T, r Release, containers ...dockerContainer) (*RuntimeBackend, *adapterRunner) {
	t.Helper()
	f := &adapterRunner{containers: containers, image: dockerImage{ID: "sha256:" + strings.Repeat("b", 64), OS: runtime.GOOS, Architecture: runtime.GOARCH}}
	root := t.TempDir()
	if err := os.Chmod(root, 0700); err != nil {
		t.Fatal(err)
	}
	return &RuntimeBackend{Root: root, Runner: f}, f
}
func adapterRegister(t *testing.T, b *RuntimeBackend, r Release) {
	t.Helper()
	s := NewState()
	s.Entries[r.Bundle.Manifest.ID] = Entry{Active: r}
	if err := (Store{Root: b.Root}).Save(s); err != nil {
		t.Fatal(err)
	}
}

func TestDisabledUpgradesRetireStoppedRuntimeBeforeAdvancingIdentity(t *testing.T) {
	for _, finish := range []string{"remove", "enable", "rollback"} {
		t.Run(finish, func(t *testing.T) {
			old := adapterRelease(t)
			c := adapterContainer(t, old)
			c.State.Running = false
			c.Config.Labels[ownerLabel], c.Config.Labels[releaseLabel] = old.Bundle.Manifest.ID, old.Digest
			b, f := adapterBackend(t, old, c)
			adapterRegister(t, b, old)
			host := schemaTestHost()
			host.ApprovedGrants = append(host.ApprovedGrants, "network:egress")
			m := Manager{Store: Store{Root: b.Root}, Host: host, Backend: b}
			f.compose = func(args []string) error {
				if args[7] == "pull" {
					return nil
				}
				state, err := m.Store.Load()
				if err != nil || state.Journal == nil {
					t.Fatal("mutation without durable journal", err)
				}
				switch args[7] {
				case "down":
					f.containers = nil
				case "up":
					r := state.Journal.Target.Active
					c := adapterContainer(t, r)
					c.Config.Labels[ownerLabel], c.Config.Labels[releaseLabel] = r.Bundle.Manifest.ID, r.Digest
					f.containers = []dockerContainer{c}
				}
				return nil
			}
			for _, version := range []string{"1.2.4", "1.2.5"} {
				target := old.Bundle
				target.Manifest.Version = version
				if err := m.Apply(context.Background(), "upgrade", old.Bundle.Manifest.ID, &target, Binding{}); err != nil {
					t.Fatal(err)
				}
				statuses, err := m.Status(context.Background())
				if err != nil || statuses[0].Version != version || statuses[0].Enabled || statuses[0].Observation.Process != "stopped" {
					t.Fatalf("disabled upgrade lost stopped identity: %+v %v", statuses, err)
				}
				if len(f.containers) != 0 {
					t.Fatal("obsolete stopped containers remain")
				}
			}
			if err := m.Apply(context.Background(), finish, old.Bundle.Manifest.ID, nil, Binding{}); err != nil {
				t.Fatal(err)
			}
			statuses, err := m.Status(context.Background())
			if err != nil {
				t.Fatal(err)
			}
			switch finish {
			case "remove":
				if len(statuses) != 0 {
					t.Fatal("remove left a registration")
				}
			case "enable":
				if statuses[0].Observation.Process != "running" || statuses[0].Version != "1.2.5" {
					t.Fatal(statuses)
				}
			case "rollback":
				if statuses[0].Observation.Process != "stopped" || statuses[0].Version != "1.2.4" || statuses[0].Enabled {
					t.Fatal(statuses)
				}
			}
		})
	}
}
func alterCompose(t *testing.T, r Release, change func(map[string]any, map[string]any)) Release {
	t.Helper()
	var root map[string]any
	if err := json.Unmarshal(r.Bundle.Compose, &root); err != nil {
		t.Fatal(err)
	}
	v := root["services"].(map[string]any)["agent"].(map[string]any)
	change(root, v)
	r.Bundle.Compose, _ = json.Marshal(root)
	r.Digest = BundleDigest(r.Bundle)
	return r
}
func TestComposeRejectsUnapprovedAuthority(t *testing.T) {
	cases := map[string]func(map[string]any, map[string]any){
		"tag":                   func(_ map[string]any, v map[string]any) { v["image"] = "example:latest" },
		"build":                 func(_ map[string]any, v map[string]any) { v["build"] = "." },
		"privileged":            func(_ map[string]any, v map[string]any) { v["privileged"] = true },
		"host network":          func(_ map[string]any, v map[string]any) { v["network_mode"] = "host" },
		"host pid":              func(_ map[string]any, v map[string]any) { v["pid"] = "host" },
		"device":                func(_ map[string]any, v map[string]any) { v["devices"] = []string{"/dev/ttyACM0"} },
		"post start hook":       func(_ map[string]any, v map[string]any) { v["post_start"] = []any{} },
		"extends":               func(_ map[string]any, v map[string]any) { v["extends"] = map[string]string{"file": "core.yaml"} },
		"unknown extension":     func(root map[string]any, _ map[string]any) { root["x-danger"] = true },
		"mutable root":          func(_ map[string]any, v map[string]any) { v["read_only"] = false },
		"restart after disable": func(_ map[string]any, v map[string]any) { v["restart"] = "always" },
		"capability":            func(_ map[string]any, v map[string]any) { v["cap_add"] = []string{"SYS_ADMIN"} },
		"security":              func(_ map[string]any, v map[string]any) { v["security_opt"] = []string{} },
		"updater owner": func(_ map[string]any, v map[string]any) {
			v["labels"] = map[string]string{"garden.mowgli.update.owner": "core"}
		},
		"docker socket": func(_ map[string]any, v map[string]any) {
			v["volumes"] = []map[string]any{{"type": "bind", "source": "/var/run/docker.sock", "target": "/socket"}}
		},
		"named mutable volume": func(root map[string]any, _ map[string]any) {
			root["volumes"] = map[string]any{"data": map[string]any{"name": "agent-data", "external": false}}
		},
		"network collision": func(root map[string]any, _ map[string]any) {
			root["networks"] = map[string]any{"private": map[string]string{"name": "core_default"}}
		},
		"interpolation": func(_ map[string]any, v map[string]any) { v["environment"] = map[string]string{"TOKEN": "${TOKEN}"} },
	}
	for name, change := range cases {
		t.Run(name, func(t *testing.T) {
			r := alterCompose(t, adapterRelease(t), change)
			if ValidateCompose(r.Bundle, r.Binding) == nil {
				t.Fatal("unsafe compose accepted")
			}
		})
	}
	r := adapterRelease(t)
	if err := ValidateCompose(r.Bundle, r.Binding); err != nil {
		t.Fatal(err)
	}
	connect := alterCompose(t, r, func(_ map[string]any, v map[string]any) {
		v["cap_add"] = []string{"NET_ADMIN", "SETUID", "SETGID", "KILL"}
	})
	connect.Bundle.Manifest.Profile = "connect"
	connect.Bundle.Manifest.Grants = append(connect.Bundle.Manifest.Grants, "network:gui-api", "network:relay-egress", "sandbox:connect-nft-bootstrap")
	if err := ValidateCompose(connect.Bundle, connect.Binding); err != nil {
		t.Fatal(err)
	}
	connect = alterCompose(t, connect, func(_ map[string]any, v map[string]any) { v["user"] = "1000" })
	if ValidateCompose(connect.Bundle, connect.Binding) == nil {
		t.Fatal("non-root connect bootstrap accepted")
	}
}

func TestPartialProjectIsNotReportedAsStopped(t *testing.T) {
	r := adapterRelease(t)
	s, err := validatedCompose(r.Bundle, r.Binding)
	if err != nil {
		t.Fatal(err)
	}
	s.Services["helper"] = s.Services["agent"]
	r.Bundle.Compose, _ = json.Marshal(s)
	r.Digest = BundleDigest(r.Bundle)
	c := adapterContainer(t, r)
	helper := adapterContainer(t, r)
	helper.ID = strings.Repeat("2", 64)
	helper.Name = "/" + r.Binding.Project + "-helper-1"
	helper.Config.Labels["com.docker.compose.service"] = "helper"
	helper.State.Running = false
	backend, _ := adapterBackend(t, r, c, helper)
	observation, err := backend.Observe(context.Background(), r)
	if err != nil || observation.Process != "partial" {
		t.Fatalf("running partial project was reported as stopped: %+v %v", observation, err)
	}
}
func TestAdoptionRequiresExactRuntimeIdentity(t *testing.T) {
	r := adapterRelease(t)
	c := adapterContainer(t, r)
	b, _ := adapterBackend(t, r, c)
	if b.Check(context.Background(), r, false) == nil {
		t.Fatal("implicit adoption allowed")
	}
	if err := b.Check(context.Background(), r, true); err != nil {
		t.Fatal(err)
	}
	if _, err := os.Stat(filepath.Join(b.Root, "releases")); !os.IsNotExist(err) {
		t.Fatal("read-only check wrote release files")
	}
	cases := map[string]func(*dockerContainer){
		"image":        func(c *dockerContainer) { c.Image = "sha256:" + strings.Repeat("c", 64) },
		"environment":  func(c *dockerContainer) { c.Config.Env = []string{"EXTRA_SECRET=unsafe"} },
		"command":      func(c *dockerContainer) { c.Config.Cmd = []string{"sh", "-c", "unsafe"} },
		"mount":        func(c *dockerContainer) { c.Mounts[1].Source = "/core-data" },
		"host network": func(c *dockerContainer) { c.HostConfig.NetworkMode = "host" },
		"privileged":   func(c *dockerContainer) { c.HostConfig.Privileged = true },
		"port": func(c *dockerContainer) {
			c.HostConfig.PortBindings = map[string]json.RawMessage{"80/tcp": json.RawMessage(`[]`)}
		},
		"updater":       func(c *dockerContainer) { c.Config.Labels["garden.mowgli.update.owner"] = "core" },
		"foreign owner": func(c *dockerContainer) { c.Config.Labels[ownerLabel] = "different" },
	}
	for name, change := range cases {
		t.Run(name, func(t *testing.T) {
			c := adapterContainer(t, r)
			change(&c)
			b, _ := adapterBackend(t, r, c)
			if b.Check(context.Background(), r, true) == nil {
				t.Fatal("mismatched adoption accepted")
			}
		})
	}
	if b, _ := adapterBackend(t, r); b.Check(context.Background(), r, true) == nil {
		t.Fatal("empty project adopted")
	}
}

func TestContainerInventoryLimitsUnrelatedMetadata(t *testing.T) {
	r := adapterRelease(t)
	own := adapterContainer(t, r)
	own.Config.Env = []string{"OWN_ENV=local-verification-only"}
	core := adapterContainer(t, r)
	core.ID = strings.Repeat("2", 64)
	core.Name = "/unrelated-core"
	core.Config.Labels = map[string]string{
		"com.docker.compose.project": r.Binding.Project + "-other",
		"com.docker.compose.service": "core",
		ownerLabel:                   "other-extension",
		releaseLabel:                 "other-release",
		"private.label":              "UNRELATED_LABEL_SECRET",
	}
	core.Config.Env = []string{"UNRELATED_ENV_SECRET=must-not-be-read"}
	core.Config.Cmd = []string{"UNRELATED_COMMAND_SECRET"}
	core.Config.User = "UNRELATED_USER_SECRET"
	core.Mounts = nil
	core.NetworkSettings.Networks = map[string]json.RawMessage{
		"core_private":  json.RawMessage(`{"Aliases":["UNRELATED_NETWORK_SECRET"]}`),
		"other_private": json.RawMessage(`{}`),
	}
	for _, withOwn := range []bool{false, true} {
		t.Run(strconv.FormatBool(withOwn), func(t *testing.T) {
			containers := []dockerContainer{core}
			if withOwn {
				containers = append(containers, own)
			}
			b, f := adapterBackend(t, r, containers...)
			f.image.Config.Env = own.Config.Env
			list, err := b.containers(context.Background(), r)
			if err != nil || len(list) != len(containers) {
				t.Fatalf("inventory failed: %v", err)
			}
			basic := list[0]
			if basic.ID != core.ID || basic.Name != core.Name || basic.Image != core.Image || len(basic.NetworkSettings.Networks) != 2 || len(basic.Config.Labels) != 4 || basic.Config.Labels[ownerLabel] != "other-extension" || basic.Config.Labels[releaseLabel] != "other-release" {
				t.Fatal("inventory lost required collision identities")
			}
			if withOwn && !reflect.DeepEqual(list[1], own) {
				t.Fatal("owned metadata required for exact verification was lost")
			}
			retained, _ := json.Marshal(basic)
			for _, marker := range []string{"UNRELATED_ENV_SECRET", "UNRELATED_LABEL_SECRET", "UNRELATED_COMMAND_SECRET", "UNRELATED_USER_SECRET", "UNRELATED_NETWORK_SECRET"} {
				if bytes.Contains(retained, []byte(marker)) || bytes.Contains(bytes.Join(f.inventoryReads, nil), []byte(marker)) {
					t.Fatalf("unrelated metadata was requested or retained: %s", marker)
				}
			}
			fullReads := 0
			for _, call := range f.calls {
				if len(call) > 1 && call[1] == "inspect" {
					if call[4] == "--format" {
						continue
					}
					fullReads++
					if !reflect.DeepEqual(call[4:], []string{own.ID}) {
						t.Fatal("full inspect included an unrelated ID")
					}
				}
			}
			if fullReads != map[bool]int{false: 0, true: 1}[withOwn] {
				t.Fatal("unexpected full metadata read")
			}
			o, err := b.Observe(context.Background(), r)
			if err != nil {
				t.Fatal(err)
			}
			status, _ := json.Marshal(o)
			if bytes.Contains(status, []byte("ENV")) || bytes.Contains(status, []byte("SECRET")) || bytes.Contains(status, []byte("local-verification-only")) {
				t.Fatal("runtime metadata was exported in the observation")
			}
			entries, err := os.ReadDir(b.Root)
			if err != nil || len(entries) != 0 {
				t.Fatal("inventory or observation persisted runtime metadata")
			}
		})
	}
}

type adapterCommandFunc func(context.Context, string, ...string) ([]byte, error)

func (f adapterCommandFunc) Run(ctx context.Context, name string, args ...string) ([]byte, error) {
	return f(ctx, name, args...)
}

func TestContainerInspectRejectsUnexpectedIdentity(t *testing.T) {
	r := adapterRelease(t)
	c := adapterContainer(t, r)
	for name, change := range map[string]func([]dockerContainer) []dockerContainer{
		"unrequested ID": func(list []dockerContainer) []dockerContainer {
			list[0].ID = strings.Repeat("f", 64)
			return list
		},
		"changed project": func(list []dockerContainer) []dockerContainer {
			list[0].Config.Labels["com.docker.compose.project"] = "core"
			return list
		},
		"missing container": func([]dockerContainer) []dockerContainer { return nil },
		"extra container":   func(list []dockerContainer) []dockerContainer { return append(list, list[0]) },
	} {
		t.Run(name, func(t *testing.T) {
			b, f := adapterBackend(t, r, c)
			b.Runner = adapterCommandFunc(func(ctx context.Context, name string, args ...string) ([]byte, error) {
				raw, err := f.Run(ctx, name, args...)
				if err == nil && len(args) > 3 && args[0] == "inspect" && args[3] != "--format" {
					var list []dockerContainer
					if err = json.Unmarshal(raw, &list); err == nil {
						return json.Marshal(change(list))
					}
				}
				return raw, err
			})
			if _, err := b.containers(context.Background(), r); err == nil {
				t.Fatal("invalid detailed identity accepted")
			}
		})
	}
}

func TestContainerInventoryHandlesUnlabeledContainers(t *testing.T) {
	r := adapterRelease(t)
	c := adapterContainer(t, r)
	c.Config.Labels = nil
	c.Config.Env = []string{"UNRELATED_ENV_SECRET=unlabeled"}
	b, f := adapterBackend(t, r, c)
	list, err := b.containers(context.Background(), r)
	if err != nil || len(list) != 1 || list[0].ID != c.ID || len(list[0].Config.Labels) != 0 || len(list[0].Config.Env) != 0 {
		t.Fatalf("unlabeled inventory failed: %v", err)
	}
	if len(f.calls) != 2 {
		t.Fatal("unlabeled container required detailed inspection")
	}
}

func TestContainerInventoryRejectsUnexpectedResponse(t *testing.T) {
	r := adapterRelease(t)
	c := adapterContainer(t, r)
	for name, change := range map[string]func([]byte) []byte{
		"unrequested ID": func(raw []byte) []byte {
			return bytes.ReplaceAll(raw, []byte(c.ID), []byte(strings.Repeat("f", 64)))
		},
		"duplicate ID": func(raw []byte) []byte { return append(raw, raw...) },
		"missing ID":   func([]byte) []byte { return nil },
		"full config": func(raw []byte) []byte {
			return bytes.Replace(raw, []byte(`"Labels":`), []byte(`"Config":{"Env":["UNRELATED_SECRET"]},"Labels":`), 1)
		},
	} {
		t.Run(name, func(t *testing.T) {
			b, f := adapterBackend(t, r, c)
			b.Runner = adapterCommandFunc(func(ctx context.Context, name string, args ...string) ([]byte, error) {
				raw, err := f.Run(ctx, name, args...)
				if err == nil && len(args) > 3 && args[0] == "inspect" && args[3] == "--format" {
					return change(raw), nil
				}
				return raw, err
			})
			list, err := b.containers(context.Background(), r)
			if err == nil || list != nil || strings.Contains(err.Error(), "UNRELATED_SECRET") {
				t.Fatal("invalid inventory returned metadata")
			}
			if len(f.calls) != 2 {
				t.Fatal("invalid inventory reached detailed inspection")
			}
		})
	}
}

func TestLimitedInventoryPreservesUnrelatedCollisionChecks(t *testing.T) {
	r := adapterRelease(t)
	for name, change := range map[string]func(*dockerContainer){
		"name": func(c *dockerContainer) { c.Name = "/" + r.Binding.Project + "-agent-1" },
		"network": func(c *dockerContainer) {
			c.NetworkSettings.Networks[r.Binding.Project+"_private"] = json.RawMessage(`{}`)
		},
		"bind": func(c *dockerContainer) { c.Mounts[0].Source = r.Binding.ConfigDir },
	} {
		t.Run(name, func(t *testing.T) {
			c := adapterContainer(t, r)
			c.Config.Labels["com.docker.compose.project"] = "core"
			c.Config.Env = []string{"UNRELATED_ENV_SECRET=not-needed-for-collisions"}
			c.Name = "/unrelated-core"
			c.Mounts[0].Source = "/unrelated-config"
			c.Mounts[1].Source = "/unrelated-data"
			c.NetworkSettings.Networks = map[string]json.RawMessage{"core_private": json.RawMessage(`{}`)}
			change(&c)
			b, f := adapterBackend(t, r, c)
			if b.Check(context.Background(), r, false) == nil {
				t.Fatal("unrelated collision accepted")
			}
			for _, call := range f.calls {
				if len(call) > 1 && call[1] == "inspect" && call[4] != "--format" {
					t.Fatal("collision detection requested unrelated full metadata")
				}
			}
		})
	}
	t.Run("named volume", func(t *testing.T) {
		r := alterCompose(t, r, func(root, service map[string]any) {
			root["volumes"] = map[string]any{"data": map[string]any{"name": "extension_data", "external": true}}
			service["volumes"] = []map[string]any{{"type": "volume", "source": "data", "target": "/data"}}
		})
		c := adapterContainer(t, r)
		c.Config.Labels["com.docker.compose.project"] = "core"
		c.Name = "/unrelated-core"
		c.Mounts[0].Name = "extension_data"
		c.NetworkSettings.Networks = map[string]json.RawMessage{"core_private": json.RawMessage(`{}`)}
		b, f := adapterBackend(t, r, c)
		if err := b.Check(context.Background(), r, false); err == nil || err.Error() != "extension storage is shared with another project" {
			t.Fatalf("named volume collision missed: %v", err)
		}
		if len(f.calls) != 2 {
			t.Fatal("named volume collision required detailed inspection")
		}
	})
}
func TestComposeCommandsPreserveDataAndUseIsolatedArtifact(t *testing.T) {
	ctx := context.Background()
	r := adapterRelease(t)
	b, f := adapterBackend(t, r)
	if err := b.Prepare(ctx, r); err != nil {
		t.Fatal(err)
	}
	if b.Start(ctx, r) == nil {
		t.Fatal("start authorized before registration")
	}
	adapterRegister(t, b, r)
	for _, op := range []func(context.Context, Release) error{b.Start, b.Stop, b.Remove} {
		if err := op(ctx, r); err != nil {
			t.Fatal(err)
		}
	}
	var commands [][]string
	for _, call := range f.calls {
		if len(call) > 1 && call[1] == "compose" {
			commands = append(commands, call)
			if len(call) < 9 || call[2] != "--project-name" || call[3] != r.Binding.Project || call[4] != "--project-directory" || call[6] != "-f" {
				t.Fatalf("unscoped command %v", call)
			}
			for _, arg := range call {
				if arg == "-v" || arg == "--volumes" || arg == "--rmi" || arg == "--remove-orphans" {
					t.Fatal("destructive command")
				}
			}
		}
	}
	if len(commands) != 4 || commands[0][8] != "pull" || commands[1][8] != "up" || commands[2][8] != "stop" || commands[3][8] != "down" {
		t.Fatalf("commands: %v", commands)
	}
	key, _ := releaseKey(r)
	path := filepath.Join(b.Root, "releases", key, "compose.json")
	info, err := os.Stat(path)
	if err != nil || info.Mode().Perm() != 0600 {
		t.Fatalf("artifact permissions %v %v", info, err)
	}
	raw, err := os.ReadFile(path)
	if err != nil {
		t.Fatal(err)
	}
	var spec composeSpec
	if json.Unmarshal(raw, &spec) != nil || spec.Services["agent"].Labels[ownerLabel] != r.Bundle.Manifest.ID {
		t.Fatal("missing ownership")
	}
	if strings.Contains(string(r.Bundle.Compose), ownerLabel) {
		t.Fatal("caller bundle changed")
	}
	if err := os.WriteFile(path, []byte(`{}`), 0600); err != nil {
		t.Fatal(err)
	}
	before := len(f.calls)
	if b.Start(ctx, r) == nil || len(f.calls) != before {
		t.Fatal("changed artifact reached Docker")
	}
}
func TestCollisionAndArtifactSymlink(t *testing.T) {
	r := adapterRelease(t)
	c := adapterContainer(t, r)
	c.Config.Labels["com.docker.compose.project"] = "core"
	b, _ := adapterBackend(t, r, c)
	if b.Check(context.Background(), r, false) == nil {
		t.Fatal("container collision allowed")
	}
	b, _ = adapterBackend(t, r)
	target := t.TempDir()
	if err := os.Symlink(target, filepath.Join(b.Root, "releases")); err != nil {
		t.Fatal(err)
	}
	if b.Prepare(context.Background(), r) == nil {
		t.Fatal("artifact symlink followed")
	}
	entries, _ := os.ReadDir(target)
	if len(entries) != 0 {
		t.Fatal("symlink target changed")
	}
}
func TestObserveUsesActualHealthAndImageIdentity(t *testing.T) {
	r := adapterRelease(t)
	r.Bundle.Manifest.Health = "application"
	r.Digest = BundleDigest(r.Bundle)
	c := adapterContainer(t, r)
	b, f := adapterBackend(t, r, c)
	o, err := b.Observe(context.Background(), r)
	if err != nil || o.Process != "running" || o.Application != "unknown" || o.Data != "unknown" || o.Identity == "" {
		t.Fatalf("%+v %v", o, err)
	}
	c.Config.Healthcheck = &dockerHealth{Test: []string{"CMD", "true"}}
	f.image.Config.Healthcheck = c.Config.Healthcheck
	c.State.Health = &struct{ Status string }{Status: "healthy"}
	f.containers = []dockerContainer{c}
	o, err = b.Observe(context.Background(), r)
	if err != nil || o.Application != "healthy" {
		t.Fatalf("%+v %v", o, err)
	}
	// Pinned image and application health are independent from a tag/string label.
	f.containers[0].Image = "sha256:" + strings.Repeat("f", 64)
	if _, err = b.Observe(context.Background(), r); err == nil {
		t.Fatal("wrong image observed as ready")
	}
	f.containers = nil
	o, err = b.Observe(context.Background(), r)
	if err != nil || o.Process != "stopped" || o.Application != "unknown" {
		t.Fatalf("%+v %v", o, err)
	}
}
func TestRuntimeErrorsDoNotExposeOutput(t *testing.T) {
	r := adapterRelease(t)
	b, f := adapterBackend(t, r)
	f.fail = "docker ps"
	err := b.Check(context.Background(), r, false)
	if err == nil || strings.Contains(err.Error(), "SECRET") {
		t.Fatal(err)
	}
}
func TestRegisteredAdoptionAndUpgradeUseOriginalIdentity(t *testing.T) {
	r := adapterRelease(t)
	c := adapterContainer(t, r)
	b, f := adapterBackend(t, r, c)
	adapterRegister(t, b, r)
	if err := b.Stop(context.Background(), r); err != nil {
		t.Fatal(err)
	}
	target := r
	target.Bundle.Manifest.Version = "1.2.4"
	target = alterCompose(t, target, func(_ map[string]any, v map[string]any) {
		v["image"] = "example/agent@sha256:" + strings.Repeat("c", 64)
	})
	target.Digest = BundleDigest(target.Bundle)
	// Check/Prepare accept the exact prior registered instance before it stops.
	if err := b.Check(context.Background(), target, false); err != nil {
		t.Fatal(err)
	}
	if err := b.Prepare(context.Background(), target); err != nil {
		t.Fatal(err)
	}
	if f.containers[0].Image != c.Image {
		t.Fatal("fake instance mutated")
	}
}
func TestImageDefaultsAreComparedExactly(t *testing.T) {
	r := adapterRelease(t)
	c := adapterContainer(t, r)
	c.Config.Env = []string{"PATH=/image/bin"}
	c.Config.Cmd = []string{"/image/agent"}
	b, f := adapterBackend(t, r, c)
	f.image.Config.Env = c.Config.Env
	f.image.Config.Cmd = c.Config.Cmd
	if err := b.Check(context.Background(), r, true); err != nil {
		t.Fatal(err)
	}
	if !reflect.DeepEqual(f.containers[0].Config.Env, c.Config.Env) {
		t.Fatal("input changed")
	}
	f.containers[0].Config.Env = append(f.containers[0].Config.Env, "EXTRA=1")
	if b.Check(context.Background(), r, true) == nil {
		t.Fatal("extra env adopted")
	}
}

func TestNormalizeComposePreservesConnectIsolationAndRejectsHiddenAuthority(t *testing.T) {
	raw := []byte(`{"name":"mowgli-connect","services":{"agent":{"image":"example/agent@sha256:aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa","command":null,"entrypoint":null,"read_only":true,"cap_drop":["ALL"],"cap_add":["NET_ADMIN","SETUID","SETGID","KILL"],"security_opt":["no-new-privileges:true"],"dns":["1.1.1.1","8.8.8.8"],"extra_hosts":["host.docker.internal=host-gateway"],"networks":{"default":null},"volumes":[{"type":"volume","source":"data","target":"/data","volume":{}}]}},"networks":{"default":{"name":"mowgli-connect_default","ipam":{}}},"volumes":{"data":{"name":"mowgli-connect_data","external":true}}}`)
	normalized, err := NormalizeCompose(raw)
	if err != nil {
		t.Fatal(err)
	}
	r := adapterRelease(t)
	r.Binding.Project = "mowgli-connect"
	r.Bundle.Manifest.ID = "mowgli-connect"
	r.Bundle.Manifest.Profile = "connect"
	r.Bundle.Manifest.Grants = append(r.Bundle.Manifest.Grants, "network:gui-api", "network:relay-egress", "sandbox:connect-nft-bootstrap")
	r.Bundle.Compose = normalized
	r.Digest = BundleDigest(r.Bundle)
	if err := ValidateCompose(r.Bundle, r.Binding); err != nil {
		t.Fatal(err)
	}
	if strings.Contains(string(normalized), "null") || strings.Contains(string(normalized), "host.docker.internal=host-gateway") {
		t.Fatal(string(normalized))
	}
	bad := strings.Replace(string(raw), `"read_only":true`, `"read_only":true,"privileged":true`, 1)
	if _, err := NormalizeCompose([]byte(bad)); err == nil {
		t.Fatal("normalization discarded authority")
	}
	duplicate := strings.Replace(string(raw), `"read_only":true`, `"read_only":true,"read_only":false`, 1)
	if _, err := NormalizeCompose([]byte(duplicate)); err == nil {
		t.Fatal("normalization hid duplicate")
	}
	nullSecurity := strings.Replace(string(raw), `"security_opt":["no-new-privileges:true"]`, `"security_opt":null`, 1)
	if _, err := NormalizeCompose([]byte(nullSecurity)); err == nil {
		t.Fatal("normalization hid security null")
	}
}
func TestComposeExternalVolumeIdentityAndResourceCollisions(t *testing.T) {
	r := adapterRelease(t)
	r = alterCompose(t, r, func(root, v map[string]any) {
		root["volumes"] = map[string]any{"data": map[string]any{"name": "connect_data", "external": true}}
		v["volumes"] = []map[string]any{{"type": "volume", "source": "data", "target": "/data"}}
	})
	b, f := adapterBackend(t, r)
	f.volume = `[{"Name":"connect_data","Driver":"local","Options":null}]`
	if err := b.Check(context.Background(), r, false); err != nil {
		t.Fatal(err)
	}
	f.volume = `[{"Name":"connect_data","Driver":"local","Options":{"device":"/etc","o":"bind","type":"none"}}]`
	if b.Check(context.Background(), r, false) == nil {
		t.Fatal("host-bind external volume allowed")
	}
	f.volume = `[{"Name":"connect_data","Driver":"local","Options":null}]`
	f.network = `{"Name":"robot-ext-test_private","Labels":""}`
	f.networkInspect = `[{"Labels":{"com.docker.compose.project":"core","com.docker.compose.network":"private"},"Driver":"bridge"}]`
	if b.Check(context.Background(), r, false) == nil {
		t.Fatal("foreign network collision accepted")
	}
}
func TestArtifactHardLinksAreRejected(t *testing.T) {
	r := adapterRelease(t)
	b, _ := adapterBackend(t, r)
	if err := b.Prepare(context.Background(), r); err != nil {
		t.Fatal(err)
	}
	key, _ := releaseKey(r)
	path := filepath.Join(b.Root, "releases", key, "compose.json")
	if err := os.Link(path, filepath.Join(b.Root, "copied-compose")); err != nil {
		t.Fatal(err)
	}
	if b.Check(context.Background(), r, false) == nil {
		t.Fatal("hard-linked artifact accepted")
	}
}

func TestComposeRejectsAllDollarExpansionForms(t *testing.T) {
	r := adapterRelease(t)
	for _, value := range []string{"$HOME", "${HOME}", "$$HOME", "literal-dollar-$"} {
		candidate := alterCompose(t, r, func(_ map[string]any, v map[string]any) { v["environment"] = map[string]string{"VALUE": value} })
		if ValidateCompose(candidate.Bundle, candidate.Binding) == nil {
			t.Fatalf("dollar form accepted: %q", value)
		}
	}
	raw := strings.Replace(string(r.Bundle.Compose), `"image":`, `"environment":{"VALUE":"\u0024HOME"},"image":`, 1)
	r.Bundle.Compose = []byte(raw)
	if ValidateCompose(r.Bundle, r.Binding) == nil {
		t.Fatal("escaped JSON dollar bypassed policy")
	}
}
func TestComposeApplicationStatesAndRestartInstance(t *testing.T) {
	r := adapterRelease(t)
	r.Bundle.Manifest.Health = "application"
	r.Digest = BundleDigest(r.Bundle)
	c := adapterContainer(t, r)
	c.Config.Healthcheck = &dockerHealth{Test: []string{"CMD", "true"}}
	c.State.Health = &struct{ Status string }{Status: "starting"}
	b, f := adapterBackend(t, r, c)
	f.image.Config.Healthcheck = c.Config.Healthcheck
	first, err := b.Observe(context.Background(), r)
	if err != nil || first.Application != "starting" {
		t.Fatalf("%+v %v", first, err)
	}
	f.containers[0].State.Health.Status = "unhealthy"
	second, err := b.Observe(context.Background(), r)
	if err != nil || second.Application != "unhealthy" {
		t.Fatalf("%+v %v", second, err)
	}
	f.containers[0].State.StartedAt = "2026-11-10T10:01:00.123456789Z"
	third, err := b.Observe(context.Background(), r)
	if err != nil || third.Identity == first.Identity {
		t.Fatalf("restart identity reused %+v %v", third, err)
	}
}
func TestComposeGrantsMatchActualAuthority(t *testing.T) {
	r := adapterRelease(t)
	r.Bundle.Manifest.Grants = []string{}
	if ValidateCompose(r.Bundle, r.Binding) == nil {
		t.Fatal("network and storage without grant")
	}
	r = adapterRelease(t)
	r.Bundle.Manifest.Grants = []string{"network:egress"}
	if ValidateCompose(r.Bundle, r.Binding) == nil {
		t.Fatal("storage without grant")
	}
	r = adapterRelease(t)
	r.Bundle.Manifest.Grants = []string{"storage:extension-data"}
	if ValidateCompose(r.Bundle, r.Binding) == nil {
		t.Fatal("egress without grant")
	}
	r = alterCompose(t, r, func(root, _ map[string]any) {
		root["networks"].(map[string]any)["private"].(map[string]any)["internal"] = true
	})
	if err := ValidateCompose(r.Bundle, r.Binding); err != nil {
		t.Fatal(err)
	}
}

// The config subcommand parses only; it never contacts a Docker daemon.
func TestDisabledHealthcheckArtifactHasNoNullTest(t *testing.T) {
	r := alterCompose(t, adapterRelease(t), func(_ map[string]any, service map[string]any) {
		service["healthcheck"] = map[string]any{"disable": true}
	})
	b, _ := adapterBackend(t, r)
	ctx, cancel := context.WithTimeout(context.Background(), 10*time.Second)
	defer cancel()
	if err := b.Prepare(ctx, r); err != nil {
		t.Fatal(err)
	}
	key, _ := releaseKey(r)
	path := filepath.Join(b.Root, "releases", key, "compose.json")
	raw, err := os.ReadFile(path)
	if err != nil {
		t.Fatal(err)
	}
	var spec map[string]any
	if err := json.Unmarshal(raw, &spec); err != nil {
		t.Fatal(err)
	}
	health := spec["services"].(map[string]any)["agent"].(map[string]any)["healthcheck"].(map[string]any)
	if _, present := health["test"]; present || health["disable"] != true {
		t.Fatal("disabled healthcheck emitted an invalid null test")
	}
	if docker, err := exec.LookPath("docker"); err == nil && exec.CommandContext(ctx, docker, "compose", "version").Run() == nil {
		if err := exec.CommandContext(ctx, docker, "compose", "--project-name", r.Binding.Project, "--project-directory", filepath.Dir(path), "-f", path, "config", "--quiet").Run(); err != nil {
			t.Fatal("disabled healthcheck artifact failed daemon-free Compose parsing")
		}
	}
}

func TestCanonicalArtifactParsesWithComposeCLI(t *testing.T) {
	docker, err := exec.LookPath("docker")
	if err != nil {
		t.Skip("Docker Compose CLI not installed")
	}
	ctx, cancel := context.WithTimeout(context.Background(), 10*time.Second)
	defer cancel()
	if exec.CommandContext(ctx, docker, "compose", "version").Run() != nil {
		t.Skip("Docker Compose CLI not installed")
	}
	r := adapterRelease(t)
	b, _ := adapterBackend(t, r)
	if err := b.Prepare(ctx, r); err != nil {
		t.Fatal(err)
	}
	key, _ := releaseKey(r)
	path := filepath.Join(b.Root, "releases", key, "compose.json")
	raw, err := exec.CommandContext(ctx, docker, "compose", "--project-name", r.Binding.Project, "--project-directory", filepath.Dir(path), "-f", path, "config", "--format", "json").Output()
	if err != nil {
		t.Fatal("canonical artifact failed daemon-free Compose parsing")
	}
	normalized, err := NormalizeCompose(raw)
	if err != nil {
		t.Fatal(err)
	}
	var spec composeSpec
	if json.Unmarshal(normalized, &spec) != nil {
		t.Fatal("invalid normalized artifact")
	}
	service := spec.Services["agent"]
	if !service.ReadOnly || len(service.Volumes) != 2 || service.Volumes[0].Bind == nil || service.Volumes[0].Bind.CreateHostPath || service.Labels[ownerLabel] != r.Bundle.Manifest.ID {
		t.Fatal("Compose changed approved runtime isolation or mount semantics")
	}
}
