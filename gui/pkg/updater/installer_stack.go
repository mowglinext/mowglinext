package updater

import (
	"context"
	"encoding/json"
	"fmt"
	"os"
	"path/filepath"
	"reflect"
	"sort"
	"strings"
)

// InstallStack registers installer choices using the same release selector.
// Once a published stack is active, installer reruns only propose choices;
// activation still goes through the updater's reviewed backup transaction.
func (b DockerBackend) InstallStack(ctx context.Context, sourceDir string, choices map[string]string) error {
	preserved := map[string]string{}
	if data, e := os.ReadFile(filepath.Join(b.Config.Directory, "stack-selection.json")); e == nil {
		var previous StackSelection
		if e = json.Unmarshal(data, &previous); e != nil {
			return e
		}
		for key, value := range previous.Options {
			preserved[key] = value
		}
	} else if !os.IsNotExist(e) {
		return e
	}
	for key, value := range choices {
		preserved[key] = value
	}
	choices = preserved
	selection := StackSelection{Options: choices}
	var err error
	selection.InstallerConfig, err = installerConfigIdentity(b.Config.Directory)
	if err != nil {
		return err
	}
	var bundle ComposeBundle
	var active *Deployment
	if data, e := os.ReadFile(filepath.Join(b.Config.Directory, "stack-release.json")); e == nil {
		if err = json.Unmarshal(data, &active); err != nil {
			return err
		}
	} else if !os.IsNotExist(e) {
		return e
	}
	if active != nil {
		if err = b.checkStackBaseline(); err != nil {
			return err
		}
		data, e := os.ReadFile(filepath.Join(b.Config.Directory, "stack-bundle.json"))
		if e != nil {
			return e
		}
		if err = json.Unmarshal(data, &bundle); err != nil {
			return err
		}
		if err = bundle.Validate(); err != nil {
			return err
		}
		if _, err = bundle.Select(choices); err != nil {
			return err
		}
		return AtomicJSON(filepath.Join(b.Config.Directory, "stack-selection.json"), selection)
	}
	bundle, err = ReadComposeBundle(sourceDir)
	if err != nil {
		return err
	}
	target, err := b.renderBundle(ctx, bundle, selection)
	if err != nil {
		return err
	}
	// MQTT remains an installer/local service. It is deliberately not retired
	// or updated by release membership reconciliation.
	legacy := filepath.Join(b.Config.Directory, "docker-compose.yaml")
	if _, e := os.Stat(legacy); e == nil {
		if _, e = os.Stat(filepath.Join(b.Config.Directory, "stack-definition.sha256")); e == nil {
			if err = b.checkStackBaseline(); err != nil {
				return err
			}
		} else {
			// Do not silently adopt manually patched legacy definitions. The
			// operator can place reviewed differences in stack-overrides.yaml.
			old, e := b.composeWithOverride(ctx, false, "config", "--no-interpolate", "--format", "json")
			if e != nil {
				return e
			}
			if difference := legacyDifference(old, target); difference != "" {
				return fmt.Errorf("legacy Compose differs in %s; back it up and review regeneration with stack-overrides.yaml before adopting release updates", difference)
			}
		}
		old, e := b.composeWithOverride(ctx, false, "config", "--no-interpolate", "--format", "json")
		if e != nil {
			return e
		}
		var c composeConfig
		if err = json.Unmarshal(old, &c); err != nil {
			return err
		}
		managed, e := managedServices(c)
		if e != nil {
			return e
		}
		// install_host_updater has already retired this project's Watchtower.
		managed["watchtower"] = managedService{}
		target, err = retainLocalServices(old, target, managed)
		if err != nil {
			return err
		}
	} else if !os.IsNotExist(e) {
		return e
	}

	dir, err := os.MkdirTemp(b.Config.Directory, ".install-stack-")
	if err != nil {
		return err
	}
	defer os.RemoveAll(dir)
	file := filepath.Join(dir, "target.json")
	if err = os.WriteFile(file, target, 0600); err != nil {
		return err
	}
	files := []string{file}
	if _, e := os.Stat(legacy); os.IsNotExist(e) {
		files = append(files, filepath.Join(sourceDir, "docker-compose.mqtt.yml"))
	}
	if _, e := os.Stat(filepath.Join(b.Config.Directory, "stack-overrides.yaml")); e == nil {
		files = append(files, filepath.Join(b.Config.Directory, "stack-overrides.yaml"))
	}
	target, err = b.renderFiles(ctx, files, false)
	if err != nil {
		return err
	}
	if err = AtomicWrite(legacy, target, 0600); err != nil {
		return err
	}
	return b.writeStackMetadata(bundle, selection, nil, target)
}

func legacyDifference(old, target []byte) string {
	var a, b composeConfig
	if e := json.Unmarshal(old, &a); e != nil {
		return "installed JSON: " + e.Error()
	}
	if e := json.Unmarshal(target, &b); e != nil {
		return "release JSON: " + e.Error()
	}
	for _, c := range []*composeConfig{&a, &b} {
		delete(c.Services, "watchtower")
		for name, sc := range c.Services {
			var doc map[string]any
			if json.Unmarshal(sc.Raw, &doc) != nil {
				return name + " definition missing or invalid"
			}
			environment := map[string]any{}
			for key, value := range sc.Environment {
				environment[key] = value
			}
			doc["environment"] = environment
			labels := map[string]any{}
			for key, value := range sc.Labels {
				labels[key] = value
			}
			doc["labels"] = labels
			delete(labels, "com.centurylinklabs.watchtower.enable")
			delete(labels, updateLabel+"image")
			if len(labels) == 0 {
				delete(doc, "labels")
			}
			delete(object(doc["environment"]), "MOWGLI_UPDATE_MAINTENANCE")
			delete(object(doc["environment"]), "MOWGLI_UPDATER_SOCKET")
			volumes := []any{}
			if values, ok := doc["volumes"].([]any); ok {
				for _, value := range values {
					dest := object(value)["target"]
					if dest != "/var/lib/mowgli-updater" && dest != "/run/mowgli-updater" {
						volumes = append(volumes, value)
					}
				}
			}
			doc["volumes"] = volumes
			sc.Raw, _ = json.Marshal(doc)
			c.Services[name] = sc
		}
	}
	for name, sc := range a.Services {
		if _, managed := Services[name]; !managed && sc.Labels[updateLabel+"image"] == "" {
			continue
		}
		var left, right map[string]any
		if json.Unmarshal(sc.Raw, &left) != nil || json.Unmarshal(b.Services[name].Raw, &right) != nil {
			return name + " definition missing or invalid"
		}
		if !reflect.DeepEqual(left, right) {
			keys := map[string]bool{}
			for key := range left {
				keys[key] = true
			}
			for key := range right {
				keys[key] = true
			}
			changed := []string{}
			for key := range keys {
				if !reflect.DeepEqual(left[key], right[key]) {
					changed = append(changed, key)
				}
			}
			sort.Strings(changed)
			return name + "." + strings.Join(changed, ",")
		}
	}
	return ""
}

func (b DockerBackend) SelectionPending() (bool, error) {
	wanted, err := os.ReadFile(filepath.Join(b.Config.Directory, "stack-selection.json"))
	if os.IsNotExist(err) {
		return false, nil
	}
	if err != nil {
		return false, err
	}
	applied, err := os.ReadFile(filepath.Join(b.Config.Directory, "stack-applied-selection.json"))
	if err != nil {
		return false, err
	}
	var a, z StackSelection
	if err = json.Unmarshal(wanted, &a); err != nil {
		return false, err
	}
	if err = json.Unmarshal(applied, &z); err != nil {
		return false, err
	}
	return !reflect.DeepEqual(a.Options, z.Options), nil
}
