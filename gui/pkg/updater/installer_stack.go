package updater

import (
	"context"
	"encoding/json"
	"os"
	"path/filepath"
	"reflect"
	"sort"
	"strings"
	"time"
)

// AdoptLegacyEnv is how the installer passes the operator's explicit consent
// to replace a Compose file that has no recorded baseline. An environment
// variable rather than an argument: an older worker ignores it and keeps
// refusing, instead of failing on an unknown command line.
const AdoptLegacyEnv = "MOWGLI_ADOPT_LEGACY_COMPOSE"

// LegacyComposeExitCode lets the installer tell "needs the operator's
// decision" apart from every other installer-stack failure.
const LegacyComposeExitCode = 3

// LegacyComposeError reports a Compose file that has NO recorded baseline
// (stack-definition.sha256) and differs from what the current fragments
// render. Without a baseline a hand edit and the release's own evolution of
// the fragments are indistinguishable — GNSS_STACK added to mowgli.environment
// by #625 tripped this on every untouched pre-updater install — so the
// decision belongs to the operator, not to a comparison that cannot know.
type LegacyComposeError struct {
	Differences []string
}

func (e *LegacyComposeError) Error() string {
	return "legacy Compose has no recorded baseline and differs from the current release definition in " + strings.Join(e.Differences, ", ")
}

// InstallStack registers installer choices using the same release selector.
// Once a published stack is active, installer reruns only propose choices;
// activation still goes through the updater's reviewed backup transaction.
//
// adoptLegacy is the operator's consent to replace a baseline-less Compose
// file that differs from the target. It never bypasses a RECORDED baseline:
// a generated file whose checksum no longer matches stays refused.
func (b DockerBackend) InstallStack(ctx context.Context, sourceDir string, choices map[string]string, adoptLegacy bool) error {
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
			// No baseline was ever recorded for this file (it predates the
			// updater), so "differs from the target" cannot tell a hand edit
			// from fragments that simply evolved since it was generated.
			// Never replace it silently: refuse until the operator consents,
			// and keep the exact previous file next to the new one.
			old, e := b.composeWithOverride(ctx, false, "config", "--no-interpolate", "--format", "json")
			if e != nil {
				return e
			}
			if differences := legacyDifferences(old, target); len(differences) > 0 {
				if !adoptLegacy {
					return &LegacyComposeError{Differences: differences}
				}
				if err = backupLegacyCompose(legacy); err != nil {
					return err
				}
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

// backupLegacyCompose keeps the replaced file byte-for-byte. The name is
// outside the installer's own `.old.<timestamp>` series on purpose: those are
// pruned when regeneration changes nothing, this one is the operator's record.
func backupLegacyCompose(path string) error {
	data, err := os.ReadFile(path)
	if err != nil {
		return err
	}
	return AtomicWrite(path+".legacy-"+time.Now().UTC().Format("20060102T150405Z"), data, 0600)
}

// legacyDifferences lists every managed `service.key` whose definition differs
// between the installed file and the target, sorted. Empty means identical.
func legacyDifferences(old, target []byte) []string {
	var a, b composeConfig
	if e := json.Unmarshal(old, &a); e != nil {
		return []string{"installed JSON: " + e.Error()}
	}
	if e := json.Unmarshal(target, &b); e != nil {
		return []string{"release JSON: " + e.Error()}
	}
	for _, c := range []*composeConfig{&a, &b} {
		delete(c.Services, "watchtower")
		for name, sc := range c.Services {
			var doc map[string]any
			if json.Unmarshal(sc.Raw, &doc) != nil {
				return []string{name + " definition missing or invalid"}
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
	differences := []string{}
	for name, sc := range a.Services {
		if _, managed := Services[name]; !managed && sc.Labels[updateLabel+"image"] == "" {
			continue
		}
		var left, right map[string]any
		if json.Unmarshal(sc.Raw, &left) != nil || json.Unmarshal(b.Services[name].Raw, &right) != nil {
			differences = append(differences, name)
			continue
		}
		if !reflect.DeepEqual(left, right) {
			keys := map[string]bool{}
			for key := range left {
				keys[key] = true
			}
			for key := range right {
				keys[key] = true
			}
			for key := range keys {
				if !reflect.DeepEqual(left[key], right[key]) {
					differences = append(differences, name+"."+key)
				}
			}
		}
	}
	sort.Strings(differences)
	return differences
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
