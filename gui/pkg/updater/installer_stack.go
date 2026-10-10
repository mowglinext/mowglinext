package updater

import (
	"context"
	"encoding/json"
	"errors"
	"fmt"
	"os"
	"path/filepath"
	"reflect"
	"sort"
	"strings"
	"time"

	"github.com/joho/godotenv"
)

// AdoptLegacyEnv is how the installer passes the operator's explicit consent
// to replace a Compose file that has no recorded baseline. An environment
// variable rather than an argument: an older worker ignores it and keeps
// refusing, instead of failing on an unknown command line.
const AdoptLegacyEnv = "MOWGLI_ADOPT_LEGACY_COMPOSE"

// EnableMQTTEnv tells installer-stack what to do with the local mosquitto
// service when it regenerates the file: "true" adds/keeps it, "false" drops
// it, unset keeps whatever the replaced file had and never adds it.
const EnableMQTTEnv = "MOWGLI_ENABLE_MQTT"

// RegenerateStackEnv makes installer-stack render docker-compose.yaml from
// the CHECKOUT's fragments even while a published release is active (the
// installer's manual `update`/`repair`). The release record is cleared so the
// file is installer-owned again; the next reviewed update adopts it.
const RegenerateStackEnv = "MOWGLI_REGENERATE_STACK"

func regenerateStack() bool { return os.Getenv(RegenerateStackEnv) == "true" }

func mqttWanted() bool   { return os.Getenv(EnableMQTTEnv) == "true" }
func mqttUnwanted() bool { return os.Getenv(EnableMQTTEnv) == "false" }

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

// editedComposeReason is what LegacyComposeError reports when the generated
// file no longer matches its recorded baseline.
const editedComposeReason = "docker-compose.yaml was edited after it was generated (its checksum differs from the recorded baseline)"

// reconcileInstalledCompose decides what happens to docker/docker-compose.yaml
// before the installer (re)generates it. Missing: regenerate, nothing to
// keep. Matching its baseline: nothing to do. Edited: refuse without the
// operator's consent (LegacyComposeError → exit 3, the installer asks); with
// consent the exact file is kept as docker-compose.yaml.edited-<UTC> and the
// definition is regenerated. Returns whether the file must be written.
func (b DockerBackend) reconcileInstalledCompose(adopt bool) (bool, error) {
	path := filepath.Join(b.Config.Directory, "docker-compose.yaml")
	if _, err := os.Stat(path); os.IsNotExist(err) {
		return true, nil
	}
	err := b.checkStackBaseline()
	if err == nil {
		return false, nil
	}
	if !errors.Is(err, ErrComposeEdited) {
		return false, err
	}
	if !adopt {
		return false, &LegacyComposeError{Differences: []string{editedComposeReason}}
	}
	if err := backupCompose(path, ".edited-"); err != nil {
		return false, err
	}
	return true, nil
}

// InstallStack registers installer choices using the same release selector.
// Once a published stack is active, installer reruns only propose choices;
// activation still goes through the updater's reviewed backup transaction —
// unless the installed file is missing or was hand-edited, in which case the
// active release is re-rendered from its own bundle (reconcileInstalledCompose).
//
// adoptLegacy is the operator's consent to replace a Compose file the updater
// cannot vouch for: one with no recorded baseline that differs from the
// target, or one edited after it was generated. The replaced file is always
// kept next to the new one.
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
	backend, err := installerBackendChoice(b.Config.Directory)
	if err != nil {
		return err
	}
	// The Mowgli board leaves the key out ("none" either way to Select), so a
	// selection recorded before the group existed still equals it and no
	// Mowgli robot is told an installer change is pending.
	delete(preserved, "backend")
	if backend != "none" {
		preserved["backend"] = backend
	}
	choices = preserved
	selection := StackSelection{Options: choices}
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
	if active != nil && regenerateStack() {
		// Manual update: the operator wants THIS checkout's definition, not
		// the release the updater installed. renderInstalledStack still asks
		// for consent on a hand-edited file and keeps it.
		bundle, err = ReadComposeBundle(sourceDir)
		if err != nil {
			return err
		}
		return b.renderInstalledStack(ctx, bundle, selection, nil, sourceDir, adoptLegacy)
	}
	if active != nil {
		regenerate, err := b.reconcileInstalledCompose(adoptLegacy)
		if err != nil {
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
		if !regenerate {
			return AtomicJSON(filepath.Join(b.Config.Directory, "stack-selection.json"), selection)
		}
		return b.renderInstalledStack(ctx, bundle, selection, active, sourceDir)
	}
	bundle, err = ReadComposeBundle(sourceDir)
	if err != nil {
		return err
	}
	return b.renderInstalledStack(ctx, bundle, selection, nil, sourceDir, adoptLegacy)
}

// renderInstalledStack writes docker/docker-compose.yaml from a bundle (the
// installer's fragments, or the active release's own bundle) plus the local
// services of the file it replaces and stack-overrides.yaml, then records
// the metadata and the baseline. adoptLegacy only matters for a file with no
// recorded baseline (see InstallStack).
func (b DockerBackend) renderInstalledStack(ctx context.Context, bundle ComposeBundle, selection StackSelection, release *Deployment, sourceDir string, adopt ...bool) error {
	adoptLegacy := len(adopt) > 0 && adopt[0]
	target, err := b.renderBundle(ctx, bundle, selection)
	if err != nil {
		return err
	}
	// MQTT remains an installer/local service. It is deliberately not retired
	// or updated by release membership reconciliation.
	legacy := filepath.Join(b.Config.Directory, "docker-compose.yaml")
	if _, e := os.Stat(legacy); e == nil {
		if _, e = os.Stat(filepath.Join(b.Config.Directory, "stack-definition.sha256")); e == nil {
			if _, err = b.reconcileInstalledCompose(adoptLegacy); err != nil {
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
		if mqttUnwanted() {
			if target, err = dropService(target, "mosquitto"); err != nil {
				return err
			}
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
	if _, e := os.Stat(legacy); os.IsNotExist(e) && mqttWanted() {
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
	return b.writeStackMetadata(bundle, selection, release, target)
}

// dropService removes one service from a rendered Compose JSON document.
func dropService(target []byte, name string) ([]byte, error) {
	var document map[string]any
	if err := json.Unmarshal(target, &document); err != nil {
		return nil, err
	}
	if services, ok := document["services"].(map[string]any); ok {
		delete(services, name)
	}
	return json.Marshal(document)
}

// backupCompose keeps the replaced file byte-for-byte under a suffix naming
// why it was replaced (.legacy-: no baseline; .edited-: hand edit). The name
// is outside the installer's own `.old.<timestamp>` series on purpose: those
// are pruned when regeneration changes nothing, this one is the operator's
// record.
func backupCompose(path, suffix string) error {
	data, err := os.ReadFile(path)
	if err != nil {
		return err
	}
	return AtomicWrite(path+suffix+time.Now().UTC().Format("20060102T150405Z"), data, 0600)
}

func backupLegacyCompose(path string) error { return backupCompose(path, ".legacy-") }

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

// installerBackendChoice maps docker/.env HARDWARE_BACKEND onto the bundle's
// "backend" group. It is derived here, never preserved from an earlier
// selection, so a robot moved back to the Mowgli board drops the OpenMower
// bridge instead of keeping it. MAVROS has no release stack.
func installerBackendChoice(dir string) (string, error) {
	env, err := godotenv.Read(filepath.Join(dir, ".env"))
	if err != nil {
		return "", err
	}
	switch backend := strings.TrimSpace(env["HARDWARE_BACKEND"]); backend {
	case "", "mowgli":
		return "none", nil
	case "openmower":
		return "openmower", nil
	default:
		return "", fmt.Errorf("release stacks do not cover HARDWARE_BACKEND=%s", backend)
	}
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
