package extensions

import (
	"context"
	"errors"
	"path/filepath"
	"strings"
	"testing"
)

type lifecycleRuntime struct {
	t          *testing.T
	store      Store
	events     []string
	running    *Release
	failures   map[string]int
	crashAfter string
	instance   int
	readiness  string
	unknown    bool
}

func (f *lifecycleRuntime) event(action string, r Release) error {
	key := action + ":" + r.Bundle.Manifest.Version
	f.events = append(f.events, key)
	if action == "start" || action == "stop" || action == "remove" {
		state, err := f.store.Load()
		if err != nil || state.Journal == nil {
			f.t.Fatal("runtime mutation without a durable journal", err)
		}
	}
	if f.failures[key] > 0 {
		f.failures[key]--
		return errors.New("injected " + key)
	}
	return nil
}
func (f *lifecycleRuntime) Check(ctx context.Context, r Release, adopt bool) error {
	return f.event("check", r)
}
func (f *lifecycleRuntime) Prepare(ctx context.Context, r Release) error {
	return f.event("prepare", r)
}
func (f *lifecycleRuntime) Start(ctx context.Context, r Release) error {
	if err := f.event("start", r); err != nil {
		return err
	}
	f.running = &r
	f.instance++
	if f.crashAfter == "start" {
		panic("simulated process death")
	}
	return nil
}
func (f *lifecycleRuntime) Stop(ctx context.Context, r Release) error {
	if err := f.event("stop", r); err != nil {
		return err
	}
	f.running = nil
	if f.crashAfter == "stop" {
		panic("simulated process death")
	}
	return nil
}
func (f *lifecycleRuntime) Remove(ctx context.Context, r Release) error {
	if err := f.event("remove", r); err != nil {
		return err
	}
	f.running = nil
	if f.crashAfter == "remove" {
		panic("simulated process death")
	}
	return nil
}
func (f *lifecycleRuntime) Observe(ctx context.Context, r Release) (Observation, error) {
	if f.unknown {
		return Observation{}, errors.New("unavailable")
	}
	observation := Observation{Process: "stopped", Application: "unknown", Data: "unknown"}
	if f.running != nil {
		if f.running.Digest != r.Digest {
			return Observation{}, errors.New("wrong artifact")
		}
		observation.Process = "running"
		observation.Identity = "instance"
		observation.Application = f.readiness
	}
	return observation, nil
}

func lifecycleFixture(t *testing.T) (Manager, *lifecycleRuntime, Bundle, Binding) {
	t.Helper()
	root := t.TempDir()
	store := Store{Root: filepath.Join(root, "state")}
	runtime := &lifecycleRuntime{t: t, store: store, failures: map[string]int{}, readiness: "healthy"}
	bundle := Bundle{Manifest: Manifest{Schema: 1, ID: "example", Version: "1.0.0",
		Source:  Source{Repository: "example/extension", Revision: strings.Repeat("a", 40)},
		Runtime: Runtime{Kind: "compose"}, Requires: Requirements{ManagerAPI: 1,
			Platforms: []string{"linux/amd64"}, Contracts: []string{"api-1"}},
		Grants: []string{}, DataSchema: 1, Health: "process-only", Profile: "isolated"},
		Compose: []byte(`{"services":{"test":{"image":"example/image@sha256:` + strings.Repeat("b", 64) + `"}}}`)}
	host := HostConfig{Schema: 1, Installation: "test", Platform: "linux/amd64", Contracts: []string{"api-1"},
		TrustedRepositories: []string{"example/extension"}, ApprovedGrants: []string{}, ExternalUnits: []string{}}
	binding := Binding{Project: "test-ext-example", ConfigDir: filepath.Join(root, "config"), DataDir: filepath.Join(root, "data")}
	return Manager{Store: store, Host: host, Backend: runtime}, runtime, bundle, binding
}

func applyOK(t *testing.T, m Manager, action string, b *Bundle, binding Binding) {
	t.Helper()
	if err := m.Apply(context.Background(), action, "example", b, binding); err != nil {
		t.Fatalf("%s: %v", action, err)
	}
}
func candidateBundle(b Bundle) Bundle {
	b.Manifest.Version = "1.0.1"
	b.Manifest.Source.Revision = strings.Repeat("c", 40)
	return b
}
func stateEntry(t *testing.T, m Manager) Entry {
	t.Helper()
	state, err := m.Store.Load()
	if err != nil {
		t.Fatal(err)
	}
	return state.Entries["example"]
}

func TestLifecyclePersistsAcrossManagerRestart(t *testing.T) {
	m, r, bundle, binding := lifecycleFixture(t)
	applyOK(t, m, "install", &bundle, binding)
	if r.running != nil || stateEntry(t, m).Enabled {
		t.Fatal("install started runtime")
	}
	m = Manager{Store: m.Store, Host: m.Host, Backend: r}
	applyOK(t, m, "enable", nil, Binding{})
	if !stateEntry(t, m).Enabled || r.running == nil {
		t.Fatal("enable did not persist")
	}
	applyOK(t, m, "enable", nil, Binding{})
	starts := 0
	for _, e := range r.events {
		if strings.HasPrefix(e, "start:") {
			starts++
		}
	}
	if starts != 1 {
		t.Fatal("enable recreated running service")
	}
	applyOK(t, m, "disable", nil, Binding{})
	applyOK(t, m, "remove", nil, Binding{})
	state, _ := m.Store.Load()
	if len(state.Entries) != 0 || state.Retained["example"].Binding != binding {
		t.Fatal("remove lost retained identity")
	}
	applyOK(t, m, "install", &bundle, binding)
}

func TestUpgradeRollbackAndPrepareFailure(t *testing.T) {
	m, r, bundle, binding := lifecycleFixture(t)
	applyOK(t, m, "install", &bundle, binding)
	applyOK(t, m, "enable", nil, Binding{})
	target := candidateBundle(bundle)
	r.failures["prepare:1.0.1"] = 1
	if err := m.Apply(context.Background(), "upgrade", "example", &target, Binding{}); err == nil {
		t.Fatal("prepare failure accepted")
	}
	if r.running.Digest != BundleDigest(bundle) {
		t.Fatal("prepare failure stopped previous runtime")
	}
	applyOK(t, m, "upgrade", &target, Binding{})
	if stateEntry(t, m).Previous == nil || r.running.Digest != BundleDigest(target) {
		t.Fatal("upgrade did not retain previous identity")
	}
	applyOK(t, m, "rollback", nil, Binding{})
	if r.running.Digest != BundleDigest(bundle) {
		t.Fatal("rollback failed")
	}
}

func TestDisabledUpgradeRecoveryNeverStartsRuntime(t *testing.T) {
	for _, failure := range []string{"remove-error", "process-death"} {
		t.Run(failure, func(t *testing.T) {
			m, runtime, bundle, binding := lifecycleFixture(t)
			applyOK(t, m, "install", &bundle, binding)
			target := candidateBundle(bundle)
			if failure == "remove-error" {
				runtime.failures["remove:1.0.0"] = 1
				if err := m.Apply(context.Background(), "upgrade", "example", &target, Binding{}); err == nil || !strings.Contains(err.Error(), "recovery completed") {
					t.Fatal("failed removal was not recovered", err)
				}
			} else {
				runtime.crashAfter = "remove"
				func() {
					defer func() {
						if recover() == nil {
							t.Fatal("process death was not injected")
						}
					}()
					_ = m.Apply(context.Background(), "upgrade", "example", &target, Binding{})
				}()
				runtime.crashAfter = ""
				m = Manager{Store: m.Store, Host: m.Host, Backend: runtime}
				if err := m.Recover(context.Background()); err != nil {
					t.Fatal(err)
				}
			}
			entry := stateEntry(t, m)
			if entry.Enabled || entry.RecoveryRequired || entry.Active.Digest != BundleDigest(bundle) {
				t.Fatal("recovery did not restore the disabled registration", entry)
			}
			applyOK(t, m, "upgrade", &target, Binding{})
			for _, event := range runtime.events {
				if strings.HasPrefix(event, "start:") {
					t.Fatal("disabled upgrade or recovery started the runtime")
				}
			}
		})
	}
}

func TestDisabledUpgradeRejectsRunningDrift(t *testing.T) {
	m, runtime, bundle, binding := lifecycleFixture(t)
	applyOK(t, m, "install", &bundle, binding)
	active := stateEntry(t, m).Active
	runtime.running = &active // manually started outside the manager
	target := candidateBundle(bundle)
	if err := m.Apply(context.Background(), "upgrade", "example", &target, Binding{}); err == nil {
		t.Fatal("disabled registration hid a running service")
	}
	if stateEntry(t, m).Active.Digest != active.Digest || runtime.running == nil {
		t.Fatal("rejected upgrade changed the registered or running release")
	}
}

func TestFailedActivationRestoresOrRecordsRecovery(t *testing.T) {
	for _, failRecovery := range []bool{false, true} {
		t.Run(map[bool]string{false: "restored", true: "blocked"}[failRecovery], func(t *testing.T) {
			m, r, bundle, binding := lifecycleFixture(t)
			applyOK(t, m, "install", &bundle, binding)
			applyOK(t, m, "enable", nil, Binding{})
			target := candidateBundle(bundle)
			r.failures["start:1.0.1"] = 1
			if failRecovery {
				r.failures["start:1.0.0"] = 1
			}
			if err := m.Apply(context.Background(), "upgrade", "example", &target, Binding{}); err == nil {
				t.Fatal("activation failure hidden")
			}
			state, _ := m.Store.Load()
			if (state.Journal != nil) != failRecovery || state.Entries["example"].RecoveryRequired != failRecovery {
				t.Fatalf("wrong recovery status: %+v", state)
			}
			if failRecovery {
				if err := m.Apply(context.Background(), "disable", "example", nil, Binding{}); err == nil {
					t.Fatal("pending recovery ignored")
				}
				if err := m.Recover(context.Background()); err != nil {
					t.Fatal(err)
				}
			}
			if r.running == nil || r.running.Digest != BundleDigest(bundle) {
				t.Fatal("previous runtime not restored")
			}
		})
	}
}

func TestProcessDeathRecoveryAfterStopOrStart(t *testing.T) {
	for _, crash := range []string{"stop", "start"} {
		t.Run(crash, func(t *testing.T) {
			m, r, bundle, binding := lifecycleFixture(t)
			applyOK(t, m, "install", &bundle, binding)
			applyOK(t, m, "enable", nil, Binding{})
			r.crashAfter = crash
			target := candidateBundle(bundle)
			func() {
				defer func() {
					if recover() == nil {
						t.Fatal("crash was not injected")
					}
				}()
				_ = m.Apply(context.Background(), "upgrade", "example", &target, Binding{})
			}()
			state, _ := m.Store.Load()
			if state.Journal == nil {
				t.Fatal("interrupted journal lost")
			}
			r.crashAfter = ""
			fresh := Manager{Store: m.Store, Host: m.Host, Backend: r}
			if err := fresh.Recover(context.Background()); err != nil {
				t.Fatal(err)
			}
			state, _ = m.Store.Load()
			if state.Journal != nil || r.running == nil || r.running.Digest != BundleDigest(bundle) {
				t.Fatal("recovery did not converge to prior release")
			}
		})
	}
}

func TestRecoveryDoesNotRestartIncompatiblePriorRelease(t *testing.T) {
	m, r, bundle, binding := lifecycleFixture(t)
	applyOK(t, m, "install", &bundle, binding)
	applyOK(t, m, "enable", nil, Binding{})
	target := candidateBundle(bundle)
	target.Manifest.Requires.Contracts = []string{}
	m.Host.Contracts = []string{}
	r.failures["start:1.0.1"] = 1
	if err := m.Apply(context.Background(), "upgrade", "example", &target, Binding{}); err == nil {
		t.Fatal("failure accepted")
	}
	if r.running != nil || !stateEntry(t, m).RecoveryRequired {
		t.Fatal("incompatible release restarted")
	}
}

func TestInterruptedDisableAndRemoveCompleteRequestedOperation(t *testing.T) {
	for _, action := range []string{"disable", "remove"} {
		t.Run(action, func(t *testing.T) {
			m, r, bundle, binding := lifecycleFixture(t)
			applyOK(t, m, "install", &bundle, binding)
			applyOK(t, m, "enable", nil, Binding{})
			r.crashAfter = "stop"
			if action == "remove" {
				r.crashAfter = ""
				applyOK(t, m, "disable", nil, Binding{})
				r.crashAfter = "remove"
			}
			func() {
				defer func() {
					if recover() == nil {
						t.Fatal("crash was not injected")
					}
				}()
				_ = m.Apply(context.Background(), action, "example", nil, Binding{})
			}()
			r.crashAfter = ""
			if err := m.Recover(context.Background()); err != nil {
				t.Fatal(err)
			}
			state, _ := m.Store.Load()
			if state.Journal != nil || r.running != nil {
				t.Fatal("interrupted teardown did not converge")
			}
			if action == "remove" {
				if len(state.Entries) != 0 || state.Retained["example"].Binding != binding {
					t.Fatal("remove recovery lost data ownership")
				}
			} else if state.Entries["example"].Enabled {
				t.Fatal("disable recovery re-enabled service")
			}
		})
	}
}

func TestFailedEnableCleanupIsVisibleUntilExplicitRecovery(t *testing.T) {
	m, r, bundle, binding := lifecycleFixture(t)
	bundle.Manifest.Health = "application"
	applyOK(t, m, "install", &bundle, binding)
	r.readiness = "unknown"
	r.failures["stop:1.0.0"] = 1
	if err := m.Apply(context.Background(), "enable", "example", nil, Binding{}); err == nil {
		t.Fatal("failed cleanup accepted")
	}
	status, err := m.Status(context.Background())
	if err != nil || status[0].Enabled || !status[0].RecoveryRequired || status[0].Observation.Process != "running" {
		t.Fatalf("pending process was hidden: %+v %v", status, err)
	}
	conflicts, err := m.Preflight(context.Background(), m.Host)
	if err != nil || len(conflicts["example"]) == 0 {
		t.Fatal("preflight ignored failed cleanup", err)
	}
	if err := m.Recover(context.Background()); err != nil {
		t.Fatal(err)
	}
	if r.running != nil || stateEntry(t, m).Enabled || stateEntry(t, m).RecoveryRequired {
		t.Fatal("explicit recovery did not restore disabled state")
	}
}

func TestRetainedDataCannotChangeSchemaOrOwner(t *testing.T) {
	m, _, bundle, binding := lifecycleFixture(t)
	applyOK(t, m, "install", &bundle, binding)
	applyOK(t, m, "remove", nil, Binding{})
	target := candidateBundle(bundle)
	target.Manifest.DataSchema = 2
	if err := m.Apply(context.Background(), "install", "example", &target, binding); err == nil {
		t.Fatal("retained schema migrated implicitly")
	}
	applyOK(t, m, "install", &bundle, binding)
}

func TestApplicationHealthFailureDoesNotInventDataFreshness(t *testing.T) {
	m, r, bundle, binding := lifecycleFixture(t)
	bundle.Manifest.Health = "application"
	applyOK(t, m, "install", &bundle, binding)
	r.readiness = "unknown"
	if err := m.Apply(context.Background(), "enable", "example", nil, Binding{}); err == nil {
		t.Fatal("unknown readiness accepted")
	}
	if r.running != nil || stateEntry(t, m).Enabled {
		t.Fatal("failed initial start not cleaned up")
	}
	r.unknown = true
	status, err := m.Status(context.Background())
	if err != nil || status[0].Observation.Data != "unknown" {
		t.Fatal("unknown observation was upgraded to health", err)
	}
}

func TestExternalAdoptionNeverMutatesRuntime(t *testing.T) {
	m, r, bundle, binding := lifecycleFixture(t)
	bundle.Compose = nil
	bundle.Manifest.Runtime = Runtime{Kind: "external-systemd", Units: []string{"mowgli-lora-robot.service"}}
	bundle.Manifest.Profile = "external"
	m.Host.ExternalUnits = []string{"mowgli-lora-robot.service"}
	applyOK(t, m, "adopt", &bundle, binding)
	for _, action := range []string{"enable", "disable", "remove", "upgrade"} {
		if err := m.Apply(context.Background(), action, "example", &bundle, binding); err == nil {
			t.Fatal("external mutation accepted", action)
		}
	}
	applyOK(t, m, "forget", nil, Binding{})
	for _, e := range r.events {
		if !strings.HasPrefix(e, "check:") {
			t.Fatal("external adoption mutated runtime", r.events)
		}
	}
}

func TestPreflightDoesNotIgnoreUnknownOrRecoveringRuntime(t *testing.T) {
	m, r, bundle, binding := lifecycleFixture(t)
	applyOK(t, m, "install", &bundle, binding)
	target := m.Host
	target.Contracts = []string{}
	r.unknown = true
	conflicts, err := m.Preflight(context.Background(), target)
	if err != nil || len(conflicts["example"]) == 0 {
		t.Fatal("unknown runtime ignored", err)
	}
	r.unknown = false
	conflicts, err = m.Preflight(context.Background(), target)
	if err != nil || len(conflicts) != 0 {
		t.Fatal("known disabled runtime unnecessarily blocked", err)
	}
	state, _ := m.Store.Load()
	entry := state.Entries["example"]
	state.Journal = &Journal{ID: "example", Action: "enable", Phase: "applying", Before: &entry, Target: &entry}
	if err := m.Store.Save(state); err != nil {
		t.Fatal(err)
	}
	conflicts, err = m.Preflight(context.Background(), m.Host)
	if err != nil || len(conflicts["example"]) == 0 {
		t.Fatal("pending recovery ignored", err)
	}
}

func TestServiceTopologyCannotBypassRollbackMigrationGate(t *testing.T) {
	m, r, bundle, binding := lifecycleFixture(t)
	applyOK(t, m, "install", &bundle, binding)
	applyOK(t, m, "enable", nil, Binding{})
	target := candidateBundle(bundle)
	target.Compose = []byte(strings.Replace(string(bundle.Compose), `"test":`, `"another":{"image":"example/other@sha256:`+strings.Repeat("d", 64)+`"},"test":`, 1))
	before := len(r.events)
	if err := m.Apply(context.Background(), "upgrade", "example", &target, Binding{}); err == nil || !strings.Contains(err.Error(), "topology") {
		t.Fatal("topology change accepted", err)
	}
	if len(r.events) != before || r.running.Digest != BundleDigest(bundle) {
		t.Fatal("migration rejection mutated runtime")
	}
}
