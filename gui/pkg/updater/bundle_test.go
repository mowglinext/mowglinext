package updater

import (
	"context"
	"encoding/json"
	"os"
	"path/filepath"
	"reflect"
	"strings"
	"testing"

	"github.com/mowglinext/mowglinext/pkg/updates"
)

func TestReleaseBundleSharesInstallerSelections(t *testing.T) {
	b, err := ReadComposeBundle("../../../install/compose")
	if err != nil {
		t.Fatal(err)
	}
	for _, gnss := range []string{"none", "universal"} {
		for _, lidar := range []string{"none", "ldlidar", "rplidar", "stl27l"} {
			files, err := b.Select(map[string]string{"gnss": gnss, "lidar": lidar})
			if err != nil {
				t.Fatal(err)
			}
			joined := strings.Join(files, " ")
			if strings.Contains(joined, "docker-compose.gps.yml") != (gnss != "none") {
				t.Fatal(files)
			}
			if strings.Contains(joined, "docker-compose.lidar-") != (lidar != "none") {
				t.Fatal(files)
			}
			if strings.Contains(joined, "watchtower") {
				t.Fatal("Watchtower entered managed selection")
			}
		}
	}
	data, _ := json.Marshal(b)
	if _, err = decodeBundle(data, strings.TrimPrefix(updates.Hash(data), "sha256:")); err != nil {
		t.Fatal(err)
	}
	if _, err = decodeBundle(append(data, ' '), strings.TrimPrefix(updates.Hash(data), "sha256:")); err == nil {
		t.Fatal("tampered bundle accepted")
	}
	b.Options["future"] = map[string][]string{"none": {}, "enabled": {"future.json"}}
	files, err := b.Select(map[string]string{"gnss": "universal"})
	if err != nil || strings.Contains(strings.Join(files, " "), "future.json") {
		t.Fatal("new optional service enabled automatically")
	}
	delete(b.Options["lidar"], "stl27l")
	if _, err = b.Select(map[string]string{"lidar": "stl27l"}); err == nil {
		t.Fatal("removed selected hardware silently disabled")
	}
	delete(b.Options, "gnss")
	if _, err = b.Select(map[string]string{"gnss": "universal"}); err == nil {
		t.Fatal("removed selected group accepted")
	}
}

func TestReleaseBundleRejectsExternalInputs(t *testing.T) {
	for _, doc := range []string{
		`{"include":"https://example.org/compose.yaml","services":{}}`,
		`{"secrets":{},"services":{}}`,
		`{"services":{"gui":{"build":"."}}}`,
		`{"services":{"gui":{"env_file":"/etc/secret"}}}`,
		`{"services":{"gui":{"extends":{"file":"/etc/secret"}}}}`,
		`{"services":{"gui":{"post_start":[{"command":"something"}]}}}`,
	} {
		var m map[string]any
		_ = json.Unmarshal([]byte(doc), &m)
		if validateComposeDocument(m) == nil {
			t.Fatalf("accepted external input: %s", doc)
		}
	}
	d := StackDefinition{Schema: 1, Required: []string{"../outside.yml"}}
	if _, err := d.Select(nil); err == nil {
		t.Fatal("path traversal accepted")
	}
}

func TestLocalServicesAndResourcesCannotBeClaimed(t *testing.T) {
	old := []byte(`{"services":{"gui":{},"obsolete":{},"mqtt":{"image":"local-mqtt"}},"volumes":{"saved":{"name":"saved-data"}}}`)
	next := []byte(`{"services":{"gui":{},"new-helper":{}}}`)
	result, err := retainLocalServices(old, next, map[string]managedService{"gui": {}, "obsolete": {}})
	if err != nil {
		t.Fatal(err)
	}
	var actual map[string]any
	_ = json.Unmarshal(result, &actual)
	services := object(actual["services"])
	if _, ok := services["obsolete"]; ok {
		t.Fatal("retired service retained")
	}
	if services["mqtt"] == nil || services["new-helper"] == nil || object(actual["volumes"])["saved"] == nil {
		t.Fatal(string(result))
	}
	for _, conflict := range []string{`{"services":{"mqtt":{}}}`, `{"services":{},"volumes":{"saved":{"driver":"other"}}}`} {
		if _, err = retainLocalServices(old, []byte(conflict), map[string]managedService{"gui": {}, "obsolete": {}}); err == nil {
			t.Fatal("resource/service takeover accepted")
		}
	}
	result, err = retainLocalServices(old, []byte(`{"services":{},"volumes":{"saved":{"name":"new-project_saved"}}}`), map[string]managedService{"gui": {}, "obsolete": {}})
	if err != nil || !strings.Contains(string(result), "saved-data") || strings.Contains(string(result), "new-project_saved") {
		t.Fatal("physical volume identity changed", err)
	}
}

func TestComposeListAndMapEnvironmentsPreserveReferences(t *testing.T) {
	for _, data := range []string{
		`{"environment":{"SERIAL":"${GNSS_SERIAL_DEVICE}","SECRET":"a=b"},"labels":{"garden.mowgli.update.image":"gps"}}`,
		`{"environment":["SERIAL=${GNSS_SERIAL_DEVICE}","SECRET=a=b"],"labels":["garden.mowgli.update.image=gps"]}`,
	} {
		var sc serviceConfig
		if err := json.Unmarshal([]byte(data), &sc); err != nil {
			t.Fatal(err)
		}
		if sc.Environment["SERIAL"] != "${GNSS_SERIAL_DEVICE}" || sc.Environment["SECRET"] != "a=b" || sc.Labels[updateLabel+"image"] != "gps" {
			t.Fatalf("lost Compose configuration: %+v", sc)
		}
	}
}

func TestStackPlanHTTPRedactsRecoveryConfiguration(t *testing.T) {
	private := &StackPlan{Compose: []byte("NTRIP_PASSWORD=secret"), Bundle: ComposeBundle{Fragments: map[string]json.RawMessage{"private": json.RawMessage(`{"secret":"value"}`)}}, Changes: []ServiceChange{{"helper", "add"}}, Selection: StackSelection{Options: map[string]string{"lidar": "none"}}}
	p := Plan{Stack: private}
	s := PublicState(State{Plans: []Plan{p}, Job: &Job{Plan: p}, History: []Job{{Plan: p}}})
	data, err := json.Marshal(s)
	if err != nil {
		t.Fatal(err)
	}
	if strings.Contains(string(data), "secret") || strings.Contains(string(data), "TlRSSVB") || s.Job.Plan.Stack.Compose != nil {
		t.Fatal("private Compose leaked")
	}
	if len(private.Compose) == 0 || s.Plans[0].Stack.Changes[0].Action != "add" {
		t.Fatal("redaction damaged journal/review")
	}
}

func TestInstallerMembershipIdentityAndPendingSelections(t *testing.T) {
	dir := t.TempDir()
	env := filepath.Join(dir, ".env")
	_ = os.WriteFile(env, []byte("LIDAR_ENABLED=false\nLIDAR_TYPE=none\nGNSS_STACK=universal\nPASSWORD=one\n"), 0600)
	a, err := installerConfigIdentity(dir)
	if err != nil {
		t.Fatal(err)
	}
	_ = os.WriteFile(env, []byte("LIDAR_ENABLED=false\nLIDAR_TYPE=none\nGNSS_STACK=universal\nPASSWORD=two\n"), 0600)
	b, _ := installerConfigIdentity(dir)
	if a != b {
		t.Fatal("ordinary setting changed membership identity")
	}
	_ = os.WriteFile(env, []byte("LIDAR_ENABLED=true\nLIDAR_TYPE=stl27l\nGNSS_STACK=universal\n"), 0600)
	c, _ := installerConfigIdentity(dir)
	if a == c {
		t.Fatal("hardware selection change went unnoticed")
	}
	backend := DockerBackend{Config: HostConfig{Directory: dir}}
	selection := StackSelection{Options: map[string]string{"lidar": "none"}, InstallerConfig: a}
	_ = AtomicJSON(filepath.Join(dir, "stack-selection.json"), selection)
	_ = AtomicJSON(filepath.Join(dir, "stack-applied-selection.json"), selection)
	if pending, e := backend.SelectionPending(); e != nil || pending {
		t.Fatal(pending, e)
	}
	selection.Options["lidar"] = "stl27l"
	_ = AtomicJSON(filepath.Join(dir, "stack-selection.json"), selection)
	if pending, e := backend.SelectionPending(); e != nil || !pending {
		t.Fatal(pending, e)
	}
}

type stackFakeBackend struct{ *fakeBackend }

func (b stackFakeBackend) PlanStack(context.Context, Deployment, map[string]Deployment) (map[string]string, *StackPlan, error) {
	return map[string]string{"gui": "new-gui", "mowgli": "new-ros", "helper": "new-helper"}, &StackPlan{Compose: []byte("private"), Changes: []ServiceChange{{"helper", "add"}}}, nil
}
func (b stackFakeBackend) BackupStack(context.Context, string, *StackPlan) (string, error) {
	return "backup", b.event("backup-stack")
}
func (b stackFakeBackend) ApplyStack(context.Context, Plan) error { return b.event("apply-stack") }

func TestManagerUsesTopologyTransactionAndRecovers(t *testing.T) {
	for _, failure := range []string{"", "verify-new"} {
		t.Run(failure, func(t *testing.T) {
			m, b, _ := setup(t, failure)
			m.backend = stackFakeBackend{b}
			m.state.Releases[0].Schema = 2
			m.state.Releases[0].Bundle = &BundleAsset{Asset: "mowgli-compose.json", SHA256: strings.Repeat("a", 64)}
			plan, err := m.MakePlan(context.Background(), fixture().ID, false)
			if err != nil {
				t.Fatal(err)
			}
			if _, err = m.Start(plan.ID); err != nil {
				t.Fatal(err)
			}
			state := settled(t, m)
			want := "succeeded"
			if failure != "" {
				want = "rolled_back"
			}
			if state.Job.Phase != want {
				t.Fatal(state.Job.Phase, state.Job.Error)
			}
			events := strings.Join(b.events, ",")
			if !strings.Contains(events, "backup-stack,apply-stack,verify-new") {
				t.Fatal(events)
			}
			if failure != "" && !strings.Contains(events, "restore,apply-old,verify-old,ungate") {
				t.Fatal(events)
			}
			if !reflect.DeepEqual(plan.Stack.Changes, state.Job.Plan.Stack.Changes) {
				t.Fatal("review lost")
			}
		})
	}
}
