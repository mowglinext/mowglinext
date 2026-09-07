package updater

import (
	"context"
	"encoding/json"
	"errors"
	"fmt"
	"net"
	"net/http"
	"os"
	"path/filepath"
	"strings"
	"testing"
	"time"

	"github.com/mowglinext/mowglinext/pkg/updates"
)

// This test uses a disposable Compose project, ordinary unprivileged containers
// and synthetic readiness. It never connects to a mower or ROS hardware.
type integrationBackend struct {
	DockerBackend
	failVerification bool
	data             string
	target           string
	bundle           *ComposeBundle
}

func (b *integrationBackend) PlanStack(ctx context.Context, d Deployment, overrides map[string]Deployment) (map[string]string, *StackPlan, error) {
	copyData, _ := json.Marshal(d)
	_ = json.Unmarshal(copyData, &d)
	if b.bundle == nil {
		return nil, nil, errors.New("missing fixture bundle")
	}
	data, err := command(ctx, "docker", "image", "inspect", "busybox:1.37")
	if err != nil {
		return nil, nil, err
	}
	var images []struct {
		ID      string   `json:"Id"`
		Digests []string `json:"RepoDigests"`
	}
	if err = json.Unmarshal(data, &images); err != nil || len(images) != 1 || len(images[0].Digests) == 0 {
		return nil, nil, errors.New("missing test image")
	}
	reference := strings.Split(images[0].Digests[0], "@")
	b.target = images[0].Digests[0]
	for _, family := range []string{"mowgli-ros2", "mowglinext-gui", "helper"} {
		d.Images[family] = updates.Image{Repository: reference[0], Digest: reference[1], Platforms: map[string]updates.Platform{"linux/amd64": {Manifest: reference[1], Config: images[0].ID}}}
	}
	return b.planBundle(ctx, d, overrides, *b.bundle, StackSelection{Options: map[string]string{"gnss": "none", "lidar": "none"}})
}

func (b *integrationBackend) PlanImages(context.Context, Deployment) (map[string]string, error) {
	return map[string]string{"gui": b.target, "mowgli": b.target, "metrics": b.target}, nil
}
func (b *integrationBackend) Verify(ctx context.Context, images map[string]string, d *Deployment) error {
	if b.failVerification && images["gui"] == b.target {
		b.failVerification = false
		_ = os.WriteFile(b.data, []byte("new incompatible data"), 0600)
		return errors.New("injected application failure")
	}
	return b.DockerBackend.Verify(ctx, images, d)
}
func TestDockerTransactionRestoresImagesAndData(t *testing.T)    { runDockerTransaction(t, false, true) }
func TestDockerReleaseTopologyFailureRestoresStack(t *testing.T) { runDockerTransaction(t, true, true) }
func TestDockerReleaseTopologyCommitAndExplicitRollback(t *testing.T) {
	runDockerTransaction(t, true, false)
}
func runDockerTransaction(t *testing.T, topology, failVerification bool) {
	if os.Getenv("MOWGLI_UPDATER_DOCKER_TESTS") != "1" {
		t.Skip("set MOWGLI_UPDATER_DOCKER_TESTS=1 on a disposable Docker test host")
	}
	ctx, cancel := context.WithTimeout(context.Background(), 5*time.Minute)
	defer cancel()
	dir := t.TempDir()
	runtimeDir := filepath.Join(dir, "docker")
	// Keep archives on Linux storage even when the fixture Compose directory
	// is shared from Docker Desktop's Windows host.
	stateDir, err := os.MkdirTemp("/tmp", "mowgli-updater-state-")
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() { _ = os.RemoveAll(stateDir) })
	db := filepath.Join(dir, "db")
	for _, p := range []string{runtimeDir, stateDir, db} {
		if err := os.MkdirAll(p, 0755); err != nil {
			t.Fatal(err)
		}
	}
	dataPath := filepath.Join(db, "data")
	_ = os.WriteFile(dataPath, []byte("original data"), 0600)
	_ = os.WriteFile(filepath.Join(runtimeDir, ".env"), nil, 0600)
	project := fmt.Sprintf("updater-test-%d", time.Now().UnixNano())
	services := map[string]any{}
	for _, name := range []string{"gui", "mowgli", "metrics"} {
		s := map[string]any{"image": "busybox:1.36", "container_name": project + "-" + name, "command": []string{"sleep", "3600"}, "stop_grace_period": "1s", "user": fmt.Sprint(os.Getuid()), "labels": map[string]string{"garden.mowgli.maintenance-api": "1"}}
		if name == "metrics" {
			s["labels"] = map[string]string{updateLabel + "image": "metrics", updateLabel + "after": "mowgli"}
		}
		if name == "gui" {
			s["volumes"] = []string{db + ":/db"}
		}
		services[name] = s
	}
	var bundle *ComposeBundle
	if topology {
		for _, name := range []string{"gui", "mowgli"} {
			service := object(services[name])
			service["environment"] = map[string]string{"MOWGLI_UPDATE_MAINTENANCE": "/var/lib/mowgli-updater/maintenance"}
			mounts, _ := service["volumes"].([]string)
			service["volumes"] = append(mounts, stateDir+":/var/lib/mowgli-updater:ro")
		}
		nextData, _ := json.Marshal(services)
		var next map[string]any
		_ = json.Unmarshal(nextData, &next)
		delete(next, "metrics")
		next["helper"] = map[string]any{"image": "busybox:1.37", "container_name": project + "-helper", "command": []string{"sleep", "3600"}, "stop_grace_period": "1s", "user": fmt.Sprint(os.Getuid()), "labels": map[string]string{updateLabel + "image": "helper", updateLabel + "after": "mowgli"}}
		fragment, _ := json.Marshal(map[string]any{"services": next})
		bundle = &ComposeBundle{StackDefinition: StackDefinition{Schema: 1, Required: []string{"stack.json"}, Options: map[string]map[string][]string{"gnss": {"none": {}}, "lidar": {"none": {}}}}, Fragments: map[string]json.RawMessage{"stack.json": fragment}}
		services["mqtt"] = map[string]any{"image": "busybox:1.36", "container_name": project + "-mqtt", "command": []string{"sleep", "3600"}, "stop_grace_period": "1s", "user": fmt.Sprint(os.Getuid())}
	}
	if err := AtomicJSON(filepath.Join(runtimeDir, "docker-compose.yaml"), map[string]any{"services": services}); err != nil {
		t.Fatal(err)
	}
	config := HostConfig{Directory: runtimeDir, Project: project, StateDir: stateDir, Trusted: []string{"mowglinext/mowglinext"}, Platform: "linux/amd64"}
	backend := &integrationBackend{DockerBackend: DockerBackend{config}, failVerification: failVerification, bundle: bundle, data: dataPath, target: "busybox:1.37"}
	if _, err := backend.compose(ctx, "up", "-d"); err != nil {
		t.Fatal(err)
	}
	defer func() { _, _ = backend.compose(context.Background(), "down") }()
	listener, err := net.Listen("tcp", "127.0.0.1:4006")
	if err != nil {
		t.Fatal(err)
	}
	server := &http.Server{Handler: http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		_, e := os.Stat(filepath.Join(stateDir, "maintenance"))
		_ = json.NewEncoder(w).Encode(Readiness{Ready: true, Maintenance: e == nil, FirmwareProtocol: 6, GPSFresh: true, LidarFresh: true})
	})}
	go server.Serve(listener)
	defer server.Close()
	deployment := fixture()
	mqttID := ""
	if topology {
		if _, err = command(ctx, "docker", "pull", "busybox:1.37"); err != nil {
			t.Fatal(err)
		}
		deployment.Schema = 2
		deployment.Bundle = &BundleAsset{Asset: "mowgli-compose.json", SHA256: strings.Repeat("a", 64)}
		ci, e := backend.inspect(ctx, project+"-mqtt")
		if e != nil {
			t.Fatal(e)
		}
		mqttID = ci.ID
	}
	source := &fakeSource{releases: []Deployment{deployment}}
	manager, err := Open(stateDir, config.Trusted, backend, source)
	if err != nil {
		t.Fatal(err)
	}
	if err = manager.Check(ctx, true); err != nil {
		t.Fatal(err)
	}
	plan, err := manager.MakePlan(ctx, fixture().ID, false)
	if err != nil {
		t.Fatal(err)
	}
	if topology {
		actions := map[string]string{}
		for _, change := range plan.Stack.Changes {
			actions[change.Service] = change.Action
		}
		if actions["helper"] != "add" || actions["metrics"] != "remove" || actions["mqtt"] != "unmanaged" {
			t.Fatal(actions)
		}
	}
	if _, err = manager.Start(plan.ID); err != nil {
		t.Fatal(err)
	}
	deadline := time.Now().Add(3 * time.Minute)
	for time.Now().Before(deadline) {
		manager.mu.Lock()
		busy := manager.busy
		manager.mu.Unlock()
		if !busy {
			break
		}
		time.Sleep(100 * time.Millisecond)
	}
	state := manager.Snapshot()
	if topology && !failVerification {
		if state.Job.Phase != "succeeded" {
			t.Fatalf("activation failed: %+v", state.Job)
		}
		if _, err = backend.inspect(ctx, project+"-helper"); err != nil {
			t.Fatal("added service missing", err)
		}
		ids, e := command(ctx, "docker", "ps", "-aq", "--filter", "name=^/"+project+"-metrics$")
		if e != nil || strings.TrimSpace(string(ids)) != "" {
			t.Fatal("retired container remains", e)
		}
		if _, err = manager.Rollback(); err != nil {
			t.Fatal(err)
		}
		deadline = time.Now().Add(2 * time.Minute)
		for time.Now().Before(deadline) {
			manager.mu.Lock()
			busy := manager.busy
			manager.mu.Unlock()
			if !busy {
				break
			}
			time.Sleep(100 * time.Millisecond)
		}
		state = manager.Snapshot()
	}
	if state.Job.Phase != "rolled_back" {
		t.Fatalf("job: %+v", state.Job)
	}
	data, err := os.ReadFile(dataPath)
	if err != nil || string(data) != "original data" {
		t.Fatalf("data not restored: %q, %v", data, err)
	}
	if _, err = os.Stat(filepath.Join(stateDir, "maintenance")); !os.IsNotExist(err) {
		t.Fatal("maintenance not released after verified rollback")
	}
	if err = backend.DockerBackend.Verify(ctx, plan.Previous, nil); err != nil {
		t.Fatal(err)
	}
	if topology {
		ci, e := backend.inspect(ctx, project+"-mqtt")
		if e != nil || ci.ID != mqttID || !ci.State.Running {
			t.Fatal("unmanaged container was changed", e)
		}
		ids, e := command(ctx, "docker", "ps", "-aq", "--filter", "name=^/"+project+"-helper$")
		if e != nil || strings.TrimSpace(string(ids)) != "" {
			t.Fatal("new container survives rollback", e)
		}
		if _, e = backend.inspect(ctx, project+"-metrics"); e != nil {
			t.Fatal("retired container not restored", e)
		}
	}
	// Remove only this transaction's retained image tags from the disposable host.
	for service := range plan.Previous {
		_, _ = command(ctx, "docker", "image", "rm", "mowgli-rollback:"+filepath.Base(state.Job.Backup)+"-"+service)
	}
}
