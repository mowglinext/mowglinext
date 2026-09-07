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
	"testing"
	"time"
)

// This test uses a disposable Compose project, ordinary unprivileged containers
// and synthetic readiness. It never connects to a mower or ROS hardware.
type integrationBackend struct {
	DockerBackend
	failVerification bool
	data             string
	target           string
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
func TestDockerTransactionRestoresImagesAndData(t *testing.T) {
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
		s := map[string]any{"image": "busybox:1.36", "container_name": project + "-" + name, "command": []string{"sleep", "3600"}, "user": fmt.Sprint(os.Getuid()), "labels": map[string]string{"garden.mowgli.maintenance-api": "1"}}
		if name == "metrics" {
			s["labels"] = map[string]string{updateLabel + "image": "metrics", updateLabel + "after": "mowgli"}
		}
		if name == "gui" {
			s["volumes"] = []string{db + ":/db"}
		}
		services[name] = s
	}
	if err := AtomicJSON(filepath.Join(runtimeDir, "docker-compose.yaml"), map[string]any{"services": services}); err != nil {
		t.Fatal(err)
	}
	config := HostConfig{Directory: runtimeDir, Project: project, StateDir: stateDir, Trusted: []string{"mowglinext/mowglinext"}, Platform: "linux/amd64"}
	backend := &integrationBackend{DockerBackend: DockerBackend{config}, failVerification: true, data: dataPath, target: "busybox:1.37"}
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
	source := &fakeSource{releases: []Deployment{fixture()}}
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
	// Remove only this transaction's retained image tags from the disposable host.
	for service := range plan.Previous {
		_, _ = command(ctx, "docker", "image", "rm", "mowgli-rollback:"+state.Job.ID+"-"+service)
	}
}
