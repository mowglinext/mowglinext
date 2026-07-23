package api

import (
	"archive/tar"
	"compress/gzip"
	"context"
	"encoding/json"
	"io"
	"net/http"
	"net/http/httptest"
	"os"
	"path/filepath"
	"strings"
	"testing"

	pkgtypes "github.com/mowglinext/mowglinext/pkg/types"
	dockertypes "github.com/docker/docker/api/types"
	"github.com/gin-gonic/gin"
)

// rosbagFakeDocker is a minimal IDockerProvider whose ContainerExec is driven
// by a caller-supplied function, so tests can simulate the in-container probe.
type rosbagFakeDocker struct {
	containers []dockertypes.Container
	execFn     func(spec pkgtypes.ContainerExecSpec) (pkgtypes.ContainerExecResult, error)
}

func (f *rosbagFakeDocker) ContainerList(context.Context) ([]dockertypes.Container, error) {
	return f.containers, nil
}
func (f *rosbagFakeDocker) ContainerLogs(context.Context, string) (io.ReadCloser, error) {
	return io.NopCloser(strings.NewReader("")), nil
}
func (f *rosbagFakeDocker) ContainerStart(context.Context, string) error   { return nil }
func (f *rosbagFakeDocker) ContainerStop(context.Context, string) error    { return nil }
func (f *rosbagFakeDocker) ContainerRestart(context.Context, string) error { return nil }
func (f *rosbagFakeDocker) ContainerInspect(context.Context, string) (pkgtypes.ContainerDetails, error) {
	return pkgtypes.ContainerDetails{}, nil
}
func (f *rosbagFakeDocker) ContainerRun(context.Context, pkgtypes.ContainerRunSpec) (pkgtypes.ContainerRunResult, error) {
	return pkgtypes.ContainerRunResult{}, nil
}
func (f *rosbagFakeDocker) ContainerExec(_ context.Context, _ string, spec pkgtypes.ContainerExecSpec) (pkgtypes.ContainerExecResult, error) {
	if f.execFn != nil {
		return f.execFn(spec)
	}
	return pkgtypes.ContainerExecResult{}, nil
}
func (f *rosbagFakeDocker) ContainerExecStart(context.Context, string, pkgtypes.ContainerExecSpec) (pkgtypes.ContainerExecHandle, error) {
	return pkgtypes.ContainerExecHandle{Reader: io.NopCloser(strings.NewReader(""))}, nil
}
func (f *rosbagFakeDocker) ContainerExecInspect(context.Context, string) (pkgtypes.ContainerExecInspectResult, error) {
	return pkgtypes.ContainerExecInspectResult{}, nil
}

func rosbagWithContainer() *rosbagFakeDocker {
	return &rosbagFakeDocker{
		containers: []dockertypes.Container{{ID: "abc123", Names: []string{"/mowgli-ros2"}}},
	}
}

func setupRosbagRouter(t *testing.T, docker pkgtypes.IDockerProvider) (*gin.Engine, string) {
	t.Helper()
	gin.SetMode(gin.TestMode)
	dir := t.TempDir()
	t.Setenv("ROSBAG_DIR", dir)
	r := gin.New()
	RosbagRoutes(r.Group("/api"), docker)
	return r, dir
}

func TestRosbagStatus_ListsRecordingsNewestFirst(t *testing.T) {
	docker := rosbagWithContainer()
	docker.execFn = func(pkgtypes.ContainerExecSpec) (pkgtypes.ContainerExecResult, error) {
		// No active pidfiles.
		return pkgtypes.ContainerExecResult{ExitCode: 0, Stdout: ""}, nil
	}
	r, dir := setupRosbagRouter(t, docker)

	for _, name := range []string{"rosbag_20260101T000000Z", "rosbag_20260202T000000Z"} {
		if err := os.MkdirAll(filepath.Join(dir, name, "bag"), 0o755); err != nil {
			t.Fatal(err)
		}
		if err := os.WriteFile(filepath.Join(dir, name, "bag", "data.mcap"), []byte("payload"), 0o644); err != nil {
			t.Fatal(err)
		}
	}

	w := httptest.NewRecorder()
	r.ServeHTTP(w, httptest.NewRequest(http.MethodGet, "/api/tools/rosbag/status", nil))
	if w.Code != http.StatusOK {
		t.Fatalf("status = %d, body=%s", w.Code, w.Body.String())
	}

	var resp RosbagStatusResponse
	if err := json.Unmarshal(w.Body.Bytes(), &resp); err != nil {
		t.Fatal(err)
	}
	if resp.Active {
		t.Fatalf("expected inactive, got active")
	}
	if len(resp.Recordings) != 2 {
		t.Fatalf("expected 2 recordings, got %d", len(resp.Recordings))
	}
	if resp.Recordings[0].Name != "rosbag_20260202T000000Z" {
		t.Fatalf("expected newest first, got %s", resp.Recordings[0].Name)
	}
	if resp.Recordings[0].SizeBytes != int64(len("payload")) {
		t.Fatalf("size = %d, want %d", resp.Recordings[0].SizeBytes, len("payload"))
	}
}

func TestRosbagStatus_ReportsActiveFromProbe(t *testing.T) {
	docker := rosbagWithContainer()
	docker.execFn = func(pkgtypes.ContainerExecSpec) (pkgtypes.ContainerExecResult, error) {
		return pkgtypes.ContainerExecResult{ExitCode: 0, Stdout: "rosbag_live\n"}, nil
	}
	r, dir := setupRosbagRouter(t, docker)
	if err := os.MkdirAll(filepath.Join(dir, "rosbag_live"), 0o755); err != nil {
		t.Fatal(err)
	}

	w := httptest.NewRecorder()
	r.ServeHTTP(w, httptest.NewRequest(http.MethodGet, "/api/tools/rosbag/status", nil))

	var resp RosbagStatusResponse
	_ = json.Unmarshal(w.Body.Bytes(), &resp)
	if !resp.Active || resp.ActiveName != "rosbag_live" {
		t.Fatalf("expected active rosbag_live, got %+v", resp)
	}
	if len(resp.Recordings) != 1 || !resp.Recordings[0].Active {
		t.Fatalf("expected recording flagged active, got %+v", resp.Recordings)
	}
}

func TestRosbagStart_RejectsWhenAlreadyRecording(t *testing.T) {
	docker := rosbagWithContainer()
	docker.execFn = func(pkgtypes.ContainerExecSpec) (pkgtypes.ContainerExecResult, error) {
		return pkgtypes.ContainerExecResult{ExitCode: 0, Stdout: "rosbag_busy\n"}, nil
	}
	r, _ := setupRosbagRouter(t, docker)

	w := httptest.NewRecorder()
	r.ServeHTTP(w, httptest.NewRequest(http.MethodPost, "/api/tools/rosbag/start", nil))
	if w.Code != http.StatusConflict {
		t.Fatalf("expected 409, got %d body=%s", w.Code, w.Body.String())
	}
}

func TestRosbagStart_LaunchesRecordAndReturnsName(t *testing.T) {
	docker := rosbagWithContainer()
	calls := 0
	docker.execFn = func(spec pkgtypes.ContainerExecSpec) (pkgtypes.ContainerExecResult, error) {
		calls++
		if calls == 1 {
			return pkgtypes.ContainerExecResult{ExitCode: 0, Stdout: ""}, nil // probe: none active
		}
		// start command: must be a `ros2 bag record -a` invocation
		joined := strings.Join(spec.Cmd, " ")
		if !strings.Contains(joined, "ros2 bag record -a") {
			t.Fatalf("start command missing ros2 bag record -a: %s", joined)
		}
		return pkgtypes.ContainerExecResult{ExitCode: 0, Stdout: "4242\n"}, nil
	}
	r, _ := setupRosbagRouter(t, docker)

	w := httptest.NewRecorder()
	r.ServeHTTP(w, httptest.NewRequest(http.MethodPost, "/api/tools/rosbag/start", nil))
	if w.Code != http.StatusOK {
		t.Fatalf("expected 200, got %d body=%s", w.Code, w.Body.String())
	}
	var resp RosbagStartResponse
	_ = json.Unmarshal(w.Body.Bytes(), &resp)
	if !strings.HasPrefix(resp.Name, "rosbag_") {
		t.Fatalf("unexpected name %q", resp.Name)
	}
}

func TestRosbagStart_SurfacesRecorderFailure(t *testing.T) {
	docker := rosbagWithContainer()
	calls := 0
	docker.execFn = func(pkgtypes.ContainerExecSpec) (pkgtypes.ContainerExecResult, error) {
		calls++
		if calls == 1 {
			return pkgtypes.ContainerExecResult{ExitCode: 0, Stdout: ""}, nil
		}
		return pkgtypes.ContainerExecResult{ExitCode: 1, Stdout: "recorder exited immediately\n"}, nil
	}
	r, _ := setupRosbagRouter(t, docker)

	w := httptest.NewRecorder()
	r.ServeHTTP(w, httptest.NewRequest(http.MethodPost, "/api/tools/rosbag/start", nil))
	if w.Code != http.StatusInternalServerError {
		t.Fatalf("expected 500, got %d", w.Code)
	}
}

func TestRosbagStop_ReportsStoppedName(t *testing.T) {
	docker := rosbagWithContainer()
	docker.execFn = func(pkgtypes.ContainerExecSpec) (pkgtypes.ContainerExecResult, error) {
		return pkgtypes.ContainerExecResult{ExitCode: 0, Stdout: "rosbag_live\n"}, nil
	}
	r, _ := setupRosbagRouter(t, docker)

	w := httptest.NewRecorder()
	r.ServeHTTP(w, httptest.NewRequest(http.MethodPost, "/api/tools/rosbag/stop", nil))
	if w.Code != http.StatusOK {
		t.Fatalf("expected 200, got %d", w.Code)
	}
	var resp RosbagStopResponse
	_ = json.Unmarshal(w.Body.Bytes(), &resp)
	if !resp.Stopped || resp.StoppedName != "rosbag_live" {
		t.Fatalf("expected stopped rosbag_live, got %+v", resp)
	}
}

func TestRosbagDownload_StreamsTarGz(t *testing.T) {
	docker := rosbagWithContainer()
	r, dir := setupRosbagRouter(t, docker)

	name := "rosbag_dl"
	bagDir := filepath.Join(dir, name, "bag")
	if err := os.MkdirAll(bagDir, 0o755); err != nil {
		t.Fatal(err)
	}
	if err := os.WriteFile(filepath.Join(bagDir, "data.mcap"), []byte("mcap-bytes"), 0o644); err != nil {
		t.Fatal(err)
	}
	if err := os.WriteFile(filepath.Join(bagDir, "metadata.yaml"), []byte("meta"), 0o644); err != nil {
		t.Fatal(err)
	}
	// Live pidfile must be excluded from the archive.
	if err := os.WriteFile(filepath.Join(dir, name, rosbagPidFile), []byte("999"), 0o644); err != nil {
		t.Fatal(err)
	}

	w := httptest.NewRecorder()
	r.ServeHTTP(w, httptest.NewRequest(http.MethodGet, "/api/tools/rosbag/download/"+name, nil))
	if w.Code != http.StatusOK {
		t.Fatalf("expected 200, got %d body=%s", w.Code, w.Body.String())
	}
	if cd := w.Header().Get("Content-Disposition"); !strings.Contains(cd, name+".tar.gz") {
		t.Fatalf("unexpected Content-Disposition %q", cd)
	}

	names := tarEntryNames(t, w.Body.Bytes())
	if _, ok := names[name+"/bag/data.mcap"]; !ok {
		t.Fatalf("archive missing data.mcap; entries=%v", names)
	}
	if _, ok := names[name+"/"+rosbagPidFile]; ok {
		t.Fatalf("pidfile must be excluded from archive")
	}
}

func TestRosbagDownload_RejectsTraversal(t *testing.T) {
	docker := rosbagWithContainer()
	r, _ := setupRosbagRouter(t, docker)
	w := httptest.NewRecorder()
	// gin will not route a raw "..", so exercise the validator with an invalid char.
	r.ServeHTTP(w, httptest.NewRequest(http.MethodGet, "/api/tools/rosbag/download/bad$name", nil))
	if w.Code != http.StatusBadRequest {
		t.Fatalf("expected 400, got %d", w.Code)
	}
}

func TestRosbagDelete_RemovesRecording(t *testing.T) {
	docker := rosbagWithContainer()
	docker.execFn = func(pkgtypes.ContainerExecSpec) (pkgtypes.ContainerExecResult, error) {
		return pkgtypes.ContainerExecResult{ExitCode: 0, Stdout: ""}, nil // none active
	}
	r, dir := setupRosbagRouter(t, docker)
	name := "rosbag_del"
	if err := os.MkdirAll(filepath.Join(dir, name), 0o755); err != nil {
		t.Fatal(err)
	}

	w := httptest.NewRecorder()
	r.ServeHTTP(w, httptest.NewRequest(http.MethodDelete, "/api/tools/rosbag/"+name, nil))
	if w.Code != http.StatusOK {
		t.Fatalf("expected 200, got %d body=%s", w.Code, w.Body.String())
	}
	if _, err := os.Stat(filepath.Join(dir, name)); !os.IsNotExist(err) {
		t.Fatalf("recording dir was not removed")
	}
}

func TestRosbagDelete_RefusesActiveRecording(t *testing.T) {
	docker := rosbagWithContainer()
	docker.execFn = func(pkgtypes.ContainerExecSpec) (pkgtypes.ContainerExecResult, error) {
		return pkgtypes.ContainerExecResult{ExitCode: 0, Stdout: "rosbag_active\n"}, nil
	}
	r, dir := setupRosbagRouter(t, docker)
	name := "rosbag_active"
	if err := os.MkdirAll(filepath.Join(dir, name), 0o755); err != nil {
		t.Fatal(err)
	}

	w := httptest.NewRecorder()
	r.ServeHTTP(w, httptest.NewRequest(http.MethodDelete, "/api/tools/rosbag/"+name, nil))
	if w.Code != http.StatusConflict {
		t.Fatalf("expected 409, got %d", w.Code)
	}
}

func tarEntryNames(t *testing.T, body []byte) map[string]bool {
	t.Helper()
	gz, err := gzip.NewReader(strings.NewReader(string(body)))
	if err != nil {
		t.Fatal(err)
	}
	tr := tar.NewReader(gz)
	names := map[string]bool{}
	for {
		hdr, err := tr.Next()
		if err == io.EOF {
			break
		}
		if err != nil {
			t.Fatal(err)
		}
		names[hdr.Name] = true
	}
	return names
}
