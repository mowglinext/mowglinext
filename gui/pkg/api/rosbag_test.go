package api

import (
	"archive/tar"
	"bytes"
	"compress/gzip"
	"context"
	"encoding/json"
	"errors"
	"io"
	"net/http"
	"net/http/httptest"
	"os"
	"path/filepath"
	"strings"
	"testing"

	dockertypes "github.com/docker/docker/api/types"
	"github.com/gin-gonic/gin"
	pkgtypes "github.com/mowglinext/mowglinext/pkg/types"
	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
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
	// A live recorder always has its pid file (buildRosbagStartCommand writes
	// and verifies it); the status probe only execs when one exists.
	if err := os.WriteFile(filepath.Join(dir, "rosbag_live", rosbagPidFile), []byte("4242\n"), 0o644); err != nil {
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
	r, dir := setupRosbagRouter(t, docker)
	if err := os.MkdirAll(filepath.Join(dir, "rosbag_busy"), 0o755); err != nil {
		t.Fatal(err)
	}
	// A live recorder always has its pid file (buildRosbagStartCommand writes
	// and verifies it); the status probe only execs when one exists.
	if err := os.WriteFile(filepath.Join(dir, "rosbag_busy", rosbagPidFile), []byte("4242\n"), 0o644); err != nil {
		t.Fatal(err)
	}

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
	// Key the fake on what is being executed rather than on call order: the
	// liveness probe is skipped entirely when no pid file exists, so the start
	// command may well be the first exec this handler issues.
	docker.execFn = func(spec pkgtypes.ContainerExecSpec) (pkgtypes.ContainerExecResult, error) {
		if strings.Contains(strings.Join(spec.Cmd, " "), "ros2 bag record") {
			return pkgtypes.ContainerExecResult{ExitCode: 1, Stdout: "recorder exited immediately\n"}, nil
		}
		return pkgtypes.ContainerExecResult{ExitCode: 0, Stdout: ""}, nil
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
	// Invalid characters must be rejected in addition to dot-segment names.
	r.ServeHTTP(w, httptest.NewRequest(http.MethodGet, "/api/tools/rosbag/download/bad$name", nil))
	if w.Code != http.StatusBadRequest {
		t.Fatalf("expected 400, got %d", w.Code)
	}
}

func TestRosbagPath_RejectsInvalidNames(t *testing.T) {
	for _, name := range []string{
		".", "..", "%2e", "%2e%2e", "%2E%2E", ".%2e", "%2e.",
		"%252e%252e", "bad$name", "bad%20name", "bad%00name", "%5c..", "bad%25name",
		"..%2f..", "%2fetc", "bad%5cname",
	} {
		for _, method := range []string{http.MethodGet, http.MethodDelete} {
			t.Run(method+"/"+name, func(t *testing.T) {
				parent := t.TempDir()
				root := filepath.Join(parent, "rosbags")
				require.NoError(t, os.MkdirAll(filepath.Join(root, "legitimate"), 0o755))
				sentinel := filepath.Join(parent, "sentinel.yaml")
				require.NoError(t, os.WriteFile(sentinel, []byte("parent must stay unchanged"), 0o600))
				t.Setenv("ROSBAG_DIR", root)
				gin.SetMode(gin.TestMode)
				r := gin.New()
				RosbagRoutes(r.Group("/api"), rosbagWithContainer())
				path := "/api/tools/rosbag/"
				if method == http.MethodGet {
					path += "download/"
				}
				w := httptest.NewRecorder()
				r.ServeHTTP(w, httptest.NewRequest(method, path+name, nil))
				// Decoded separators may be rejected by Gin before the handler.
				require.Contains(t, []int{http.StatusBadRequest, http.StatusNotFound}, w.Code, w.Body.String())
				if w.Code == http.StatusBadRequest {
					var response ErrorResponse
					require.NoError(t, json.Unmarshal(w.Body.Bytes(), &response))
					assert.Equal(t, "invalid recording name", response.Error)
				}
				assert.NotContains(t, w.Header().Get("Content-Type"), "gzip")
				data, err := os.ReadFile(sentinel)
				require.NoError(t, err)
				assert.Equal(t, "parent must stay unchanged", string(data))
				info, err := os.Stat(sentinel)
				require.NoError(t, err)
				assert.Equal(t, os.FileMode(0o600), info.Mode().Perm())
				_, err = os.Stat(filepath.Join(root, "legitimate"))
				assert.NoError(t, err)
			})
		}
	}
}

func TestRosbagPath_RejectsSymlinks(t *testing.T) {
	for _, link := range []string{"recording-absolute", "recording-relative", "nested-file", "nested-relative-file", "nested-directory", "dangling", "internal"} {
		for _, method := range []string{http.MethodGet, http.MethodDelete} {
			t.Run(method+"/"+link, func(t *testing.T) {
				parent := t.TempDir()
				root := filepath.Join(parent, "rosbags")
				outside := filepath.Join(parent, "outside")
				require.NoError(t, os.MkdirAll(outside, 0o755))
				sentinel := filepath.Join(outside, "sentinel.mcap")
				require.NoError(t, os.WriteFile(sentinel, []byte("external sentinel"), 0o600))
				capture := filepath.Join(root, "capture")
				require.NoError(t, os.MkdirAll(capture, 0o755))
				switch link {
				case "recording-absolute":
					require.NoError(t, os.Remove(capture))
					require.NoError(t, os.Symlink(outside, capture))
				case "recording-relative":
					require.NoError(t, os.Remove(capture))
					require.NoError(t, os.Symlink("../outside", capture))
				case "nested-file":
					require.NoError(t, os.Symlink(sentinel, filepath.Join(capture, "data.mcap")))
				case "nested-relative-file":
					require.NoError(t, os.Symlink("../../outside/sentinel.mcap", filepath.Join(capture, "data.mcap")))
				case "nested-directory":
					require.NoError(t, os.Symlink(outside, filepath.Join(capture, "bag")))
				case "dangling":
					require.NoError(t, os.Symlink(filepath.Join(outside, "missing"), filepath.Join(capture, "data.mcap")))
				case "internal":
					require.NoError(t, os.WriteFile(filepath.Join(capture, "original.mcap"), []byte("payload"), 0o600))
					require.NoError(t, os.Symlink("original.mcap", filepath.Join(capture, "data.mcap")))
				}
				t.Setenv("ROSBAG_DIR", root)
				r := gin.New()
				RosbagRoutes(r.Group("/api"), rosbagWithContainer())
				path := "/api/tools/rosbag/"
				if method == http.MethodGet {
					path += "download/"
				}
				w := httptest.NewRecorder()
				r.ServeHTTP(w, httptest.NewRequest(method, path+"capture", nil))
				require.Equal(t, http.StatusBadRequest, w.Code, w.Body.String())
				var response ErrorResponse
				require.NoError(t, json.Unmarshal(w.Body.Bytes(), &response))
				assert.Equal(t, "invalid recording path", response.Error)
				assert.NotContains(t, w.Header().Get("Content-Type"), "gzip")
				data, err := os.ReadFile(sentinel)
				require.NoError(t, err)
				assert.Equal(t, "external sentinel", string(data))
				_, err = os.Lstat(capture)
				assert.NoError(t, err, "rejected capture must not be deleted")
			})
		}
	}
}

func TestRosbagPath_PreservesExistingNames(t *testing.T) {
	for _, name := range []string{"rosbag_20261021T120000Z", "legacy-capture.v1_2", ".legacy", "capture..old"} {
		t.Run(name, func(t *testing.T) {
			r, root := setupRosbagRouter(t, rosbagWithContainer())
			require.NoError(t, os.MkdirAll(filepath.Join(root, name, "bag"), 0o755))
			require.NoError(t, os.WriteFile(filepath.Join(root, name, "bag", "data.mcap"), []byte("payload"), 0o600))
			w := httptest.NewRecorder()
			r.ServeHTTP(w, httptest.NewRequest(http.MethodGet, "/api/tools/rosbag/download/"+name, nil))
			require.Equal(t, http.StatusOK, w.Code, w.Body.String())
			assert.True(t, tarEntryNames(t, w.Body.Bytes())[name+"/bag/data.mcap"])
			w = httptest.NewRecorder()
			r.ServeHTTP(w, httptest.NewRequest(http.MethodDelete, "/api/tools/rosbag/"+name, nil))
			require.Equal(t, http.StatusOK, w.Code, w.Body.String())
			_, err := os.Stat(filepath.Join(root, name))
			assert.True(t, os.IsNotExist(err))
		})
	}
}

type rosbagSwapWriter struct {
	io.Writer
	beforeFirstWrite func()
}

func (w *rosbagSwapWriter) Write(p []byte) (int, error) {
	if swap := w.beforeFirstWrite; swap != nil {
		w.beforeFirstWrite = nil
		swap()
	}
	return w.Writer.Write(p)
}

func TestRosbagDownload_ConfinesReadsDuringPathSwap(t *testing.T) {
	parent := t.TempDir()
	capture := filepath.Join(parent, "capture")
	require.NoError(t, os.Mkdir(capture, 0o755))
	require.NoError(t, os.WriteFile(filepath.Join(capture, "a.mcap"), []byte("recorded data"), 0o600))
	replaced := filepath.Join(capture, "b.mcap")
	require.NoError(t, os.WriteFile(replaced, []byte("original b"), 0o600))
	sentinel := filepath.Join(parent, "sentinel.mcap")
	require.NoError(t, os.WriteFile(sentinel, []byte("external sentinel"), 0o600))
	root, err := os.OpenRoot(capture)
	require.NoError(t, err)
	defer root.Close()
	var body bytes.Buffer
	w := &rosbagSwapWriter{Writer: &body, beforeFirstWrite: func() {
		// Both entries were already enumerated. Replace b after the first
		// archive header, before the second entry is opened: no timing sleeps.
		require.NoError(t, os.Remove(replaced))
		require.NoError(t, os.Symlink(sentinel, replaced))
	}}
	require.Error(t, writeRecordingTarGz(w, root, "capture"))
	gz, err := gzip.NewReader(&body)
	require.NoError(t, err)
	defer gz.Close()
	plain, err := io.ReadAll(gz)
	require.NoError(t, err)
	assert.Contains(t, string(plain), "recorded data")
	assert.NotContains(t, string(plain), "external sentinel")
	data, err := os.ReadFile(sentinel)
	require.NoError(t, err)
	assert.Equal(t, "external sentinel", string(data))
}

func TestRosbagDelete_RequiresKnownInactiveRecorder(t *testing.T) {
	for _, tc := range []struct {
		name        string
		result      pkgtypes.ContainerExecResult
		probeError  error
		noContainer bool
		wantCode    int
		wantError   string
	}{
		{name: "active", result: pkgtypes.ContainerExecResult{Stdout: "rosbag_live\n"}, wantCode: http.StatusConflict, wantError: "recording is still in progress; stop it first"},
		{name: "inactive", wantCode: http.StatusOK},
		{name: "probe-error", probeError: errors.New("docker unavailable"), wantCode: http.StatusServiceUnavailable, wantError: "cannot verify recording state: docker unavailable"},
		{name: "probe-nonzero-exit", result: pkgtypes.ContainerExecResult{ExitCode: 7, Stderr: "probe failed"}, wantCode: http.StatusServiceUnavailable, wantError: "cannot verify recording state: failed to probe recordings: probe failed"},
		{name: "unknown-container-state", noContainer: true, wantCode: http.StatusServiceUnavailable, wantError: "cannot verify recording state: mowgli-ros2 container not found"},
	} {
		t.Run(tc.name, func(t *testing.T) {
			docker := rosbagWithContainer()
			if tc.noContainer {
				docker.containers = nil
			}
			docker.execFn = func(pkgtypes.ContainerExecSpec) (pkgtypes.ContainerExecResult, error) {
				return tc.result, tc.probeError
			}
			r, root := setupRosbagRouter(t, docker)
			capture := filepath.Join(root, "rosbag_live")
			require.NoError(t, os.MkdirAll(capture, 0o755))
			require.NoError(t, os.WriteFile(filepath.Join(capture, rosbagPidFile), []byte("4242\n"), 0o600))
			file := filepath.Join(capture, "data.mcap")
			require.NoError(t, os.WriteFile(file, []byte("capture data"), 0o600))
			w := httptest.NewRecorder()
			r.ServeHTTP(w, httptest.NewRequest(http.MethodDelete, "/api/tools/rosbag/rosbag_live", nil))
			require.Equal(t, tc.wantCode, w.Code, w.Body.String())
			if tc.wantCode == http.StatusOK {
				_, err := os.Stat(capture)
				assert.True(t, os.IsNotExist(err))
			} else {
				var response ErrorResponse
				require.NoError(t, json.Unmarshal(w.Body.Bytes(), &response))
				assert.Equal(t, tc.wantError, response.Error)
				data, err := os.ReadFile(file)
				require.NoError(t, err)
				assert.Equal(t, "capture data", string(data))
				pid, err := os.ReadFile(filepath.Join(capture, rosbagPidFile))
				require.NoError(t, err)
				assert.Equal(t, "4242\n", string(pid))
			}
		})
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
	// A live recording always carries its pid file: buildRosbagStartCommand
	// writes `echo $! > .rosbag.pid` and refuses to report success without it.
	// The status probe now only execs into the ros2 container when a pid file
	// exists, so the fixture must reflect that invariant.
	if err := os.WriteFile(filepath.Join(dir, name, rosbagPidFile), []byte("4242\n"), 0o644); err != nil {
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

// The status endpoint is polled every 4 s per tab. On an idle robot there is
// no pid file anywhere under the recordings root, and the probe must not cost
// a docker exec just to discover that.
func TestRosbagStatus_SkipsExecWhenNoPidFileExists(t *testing.T) {
	execCalls := 0
	docker := &rosbagFakeDocker{
		containers: []dockertypes.Container{{ID: "abc123", Names: []string{"/mowgli-ros2"}}},
		execFn: func(spec pkgtypes.ContainerExecSpec) (pkgtypes.ContainerExecResult, error) {
			execCalls++
			return pkgtypes.ContainerExecResult{}, nil
		},
	}
	r, dir := setupRosbagRouter(t, docker)
	require.NoError(t, os.MkdirAll(filepath.Join(dir, "finished_run", rosbagBagSubdir), 0o755))
	w := httptest.NewRecorder()
	r.ServeHTTP(w, httptest.NewRequest(http.MethodGet, "/api/tools/rosbag/status", nil))
	require.Equal(t, http.StatusOK, w.Code)
	assert.Equal(t, 0, execCalls, "no pid file anywhere: the probe must not exec into the ros2 container")
}

// With a pid file present the recorder may still be alive, and only the ros2
// container's pid namespace can say — the exec must still happen.
func TestRosbagStatus_ExecsWhenAPidFileExists(t *testing.T) {
	execCalls := 0
	docker := &rosbagFakeDocker{
		containers: []dockertypes.Container{{ID: "abc123", Names: []string{"/mowgli-ros2"}}},
		execFn: func(spec pkgtypes.ContainerExecSpec) (pkgtypes.ContainerExecResult, error) {
			execCalls++
			return pkgtypes.ContainerExecResult{Stdout: "live_run\n"}, nil
		},
	}
	r, dir := setupRosbagRouter(t, docker)
	require.NoError(t, os.MkdirAll(filepath.Join(dir, "live_run", rosbagBagSubdir), 0o755))
	require.NoError(t, os.WriteFile(filepath.Join(dir, "live_run", rosbagPidFile), []byte("4242\n"), 0o644))
	w := httptest.NewRecorder()
	r.ServeHTTP(w, httptest.NewRequest(http.MethodGet, "/api/tools/rosbag/status", nil))
	require.Equal(t, http.StatusOK, w.Code)
	assert.Equal(t, 1, execCalls, "a pid file exists: liveness must still be probed in the ros2 container")
	assert.Contains(t, w.Body.String(), `"active":true`)
}
