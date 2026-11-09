package api

import (
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
	"time"

	"github.com/gin-gonic/gin"
	"github.com/mowglinext/mowglinext/pkg/blackbox"
	"github.com/mowglinext/mowglinext/pkg/types"
	"github.com/stretchr/testify/require"
)

type failingBlackboxDB struct{ *types.MockDBProvider }

func (failingBlackboxDB) Set(string, []byte) error { return errors.New("injected store failure") }

func TestBlackboxConfigPersistenceFailureIsAtomic(t *testing.T) {
	r, m, db := setupBlackboxRouter(t, nil)
	m.db = failingBlackboxDB{db}
	old := m.recorder.Status().Config
	updated := old
	updated.Enabled = false
	raw, _ := json.Marshal(updated)
	w := blackboxRequest(r, "PUT", "/config", string(raw))
	require.Equal(t, 500, w.Code)
	require.Equal(t, old, m.recorder.Status().Config)
	w = blackboxRequest(r, "POST", "/save", "")
	require.Equal(t, 202, w.Code)
}

func setupBlackboxRouter(t *testing.T, ros types.IRosProvider) (*gin.Engine, *blackboxManager, *types.MockDBProvider) {
	t.Helper()
	gin.SetMode(gin.TestMode)
	t.Setenv("BLACKBOX_DIR", t.TempDir())
	db := types.NewMockDBProvider()
	cfg := blackbox.DefaultConfig()
	cfg.PostSeconds = 1
	raw, _ := json.Marshal(cfg)
	require.NoError(t, db.Set(blackboxConfigKey, raw))
	r := gin.New()
	m := BlackboxRoutes(r.Group("/api"), db, ros)
	require.NotNil(t, m.recorder)
	t.Cleanup(func() {
		close(m.reconcile)
		ctx, cancel := context.WithTimeout(context.Background(), 3*time.Second)
		defer cancel()
		require.NoError(t, m.recorder.Close(ctx))
	})
	return r, m, db
}

func blackboxRequest(r *gin.Engine, method, path, body string) *httptest.ResponseRecorder {
	w := httptest.NewRecorder()
	req := httptest.NewRequest(method, "/api/tools/blackbox"+path, strings.NewReader(body))
	req.Header.Set("Content-Type", "application/json")
	r.ServeHTTP(w, req)
	return w
}

func TestBlackboxAutomaticSaveDownloadDelete(t *testing.T) {
	ros := types.NewMockRosProvider()
	r, m, _ := setupBlackboxRouter(t, ros)
	// Publishers can appear after the recorder. No UI websocket is involved.
	require.Eventually(t, func() bool {
		ros.Dispatch("wheelOdom", []byte(`{"header":{"stamp":{"sec":3}},"twist":{"twist":{"linear":{"x":0.4}}}}`))
		return m.recorder.Status().BufferedRecords > 0
	}, 2*time.Second, 10*time.Millisecond)
	ros.Dispatch("emergency", []byte(`{"active_emergency":true,"latched_emergency":false,"reason":"synthetic"}`))
	require.Eventually(t, func() bool { return m.recorder.Status().Phase == "capturing" }, time.Second, 10*time.Millisecond)
	// A manual request and repeating emergency join the same incident.
	w := blackboxRequest(r, "POST", "/save", "")
	require.Equal(t, 202, w.Code, w.Body.String())
	require.Contains(t, w.Body.String(), `"accepted":false`)
	ros.Dispatch("wheelOdom", []byte(`{"header":{"stamp":{"sec":-20}},"twist":{"twist":{"linear":{"x":0.0}}}}`))
	ros.Dispatch("emergency", []byte(`{"active_emergency":true,"latched_emergency":true}`))
	require.Eventually(t, func() bool { return m.recorder.Status().CompletedSnapshots == 1 }, 4*time.Second, 10*time.Millisecond)
	w = blackboxRequest(r, "GET", "/status", "")
	require.Equal(t, 200, w.Code)
	var status BlackboxStatusResponse
	require.NoError(t, json.Unmarshal(w.Body.Bytes(), &status))
	require.Len(t, status.Recordings, 1)
	name := status.Recordings[0].Name
	w = blackboxRequest(r, "GET", "/download/"+name, "")
	require.Equal(t, 200, w.Code, w.Body.String())
	require.Equal(t, "application/x-ndjson", w.Header().Get("Content-Type"))
	require.Contains(t, w.Header().Get("Content-Disposition"), "attachment;")
	require.Contains(t, w.Body.String(), "blackbox_metadata")
	require.Contains(t, w.Body.String(), "wheelOdom")
	require.Contains(t, w.Body.String(), `"sec":-20`)
	require.Contains(t, w.Body.String(), "emergency")
	require.Empty(t, ros.ServiceCalls)
	require.Empty(t, ros.Publishes)
	w = blackboxRequest(r, "DELETE", "/"+name, "")
	require.Equal(t, 200, w.Code)
	w = blackboxRequest(r, "GET", "/download/"+name, "")
	require.Equal(t, 404, w.Code)
}

func TestBlackboxManualConfigRestartAndValidation(t *testing.T) {
	r, m, db := setupBlackboxRouter(t, nil)
	cfg := m.recorder.Status().Config
	cfg.Enabled = false
	cfg.PreSeconds = 12
	raw, _ := json.Marshal(cfg)
	w := blackboxRequest(r, "PUT", "/config", string(raw))
	require.Equal(t, 200, w.Code, w.Body.String())
	w = blackboxRequest(r, "POST", "/save", "")
	require.Equal(t, 409, w.Code)
	persisted, err := db.Get(blackboxConfigKey)
	require.NoError(t, err)
	require.JSONEq(t, string(raw), string(persisted))
	w = blackboxRequest(r, "PUT", "/config", `{"enabled":true,"memory_bytes":999999999999}`)
	require.Equal(t, 400, w.Code)
	cfg.Enabled = true
	raw, _ = json.Marshal(cfg)
	w = blackboxRequest(r, "PUT", "/config", string(raw))
	require.Equal(t, 200, w.Code)
	m.recorder.Ingest("status", []byte(`{"blade_rpm":1234}`))
	w = blackboxRequest(r, "POST", "/save", "")
	require.Equal(t, 202, w.Code)
	w = blackboxRequest(r, "PUT", "/config", string(raw))
	require.Equal(t, 409, w.Code)
	require.Eventually(t, func() bool { return m.recorder.Status().CompletedSnapshots == 1 }, 4*time.Second, 10*time.Millisecond)
	// Re-open the same storage to demonstrate completed captures survive a restart.
	restarted, err := blackbox.New(os.Getenv("BLACKBOX_DIR"), cfg, nil)
	require.NoError(t, err)
	defer restarted.Close(context.Background())
	files, err := restarted.List()
	require.NoError(t, err)
	require.Len(t, files, 1)
	require.Equal(t, 0, restarted.Status().BufferedRecords)
}

func TestBlackboxUnsafeFilesAndUnavailableStorage(t *testing.T) {
	r, m, _ := setupBlackboxRouter(t, nil)
	for _, path := range []string{"/download/not-a-snapshot", "/not-a-snapshot"} {
		method := "GET"
		if !strings.HasPrefix(path, "/download/") {
			method = "DELETE"
		}
		w := blackboxRequest(r, method, path, "")
		require.Equal(t, 404, w.Code)
	}
	// Stored invalid configuration disables collection, rather than silently
	// overriding an operator's disabled/malformed resource setting.
	db := types.NewMockDBProvider()
	require.NoError(t, db.Set(blackboxConfigKey, []byte("broken")))
	other := gin.New()
	otherManager := BlackboxRoutes(other.Group("/api"), db, nil)
	defer otherManager.recorder.Close(context.Background())
	w := blackboxRequest(other, "GET", "/status", "")
	require.Equal(t, 200, w.Code)
	require.Contains(t, w.Body.String(), "invalid stored")
	require.False(t, otherManager.recorder.Status().Config.Enabled)
	_ = m
	file := filepath.Join(t.TempDir(), "file")
	require.NoError(t, os.WriteFile(file, []byte("x"), 0600))
	t.Setenv("BLACKBOX_DIR", file)
	failed := gin.New()
	BlackboxRoutes(failed.Group("/api"), types.NewMockDBProvider(), nil)
	w = blackboxRequest(failed, "GET", "/status", "")
	require.Equal(t, http.StatusServiceUnavailable, w.Code)
	_, _ = io.Copy(io.Discard, w.Result().Body)
}

func TestBlackboxBehaviorFailureUsesExistingNotificationClassification(t *testing.T) {
	ros := types.NewMockRosProvider()
	_, m, _ := setupBlackboxRouter(t, ros)
	require.Eventually(t, func() bool {
		ros.Dispatch("highLevelStatus", []byte(`{"state":2,"state_name":"NAV_TO_DOCK_FAILED"}`))
		return m.recorder.Status().Phase == "capturing"
	}, time.Second, 10*time.Millisecond)
	for i := 0; i < 100; i++ {
		ros.Dispatch("highLevelStatus", []byte(`{"state":2,"state_name":"NAV_TO_DOCK_FAILED"}`))
	}
	require.Eventually(t, func() bool { return m.recorder.Status().CompletedSnapshots == 1 }, 4*time.Second, 10*time.Millisecond)
	files, err := m.recorder.List()
	require.NoError(t, err)
	require.Len(t, files, 1)
	require.Contains(t, files[0].Reasons, "behavior: navFailed: NAV_TO_DOCK_FAILED")
	require.Empty(t, ros.Publishes)
	require.Empty(t, ros.ServiceCalls)
}
