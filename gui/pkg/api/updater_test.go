package api

import (
	"encoding/json"
	"fmt"
	"github.com/gin-gonic/gin"
	"github.com/mowglinext/mowglinext/pkg/types"
	"github.com/mowglinext/mowglinext/pkg/updater"
	"net"
	"net/http"
	"net/http/httptest"
	"os"
	"path/filepath"
	"sync"
	"testing"
	"time"
)

func TestUpdaterMutationsRequireSameOriginJSON(t *testing.T) {
	for _, test := range []struct {
		origin, content, header string
		want                    bool
	}{
		{"http://mower:4006", "application/json", "1", true},
		{"http://evil.example", "application/json", "1", false},
		{"", "application/json", "1", false},
		{"http://mower:4006", "text/plain", "1", false},
		{"http://mower:4006", "application/json", "", false},
	} {
		r := httptest.NewRequest("POST", "http://mower:4006/api/system/updater/apply", nil)
		r.Header.Set("Origin", test.origin)
		r.Header.Set("Content-Type", test.content)
		r.Header.Set("X-Mowgli-Update", test.header)
		if updaterOriginAllowed(r) != test.want {
			t.Fatalf("unexpected origin verdict: %#v", test)
		}
	}
}
func TestUpdateReadinessRequiresLiveStoppedHardware(t *testing.T) {
	ros := types.NewMockRosProvider()
	r := gin.New()
	UpdaterRoutes(r.Group("/api"), ros)
	now := time.Now()
	stamp := map[string]any{"sec": now.Unix(), "nanosec": now.Nanosecond()}
	emit := func(topic string, data any) { body, _ := json.Marshal(data); ros.Dispatch(topic, body) }
	read := func() updater.Readiness {
		w := httptest.NewRecorder()
		r.ServeHTTP(w, httptest.NewRequest("GET", "/api/system/update-readiness", nil))
		var result updater.Readiness
		if err := json.Unmarshal(w.Body.Bytes(), &result); err != nil {
			t.Fatal(err)
		}
		return result
	}
	if read().Ready {
		t.Fatal("missing telemetry accepted")
	}
	emit("gps", map[string]any{"header": map[string]any{"stamp": stamp}})
	if !read().GPSFresh {
		t.Fatal("fresh GPS observation rejected")
	}
	for i := 0; i < 2; i++ {
		emit("gps", map[string]any{"header": map[string]any{"stamp": map[string]any{"sec": 1}}})
		if read().GPSFresh {
			t.Fatal("republication refreshed an old GPS observation")
		}
	}
	emit("highLevelStatus", map[string]any{"state": 1, "state_name": "CHARGING"})
	emit("wheelOdom", map[string]any{"header": map[string]any{"stamp": stamp}, "twist": map[string]any{"twist": map[string]any{"linear": map[string]any{"x": 0}, "angular": map[string]any{"z": 0}}}})
	status := map[string]any{"stamp": stamp, "blade_status_stamp": stamp, "firmware_protocol_version": 6, "firmware_compatible": true, "mow_enabled": false, "mower_motor_rpm": 0, "is_charging": false}
	emit("status", status)
	if !read().Ready {
		t.Fatal(read().Reason)
	}
	// A protocol-first upgrade may enter maintenance only with independently
	// fresh stopped telemetry. Full readiness must remain false until the
	// matching bridge is running; neither verdict permits mowing.
	status["firmware_compatible"] = false
	status["firmware_protocol_version"] = 7
	emit("status", status)
	if result := read(); result.Ready || !result.MaintenanceReady {
		t.Fatalf("protocol transition verdict: %+v", result)
	}
	for _, field := range []string{"mow_enabled", "mower_motor_rpm", "blade_status_stamp", "stamp", "firmware_protocol_version"} {
		previous := status[field]
		switch field {
		case "mow_enabled":
			status[field] = true
		case "mower_motor_rpm":
			status[field] = 50
		case "firmware_protocol_version":
			status[field] = 0
		default:
			status[field] = map[string]any{"sec": 1}
		}
		emit("status", status)
		if result := read(); result.Ready || result.MaintenanceReady {
			t.Fatalf("unsafe %s accepted: %+v", field, result)
		}
		status[field] = previous
	}
	emit("status", status)
	emit("highLevelStatus", map[string]any{"state": 2})
	if read().MaintenanceReady {
		t.Fatal("non-idle incompatible mower accepted")
	}
	emit("highLevelStatus", map[string]any{"state": 1})
	for _, velocity := range []float64{0.1, 0} {
		wheelStamp := stamp
		if velocity == 0 {
			wheelStamp = map[string]any{"sec": 1}
		}
		emit("wheelOdom", map[string]any{"header": map[string]any{"stamp": wheelStamp}, "twist": map[string]any{"twist": map[string]any{"linear": map[string]any{"x": velocity}}}})
		if read().MaintenanceReady {
			t.Fatal("moving or stale wheels accepted")
		}
	}
	emit("wheelOdom", map[string]any{"header": map[string]any{"stamp": stamp}})
	status["firmware_compatible"] = true
	emit("status", status)
	// No charging-current threshold: an LFP ramp at zero current still permits
	// an explicitly idle maintenance operation.
	status["mower_motor_rpm"] = 25
	emit("status", status)
	if read().Ready {
		t.Fatal("spinning blade accepted")
	}
	status["mower_motor_rpm"] = 0
	status["stamp"] = map[string]any{"sec": 1}
	emit("status", status)
	if read().Ready {
		t.Fatal("stale firmware accepted")
	}
	path := filepath.Join(t.TempDir(), "maintenance")
	t.Setenv("MOWGLI_UPDATE_MAINTENANCE", path)
	if updateMaintenance() {
		t.Fatal("absent marker active")
	}
	_ = os.WriteFile(path, []byte("pending"), 0644)
	if !read().Maintenance {
		t.Fatal("maintenance marker not reported")
	}
}

// fakeUpdater serves a minimal /v1/state on a unix socket, like the host updater.
type fakeUpdater struct {
	mu        sync.Mutex
	release   string
	checkedAt string
	status    int
}

func (f *fakeUpdater) set(release, checkedAt string) {
	f.mu.Lock()
	defer f.mu.Unlock()
	f.release, f.checkedAt = release, checkedAt
}

func startFakeUpdater(t *testing.T) *fakeUpdater {
	t.Helper()
	f := &fakeUpdater{release: "r1", checkedAt: "2026-10-07T10:00:00Z", status: 200}
	socket := filepath.Join(t.TempDir(), "u.sock")
	listener, err := net.Listen("unix", socket)
	if err != nil {
		t.Skipf("unix sockets unavailable: %v", err)
	}
	srv := &http.Server{Handler: http.HandlerFunc(func(w http.ResponseWriter, _ *http.Request) {
		f.mu.Lock()
		defer f.mu.Unlock()
		w.Header().Set("Content-Type", "application/json")
		w.WriteHeader(f.status)
		_, _ = fmt.Fprintf(w, `{"api":1,"agent":{},"state":{"releases":[{"id":%q}]},"runtime":{"health":"ok","checked_at":%q}}`, f.release, f.checkedAt)
	})}
	go func() { _ = srv.Serve(listener) }()
	t.Cleanup(func() { _ = srv.Close() })
	t.Setenv("MOWGLI_UPDATER_SOCKET", socket)
	return f
}

func TestUpdaterStateIsRevalidatedWithETag(t *testing.T) {
	fake := startFakeUpdater(t)
	r := gin.New()
	UpdaterRoutes(r.Group("/api"), types.NewMockRosProvider())
	get := func(inm string) *httptest.ResponseRecorder {
		req := httptest.NewRequest("GET", "/api/system/updater/state", nil)
		if inm != "" {
			req.Header.Set("If-None-Match", inm)
		}
		w := httptest.NewRecorder()
		r.ServeHTTP(w, req)
		return w
	}

	first := get("")
	etag := first.Header().Get("ETag")
	if first.Code != 200 || etag == "" || first.Body.Len() == 0 {
		t.Fatalf("first poll: code=%d etag=%q body=%d bytes", first.Code, etag, first.Body.Len())
	}

	same := get(etag)
	if same.Code != 304 || same.Body.Len() != 0 || same.Header().Get("ETag") != etag {
		t.Fatalf("repeat poll should be 304 with no body: code=%d body=%d etag=%q", same.Code, same.Body.Len(), same.Header().Get("ETag"))
	}

	// The updater re-stamps runtime.checked_at every 15 s; that alone is not a change.
	fake.set("r1", "2026-10-07T10:00:15Z")
	if w := get(etag); w.Code != 304 {
		t.Fatalf("a new checked_at alone must not defeat the ETag: code=%d", w.Code)
	}

	// A real change (a new release) is sent in full, with a new ETag.
	fake.set("r2", "2026-10-07T10:00:30Z")
	changed := get(etag)
	if changed.Code != 200 || changed.Header().Get("ETag") == etag || changed.Body.Len() == 0 {
		t.Fatalf("a changed state must be sent: code=%d etag=%q", changed.Code, changed.Header().Get("ETag"))
	}

	// An unconditional poll (the open updater panel) always gets the fresh timestamp.
	fake.set("r2", "2026-10-07T10:01:00Z")
	if w := get(""); w.Code != 200 || !json.Valid(w.Body.Bytes()) {
		t.Fatalf("unconditional poll must return the state: code=%d", w.Code)
	}
}

func TestUpdaterStateErrorsAreNotCachedOrRevalidated(t *testing.T) {
	fake := startFakeUpdater(t)
	fake.status = 409
	r := gin.New()
	UpdaterRoutes(r.Group("/api"), types.NewMockRosProvider())
	req := httptest.NewRequest("GET", "/api/system/updater/state", nil)
	req.Header.Set("If-None-Match", "*")
	w := httptest.NewRecorder()
	r.ServeHTTP(w, req)
	if w.Code != 409 || w.Header().Get("ETag") != "" {
		t.Fatalf("an updater error must pass through untouched: code=%d etag=%q", w.Code, w.Header().Get("ETag"))
	}
}

func TestUpdaterETagMatching(t *testing.T) {
	for _, test := range []struct {
		header string
		want   bool
	}{
		{`"abc"`, true},
		{`W/"abc"`, true},
		{`"zzz", "abc"`, true},
		{`*`, true},
		{`"abd"`, false},
		{``, false},
	} {
		if got := updaterETagMatches(test.header, `"abc"`); got != test.want {
			t.Fatalf("If-None-Match %q: got %v, want %v", test.header, got, test.want)
		}
	}
}
