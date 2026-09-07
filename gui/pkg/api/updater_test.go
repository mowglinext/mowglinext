package api

import (
	"encoding/json"
	"github.com/gin-gonic/gin"
	"github.com/mowglinext/mowglinext/pkg/types"
	"github.com/mowglinext/mowglinext/pkg/updater"
	"net/http/httptest"
	"os"
	"path/filepath"
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
	emit("highLevelStatus", map[string]any{"state": 1, "state_name": "CHARGING"})
	emit("wheelOdom", map[string]any{"header": map[string]any{"stamp": stamp}, "twist": map[string]any{"twist": map[string]any{"linear": map[string]any{"x": 0}, "angular": map[string]any{"z": 0}}}})
	status := map[string]any{"stamp": stamp, "blade_status_stamp": stamp, "firmware_protocol_version": 6, "firmware_compatible": true, "mow_enabled": false, "mower_motor_rpm": 0, "is_charging": false}
	emit("status", status)
	if !read().Ready {
		t.Fatal(read().Reason)
	}
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
