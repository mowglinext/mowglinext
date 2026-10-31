package api

import (
	"context"
	"encoding/json"
	"net/http"
	"net/http/httptest"
	"os"
	"os/exec"
	"path/filepath"
	"runtime"
	"strings"
	"testing"
	"time"

	"github.com/gin-gonic/gin"
	"github.com/mowglinext/mowglinext/pkg/foxglove"
	"github.com/mowglinext/mowglinext/pkg/types"
	"github.com/mowglinext/mowglinext/pkg/updater"
	"gopkg.in/yaml.v3"
)

type healthROSProvider struct {
	*types.MockRosProvider
	health foxglove.TransportHealth
	reads  int
}

func (p *healthROSProvider) TransportHealth() foxglove.TransportHealth {
	p.reads++
	return p.health
}

func TestGUILivenessDoesNotDependOnROSTransport(t *testing.T) {
	marker := filepath.Join(t.TempDir(), "maintenance")
	if err := os.WriteFile(marker, nil, 0600); err != nil {
		t.Fatal(err)
	}
	t.Setenv("MOWGLI_UPDATE_MAINTENANCE", marker)
	ros := &healthROSProvider{MockRosProvider: types.NewMockRosProvider()}
	r := gin.New()
	r.Use(updateMaintenanceMiddleware)
	HealthRoutes(r.Group("/api"), ros)
	for _, test := range []struct {
		health foxglove.TransportHealth
		status int
	}{
		{foxglove.TransportHealth{}, http.StatusServiceUnavailable},
		{foxglove.TransportHealth{Connected: true}, http.StatusServiceUnavailable},
		{foxglove.TransportHealth{Connected: true, HeartbeatOK: true}, http.StatusOK},
		{foxglove.TransportHealth{HeartbeatOK: true}, http.StatusServiceUnavailable},
	} {
		ros.health = test.health
		before := ros.reads
		w := httptest.NewRecorder()
		r.ServeHTTP(w, httptest.NewRequest(http.MethodGet, "/api/health/live", nil))
		if w.Code != http.StatusOK || ros.reads != before {
			t.Fatalf("liveness consulted ROS or failed: status=%d reads=%d", w.Code, ros.reads-before)
		}
		if w.Header().Get("Cache-Control") != "no-store" {
			t.Fatal("liveness response may be cached")
		}
		w = httptest.NewRecorder()
		r.ServeHTTP(w, httptest.NewRequest(http.MethodGet, "/api/health/ros", nil))
		var health ROSHealth
		if err := json.Unmarshal(w.Body.Bytes(), &health); err != nil {
			t.Fatal(err)
		}
		if w.Code != test.status || health.Ready != (test.status == http.StatusOK) || health.Transport == nil || health.Transport.Connected != test.health.Connected || health.Transport.HeartbeatOK != test.health.HeartbeatOK {
			t.Fatalf("incorrect ROS readiness: status=%d body=%s", w.Code, w.Body.String())
		}
	}
}

func TestDiagnosticsAndUpdaterSeparateTransportFromMowerReadiness(t *testing.T) {
	ros := &healthROSProvider{MockRosProvider: types.NewMockRosProvider(), health: foxglove.TransportHealth{Connected: true, HeartbeatOK: true}}
	r := gin.New()
	UpdaterRoutes(r.Group("/api"), ros, types.NewMockDBProvider())
	r.GET("/snapshot", getDiagnosticsSnapshot(&versionDocker{}, ros, types.NewMockDBProvider()))
	for _, connected := range []bool{true, false} {
		ros.health.Connected = connected
		ros.health.HeartbeatOK = connected
		w := httptest.NewRecorder()
		r.ServeHTTP(w, httptest.NewRequest(http.MethodGet, "/snapshot", nil))
		var snapshot DiagnosticsSnapshot
		if err := json.Unmarshal(w.Body.Bytes(), &snapshot); err != nil {
			t.Fatal(err)
		}
		if snapshot.ROS.Ready != connected {
			t.Fatalf("diagnostics ROS transport: %+v", snapshot.ROS)
		}
		w = httptest.NewRecorder()
		r.ServeHTTP(w, httptest.NewRequest(http.MethodGet, "/api/system/update-readiness", nil))
		var readiness struct {
			updater.Readiness
			ROS ROSHealth `json:"ros"`
		}
		if err := json.Unmarshal(w.Body.Bytes(), &readiness); err != nil {
			t.Fatal(err)
		}
		if readiness.ROS.Ready != connected || readiness.Ready || readiness.MaintenanceReady {
			t.Fatalf("transport readiness must not grant update/mower readiness: %+v", readiness)
		}
	}
	if health := readROSHealth(types.NewMockRosProvider()); health.Ready || health.Transport != nil {
		t.Fatalf("unsupported transport must be unknown: %+v", health)
	}
}

func TestComposeGUIHealthProbeDetectsStalledHTTPHandler(t *testing.T) {
	if _, err := exec.LookPath("curl"); err != nil {
		t.Skip("curl is required to execute the image's healthcheck")
	}
	_, source, _, _ := runtime.Caller(0)
	data, err := os.ReadFile(filepath.Join(filepath.Dir(source), "../../../install/compose/docker-compose.gui.yml"))
	if err != nil {
		t.Fatal(err)
	}
	var compose struct {
		Services map[string]struct {
			Healthcheck struct {
				Test    []string `yaml:"test"`
				Timeout string   `yaml:"timeout"`
			} `yaml:"healthcheck"`
		} `yaml:"services"`
	}
	if err = yaml.Unmarshal(data, &compose); err != nil {
		t.Fatal(err)
	}
	probe := compose.Services["gui"].Healthcheck
	if len(probe.Test) < 3 || probe.Test[0] != "CMD" || probe.Test[1] != "curl" || probe.Timeout != "3s" {
		t.Fatalf("GUI needs a bounded HTTP healthcheck: %+v", probe)
	}
	router := gin.New()
	HealthRoutes(router.Group("/api"), nil)
	for _, stalled := range []bool{false, true} {
		stop := make(chan struct{})
		server := httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
			if r.URL.Path != "/api/health/live" {
				t.Errorf("wrong health route: %s", r.URL.Path)
				w.WriteHeader(http.StatusNotFound)
				return
			}
			if stalled {
				<-stop
				return
			}
			router.ServeHTTP(w, r)
		}))
		args := append([]string{}, probe.Test[2:]...)
		for i := range args {
			args[i] = strings.Replace(args[i], "http://127.0.0.1:4006", server.URL, 1)
		}
		started := time.Now()
		ctx, cancel := context.WithTimeout(context.Background(), 5*time.Second)
		output, err := exec.CommandContext(ctx, "curl", args...).CombinedOutput()
		cancel()
		elapsed := time.Since(started)
		close(stop)
		server.Close()
		if (err != nil) != stalled || elapsed > 4*time.Second {
			t.Fatalf("HTTP health probe stalled=%v elapsed=%s error=%v output=%s", stalled, elapsed, err, output)
		}
	}
}
