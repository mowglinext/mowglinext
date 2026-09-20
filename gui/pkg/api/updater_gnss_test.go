package api

import (
	"encoding/json"
	"errors"
	"fmt"
	"net/http/httptest"
	"testing"
	"time"

	"github.com/gin-gonic/gin"
	"github.com/mowglinext/mowglinext/pkg/types"
	"github.com/mowglinext/mowglinext/pkg/updater"
)

func receiverSample(t *testing.T, source, incarnation, count string, transport, parser bool) receiverSnapshot {
	t.Helper()
	// This is the pinned Universal GNSS get_snapshot wire contract. No position
	// fix is present; runtime_observations includes accepted no-fix input.
	data := fmt.Sprintf(`{"status":{"source_id":%q,"source_incarnation":%q,"fix_valid":false},"diagnostics":{"status":[
	{"name":"universal_gnss/summary","hardware_id":%q,"values":[{"key":"transport_healthy","value":%q},{"key":"parser_healthy","value":%q}]},
	{"name":"universal_gnss/parser_counters","hardware_id":%q,"values":[{"key":"runtime_observations","value":%q}]}]}}`, source, incarnation, source, fmt.Sprint(transport), fmt.Sprint(parser), source, count)
	var s receiverSnapshot
	if err := json.Unmarshal([]byte(data), &s); err != nil {
		t.Fatal(err)
	}
	return s
}

func TestReceiverProgressRejectsFrozenRepublication(t *testing.T) {
	var p receiverProgress
	now := time.Now()
	initial := receiverSample(t, "serial:/dev/gps", "boot-1", "900", true, true)
	if ok, _ := p.observe(initial, now); ok {
		t.Fatal("first cached snapshot accepted")
	}
	next := receiverSample(t, "serial:/dev/gps", "boot-1", "901", true, true)
	if ok, reason := p.observe(next, now.Add(time.Second)); !ok {
		t.Fatal(reason)
	}
	if ok, _ := p.observe(next, now.Add(2*time.Second)); !ok {
		t.Fatal("normal inter-observation gap rejected")
	}
	for i := 4; i < 8; i++ {
		if ok, _ := p.observe(next, now.Add(time.Duration(i)*time.Second)); ok {
			t.Fatal("cached republication extended freshness")
		}
	}
}

func TestReceiverProgressRequiresIdentityAndHealthyInput(t *testing.T) {
	for _, tc := range []struct {
		name, source, incarnation, count string
		transport, parser                bool
	}{
		{"missing identity", "", "boot-1", "902", true, true},
		{"missing incarnation", "serial:/dev/gps", "", "902", true, true},
		{"counter missing", "serial:/dev/gps", "boot-1", "", true, true},
		{"counter invalid", "serial:/dev/gps", "boot-1", "-1", true, true},
		{"receiver changed", "other", "boot-1", "902", true, true},
		{"receiver restarted", "serial:/dev/gps", "boot-2", "902", true, true},
		{"counter regressed", "serial:/dev/gps", "boot-1", "1", true, true},
		{"disconnected", "serial:/dev/gps", "boot-1", "902", false, true},
		{"invalid receiver data", "serial:/dev/gps", "boot-1", "902", true, false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			now := time.Now()
			p := receiverProgress{source: "serial:/dev/gps", incarnation: "boot-1", count: 901, advancedAt: now}
			if ok, reason := p.observe(receiverSample(t, tc.source, tc.incarnation, tc.count, tc.transport, tc.parser), now); ok || reason == "" {
				t.Fatalf("invalid input accepted: %v %s", ok, reason)
			}
		})
	}
	t.Run("mixed diagnostic producer", func(t *testing.T) {
		var p receiverProgress
		s := receiverSample(t, "serial:/dev/gps", "boot-1", "902", true, true)
		s.Diagnostics.Status[1].HardwareID = "another-receiver"
		if ok, _ := p.observe(s, time.Now()); ok {
			t.Fatal("foreign counter accepted")
		}
	})
}

func TestReadinessUsesReceiverProgressWithoutChangingMowerSafety(t *testing.T) {
	ros := types.NewMockRosProvider()
	count := 30
	ros.ServiceResponder = func(service string, req, res any) {
		if service != "/universal_gnss_receiver/get_snapshot" {
			t.Fatalf("unexpected service %s", service)
		}
		*res.(*receiverSnapshot) = receiverSample(t, "serial:/dev/gps", "boot-1", fmt.Sprint(count), true, true)
	}
	r := gin.New()
	UpdaterRoutes(r.Group("/api"), ros)
	read := func() updater.Readiness {
		w := httptest.NewRecorder()
		r.ServeHTTP(w, httptest.NewRequest("GET", "/api/system/update-readiness", nil))
		var result updater.Readiness
		if err := json.Unmarshal(w.Body.Bytes(), &result); err != nil {
			t.Fatal(err)
		}
		return result
	}
	if read().GPSReceiverFresh {
		t.Fatal("one snapshot was sufficient")
	}
	count++
	result := read()
	if !result.GPSReceiverFresh || result.GPSFresh || result.Ready {
		t.Fatalf("no-fix receiver or missing mower telemetry classified incorrectly: %+v", result)
	}
	ros.ServiceErr = errors.New("service disconnected")
	if result = read(); result.GPSReceiverFresh || result.GPSReason == "" {
		t.Fatal("unavailable service retained healthy state")
	}
	ros.ServiceErr = nil
	count++
	if read().GPSReceiverFresh {
		t.Fatal("service reconnection reused previous progress")
	}
}
