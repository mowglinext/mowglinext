package api

import (
	"crypto/sha256"
	"encoding/hex"
	"encoding/json"
	"io"
	"math"
	"net/http"
	"net/url"
	"os"
	"path/filepath"
	"regexp"
	"strings"
	"sync"
	"time"

	"github.com/gin-gonic/gin"
	"github.com/mowglinext/mowglinext/pkg/msgs/geometry"
	"github.com/mowglinext/mowglinext/pkg/msgs/mowgli"
	"github.com/mowglinext/mowglinext/pkg/types"
	"github.com/mowglinext/mowglinext/pkg/updater"
)

func updateMaintenance() bool {
	path := os.Getenv("MOWGLI_UPDATE_MAINTENANCE")
	if path == "" {
		return false
	}
	if _, err := os.Stat(filepath.Dir(path)); err != nil {
		return true
	}
	_, err := os.Stat(path)
	return err == nil || !os.IsNotExist(err)
}
func updateMaintenanceMiddleware(c *gin.Context) {
	path := c.Request.URL.Path
	if c.Request.Method != "GET" && c.Request.Method != "HEAD" && c.Request.Method != "OPTIONS" && updateMaintenance() && !strings.HasPrefix(path, "/api/system/updater/") && path != "/api/mowglinext/call/emergency" && path != "/api/mowglinext/call/high_level_control" {
		c.AbortWithStatusJSON(423, ErrorResponse{Error: "Update maintenance is active"})
		return
	}
	c.Next()
}

// Mutations use a narrow same-origin JSON interface. The daemon socket never
// accepts user-selected URLs, Docker commands or filesystem paths.
func updaterOriginAllowed(r *http.Request) bool {
	origin, err := url.Parse(r.Header.Get("Origin"))
	return err == nil && origin.Host == r.Host && (origin.Scheme == "http" || origin.Scheme == "https") && strings.HasPrefix(r.Header.Get("Content-Type"), "application/json") && r.Header.Get("X-Mowgli-Update") == "1"
}

// runtime.checked_at is re-stamped by the updater every 15 s whether or not anything
// else moved. Left in the ETag it would make nearly every conditional poll miss; the
// open updater panel polls unconditionally, so the time it shows stays current.
var updaterVolatileCheckedAt = regexp.MustCompile(`"checked_at"\s*:\s*"[^"]*"`)

// updaterStateETag identifies the updater state apart from its per-check timestamp.
func updaterStateETag(body []byte) string {
	sum := sha256.Sum256(updaterVolatileCheckedAt.ReplaceAll(body, []byte(`"checked_at":""`)))
	return `"` + hex.EncodeToString(sum[:16]) + `"`
}

// updaterETagMatches implements the If-None-Match comparison (weak match, list, "*").
func updaterETagMatches(header, etag string) bool {
	for _, candidate := range strings.Split(header, ",") {
		candidate = strings.TrimPrefix(strings.TrimSpace(candidate), "W/")
		if candidate == "*" || candidate == etag {
			return true
		}
	}
	return false
}

// UpdaterRoutes proxies fixed installation operations to the local host service.
// @Summary Review or control a coordinated software update
// @Description Same-origin JSON and X-Mowgli-Update: 1 are required. See docs/UPDATES.md for the operation contract.
// @Tags updates
// @Accept json
// @Produce json
// @Param operation path string true "Operation" Enums(policy,check,plan,apply,rollback,recover,notice,agent-update)
// @Param X-Mowgli-Update header string true "Must be 1"
// @Param request body map[string]interface{} true "Operation-specific request"
// @Success 200 {object} map[string]interface{}
// @Success 202 {object} map[string]interface{}
// @Failure 403 {object} ErrorResponse
// @Failure 409 {object} ErrorResponse
// @Failure 503 {object} ErrorResponse
// @Router /system/updater/{operation} [post]
func UpdaterRoutes(r *gin.RouterGroup, ros types.IRosProvider) {
	socket := os.Getenv("MOWGLI_UPDATER_SOCKET")
	if socket == "" {
		socket = "/run/mowgli-updater/updater.sock"
	}
	client := updater.Client(socket)
	client.Timeout = 4 * time.Minute
	proxy := func(c *gin.Context) {
		op := c.Param("operation")
		if op == "" {
			op = "state"
		}
		allowed := op == "state" && c.Request.Method == "GET"
		if c.Request.Method == "POST" {
			switch op {
			case "policy", "check", "plan", "custom-plan", "apply", "rollback", "recover", "notice", "agent-update":
				allowed = true
			}
			if !updaterOriginAllowed(c.Request) {
				c.JSON(403, ErrorResponse{Error: "same-origin JSON update request required"})
				return
			}
		}
		if !allowed {
			c.Status(404)
			return
		}
		req, err := http.NewRequestWithContext(c.Request.Context(), c.Request.Method, "http://updater/v1/"+op, http.MaxBytesReader(c.Writer, c.Request.Body, 8192))
		if err != nil {
			c.JSON(400, ErrorResponse{Error: err.Error()})
			return
		}
		req.Header.Set("Content-Type", "application/json")
		response, err := client.Do(req)
		if err != nil {
			c.JSON(503, ErrorResponse{Error: "Host updater unavailable; run the installer upgrade on a supported Linux host"})
			return
		}
		defer response.Body.Close()
		c.Header("Cache-Control", "no-store")
		c.Header("Content-Type", "application/json")
		if op == "state" && c.Request.Method == "GET" && response.StatusCode == http.StatusOK {
			// The state is the whole updater ledger (every known release, plan and job
			// with its image manifests) — over a megabyte on a long-lived install — and
			// the GUI polls it every few seconds. It rarely changes, so answer a repeat
			// poll with 304 instead of resending it.
			body, err := io.ReadAll(io.LimitReader(response.Body, 4*1024*1024))
			if err != nil {
				c.JSON(503, ErrorResponse{Error: "Host updater unavailable; run the installer upgrade on a supported Linux host"})
				return
			}
			etag := updaterStateETag(body)
			c.Header("ETag", etag)
			if updaterETagMatches(c.GetHeader("If-None-Match"), etag) {
				c.Status(http.StatusNotModified)
				return
			}
			c.Status(http.StatusOK)
			_, _ = c.Writer.Write(body)
			return
		}
		c.Status(response.StatusCode)
		_, _ = io.Copy(c.Writer, io.LimitReader(response.Body, 4*1024*1024))
	}
	r.POST("/system/updater/:operation", proxy)
	registerUpdaterStatus(r, proxy)
	var mu sync.Mutex
	var status mowgli.Status
	var state mowgli.HighLevelStatus
	var statusAt, stateAt, odomAt time.Time
	var gpsAt, lidarAt time.Time
	var linear, angular float64
	var gnss receiverProbe
	ros.Subscribe("status", "updater-readiness", 0, func(data []byte) {
		var s mowgli.Status
		if json.Unmarshal(data, &s) == nil {
			mu.Lock()
			status = s
			statusAt = stampTime(s.Stamp)
			mu.Unlock()
		}
	})
	ros.Subscribe("highLevelStatus", "updater-readiness", 0, func(data []byte) {
		var s mowgli.HighLevelStatus
		if json.Unmarshal(data, &s) == nil {
			mu.Lock()
			state = s
			stateAt = time.Now()
			mu.Unlock()
		}
	})
	ros.Subscribe("wheelOdom", "updater-readiness", 0, func(data []byte) {
		var s struct {
			Header geometry.Header `json:"header"`
			Twist  struct {
				Twist geometry.Twist `json:"twist"`
			} `json:"twist"`
		}
		if json.Unmarshal(data, &s) == nil {
			mu.Lock()
			odomAt = stampTime(s.Header.Stamp)
			linear = s.Twist.Twist.Linear.X
			angular = s.Twist.Twist.Angular.Z
			mu.Unlock()
		}
	})
	for _, topic := range []string{"gps", "lidar"} {
		key := topic
		ros.Subscribe(key, "updater-readiness", 1000, func(data []byte) {
			var sample struct {
				Header geometry.Header `json:"header"`
			}
			if json.Unmarshal(data, &sample) == nil {
				mu.Lock()
				if key == "gps" {
					gpsAt = stampTime(sample.Header.Stamp)
				} else {
					lidarAt = stampTime(sample.Header.Stamp)
				}
				mu.Unlock()
			}
		})
	}
	r.GET("/system/update-readiness", func(c *gin.Context) {
		mu.Lock()
		needsReceiverProbe := !updateSampleFresh(gpsAt, time.Now())
		mu.Unlock()
		var receiverFresh bool
		var receiverReason string
		if needsReceiverProbe {
			receiverFresh, receiverReason = gnss.check(c.Request.Context(), ros)
		}
		// Evaluate the motion/firmware gate after the bounded service call, so a
		// slow receiver cannot return an earlier, now-stale safety verdict.
		mu.Lock()
		now := time.Now()
		fresh := func(t time.Time) bool { return updateSampleFresh(t, now) }
		result := updater.Readiness{Maintenance: updateMaintenance(), FirmwareProtocol: int(status.FirmwareProtocolVersion)}
		result.GPSFresh = fresh(gpsAt)
		result.LidarFresh = fresh(lidarAt)
		switch {
		case !fresh(statusAt) || !fresh(stateAt) || !fresh(odomAt):
			result.Reason = "Fresh firmware, behaviour and wheel telemetry required"
		case state.State != 1:
			result.Reason = "Mower must be idle before updating"
		case math.IsNaN(linear) || math.IsNaN(angular) || math.Abs(linear) > 0.005 || math.Abs(angular) > 0.01:
			result.Reason = "Mower must be stationary"
		case status.MowEnabled || !fresh(stampTime(status.BladeStatusStamp)) || math.IsNaN(float64(status.MowerMotorRpm)) || math.Abs(float64(status.MowerMotorRpm)) > 1:
			result.Reason = "Fresh blade-off telemetry required"
		default:
			// A protocol-first upgrade can leave the old ROS bridge incompatible.
			// This verdict proves only safe maintenance entry from live telemetry;
			// it must never authorize motion or release the maintenance gate.
			result.MaintenanceReady = status.FirmwareProtocolVersion > 0 || !firmwareProtocolApplies()
			result.Ready = result.MaintenanceReady && status.FirmwareCompatible
			if !result.Ready {
				result.Reason = "Firmware communication is incompatible"
			}
		}
		mu.Unlock()
		if !result.GPSFresh {
			result.GPSReceiverFresh, result.GPSReason = receiverFresh, receiverReason
		}
		c.JSON(200, result)
	})
}

// firmwareProtocolApplies is false on the OpenMower backend: its bridge talks
// to the stock LowLevel and xESC firmware, so it reports protocol 0, and
// firmware_compatible alone says it reaches the boards. Read from the
// container's HARDWARE_BACKEND (docker-compose.base.yml), like the bridge's.
func firmwareProtocolApplies() bool {
	return activeHardwareBackend(nil) != "openmower"
}

func updateSampleFresh(stamp, now time.Time) bool {
	return !stamp.IsZero() && now.Sub(stamp) < 3*time.Second && now.Sub(stamp) > -time.Second
}
func stampTime(s geometry.Stamp) time.Time { return time.Unix(int64(s.Sec), int64(s.Nanosec)) }

// registerUpdaterStatus serves cached host status without initiating remote checks.
// @Summary Read cached host updater status
// @Tags updates
// @Produce json
// @Success 200 {object} map[string]interface{}
// @Failure 503 {object} ErrorResponse
// @Router /system/updater/state [get]
func registerUpdaterStatus(r *gin.RouterGroup, proxy gin.HandlerFunc) {
	r.GET("/system/updater/state", proxy)
}
