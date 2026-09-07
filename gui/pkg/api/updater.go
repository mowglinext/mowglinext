package api

import (
	"encoding/json"
	"io"
	"math"
	"net/http"
	"net/url"
	"os"
	"path/filepath"
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
			case "policy", "check", "plan", "apply", "rollback", "recover", "notice", "agent-update":
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
		defer mu.Unlock()
		now := time.Now()
		fresh := func(t time.Time) bool { return !t.IsZero() && now.Sub(t) < 3*time.Second && now.Sub(t) > -time.Second }
		result := updater.Readiness{Maintenance: updateMaintenance(), FirmwareProtocol: int(status.FirmwareProtocolVersion)}
		result.GPSFresh = fresh(gpsAt)
		result.LidarFresh = fresh(lidarAt)
		switch {
		case !fresh(statusAt) || !fresh(stateAt) || !fresh(odomAt):
			result.Reason = "Fresh firmware, behaviour and wheel telemetry required"
		case !status.FirmwareCompatible:
			result.Reason = "Firmware communication is incompatible"
		case state.State != 1:
			result.Reason = "Mower must be idle before updating"
		case math.IsNaN(linear) || math.IsNaN(angular) || math.Abs(linear) > 0.005 || math.Abs(angular) > 0.01:
			result.Reason = "Mower must be stationary"
		case status.MowEnabled || !fresh(stampTime(status.BladeStatusStamp)) || math.IsNaN(float64(status.MowerMotorRpm)) || math.Abs(float64(status.MowerMotorRpm)) > 1:
			result.Reason = "Fresh blade-off telemetry required"
		default:
			result.Ready = true
		}
		c.JSON(200, result)
	})
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
