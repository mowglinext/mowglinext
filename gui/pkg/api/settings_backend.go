package api

import (
	"log"
	"os"
	"strings"

	"github.com/gin-gonic/gin"
	"github.com/mowglinext/mowglinext/pkg/types"
)

const defaultHardwareBackend = "mowgli"

var supportedHardwareBackends = []string{"mowgli", "mavros"}

// HardwareParameterRoute describes how one canonical mowgli_robot.yaml key is
// addressed through the Foxglove ROS parameter API for the active backend.
// Persistence always uses the canonical key; only the live ROS destination is
// backend-specific.
type HardwareParameterRoute struct {
	Parameter string `json:"parameter"`
	Runtime   string `json:"runtime"`
}

var hardwareParameterRoutes = map[string]map[string]HardwareParameterRoute{
	"mowgli": {
		"ticks_per_meter":          {Parameter: "hardware_bridge.ticks_per_meter", Runtime: "available"},
		"wheel_pid_kp":             {Parameter: "hardware_bridge.wheel_pid_kp", Runtime: "available"},
		"wheel_pid_ki":             {Parameter: "hardware_bridge.wheel_pid_ki", Runtime: "available"},
		"wheel_pid_kd":             {Parameter: "hardware_bridge.wheel_pid_kd", Runtime: "available"},
		"wheel_pid_integral_limit": {Parameter: "hardware_bridge.wheel_pid_integral_limit", Runtime: "available"},
		"wheel_pid_pwm_per_mps":    {Parameter: "hardware_bridge.wheel_pid_pwm_per_mps", Runtime: "available"},
	},
	"mavros": {
		// Contract baseline: Pepeuch/mowglimavros feat/esc-odometry at
		// 7282fd473286b33d937baac164a3fb5974b45428. The plugin owns the
		// /mavros/esc_wheel_odometry parameter service. The new image is not
		// built/deployed yet, so these routes are intentional but runtime-pending.
		"ticks_per_meter": {Parameter: "mavros/esc_wheel_odometry.ticks_per_meter", Runtime: "pending_image"},
		"wheel_track":     {Parameter: "mavros/esc_wheel_odometry.track_width_m", Runtime: "pending_image"},
	},
}

func normalizeHardwareBackend(value string) string {
	name := strings.ToLower(strings.TrimSpace(value))
	for _, known := range supportedHardwareBackends {
		if name == known {
			return name
		}
	}
	return defaultHardwareBackend
}

func loadRuntimeEnv(dbProvider types.IDBProvider) map[string]string {
	path, err := dbProvider.Get("system.mower.runtimeEnvFile")
	if err != nil || len(path) == 0 {
		return map[string]string{}
	}
	env, err := loadGNSSRuntimeEnv(string(path))
	if err != nil {
		log.Printf("settings: %v; hardware backend falls back to %q", err, defaultHardwareBackend)
		return map[string]string{}
	}
	return env
}

func activeHardwareBackend(runtimeEnv map[string]string) string {
	if value := strings.TrimSpace(runtimeEnv["HARDWARE_BACKEND"]); value != "" {
		return normalizeHardwareBackend(value)
	}
	return normalizeHardwareBackend(os.Getenv("HARDWARE_BACKEND"))
}

func activeHardwareBackendForDB(dbProvider types.IDBProvider) string {
	return activeHardwareBackend(loadRuntimeEnv(dbProvider))
}

type HardwareBackendResponse struct {
	Backend         string                            `json:"backend"`
	Supported       []string                          `json:"supported"`
	ParameterRoutes map[string]HardwareParameterRoute `json:"parameter_routes"`
	RuntimeRouting  string                            `json:"runtime_routing"`
}

func GetSettingsHardwareBackend(r *gin.RouterGroup, dbProvider types.IDBProvider) gin.IRoutes {
	return r.GET("/settings/hardware-backend", func(c *gin.Context) {
		backend := activeHardwareBackendForDB(dbProvider)
		runtimeRouting := "available"
		if backend == "mavros" {
			runtimeRouting = "pending_image"
		}
		c.JSON(200, HardwareBackendResponse{
			Backend:         backend,
			Supported:       append([]string(nil), supportedHardwareBackends...),
			ParameterRoutes: hardwareParameterRoutes[backend],
			RuntimeRouting:  runtimeRouting,
		})
	})
}

func requireMowgliHardwareBackend(dbProvider types.IDBProvider) gin.HandlerFunc {
	return func(c *gin.Context) {
		if backend := activeHardwareBackendForDB(dbProvider); backend != "mowgli" {
			c.AbortWithStatusJSON(409, ErrorResponse{
				Error: "Mowgli drive PID/feed-forward tools are unavailable for HARDWARE_BACKEND=" + backend,
			})
			return
		}
		c.Next()
	}
}
