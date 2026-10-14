package api

import (
	"context"
	"fmt"
	"math"
	"net/http"
	"strings"
	"time"

	"github.com/gin-gonic/gin"
	"github.com/mowglinext/mowglinext/pkg/types"
)

// ParamsRoutes registers the live ROS2 parameter endpoints. These read and
// write parameters directly on the running nodes via the foxglove bridge, so
// edits take effect immediately (no container restart). Persisting to
// mowgli_robot.yaml is a separate concern handled by the YAML settings flow.
func ParamsRoutes(r *gin.RouterGroup, rosProvider types.IRosProvider, dbProvider types.IDBProvider) {
	r.GET("/params", getParams(rosProvider))
	r.POST("/params", setParams(rosProvider, dbProvider))
}

// ParamsListResponse is the response for GET /params.
type ParamsListResponse struct {
	Parameters []types.RosParameter `json:"parameters"`
}

// SetParamsRequest is the body for POST /params.
type SetParamsRequest struct {
	Parameters []types.RosParameter `json:"parameters"`
}

func parameterWasApplied(updated []types.RosParameter, want types.RosParameter) bool {
	for _, parameter := range updated {
		if strings.TrimPrefix(parameter.Name, "/") == strings.TrimPrefix(want.Name, "/") &&
			valuesEqual(parameter.Value, want.Value) {
			return true
		}
	}
	return false
}

func getParams(rosProvider types.IRosProvider) gin.HandlerFunc {
	return func(c *gin.Context) {
		ctx, cancel := context.WithTimeout(c.Request.Context(), 12*time.Second)
		defer cancel()
		params, err := rosProvider.GetParameters(ctx, nil)
		if err != nil {
			c.JSON(http.StatusServiceUnavailable, ErrorResponse{Error: err.Error()})
			return
		}
		c.JSON(http.StatusOK, ParamsListResponse{Parameters: params})
	}
}

func setParams(rosProvider types.IRosProvider, dbProvider types.IDBProvider) gin.HandlerFunc {
	return func(c *gin.Context) {
		var req SetParamsRequest
		if err := c.BindJSON(&req); err != nil {
			c.JSON(http.StatusBadRequest, ErrorResponse{Error: "invalid parameter payload: " + err.Error()})
			return
		}
		if len(req.Parameters) == 0 {
			c.JSON(http.StatusBadRequest, ErrorResponse{Error: "no parameters provided"})
			return
		}
		// Resolve backend once per request. Generic node parameters remain usable
		// even when hardware identity is unresolved; hardware targets do not.
		backend, backendErr := resolveHardwareBackendForDB(dbProvider)
		for _, parameter := range req.Parameters {
			if err := validateHardwareParameter(backend, backendErr, parameter); err != nil {
				c.JSON(http.StatusConflict, ErrorResponse{Error: err.Error()})
				return
			}
		}
		ctx, cancel := context.WithTimeout(c.Request.Context(), 12*time.Second)
		defer cancel()
		updated, err := rosProvider.SetParameters(ctx, req.Parameters)
		if err != nil {
			c.JSON(http.StatusServiceUnavailable, ErrorResponse{Error: err.Error()})
			return
		}
		if backend == "mavros" || containsHardwareParameters(req.Parameters) {
			for _, parameter := range req.Parameters {
				if !parameterWasApplied(updated, parameter) {
					c.JSON(http.StatusServiceUnavailable, ErrorResponse{Error: "Runtime parameter update was not confirmed: " + parameter.Name})
					return
				}
			}
		}
		c.JSON(http.StatusOK, ParamsListResponse{Parameters: updated})
	}
}

// Classify the node basename as well as its canonical name: an added ROS
// namespace must not turn a bridge/odometry parameter into a generic target.
func isHardwareParameter(name string) bool {
	name = strings.TrimLeft(name, "/")
	node, _, found := strings.Cut(name, ".")
	if !found {
		return false
	}
	parts := strings.Split(node, "/")
	basename := parts[len(parts)-1]
	return basename == "hardware_bridge" || basename == "esc_wheel_odometry"
}

// Direct API compatibility exceptions backed by actual dynamic consumers.
// These are separate from the canonical YAML-key routes published as metadata.
// Other bridge settings are startup-only or unsupported and require YAML/restart.
var directHardwareParameterTypes = map[string]map[string]string{
	"mowgli": {
		"hardware_bridge.min_linear_vel":     "float64",
		"hardware_bridge.yaw_kp":             "float64",
		"hardware_bridge.yaw_ki":             "float64",
		"hardware_bridge.yaw_trim_limit_mps": "float64",
	},
	"mavros": {
		"hardware_bridge.manual_control_yaw_scale": "float64",
		"hardware_bridge.right_esc_slot":           "int64",
		"hardware_bridge.left_esc_slot":            "int64",
		"hardware_bridge.blade_esc_slot":           "int64",
	},
}

func validateHardwareParameterType(parameter types.RosParameter, expected string) error {
	validType := parameter.Type == "" || parameter.Type == expected || (expected == "float64" && parameter.Type == "double")
	validValue := false
	switch expected {
	case "bool":
		_, validValue = parameter.Value.(bool)
	case "float64", "int64":
		if value, ok := parameter.Value.(float64); ok && !math.IsNaN(value) && !math.IsInf(value, 0) {
			validValue = expected == "float64" || (math.Trunc(value) == value && value >= -9223372036854775808.0 && value < 9223372036854775808.0)
		}
	}
	if !validType || !validValue {
		return fmt.Errorf("parameter %s requires a valid %s", parameter.Name, expected)
	}
	return nil
}

func containsHardwareParameters(parameters []types.RosParameter) bool {
	for _, p := range parameters {
		if isHardwareParameter(p.Name) {
			return true
		}
	}
	return false
}
func validateHardwareParameter(backend string, backendErr error, parameter types.RosParameter) error {
	if !isHardwareParameter(parameter.Name) {
		return nil
	}
	if backendErr != nil {
		return backendErr
	}
	name := strings.TrimLeft(parameter.Name, "/")
	for _, route := range hardwareParameterRoutes[backend] {
		if name != route.Parameter {
			continue
		}
		if route.Runtime != "available" {
			return fmt.Errorf("parameter %s requires %s", name, route.Runtime)
		}
		return validateHardwareParameterType(parameter, "float64")
	}
	if expected, ok := directHardwareParameterTypes[backend][name]; ok {
		return validateHardwareParameterType(parameter, expected)
	}
	// rclcpp's standard clock parameter is generic even on a hardware node.
	// Keep this explicit; it does not authorize arbitrary undeclared settings.
	if name == "hardware_bridge.use_sim_time" || (backend == "mavros" && name == "mavros/esc_wheel_odometry.use_sim_time") {
		return validateHardwareParameterType(parameter, "bool")
	}
	return fmt.Errorf("parameter %s is not authorized for HARDWARE_BACKEND=%s", name, backend)
}
