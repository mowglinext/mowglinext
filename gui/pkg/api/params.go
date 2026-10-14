package api

import (
	"context"
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
		if activeHardwareBackendForDB(dbProvider) == "mavros" {
			for _, parameter := range req.Parameters {
				name := strings.TrimPrefix(parameter.Name, "/")
				if strings.HasPrefix(name, "hardware_bridge.wheel_pid_") {
					c.JSON(http.StatusConflict, ErrorResponse{Error: "Mowgli wheel PID/feed-forward parameters are unavailable for HARDWARE_BACKEND=mavros"})
					return
				}
			}
		}
		ctx, cancel := context.WithTimeout(c.Request.Context(), 12*time.Second)
		defer cancel()
		updated, err := rosProvider.SetParameters(ctx, req.Parameters)
		if err != nil {
			c.JSON(http.StatusServiceUnavailable, ErrorResponse{Error: err.Error()})
			return
		}
		if activeHardwareBackendForDB(dbProvider) == "mavros" {
			for _, parameter := range req.Parameters {
				if !parameterWasApplied(updated, parameter) {
					c.JSON(http.StatusServiceUnavailable, ErrorResponse{Error: "MAVROS parameter update was not confirmed: " + parameter.Name})
					return
				}
			}
		}
		c.JSON(http.StatusOK, ParamsListResponse{Parameters: updated})
	}
}
