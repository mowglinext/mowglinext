package api

import (
	"context"
	"net/http"
	"time"

	"github.com/cedbossneo/mowglinext/pkg/types"
	"github.com/gin-gonic/gin"
)

// ParamsRoutes registers the live ROS2 parameter endpoints. These read and
// write parameters directly on the running nodes via the foxglove bridge, so
// edits take effect immediately (no container restart). Persisting to
// mowgli_robot.yaml is a separate concern handled by the YAML settings flow.
func ParamsRoutes(r *gin.RouterGroup, rosProvider types.IRosProvider) {
	r.GET("/params", getParams(rosProvider))
	r.POST("/params", setParams(rosProvider))
}

// ParamsListResponse is the response for GET /params.
type ParamsListResponse struct {
	Parameters []types.RosParameter `json:"parameters"`
}

// SetParamsRequest is the body for POST /params.
type SetParamsRequest struct {
	Parameters []types.RosParameter `json:"parameters"`
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

func setParams(rosProvider types.IRosProvider) gin.HandlerFunc {
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
		ctx, cancel := context.WithTimeout(c.Request.Context(), 12*time.Second)
		defer cancel()
		updated, err := rosProvider.SetParameters(ctx, req.Parameters)
		if err != nil {
			c.JSON(http.StatusServiceUnavailable, ErrorResponse{Error: err.Error()})
			return
		}
		c.JSON(http.StatusOK, ParamsListResponse{Parameters: updated})
	}
}
