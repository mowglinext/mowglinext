package api

import (
	"context"
	"fmt"
	"net/http"
	"time"

	"github.com/gin-gonic/gin"
	"github.com/mowglinext/mowglinext/pkg/msgs/mowgli"
	"github.com/mowglinext/mowglinext/pkg/types"
)

const bladeLatchTimeout = 250 * time.Millisecond
const bladeOffTimeout = 2 * time.Second

func handleBladeControl(c *gin.Context, provider types.IRosProvider) {
	var req mowgli.BladeControlReq
	if err := c.ShouldBindJSON(&req); err != nil {
		c.JSON(http.StatusBadRequest, ErrorResponse{Error: err.Error()})
		return
	}
	if req.MowEnabled > 1 || (req.MowEnabled != 0 && req.MowDirection > 1) {
		c.JSON(http.StatusBadRequest, ErrorResponse{Error: "blade enable and ON direction must be 0 or 1"})
		return
	}
	if req.MowEnabled == 0 {
		req.MowDirection = 0
	}
	ctx := c.Request.Context()
	if req.MowEnabled == 0 {
		// Once OFF is accepted, closing the browser must not cancel its
		// independent fallback. Both attempts still have hard time bounds.
		ctx = context.WithoutCancel(ctx)
	}
	latchCtx, cancelLatch := context.WithTimeout(ctx, bladeLatchTimeout)
	var result mowgli.BladeControlRes
	latchErr := provider.CallService(latchCtx, "/behavior_tree_node/blade_control", &req, &result, "mowgli_interfaces/srv/BladeControl")
	cancelLatch()
	if latchErr == nil && !result.Success {
		latchErr = fmt.Errorf("%s", result.Message)
	}
	if req.MowEnabled != 0 {
		if latchErr != nil {
			c.JSON(http.StatusServiceUnavailable, ErrorResponse{Error: fmt.Sprintf("Blade direction was not accepted: %v. Check the ROS connection and update ROS and GUI together if their versions differ.", latchErr)})
		} else if !result.Forwarded {
			c.JSON(http.StatusOK, gin.H{"warning": result.Message})
		} else {
			c.JSON(http.StatusOK, gin.H{"message": result.Message})
		}
		return
	}
	// Commit inhibition first, then OFF. If the tree is absent, stalled or
	// rejects the request, this compatible hardware path still runs promptly.
	offCtx, cancelOff := context.WithTimeout(ctx, bladeOffTimeout)
	defer cancelOff()
	var hardware mowgli.MowerControlRes
	offErr := provider.CallService(offCtx, "/hardware_bridge/mower_control",
		&mowgli.MowerControlReq{MowEnabled: 0, MowDirection: 0}, &hardware, "mowgli_interfaces/srv/MowerControl")
	if offErr == nil && !hardware.Success {
		offErr = fmt.Errorf("hardware bridge rejected OFF")
	}
	switch {
	case latchErr == nil && offErr == nil:
		c.JSON(http.StatusOK, gin.H{"message": "Blade OFF latched and requested; physical stoppage is not confirmed"})
	case latchErr != nil && offErr == nil:
		c.JSON(http.StatusOK, gin.H{"warning": "Blade OFF requested, but persistent inhibition is unavailable. The behavior tree may re-enable it. Check the ROS connection and update ROS and GUI together."})
	case latchErr == nil:
		c.JSON(http.StatusOK, gin.H{"warning": fmt.Sprintf("Blade OFF is latched, but the hardware OFF request could not be confirmed: %v", offErr)})
	default:
		c.JSON(http.StatusServiceUnavailable, ErrorResponse{Error: fmt.Sprintf("Blade OFF could not be confirmed. Session controller: %v. Hardware bridge: %v", latchErr, offErr)})
	}
}
