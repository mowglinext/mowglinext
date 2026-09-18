package api

import (
	"context"
	"encoding/json"
	"io"
	"net/http"
	"strings"
	"time"

	"github.com/gin-gonic/gin"
	"github.com/mowglinext/mowglinext/pkg/providers"
)

// Fleet routes: identity + peer registry + snapshot + command proxy. These are
// the ONLY routes a peer robot calls on us, and the only way the browser
// reaches a peer (through us). See docs/MULTI_ROBOT.md.

const fleetCallTimeout = 12 * time.Second

// FleetRoutes registers /fleet/* on the API group.
func FleetRoutes(r *gin.RouterGroup, fleet *providers.FleetProvider, coord *providers.FleetCoordinator) {
	g := r.Group("/fleet")
	g.GET("/coordination", func(c *gin.Context) { getFleetCoordination(c, coord) })
	g.PUT("/coordination", func(c *gin.Context) { putFleetCoordination(c, coord) })
	g.POST("/coordination/reset", func(c *gin.Context) { postFleetCoordinationReset(c, fleet, coord) })
	g.POST("/map/push", func(c *gin.Context) { postFleetMapPush(c, fleet) })
	g.GET("/identity", func(c *gin.Context) { getFleetIdentity(c, fleet) })
	g.GET("/robots", func(c *gin.Context) { getFleetRobots(c, fleet) })
	g.GET("/peers", func(c *gin.Context) { c.JSON(http.StatusOK, fleet.Peers()) })
	g.POST("/peers", func(c *gin.Context) { postFleetPeer(c, fleet) })
	g.POST("/peers/register", func(c *gin.Context) { postFleetRegister(c, fleet) })
	g.POST("/peers/unregister", func(c *gin.Context) { postFleetUnregister(c, fleet) })
	g.DELETE("/peers/:id", func(c *gin.Context) { deleteFleetPeer(c, fleet) })
	g.POST("/robots/:id/call/:command", func(c *gin.Context) { postFleetCall(c, fleet) })
}

// getFleetIdentity returns who this robot is.
//
// @Summary this robot's fleet identity
// @Tags fleet
// @Produce json
// @Success 200 {object} providers.RobotIdentity
// @Failure 500 {object} ErrorResponse
// @Router /fleet/identity [get]
func getFleetIdentity(c *gin.Context, fleet *providers.FleetProvider) {
	id, err := fleet.Identity()
	if err != nil {
		c.JSON(http.StatusInternalServerError, ErrorResponse{Error: err.Error()})
		return
	}
	c.JSON(http.StatusOK, id)
}

// getFleetRobots returns the live fleet snapshot (self first).
//
// @Summary fleet snapshot
// @Tags fleet
// @Produce json
// @Success 200 {array} providers.FleetRobot
// @Failure 500 {object} ErrorResponse
// @Router /fleet/robots [get]
func getFleetRobots(c *gin.Context, fleet *providers.FleetProvider) {
	rows, err := fleet.Robots()
	if err != nil {
		c.JSON(http.StatusInternalServerError, ErrorResponse{Error: err.Error()})
		return
	}
	c.JSON(http.StatusOK, rows)
}

type addPeerRequest struct {
	Address string `json:"address"`
}

// postFleetPeer adds a peer by the address of its GUI.
//
// @Summary add a fleet peer
// @Tags fleet
// @Accept json
// @Produce json
// @Param body body addPeerRequest true "peer GUI address (host[:port])"
// @Success 200 {object} providers.AddPeerResult
// @Failure 400 {object} ErrorResponse
// @Router /fleet/peers [post]
func postFleetPeer(c *gin.Context, fleet *providers.FleetProvider) {
	var req addPeerRequest
	if err := c.BindJSON(&req); err != nil {
		c.JSON(http.StatusBadRequest, ErrorResponse{Error: err.Error()})
		return
	}
	ctx, cancel := context.WithTimeout(c.Request.Context(), fleetCallTimeout)
	defer cancel()
	res, err := fleet.AddPeer(ctx, req.Address)
	if err != nil {
		c.JSON(http.StatusBadRequest, ErrorResponse{Error: err.Error()})
		return
	}
	c.JSON(http.StatusOK, res)
}

type registerPeerRequest struct {
	ID   string `json:"id"`
	Name string `json:"name"`
	Port int    `json:"port"`
}

// postFleetRegister is called BY a peer that just added us; its IP comes from
// the connection, its API port from the body.
//
// @Summary reverse-register a peer
// @Tags fleet
// @Accept json
// @Produce json
// @Param body body registerPeerRequest true "peer identity + API port"
// @Success 200 {object} providers.FleetPeer
// @Failure 400 {object} ErrorResponse
// @Router /fleet/peers/register [post]
func postFleetRegister(c *gin.Context, fleet *providers.FleetProvider) {
	var req registerPeerRequest
	if err := c.BindJSON(&req); err != nil {
		c.JSON(http.StatusBadRequest, ErrorResponse{Error: err.Error()})
		return
	}
	peer, err := fleet.RegisterPeer(req.ID, req.Name, c.ClientIP(), req.Port)
	if err != nil {
		c.JSON(http.StatusBadRequest, ErrorResponse{Error: err.Error()})
		return
	}
	c.JSON(http.StatusOK, peer)
}

// postFleetUnregister is called BY a peer that removed us.
//
// @Summary a peer asks to be forgotten
// @Tags fleet
// @Accept json
// @Produce json
// @Success 200 {object} OkResponse
// @Router /fleet/peers/unregister [post]
func postFleetUnregister(c *gin.Context, fleet *providers.FleetProvider) {
	var req struct {
		ID string `json:"id"`
	}
	if err := c.BindJSON(&req); err != nil || req.ID == "" {
		c.JSON(http.StatusBadRequest, ErrorResponse{Error: "id is required"})
		return
	}
	fleet.UnregisterPeer(req.ID)
	c.JSON(http.StatusOK, OkResponse{})
}

// deleteFleetPeer removes a peer (and tells it to forget us).
//
// @Summary remove a fleet peer
// @Tags fleet
// @Produce json
// @Param id path string true "peer robot id"
// @Success 200 {object} OkResponse
// @Failure 404 {object} ErrorResponse
// @Router /fleet/peers/{id} [delete]
func deleteFleetPeer(c *gin.Context, fleet *providers.FleetProvider) {
	ctx, cancel := context.WithTimeout(c.Request.Context(), fleetCallTimeout)
	defer cancel()
	if err := fleet.RemovePeer(ctx, c.Param("id")); err != nil {
		c.JSON(http.StatusNotFound, ErrorResponse{Error: err.Error()})
		return
	}
	c.JSON(http.StatusOK, OkResponse{})
}

// postFleetCall sends a fleet command to one robot (self or peer).
//
// @Summary send a command to a fleet robot
// @Tags fleet
// @Accept json
// @Produce json
// @Param id path string true "robot id"
// @Param command path string true "high_level_control | emergency | coverage_clear_resume"
// @Success 200 {object} OkResponse
// @Failure 400 {object} ErrorResponse
// @Failure 502 {object} ErrorResponse
// @Router /fleet/robots/{id}/call/{command} [post]
func postFleetCall(c *gin.Context, fleet *providers.FleetProvider) {
	body, err := io.ReadAll(io.LimitReader(c.Request.Body, 64<<10))
	if err != nil {
		c.JSON(http.StatusBadRequest, ErrorResponse{Error: err.Error()})
		return
	}
	if len(body) == 0 {
		body = []byte(`{}`)
	}
	if !json.Valid(body) {
		c.JSON(http.StatusBadRequest, ErrorResponse{Error: "body must be JSON"})
		return
	}
	ctx, cancel := context.WithTimeout(c.Request.Context(), fleetCallTimeout)
	defer cancel()
	status, resp, err := fleet.Call(ctx, c.Param("id"), c.Param("command"), body)
	if err != nil {
		c.JSON(status, ErrorResponse{Error: err.Error()})
		return
	}
	c.Data(status, "application/json", resp)
}

// FleetCoordinationResponse bundles the coordinator settings and live state.
type FleetCoordinationResponse struct {
	Settings providers.CoordinatorSettings `json:"settings"`
	Status   providers.CoordinatorStatus   `json:"status"`
}

// getFleetCoordination returns coordinated-mowing settings + status.
//
// @Summary coordinated mowing settings and status
// @Tags fleet
// @Produce json
// @Success 200 {object} FleetCoordinationResponse
// @Router /fleet/coordination [get]
func getFleetCoordination(c *gin.Context, coord *providers.FleetCoordinator) {
	c.JSON(http.StatusOK, FleetCoordinationResponse{Settings: coord.Settings(), Status: coord.Status()})
}

// putFleetCoordination updates coordinated-mowing settings.
//
// @Summary update coordinated mowing settings
// @Tags fleet
// @Accept json
// @Produce json
// @Param body body providers.CoordinatorSettings true "settings"
// @Success 200 {object} FleetCoordinationResponse
// @Failure 400 {object} ErrorResponse
// @Router /fleet/coordination [put]
func putFleetCoordination(c *gin.Context, coord *providers.FleetCoordinator) {
	var s providers.CoordinatorSettings
	if err := c.BindJSON(&s); err != nil {
		c.JSON(http.StatusBadRequest, ErrorResponse{Error: err.Error()})
		return
	}
	if err := coord.SetSettings(s); err != nil {
		c.JSON(http.StatusInternalServerError, ErrorResponse{Error: err.Error()})
		return
	}
	c.JSON(http.StatusOK, FleetCoordinationResponse{Settings: coord.Settings(), Status: coord.Status()})
}

// postFleetCoordinationReset forgets the fleet session's completed areas and
// clears every member's coverage resume ("start fresh" for the whole fleet).
//
// @Summary fleet-wide start fresh
// @Tags fleet
// @Produce json
// @Success 200 {object} OkResponse
// @Failure 500 {object} ErrorResponse
// @Router /fleet/coordination/reset [post]
func postFleetCoordinationReset(c *gin.Context, fleet *providers.FleetProvider, coord *providers.FleetCoordinator) {
	ctx, cancel := context.WithTimeout(c.Request.Context(), fleetCallTimeout)
	defer cancel()
	rows, err := fleet.Robots()
	if err != nil {
		c.JSON(http.StatusInternalServerError, ErrorResponse{Error: err.Error()})
		return
	}
	var failures []string
	for _, r := range rows {
		if !r.Online {
			continue
		}
		if _, _, err := fleet.Call(ctx, r.Identity.ID, "coverage_clear_resume", json.RawMessage(`{}`)); err != nil {
			failures = append(failures, r.Identity.Name+": "+err.Error())
		}
	}
	if err := coord.Reset(); err != nil {
		failures = append(failures, "coordinator: "+err.Error())
	}
	if len(failures) > 0 {
		c.JSON(http.StatusInternalServerError, ErrorResponse{Error: "start fresh incomplete — " + strings.Join(failures, "; ")})
		return
	}
	c.JSON(http.StatusOK, OkResponse{})
}

// postFleetMapPush copies this robot's map onto every peer.
//
// @Summary push this robot's map to the fleet
// @Tags fleet
// @Produce json
// @Success 200 {object} providers.MapPushResult
// @Failure 400 {object} ErrorResponse
// @Router /fleet/map/push [post]
func postFleetMapPush(c *gin.Context, fleet *providers.FleetProvider) {
	res, err := fleet.PushMap(c.Request.Context())
	if err != nil {
		c.JSON(http.StatusBadRequest, ErrorResponse{Error: err.Error()})
		return
	}
	c.JSON(http.StatusOK, res)
}
