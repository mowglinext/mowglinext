package api

import (
	"net/http"

	"github.com/gin-gonic/gin"
	"github.com/mowglinext/mowglinext/pkg/foxglove"
	"github.com/mowglinext/mowglinext/pkg/types"
)

// ROSHealth describes only the GUI's upstream transport. A responding bridge
// does not establish sensor freshness or the mower's readiness to move/update.
type ROSHealth struct {
	Ready     bool                      `json:"ready"`
	Transport *foxglove.TransportHealth `json:"transport"`
}

func readROSHealth(ros types.IRosProvider) ROSHealth {
	provider, ok := ros.(interface {
		TransportHealth() foxglove.TransportHealth
	})
	if !ok {
		return ROSHealth{}
	}
	health := provider.TransportHealth()
	return ROSHealth{Ready: health.Connected && health.HeartbeatOK, Transport: &health}
}

// HealthRoutes keeps HTTP liveness independent of every external dependency.
func HealthRoutes(r *gin.RouterGroup, ros types.IRosProvider) {
	r.GET("/health/live", getGUILiveness)
	r.GET("/health/ros", getROSReadiness(ros))
}

// getGUILiveness answers from the HTTP handler without consulting ROS or Docker.
// @Summary Check GUI HTTP liveness
// @Tags health
// @Produce json
// @Success 200 {object} map[string]string
// @Router /health/live [get]
func getGUILiveness(c *gin.Context) {
	c.Header("Cache-Control", "no-store")
	c.JSON(http.StatusOK, gin.H{"status": "ok"})
}

// getROSReadiness reports bridge heartbeat evidence without issuing a probe.
// @Summary Check the GUI to ROS transport
// @Description A healthy transport does not establish sensor freshness or mower safety readiness.
// @Tags health
// @Produce json
// @Success 200 {object} ROSHealth
// @Failure 503 {object} ROSHealth
// @Router /health/ros [get]
func getROSReadiness(ros types.IRosProvider) gin.HandlerFunc {
	return func(c *gin.Context) {
		health := readROSHealth(ros)
		status := http.StatusOK
		if !health.Ready {
			status = http.StatusServiceUnavailable
		}
		c.Header("Cache-Control", "no-store")
		c.JSON(status, health)
	}
}
