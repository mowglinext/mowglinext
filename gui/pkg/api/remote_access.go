package api

import (
	"context"
	"net/http"
	"strings"
	"time"

	"github.com/gin-gonic/gin"
	"github.com/mowglinext/mowglinext/pkg/providers"
)

// RemoteAccessSettingsResponse is the operator-facing view of the settings.
// The auth key never leaves the backend: only whether one is stored and a
// recognisable masked prefix.
type RemoteAccessSettingsResponse struct {
	Enabled       bool   `json:"enabled"`
	Hostname      string `json:"hostname"`
	AuthKeySet    bool   `json:"authKeySet"`
	AuthKeyMasked string `json:"authKeyMasked"`
	ServeHttps    bool   `json:"serveHttps"`
	Image         string `json:"image"`
	DefaultImage  string `json:"defaultImage"`
	ContainerName string `json:"containerName"`
}

// RemoteAccessSettingsUpdate is a partial update: absent fields keep their
// current value. AuthKey is write-only — send it to replace the stored key,
// or ClearAuthKey to forget it.
type RemoteAccessSettingsUpdate struct {
	Enabled      *bool   `json:"enabled"`
	Hostname     *string `json:"hostname"`
	AuthKey      *string `json:"authKey"`
	ClearAuthKey bool    `json:"clearAuthKey"`
	ServeHttps   *bool   `json:"serveHttps"`
	Image        *string `json:"image"`
}

const remoteAccessStatusTimeout = 15 * time.Second
const remoteAccessLogoutTimeout = 30 * time.Second

// RemoteAccessRoutes registers the remote-access (Tailscale sidecar) endpoints.
func RemoteAccessRoutes(r *gin.RouterGroup, p *providers.RemoteAccessProvider) {
	group := r.Group("/remote-access")
	group.GET("/settings", getRemoteAccessSettings(p))
	group.PUT("/settings", putRemoteAccessSettings(p))
	group.GET("/status", getRemoteAccessStatus(p))
	group.POST("/apply", postRemoteAccessApply(p))
	group.POST("/logout", postRemoteAccessLogout(p))
}

// getRemoteAccessSettings returns the settings (auth key masked)
//
// @Summary Remote access settings
// @Tags remote-access
// @Produce json
// @Success 200 {object} RemoteAccessSettingsResponse
// @Router /remote-access/settings [get]
func getRemoteAccessSettings(p *providers.RemoteAccessProvider) gin.HandlerFunc {
	return func(c *gin.Context) {
		c.JSON(http.StatusOK, remoteAccessSettingsView(p.Config()))
	}
}

// putRemoteAccessSettings updates the settings and reconciles the sidecar
//
// @Summary update remote access settings
// @Tags remote-access
// @Accept json
// @Produce json
// @Param settings body RemoteAccessSettingsUpdate true "partial settings"
// @Success 200 {object} RemoteAccessSettingsResponse
// @Failure 400 {object} ErrorResponse
// @Router /remote-access/settings [put]
func putRemoteAccessSettings(p *providers.RemoteAccessProvider) gin.HandlerFunc {
	return func(c *gin.Context) {
		var update RemoteAccessSettingsUpdate
		if err := c.BindJSON(&update); err != nil {
			c.JSON(http.StatusBadRequest, ErrorResponse{Error: err.Error()})
			return
		}
		cfg := applyRemoteAccessUpdate(p.Config(), update)
		if err := p.UpdateConfig(cfg); err != nil {
			c.JSON(http.StatusBadRequest, ErrorResponse{Error: err.Error()})
			return
		}
		c.JSON(http.StatusOK, remoteAccessSettingsView(p.Config()))
	}
}

// getRemoteAccessStatus returns the sidecar and tailnet state
//
// @Summary Remote access status
// @Description container phase, tailscaled login state, login URL while waiting, reachable URLs once connected
// @Tags remote-access
// @Produce json
// @Success 200 {object} providers.RemoteAccessStatus
// @Router /remote-access/status [get]
func getRemoteAccessStatus(p *providers.RemoteAccessProvider) gin.HandlerFunc {
	return func(c *gin.Context) {
		ctx, cancel := context.WithTimeout(c.Request.Context(), remoteAccessStatusTimeout)
		defer cancel()
		c.JSON(http.StatusOK, p.Status(ctx))
	}
}

// postRemoteAccessApply re-runs the reconcile (retry after an error)
//
// @Summary Retry applying remote access settings
// @Tags remote-access
// @Produce json
// @Success 200 {object} OkResponse
// @Router /remote-access/apply [post]
func postRemoteAccessApply(p *providers.RemoteAccessProvider) gin.HandlerFunc {
	return func(c *gin.Context) {
		p.Apply()
		c.JSON(http.StatusOK, OkResponse{Ok: "ok"})
	}
}

// postRemoteAccessLogout forgets the tailnet identity and re-logs in
//
// @Summary Log the robot out of the tailnet
// @Description the node key is discarded; the sidecar restarts and logs in again (interactively or with the stored auth key)
// @Tags remote-access
// @Produce json
// @Success 200 {object} OkResponse
// @Failure 400 {object} ErrorResponse
// @Router /remote-access/logout [post]
func postRemoteAccessLogout(p *providers.RemoteAccessProvider) gin.HandlerFunc {
	return func(c *gin.Context) {
		ctx, cancel := context.WithTimeout(c.Request.Context(), remoteAccessLogoutTimeout)
		defer cancel()
		if err := p.Logout(ctx); err != nil {
			c.JSON(http.StatusBadRequest, ErrorResponse{Error: err.Error()})
			return
		}
		c.JSON(http.StatusOK, OkResponse{Ok: "ok"})
	}
}

func remoteAccessSettingsView(cfg providers.RemoteAccessConfig) RemoteAccessSettingsResponse {
	return RemoteAccessSettingsResponse{
		Enabled:       cfg.Enabled,
		Hostname:      cfg.Hostname,
		AuthKeySet:    cfg.AuthKey != "",
		AuthKeyMasked: cfg.MaskedAuthKey(),
		ServeHttps:    cfg.ServeHTTPS,
		Image:         cfg.Image,
		DefaultImage:  providers.DefaultRemoteAccessImage,
		ContainerName: providers.RemoteAccessContainerName,
	}
}

// applyRemoteAccessUpdate returns a NEW config with the update's present
// fields laid over the current one; the input is not mutated.
func applyRemoteAccessUpdate(current providers.RemoteAccessConfig, u RemoteAccessSettingsUpdate) providers.RemoteAccessConfig {
	next := current
	if u.Enabled != nil {
		next.Enabled = *u.Enabled
	}
	if u.Hostname != nil {
		next.Hostname = providers.NormalizeRemoteAccessHostname(*u.Hostname)
	}
	if u.ClearAuthKey {
		next.AuthKey = ""
	} else if u.AuthKey != nil && strings.TrimSpace(*u.AuthKey) != "" {
		next.AuthKey = strings.TrimSpace(*u.AuthKey)
	}
	if u.ServeHttps != nil {
		next.ServeHTTPS = *u.ServeHttps
	}
	if u.Image != nil {
		image := strings.TrimSpace(*u.Image)
		if image == "" {
			image = providers.DefaultRemoteAccessImage
		}
		next.Image = image
	}
	return next
}
