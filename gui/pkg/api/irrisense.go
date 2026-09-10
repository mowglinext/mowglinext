package api

import (
	"context"
	"errors"
	"net/http"
	"strings"
	"time"

	"github.com/gin-gonic/gin"
	"github.com/mowglinext/mowglinext/pkg/providers"
	"github.com/mowglinext/mowglinext/pkg/types"
)

// IrriSenseSettingsResponse is the operator-facing view of the integration
// settings. The token itself never leaves the backend: only whether one is
// stored and a recognisable masked prefix.
type IrriSenseSettingsResponse struct {
	Enabled               bool     `json:"enabled"`
	BaseUrl               string   `json:"baseUrl"`
	TokenSet              bool     `json:"tokenSet"`
	TokenMasked           string   `json:"tokenMasked"`
	GardenId              string   `json:"gardenId"`
	ZoneIds               []string `json:"zoneIds"`
	WetDeficitMm          float64  `json:"wetDeficitMm"`
	DryAfterWateringHours float64  `json:"dryAfterWateringHours"`
	MaxStaleMinutes       float64  `json:"maxStaleMinutes"`
	GateScheduler         bool     `json:"gateScheduler"`
}

// IrriSenseSettingsUpdate is a partial update: absent fields keep their
// current value. Token is write-only — send it to replace the stored token,
// or ClearToken to forget it.
type IrriSenseSettingsUpdate struct {
	Enabled               *bool     `json:"enabled"`
	BaseUrl               *string   `json:"baseUrl"`
	Token                 *string   `json:"token"`
	ClearToken            bool      `json:"clearToken"`
	GardenId              *string   `json:"gardenId"`
	ZoneIds               *[]string `json:"zoneIds"`
	WetDeficitMm          *float64  `json:"wetDeficitMm"`
	DryAfterWateringHours *float64  `json:"dryAfterWateringHours"`
	MaxStaleMinutes       *float64  `json:"maxStaleMinutes"`
	GateScheduler         *bool     `json:"gateScheduler"`
}

// IrriSenseGardensResponse lists the gardens the stored token may read.
type IrriSenseGardensResponse struct {
	Gardens []providers.IrriSenseGardenSummary `json:"gardens"`
}

// IrriSenseErrorResponse carries a machine-readable code next to the message
// so the GUI can tell "bad token" from "service down".
type IrriSenseErrorResponse struct {
	Error string `json:"error"`
	Code  string `json:"code,omitempty"`
}

const irriSenseGardensTimeout = 12 * time.Second

// IrriSenseRoutes registers the soil-moisture integration endpoints.
func IrriSenseRoutes(r *gin.RouterGroup, p *providers.IrriSenseProvider) {
	group := r.Group("/irrisense")
	group.GET("/status", getIrriSenseStatus(p))
	group.GET("/settings", getIrriSenseSettings(p))
	group.PUT("/settings", putIrriSenseSettings(p))
	group.GET("/gardens", getIrriSenseGardens(p))
}

// getIrriSenseStatus returns the cached soil verdict
//
// @Summary IrriSense soil status
// @Description cached wet/dry/unknown verdict the scheduler gate reads
// @Tags irrisense
// @Produce json
// @Success 200 {object} types.SoilStatus
// @Router /irrisense/status [get]
func getIrriSenseStatus(p *providers.IrriSenseProvider) gin.HandlerFunc {
	return func(c *gin.Context) {
		c.JSON(http.StatusOK, p.SoilStatus())
	}
}

// getIrriSenseSettings returns the integration settings (token masked)
//
// @Summary IrriSense settings
// @Tags irrisense
// @Produce json
// @Success 200 {object} IrriSenseSettingsResponse
// @Router /irrisense/settings [get]
func getIrriSenseSettings(p *providers.IrriSenseProvider) gin.HandlerFunc {
	return func(c *gin.Context) {
		c.JSON(http.StatusOK, irriSenseSettingsView(p.Config()))
	}
}

// putIrriSenseSettings updates the integration settings and refreshes
//
// @Summary update IrriSense settings
// @Tags irrisense
// @Accept json
// @Produce json
// @Param settings body IrriSenseSettingsUpdate true "partial settings"
// @Success 200 {object} IrriSenseSettingsResponse
// @Failure 400 {object} ErrorResponse
// @Router /irrisense/settings [put]
func putIrriSenseSettings(p *providers.IrriSenseProvider) gin.HandlerFunc {
	return func(c *gin.Context) {
		var update IrriSenseSettingsUpdate
		if err := c.BindJSON(&update); err != nil {
			c.JSON(http.StatusBadRequest, ErrorResponse{Error: err.Error()})
			return
		}
		cfg := applyIrriSenseUpdate(p.Config(), update)
		if err := p.UpdateConfig(cfg); err != nil {
			c.JSON(http.StatusBadRequest, ErrorResponse{Error: err.Error()})
			return
		}
		c.JSON(http.StatusOK, irriSenseSettingsView(p.Config()))
	}
}

// getIrriSenseGardens proxies the read-only garden list with the stored token
//
// @Summary list IrriSense gardens
// @Description gardens readable by the stored token, for the picker and "test connection"
// @Tags irrisense
// @Produce json
// @Success 200 {object} IrriSenseGardensResponse
// @Failure 400 {object} IrriSenseErrorResponse
// @Failure 502 {object} IrriSenseErrorResponse
// @Router /irrisense/gardens [get]
func getIrriSenseGardens(p *providers.IrriSenseProvider) gin.HandlerFunc {
	return func(c *gin.Context) {
		ctx, cancel := context.WithTimeout(c.Request.Context(), irriSenseGardensTimeout)
		defer cancel()
		gardens, err := p.ListGardens(ctx)
		if err != nil {
			status, code := classifyIrriSenseError(err)
			c.JSON(status, IrriSenseErrorResponse{Error: err.Error(), Code: code})
			return
		}
		c.JSON(http.StatusOK, IrriSenseGardensResponse{Gardens: gardens})
	}
}

func classifyIrriSenseError(err error) (int, string) {
	var rateLimited *providers.IrriSenseRateLimitedError
	switch {
	case errors.Is(err, providers.ErrIrriSenseUnauthorized):
		return http.StatusBadRequest, "unauthorized"
	case errors.As(err, &rateLimited):
		return http.StatusTooManyRequests, "rate_limited"
	case strings.Contains(err.Error(), "no IrriSense API token"):
		return http.StatusBadRequest, "no_token"
	default:
		return http.StatusBadGateway, "upstream"
	}
}

func irriSenseSettingsView(cfg providers.IrriSenseConfig) IrriSenseSettingsResponse {
	zoneIDs := cfg.ZoneIDs
	if zoneIDs == nil {
		zoneIDs = []string{}
	}
	return IrriSenseSettingsResponse{
		Enabled:               cfg.Enabled,
		BaseUrl:               cfg.BaseURL,
		TokenSet:              cfg.Token != "",
		TokenMasked:           cfg.MaskedToken(),
		GardenId:              cfg.GardenID,
		ZoneIds:               zoneIDs,
		WetDeficitMm:          cfg.WetDeficitMm,
		DryAfterWateringHours: cfg.DryAfterWateringHours,
		MaxStaleMinutes:       cfg.MaxStaleMinutes,
		GateScheduler:         cfg.GateScheduler,
	}
}

// applyIrriSenseUpdate returns a NEW config with the update's present fields
// laid over the current one; the input is not mutated.
func applyIrriSenseUpdate(current providers.IrriSenseConfig, u IrriSenseSettingsUpdate) providers.IrriSenseConfig {
	next := current
	next.ZoneIDs = append([]string{}, current.ZoneIDs...)
	if u.Enabled != nil {
		next.Enabled = *u.Enabled
	}
	if u.BaseUrl != nil {
		next.BaseURL = strings.TrimSpace(*u.BaseUrl)
	}
	if u.ClearToken {
		next.Token = ""
	} else if u.Token != nil && strings.TrimSpace(*u.Token) != "" {
		next.Token = strings.TrimSpace(*u.Token)
	}
	if u.GardenId != nil {
		next.GardenID = strings.TrimSpace(*u.GardenId)
	}
	if u.ZoneIds != nil {
		next.ZoneIDs = cleanZoneIDs(*u.ZoneIds)
	}
	if u.WetDeficitMm != nil {
		next.WetDeficitMm = *u.WetDeficitMm
	}
	if u.DryAfterWateringHours != nil {
		next.DryAfterWateringHours = *u.DryAfterWateringHours
	}
	if u.MaxStaleMinutes != nil {
		next.MaxStaleMinutes = *u.MaxStaleMinutes
	}
	if u.GateScheduler != nil {
		next.GateScheduler = *u.GateScheduler
	}
	return next
}

func cleanZoneIDs(in []string) []string {
	out := make([]string, 0, len(in))
	for _, id := range in {
		if trimmed := strings.TrimSpace(id); trimmed != "" {
			out = append(out, trimmed)
		}
	}
	return out
}

// Compile-time check: the provider satisfies the scheduler's interface.
var _ types.ISoilProvider = (*providers.IrriSenseProvider)(nil)
