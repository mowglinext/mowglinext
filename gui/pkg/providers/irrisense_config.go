package providers

import (
	"encoding/json"
	"fmt"
	"net/url"
	"strconv"
	"strings"
	"time"

	"github.com/mowglinext/mowglinext/pkg/types"
	"github.com/sirupsen/logrus"
)

// IrriSense settings live in the GUI's key-value DB, NOT in mowgli_robot.yaml:
// the token is a secret the operator minted in IrriSense, and nothing on the
// ROS2 side consumes any of these values.
const (
	irriSenseKeyEnabled               = "irrisense.enabled"
	irriSenseKeyBaseURL               = "irrisense.baseUrl"
	irriSenseKeyToken                 = "irrisense.token"
	irriSenseKeyGardenID              = "irrisense.gardenId"
	irriSenseKeyZoneIDs               = "irrisense.zoneIds"
	irriSenseKeyWetDeficitMm          = "irrisense.wetDeficitMm"
	irriSenseKeyDryAfterWateringHours = "irrisense.dryAfterWateringHours"
	irriSenseKeyMaxStaleMinutes       = "irrisense.maxStaleMinutes"
	irriSenseKeyGateScheduler         = "irrisense.gateScheduler"
)

const (
	DefaultIrriSenseBaseURL               = "https://irrisense-cloud.fly.dev"
	DefaultIrriSenseWetDeficitMm          = 2.0
	DefaultIrriSenseDryAfterWateringHours = 3.0
	DefaultIrriSenseMaxStaleMinutes       = 90.0
)

// IrriSenseConfig is the operator's IrriSense integration settings.
type IrriSenseConfig struct {
	Enabled               bool
	BaseURL               string
	Token                 string
	GardenID              string
	ZoneIDs               []string
	WetDeficitMm          float64
	DryAfterWateringHours float64
	MaxStaleMinutes       float64
	GateScheduler         bool
}

// DefaultIrriSenseConfig is a fresh install: disabled, pointing at the hosted
// service, with the documented thresholds.
func DefaultIrriSenseConfig() IrriSenseConfig {
	return IrriSenseConfig{
		Enabled:               false,
		BaseURL:               DefaultIrriSenseBaseURL,
		ZoneIDs:               []string{},
		WetDeficitMm:          DefaultIrriSenseWetDeficitMm,
		DryAfterWateringHours: DefaultIrriSenseDryAfterWateringHours,
		MaxStaleMinutes:       DefaultIrriSenseMaxStaleMinutes,
		GateScheduler:         true,
	}
}

// IsConfigured reports whether a poll can be attempted at all.
func (c IrriSenseConfig) IsConfigured() bool {
	return c.Token != "" && c.GardenID != "" && ValidateIrriSenseBaseURL(c.BaseURL) == nil
}

// Wetness is the pure-rule view of the config.
func (c IrriSenseConfig) Wetness() WetnessConfig {
	return WetnessConfig{
		WetDeficitMm:          c.WetDeficitMm,
		DryAfterWateringHours: c.DryAfterWateringHours,
		ZoneIDs:               c.ZoneIDs,
	}
}

// MaxStale is the freshness window as a duration.
func (c IrriSenseConfig) MaxStale() time.Duration {
	return time.Duration(c.MaxStaleMinutes * float64(time.Minute))
}

// MaskedToken is the only form of the token that ever leaves the backend:
// enough to recognise which token was pasted, never enough to use it.
func (c IrriSenseConfig) MaskedToken() string {
	if c.Token == "" {
		return ""
	}
	if len(c.Token) <= 8 {
		return "••••••••"
	}
	return c.Token[:4] + "••••••••"
}

// Validate rejects a config the provider could not act on.
func (c IrriSenseConfig) Validate() error {
	if err := ValidateIrriSenseBaseURL(c.BaseURL); err != nil {
		return err
	}
	if c.WetDeficitMm < 0 {
		return fmt.Errorf("wetDeficitMm must be >= 0")
	}
	if c.DryAfterWateringHours < 0 {
		return fmt.Errorf("dryAfterWateringHours must be >= 0")
	}
	if c.MaxStaleMinutes <= 0 {
		return fmt.Errorf("maxStaleMinutes must be > 0")
	}
	return nil
}

// ValidateIrriSenseBaseURL accepts an absolute http(s) URL with a host and no
// query/fragment — the provider appends the read-only API paths to it.
func ValidateIrriSenseBaseURL(raw string) error {
	u, err := url.Parse(strings.TrimSpace(raw))
	if err != nil {
		return fmt.Errorf("baseUrl is not a valid URL: %w", err)
	}
	if u.Scheme != "http" && u.Scheme != "https" {
		return fmt.Errorf("baseUrl must start with http:// or https://")
	}
	if u.Host == "" {
		return fmt.Errorf("baseUrl must include a host")
	}
	if u.RawQuery != "" || u.Fragment != "" {
		return fmt.Errorf("baseUrl must not carry a query string or fragment")
	}
	return nil
}

// LoadIrriSenseConfig reads the settings from the DB, falling back to the
// defaults for every absent or unparseable key (so a half-written config can
// never crash the GUI at startup).
func LoadIrriSenseConfig(db types.IDBProvider) IrriSenseConfig {
	cfg := DefaultIrriSenseConfig()
	cfg.Enabled = dbBool(db, irriSenseKeyEnabled, cfg.Enabled)
	cfg.BaseURL = dbString(db, irriSenseKeyBaseURL, cfg.BaseURL)
	cfg.Token = dbString(db, irriSenseKeyToken, "")
	cfg.GardenID = dbString(db, irriSenseKeyGardenID, "")
	cfg.ZoneIDs = dbStringList(db, irriSenseKeyZoneIDs)
	cfg.WetDeficitMm = dbFloat(db, irriSenseKeyWetDeficitMm, cfg.WetDeficitMm)
	cfg.DryAfterWateringHours = dbFloat(db, irriSenseKeyDryAfterWateringHours, cfg.DryAfterWateringHours)
	cfg.MaxStaleMinutes = dbFloat(db, irriSenseKeyMaxStaleMinutes, cfg.MaxStaleMinutes)
	cfg.GateScheduler = dbBool(db, irriSenseKeyGateScheduler, cfg.GateScheduler)
	return cfg
}

// SaveIrriSenseConfig persists every key. The token is written verbatim to the
// DB and nowhere else; an empty token deletes the key.
func SaveIrriSenseConfig(db types.IDBProvider, cfg IrriSenseConfig) error {
	zoneIDs, err := json.Marshal(nonNilStrings(cfg.ZoneIDs))
	if err != nil {
		return fmt.Errorf("encode zone ids: %w", err)
	}
	writes := []struct {
		key   string
		value string
	}{
		{irriSenseKeyEnabled, strconv.FormatBool(cfg.Enabled)},
		{irriSenseKeyBaseURL, strings.TrimSpace(cfg.BaseURL)},
		{irriSenseKeyGardenID, strings.TrimSpace(cfg.GardenID)},
		{irriSenseKeyZoneIDs, string(zoneIDs)},
		{irriSenseKeyWetDeficitMm, strconv.FormatFloat(cfg.WetDeficitMm, 'f', -1, 64)},
		{irriSenseKeyDryAfterWateringHours, strconv.FormatFloat(cfg.DryAfterWateringHours, 'f', -1, 64)},
		{irriSenseKeyMaxStaleMinutes, strconv.FormatFloat(cfg.MaxStaleMinutes, 'f', -1, 64)},
		{irriSenseKeyGateScheduler, strconv.FormatBool(cfg.GateScheduler)},
	}
	for _, w := range writes {
		if err := db.Set(w.key, []byte(w.value)); err != nil {
			return fmt.Errorf("persist %s: %w", w.key, err)
		}
	}
	if cfg.Token == "" {
		if err := db.Delete(irriSenseKeyToken); err != nil {
			return fmt.Errorf("clear IrriSense token: %w", err)
		}
		return nil
	}
	if err := db.Set(irriSenseKeyToken, []byte(cfg.Token)); err != nil {
		return fmt.Errorf("persist IrriSense token: %w", err)
	}
	return nil
}

func nonNilStrings(in []string) []string {
	if in == nil {
		return []string{}
	}
	return in
}

func dbString(db types.IDBProvider, key, def string) string {
	v, err := db.Get(key)
	if err != nil || len(v) == 0 {
		return def
	}
	return string(v)
}

func dbBool(db types.IDBProvider, key string, def bool) bool {
	raw := dbString(db, key, "")
	if raw == "" {
		return def
	}
	b, err := strconv.ParseBool(raw)
	if err != nil {
		logrus.Warnf("IrriSense: %s=%q is not a boolean, using %v", key, raw, def)
		return def
	}
	return b
}

func dbFloat(db types.IDBProvider, key string, def float64) float64 {
	raw := dbString(db, key, "")
	if raw == "" {
		return def
	}
	f, err := strconv.ParseFloat(raw, 64)
	if err != nil {
		logrus.Warnf("IrriSense: %s=%q is not a number, using %v", key, raw, def)
		return def
	}
	return f
}

func dbStringList(db types.IDBProvider, key string) []string {
	raw := dbString(db, key, "")
	if raw == "" {
		return []string{}
	}
	var list []string
	if err := json.Unmarshal([]byte(raw), &list); err != nil {
		logrus.Warnf("IrriSense: %s is not a JSON list of strings, ignoring the zone filter", key)
		return []string{}
	}
	return nonNilStrings(list)
}
