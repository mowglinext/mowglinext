package providers

import (
	"context"
	"errors"
	"fmt"
	"net/http"
	"sync"
	"time"

	"github.com/mowglinext/mowglinext/pkg/types"
	"github.com/sirupsen/logrus"
)

const (
	irriSensePollInterval = 10 * time.Minute
	irriSenseMinBackoff   = 1 * time.Minute
	irriSenseMaxBackoff   = 30 * time.Minute
)

// irriSenseSample is the last successful fetch, kept whole so a later failed
// poll can keep serving it until it goes stale.
type irriSenseSample struct {
	garden    IrriSenseGarden
	fetchedAt time.Time
}

// IrriSenseProvider polls the operator's IrriSense Cloud garden every
// irriSensePollInterval (and immediately after a config change), applies the
// wetness rule (irrisense_wetness.go) and caches the verdict for the scheduler
// and the GUI.
//
// Fail-open by construction: SoilStatus() only ever reports Wet && Fresh from a
// successful fetch younger than the staleness window. Any error, rate-limit
// backoff or outage degrades to Unknown, which the scheduler treats as "mow as
// usual" — and it logs the outage once rather than once per poll.
type IrriSenseProvider struct {
	db     types.IDBProvider
	client *irriSenseClient
	now    func() time.Time
	wake   chan struct{}

	mu           sync.RWMutex
	cfg          IrriSenseConfig
	sample       *irriSenseSample
	lastErr      error
	backoff      time.Duration
	notBefore    time.Time
	outageLogged bool
}

// NewIrriSenseProvider loads the config from the DB and starts the poll loop.
func NewIrriSenseProvider(db types.IDBProvider) *IrriSenseProvider {
	p := newIrriSenseProvider(db, nil, time.Now)
	go p.run()
	return p
}

// NewIdleIrriSenseProvider builds a provider WITHOUT the poll loop: it only
// fetches when Refresh() is called. For tests of the API layer and tooling.
func NewIdleIrriSenseProvider(db types.IDBProvider) *IrriSenseProvider {
	return newIrriSenseProvider(db, nil, time.Now)
}

// newIrriSenseProvider builds a provider without starting the goroutine, so
// tests drive Refresh() by hand with an injected clock and HTTP client.
func newIrriSenseProvider(db types.IDBProvider, httpClient *http.Client, now func() time.Time) *IrriSenseProvider {
	return &IrriSenseProvider{
		db:     db,
		client: newIrriSenseClient(httpClient),
		now:    now,
		wake:   make(chan struct{}, 1),
		cfg:    LoadIrriSenseConfig(db),
	}
}

func (p *IrriSenseProvider) run() {
	p.Refresh()
	ticker := time.NewTicker(irriSensePollInterval)
	defer ticker.Stop()
	for {
		select {
		case <-ticker.C:
		case <-p.wake:
		}
		p.Refresh()
	}
}

// RequestRefresh nudges the poll loop without blocking the caller.
func (p *IrriSenseProvider) RequestRefresh() {
	select {
	case p.wake <- struct{}{}:
	default:
	}
}

// Config returns a copy of the current settings (token included — callers
// that expose it must mask it, see IrriSenseConfig.MaskedToken).
func (p *IrriSenseProvider) Config() IrriSenseConfig {
	p.mu.RLock()
	defer p.mu.RUnlock()
	return copyIrriSenseConfig(p.cfg)
}

// UpdateConfig validates, persists and applies new settings, drops the cached
// sample when it can no longer be trusted (different garden or service) and
// schedules an immediate poll.
func (p *IrriSenseProvider) UpdateConfig(cfg IrriSenseConfig) error {
	if err := cfg.Validate(); err != nil {
		return err
	}
	if err := SaveIrriSenseConfig(p.db, cfg); err != nil {
		return err
	}

	p.mu.Lock()
	sourceChanged := cfg.BaseURL != p.cfg.BaseURL || cfg.GardenID != p.cfg.GardenID || cfg.Token != p.cfg.Token
	p.cfg = copyIrriSenseConfig(cfg)
	if sourceChanged {
		p.sample = nil
		p.lastErr = nil
	}
	// A settings change is an operator action: retry now even if the last
	// poll was rate-limited, the backoff only guards the automatic loop.
	p.backoff = 0
	p.notBefore = time.Time{}
	p.outageLogged = false
	p.mu.Unlock()

	p.RequestRefresh()
	return nil
}

// ListGardens fetches every garden the stored token may read, for the GUI's
// garden picker and its "test connection" button.
func (p *IrriSenseProvider) ListGardens(ctx context.Context) ([]IrriSenseGardenSummary, error) {
	cfg := p.Config()
	if cfg.Token == "" {
		return nil, fmt.Errorf("no IrriSense API token stored")
	}
	if err := ValidateIrriSenseBaseURL(cfg.BaseURL); err != nil {
		return nil, err
	}
	gardens, err := p.client.fetchGardens(ctx, cfg.BaseURL, cfg.Token)
	if err != nil {
		return nil, err
	}
	out := make([]IrriSenseGardenSummary, 0, len(gardens))
	for _, g := range gardens {
		out = append(out, summarizeGarden(g))
	}
	return out, nil
}

// Refresh performs one poll if the integration is enabled, configured and not
// inside a rate-limit backoff. Safe to call from any goroutine.
func (p *IrriSenseProvider) Refresh() {
	cfg := p.Config()
	if !cfg.Enabled || !cfg.IsConfigured() {
		return
	}
	now := p.now()
	p.mu.RLock()
	notBefore := p.notBefore
	p.mu.RUnlock()
	if now.Before(notBefore) {
		logrus.Debugf("IrriSense: poll skipped, backing off until %s", notBefore.Format(time.RFC3339))
		return
	}

	ctx, cancel := context.WithTimeout(context.Background(), irriSenseHTTPTimeout)
	garden, err := p.client.fetchGarden(ctx, cfg.BaseURL, cfg.Token, cfg.GardenID)
	cancel()

	p.mu.Lock()
	defer p.mu.Unlock()
	if err != nil {
		p.recordFailure(err, now)
		return
	}
	p.sample = &irriSenseSample{garden: garden, fetchedAt: p.now()}
	p.lastErr = nil
	p.backoff = 0
	p.notBefore = time.Time{}
	if p.outageLogged {
		logrus.Infof("IrriSense: connection restored (garden %q)", garden.Name)
		p.outageLogged = false
	}
}

// recordFailure (caller holds p.mu) stores the error, grows the backoff and
// logs the outage exactly once until the next success.
func (p *IrriSenseProvider) recordFailure(err error, now time.Time) {
	p.lastErr = err

	var rateLimited *IrriSenseRateLimitedError
	if errors.As(err, &rateLimited) {
		p.backoff = maxDuration(nextBackoff(p.backoff), rateLimited.RetryAfter)
	} else {
		p.backoff = nextBackoff(p.backoff)
	}
	p.notBefore = now.Add(p.backoff)

	if p.outageLogged {
		logrus.Debugf("IrriSense: poll still failing (%v), next attempt in %s", err, p.backoff)
		return
	}
	p.outageLogged = true
	logrus.Warnf("IrriSense: poll failed (%v); scheduled mowing is NOT blocked while the soil state is unknown, next attempt in %s", err, p.backoff)
}

func nextBackoff(current time.Duration) time.Duration {
	if current <= 0 {
		return irriSenseMinBackoff
	}
	return minDuration(current*2, irriSenseMaxBackoff)
}

// SoilStatus is the scheduler- and GUI-facing verdict, evaluated against the
// clock at call time so a sample ages into Unknown without another poll.
func (p *IrriSenseProvider) SoilStatus() types.SoilStatus {
	p.mu.RLock()
	defer p.mu.RUnlock()

	cfg := p.cfg
	status := types.SoilStatus{
		Enabled:       cfg.Enabled,
		Configured:    cfg.IsConfigured(),
		GateScheduler: cfg.GateScheduler,
		Unknown:       true,
		Zones:         []types.SoilZoneStatus{},
	}
	if p.lastErr != nil {
		status.Error = p.lastErr.Error()
	}

	switch {
	case !cfg.Enabled:
		status.Reason = "IrriSense integration disabled"
		return status
	case !cfg.IsConfigured():
		status.Reason = "IrriSense not configured (API token and garden required)"
		return status
	case p.sample == nil:
		status.Reason = "no IrriSense data yet"
		return status
	}

	now := p.now()
	fetchedAt := p.sample.fetchedAt
	status.FetchedAt = &fetchedAt
	status.GardenName = p.sample.garden.Name
	status.Zones = EvaluateZones(p.sample.garden, cfg.Wetness(), now)
	wet, reason := EvaluateWetness(p.sample.garden, cfg.Wetness(), now)

	age := now.Sub(fetchedAt)
	if age > cfg.MaxStale() {
		status.Reason = fmt.Sprintf("IrriSense data is %s old (stale after %.0f min); last known: %s",
			formatAgo(age), cfg.MaxStaleMinutes, reason)
		return status
	}

	status.Fresh = true
	status.Unknown = false
	status.Wet = wet
	status.Reason = reason
	return status
}

func copyIrriSenseConfig(cfg IrriSenseConfig) IrriSenseConfig {
	out := cfg
	out.ZoneIDs = append([]string{}, cfg.ZoneIDs...)
	return out
}

func maxDuration(a, b time.Duration) time.Duration {
	if a > b {
		return a
	}
	return b
}

func minDuration(a, b time.Duration) time.Duration {
	if a < b {
		return a
	}
	return b
}
