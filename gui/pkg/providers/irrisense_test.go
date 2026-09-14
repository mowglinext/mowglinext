package providers

import (
	"context"
	"encoding/json"
	"net/http"
	"net/http/httptest"
	"sync/atomic"
	"testing"
	"time"

	"github.com/mowglinext/mowglinext/pkg/types"
	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
)

const testToken = "irs_test_token_0123456789"

// fakeIrriSense is a minimal stand-in for irrisense-cloud's /api/ha routes:
// bearer auth, one garden, optional forced status.
type fakeIrriSense struct {
	srv         *httptest.Server
	garden      IrriSenseGarden
	requests    atomic.Int32
	forceStatus atomic.Int32
	retryAfter  string
}

func newFakeIrriSense(t *testing.T, garden IrriSenseGarden) *fakeIrriSense {
	t.Helper()
	f := &fakeIrriSense{garden: garden}
	mux := http.NewServeMux()
	handler := func(w http.ResponseWriter, r *http.Request, body any) {
		f.requests.Add(1)
		if r.Header.Get("Authorization") != "Bearer "+testToken {
			w.WriteHeader(http.StatusUnauthorized)
			return
		}
		if code := int(f.forceStatus.Load()); code != 0 {
			if code == http.StatusTooManyRequests && f.retryAfter != "" {
				w.Header().Set("Retry-After", f.retryAfter)
			}
			w.WriteHeader(code)
			return
		}
		w.Header().Set("Content-Type", "application/json")
		_ = json.NewEncoder(w).Encode(body)
	}
	mux.HandleFunc("/api/ha/gardens", func(w http.ResponseWriter, r *http.Request) {
		handler(w, r, []IrriSenseGarden{f.garden})
	})
	mux.HandleFunc("/api/ha/gardens/", func(w http.ResponseWriter, r *http.Request) {
		if r.URL.Path != "/api/ha/gardens/"+f.garden.ID {
			f.requests.Add(1)
			w.WriteHeader(http.StatusNotFound)
			return
		}
		handler(w, r, f.garden)
	})
	f.srv = httptest.NewServer(mux)
	t.Cleanup(f.srv.Close)
	return f
}

func testGarden() IrriSenseGarden {
	watered := time.Date(2026, 9, 3, 7, 20, 0, 0, time.UTC)
	return IrriSenseGarden{
		ID:   "garden-1",
		Name: "Jardin",
		Zones: []IrriSenseZone{
			{ID: "z1", Label: "Pelouse nord", Enabled: true, DeficitMm: 0.8, LastWateredAt: &watered},
			{ID: "z2", Label: "Pelouse sud", Enabled: true, DeficitMm: 5.2},
		},
	}
}

type fakeClock struct{ t time.Time }

func (c *fakeClock) now() time.Time          { return c.t }
func (c *fakeClock) advance(d time.Duration) { c.t = c.t.Add(d) }

func enabledConfig(baseURL string) IrriSenseConfig {
	cfg := DefaultIrriSenseConfig()
	cfg.Enabled = true
	cfg.BaseURL = baseURL
	cfg.Token = testToken
	cfg.GardenID = "garden-1"
	return cfg
}

func buildProvider(t *testing.T, cfg IrriSenseConfig) (*IrriSenseProvider, *fakeClock, *types.MockDBProvider) {
	t.Helper()
	db := types.NewMockDBProvider()
	require.NoError(t, SaveIrriSenseConfig(db, cfg))
	clock := &fakeClock{t: time.Date(2026, 9, 3, 8, 0, 0, 0, time.UTC)}
	p := newIrriSenseProvider(db, nil, clock.now)
	return p, clock, db
}

func TestIrriSense_RefreshReportsWetAndFresh(t *testing.T) {
	fake := newFakeIrriSense(t, testGarden())
	p, _, _ := buildProvider(t, enabledConfig(fake.srv.URL))

	p.Refresh()
	status := p.SoilStatus()

	assert.True(t, status.Enabled)
	assert.True(t, status.Configured)
	assert.True(t, status.Fresh)
	assert.True(t, status.Wet)
	assert.False(t, status.Unknown)
	assert.True(t, status.BlocksScheduledMowing())
	assert.Equal(t, `zone "Pelouse nord": deficit 0.8 mm ≤ 2.0 mm, watered 40 min ago`, status.Reason)
	assert.Equal(t, "Jardin", status.GardenName)
	require.Len(t, status.Zones, 2)
	assert.True(t, status.Zones[0].Wet)
	assert.False(t, status.Zones[1].Wet)
	assert.Empty(t, status.Error)
	assert.Equal(t, int32(1), fake.requests.Load())
}

func TestIrriSense_DisabledMakesNoRequestAndIsUnknown(t *testing.T) {
	fake := newFakeIrriSense(t, testGarden())
	cfg := enabledConfig(fake.srv.URL)
	cfg.Enabled = false
	p, _, _ := buildProvider(t, cfg)

	p.Refresh()
	status := p.SoilStatus()

	assert.False(t, status.Enabled)
	assert.True(t, status.Unknown)
	assert.False(t, status.BlocksScheduledMowing())
	assert.Equal(t, int32(0), fake.requests.Load())
}

func TestIrriSense_MissingTokenIsNotConfigured(t *testing.T) {
	fake := newFakeIrriSense(t, testGarden())
	cfg := enabledConfig(fake.srv.URL)
	cfg.Token = ""
	p, _, _ := buildProvider(t, cfg)

	p.Refresh()
	status := p.SoilStatus()

	assert.True(t, status.Enabled)
	assert.False(t, status.Configured)
	assert.True(t, status.Unknown)
	assert.False(t, status.BlocksScheduledMowing())
	assert.Equal(t, int32(0), fake.requests.Load())
}

func TestIrriSense_UnauthorizedIsUnknownAndFailOpen(t *testing.T) {
	fake := newFakeIrriSense(t, testGarden())
	cfg := enabledConfig(fake.srv.URL)
	cfg.Token = "wrong-token-but-long-enough"
	p, _, _ := buildProvider(t, cfg)

	p.Refresh()
	status := p.SoilStatus()

	assert.True(t, status.Unknown)
	assert.False(t, status.Fresh)
	assert.False(t, status.BlocksScheduledMowing())
	assert.Contains(t, status.Error, "401")
	assert.NotContains(t, status.Error, "wrong-token", "the token must never leak into a status message")
}

func TestIrriSense_StaleSampleDegradesToUnknown(t *testing.T) {
	fake := newFakeIrriSense(t, testGarden())
	cfg := enabledConfig(fake.srv.URL)
	cfg.MaxStaleMinutes = 90
	p, clock, _ := buildProvider(t, cfg)

	p.Refresh()
	require.True(t, p.SoilStatus().Fresh)

	// Service goes dark; the sample keeps serving while inside the window...
	fake.forceStatus.Store(http.StatusInternalServerError)
	clock.advance(60 * time.Minute)
	p.Refresh()
	status := p.SoilStatus()
	assert.True(t, status.Fresh, "a 60 min old sample is inside the 90 min window")
	assert.Contains(t, status.Error, "500")
	assert.NotNil(t, status.FetchedAt)

	// ...and degrades to Unknown once it is older than the window.
	clock.advance(45 * time.Minute)
	status = p.SoilStatus()
	assert.False(t, status.Fresh)
	assert.True(t, status.Unknown)
	assert.False(t, status.BlocksScheduledMowing())
	assert.Contains(t, status.Reason, "stale")
}

func TestIrriSense_RateLimitBacksOffUntilRetryAfter(t *testing.T) {
	fake := newFakeIrriSense(t, testGarden())
	fake.retryAfter = "300"
	fake.forceStatus.Store(http.StatusTooManyRequests)
	p, clock, _ := buildProvider(t, enabledConfig(fake.srv.URL))

	p.Refresh()
	require.Equal(t, int32(1), fake.requests.Load())
	assert.Contains(t, p.SoilStatus().Error, "429")

	// Inside the Retry-After window nothing is sent.
	clock.advance(2 * time.Minute)
	p.Refresh()
	assert.Equal(t, int32(1), fake.requests.Load(), "must honour Retry-After")

	// After it, the poll resumes and a success clears the backoff.
	fake.forceStatus.Store(0)
	clock.advance(4 * time.Minute)
	p.Refresh()
	assert.Equal(t, int32(2), fake.requests.Load())
	assert.True(t, p.SoilStatus().Fresh)
	assert.Empty(t, p.SoilStatus().Error)
}

func TestIrriSense_GenericFailureBacksOffExponentially(t *testing.T) {
	fake := newFakeIrriSense(t, testGarden())
	fake.forceStatus.Store(http.StatusBadGateway)
	p, clock, _ := buildProvider(t, enabledConfig(fake.srv.URL))

	p.Refresh() // 1st failure → 1 min
	clock.advance(30 * time.Second)
	p.Refresh() // skipped
	assert.Equal(t, int32(1), fake.requests.Load())

	clock.advance(31 * time.Second)
	p.Refresh() // 2nd failure → 2 min
	assert.Equal(t, int32(2), fake.requests.Load())

	clock.advance(90 * time.Second)
	p.Refresh() // still inside the 2 min backoff
	assert.Equal(t, int32(2), fake.requests.Load())
}

func TestIrriSense_UpdateConfigPersistsAndResetsSample(t *testing.T) {
	fake := newFakeIrriSense(t, testGarden())
	p, _, db := buildProvider(t, enabledConfig(fake.srv.URL))
	p.Refresh()
	require.True(t, p.SoilStatus().Fresh)

	next := p.Config()
	next.GardenID = "garden-2"
	next.ZoneIDs = []string{"z1"}
	next.WetDeficitMm = 1.5
	require.NoError(t, p.UpdateConfig(next))

	// A different garden invalidates the cached sample.
	assert.True(t, p.SoilStatus().Unknown)
	assert.Equal(t, "no IrriSense data yet", p.SoilStatus().Reason)

	loaded := LoadIrriSenseConfig(db)
	assert.Equal(t, "garden-2", loaded.GardenID)
	assert.Equal(t, []string{"z1"}, loaded.ZoneIDs)
	assert.Equal(t, 1.5, loaded.WetDeficitMm)
	assert.Equal(t, testToken, loaded.Token)

	// A refresh request was queued.
	select {
	case <-p.wake:
	default:
		t.Fatal("UpdateConfig must request an immediate refresh")
	}
}

func TestIrriSense_UpdateConfigRejectsBadURL(t *testing.T) {
	p, _, _ := buildProvider(t, DefaultIrriSenseConfig())
	cfg := p.Config()
	cfg.BaseURL = "ftp://nope"
	assert.Error(t, p.UpdateConfig(cfg))
	cfg.BaseURL = "https://irrisense-cloud.fly.dev/?x=1"
	assert.Error(t, p.UpdateConfig(cfg))
	cfg.BaseURL = "https://irrisense-cloud.fly.dev/"
	assert.NoError(t, p.UpdateConfig(cfg))
}

func TestIrriSense_ClearingTokenDeletesTheKey(t *testing.T) {
	fake := newFakeIrriSense(t, testGarden())
	p, _, db := buildProvider(t, enabledConfig(fake.srv.URL))

	cfg := p.Config()
	cfg.Token = ""
	require.NoError(t, p.UpdateConfig(cfg))

	_, err := db.Get(irriSenseKeyToken)
	assert.Error(t, err, "token key must be deleted, not written empty")
	assert.False(t, p.SoilStatus().Configured)
}

func TestIrriSense_MaskedTokenNeverRevealsTheSecret(t *testing.T) {
	cfg := IrriSenseConfig{Token: testToken}
	masked := cfg.MaskedToken()
	assert.Equal(t, "irs_••••••••", masked)
	assert.NotContains(t, masked, "0123456789")
	assert.Equal(t, "••••••••", IrriSenseConfig{Token: "short"}.MaskedToken())
	assert.Equal(t, "", IrriSenseConfig{}.MaskedToken())
}

func TestIrriSense_ListGardens(t *testing.T) {
	fake := newFakeIrriSense(t, testGarden())
	p, _, _ := buildProvider(t, enabledConfig(fake.srv.URL))

	gardens, err := p.ListGardens(context.Background())
	require.NoError(t, err)
	require.Len(t, gardens, 1)
	assert.Equal(t, "garden-1", gardens[0].ID)
	assert.Equal(t, "Jardin", gardens[0].Name)
	require.Len(t, gardens[0].Zones, 2)
	assert.Equal(t, "Pelouse nord", gardens[0].Zones[0].Label)
}

func TestIrriSense_ListGardensUnauthorized(t *testing.T) {
	fake := newFakeIrriSense(t, testGarden())
	cfg := enabledConfig(fake.srv.URL)
	cfg.Token = "not-the-right-token-at-all"
	p, _, _ := buildProvider(t, cfg)

	_, err := p.ListGardens(context.Background())
	assert.ErrorIs(t, err, ErrIrriSenseUnauthorized)
}

func TestLoadIrriSenseConfig_DefaultsOnEmptyDB(t *testing.T) {
	cfg := LoadIrriSenseConfig(types.NewMockDBProvider())
	assert.Equal(t, DefaultIrriSenseConfig(), cfg)
	assert.False(t, cfg.IsConfigured())
}

func TestLoadIrriSenseConfig_ToleratesGarbage(t *testing.T) {
	db := types.NewMockDBProvider()
	require.NoError(t, db.Set(irriSenseKeyEnabled, []byte("maybe")))
	require.NoError(t, db.Set(irriSenseKeyWetDeficitMm, []byte("wet")))
	require.NoError(t, db.Set(irriSenseKeyZoneIDs, []byte("{not a list")))

	cfg := LoadIrriSenseConfig(db)
	assert.False(t, cfg.Enabled)
	assert.Equal(t, DefaultIrriSenseWetDeficitMm, cfg.WetDeficitMm)
	assert.Equal(t, []string{}, cfg.ZoneIDs)
}
