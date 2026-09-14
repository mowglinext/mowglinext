package api

import (
	"bytes"
	"encoding/json"
	"net/http"
	"net/http/httptest"
	"testing"

	"github.com/gin-gonic/gin"
	"github.com/mowglinext/mowglinext/pkg/providers"
	"github.com/mowglinext/mowglinext/pkg/types"
	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
)

const irriSenseTestToken = "irs_api_token_abcdef0123456789"

// newFakeIrriSenseCloud mimics irrisense-cloud's /api/ha/gardens routes: one
// garden with a soaking zone, bearer-token auth, 404 for any other garden.
func newFakeIrriSenseCloud(t *testing.T) *httptest.Server {
	t.Helper()
	garden := map[string]any{
		"id": "garden-1", "name": "Jardin", "mode": "auto", "timezone": "Europe/Paris",
		"latitude": 48.8, "longitude": 2.3, "worst_deficit_mm": 5.0,
		"zones": []map[string]any{
			{"id": "z1", "label": "Pelouse nord", "mode": "auto", "enabled": true,
				"deficit_mm": 0.4, "trigger_mm": 8, "dose_mm": 6, "readiness": 0.05,
				"last_watered_at": nil, "device_serial": "AB12", "is_online": true},
			{"id": "z2", "label": "Potager", "mode": "auto", "enabled": false,
				"deficit_mm": 5.0, "trigger_mm": 8, "dose_mm": 6, "readiness": 0.6,
				"last_watered_at": nil, "device_serial": "AB12", "is_online": nil},
		},
		"devices": []any{},
	}
	mux := http.NewServeMux()
	auth := func(w http.ResponseWriter, r *http.Request) bool {
		if r.Header.Get("Authorization") != "Bearer "+irriSenseTestToken {
			w.WriteHeader(http.StatusUnauthorized)
			return false
		}
		return true
	}
	mux.HandleFunc("/api/ha/gardens", func(w http.ResponseWriter, r *http.Request) {
		if !auth(w, r) {
			return
		}
		_ = json.NewEncoder(w).Encode([]any{garden})
	})
	mux.HandleFunc("/api/ha/gardens/garden-1", func(w http.ResponseWriter, r *http.Request) {
		if !auth(w, r) {
			return
		}
		_ = json.NewEncoder(w).Encode(garden)
	})
	srv := httptest.NewServer(mux)
	t.Cleanup(srv.Close)
	return srv
}

func setupIrriSenseRouter(t *testing.T) (*gin.Engine, *providers.IrriSenseProvider, *types.MockDBProvider) {
	t.Helper()
	gin.SetMode(gin.TestMode)
	db := types.NewMockDBProvider()
	p := providers.NewIdleIrriSenseProvider(db)
	r := gin.New()
	IrriSenseRoutes(r.Group("/api"), p)
	return r, p, db
}

func doJSON(t *testing.T, r *gin.Engine, method, path string, body any) *httptest.ResponseRecorder {
	t.Helper()
	var buf bytes.Buffer
	switch b := body.(type) {
	case nil:
	case string:
		buf.WriteString(b) // raw payload, deliberately not encoded
	default:
		require.NoError(t, json.NewEncoder(&buf).Encode(body))
	}
	req, err := http.NewRequest(method, path, &buf)
	require.NoError(t, err)
	req.Header.Set("Content-Type", "application/json")
	w := httptest.NewRecorder()
	r.ServeHTTP(w, req)
	return w
}

func TestIrriSenseSettings_DefaultsOnFreshInstall(t *testing.T) {
	r, _, _ := setupIrriSenseRouter(t)

	w := doJSON(t, r, http.MethodGet, "/api/irrisense/settings", nil)
	require.Equal(t, http.StatusOK, w.Code)

	var resp IrriSenseSettingsResponse
	require.NoError(t, json.Unmarshal(w.Body.Bytes(), &resp))
	assert.False(t, resp.Enabled)
	assert.Equal(t, providers.DefaultIrriSenseBaseURL, resp.BaseUrl)
	assert.False(t, resp.TokenSet)
	assert.Equal(t, "", resp.TokenMasked)
	assert.Equal(t, []string{}, resp.ZoneIds)
	assert.Equal(t, 2.0, resp.WetDeficitMm)
	assert.Equal(t, 3.0, resp.DryAfterWateringHours)
	assert.Equal(t, 90.0, resp.MaxStaleMinutes)
	assert.True(t, resp.GateScheduler)
}

func TestIrriSenseSettings_PutStoresTokenAndMasksIt(t *testing.T) {
	cloud := newFakeIrriSenseCloud(t)
	r, _, db := setupIrriSenseRouter(t)

	w := doJSON(t, r, http.MethodPut, "/api/irrisense/settings", map[string]any{
		"enabled":  true,
		"baseUrl":  cloud.URL,
		"token":    irriSenseTestToken,
		"gardenId": "garden-1",
		"zoneIds":  []string{" z1 ", ""},
	})
	require.Equal(t, http.StatusOK, w.Code, w.Body.String())

	var resp IrriSenseSettingsResponse
	require.NoError(t, json.Unmarshal(w.Body.Bytes(), &resp))
	assert.True(t, resp.Enabled)
	assert.True(t, resp.TokenSet)
	assert.Equal(t, "irs_••••••••", resp.TokenMasked)
	assert.Equal(t, []string{"z1"}, resp.ZoneIds)
	assert.NotContains(t, w.Body.String(), irriSenseTestToken, "the token must never be echoed")

	stored, err := db.Get("irrisense.token")
	require.NoError(t, err)
	assert.Equal(t, irriSenseTestToken, string(stored))
}

func TestIrriSenseSettings_PutWithoutTokenKeepsTheStoredOne(t *testing.T) {
	cloud := newFakeIrriSenseCloud(t)
	r, _, db := setupIrriSenseRouter(t)
	w := doJSON(t, r, http.MethodPut, "/api/irrisense/settings", map[string]any{"token": irriSenseTestToken})
	require.Equal(t, http.StatusOK, w.Code)

	// A second save from a form that never held the token (write-only field)
	// must not wipe it.
	w = doJSON(t, r, http.MethodPut, "/api/irrisense/settings", map[string]any{
		"baseUrl": cloud.URL, "wetDeficitMm": 1.0,
	})
	require.Equal(t, http.StatusOK, w.Code)

	stored, err := db.Get("irrisense.token")
	require.NoError(t, err)
	assert.Equal(t, irriSenseTestToken, string(stored))

	var resp IrriSenseSettingsResponse
	require.NoError(t, json.Unmarshal(w.Body.Bytes(), &resp))
	assert.Equal(t, 1.0, resp.WetDeficitMm)
	assert.True(t, resp.TokenSet)
}

func TestIrriSenseSettings_ClearTokenForgetsIt(t *testing.T) {
	r, _, db := setupIrriSenseRouter(t)
	w := doJSON(t, r, http.MethodPut, "/api/irrisense/settings", map[string]any{"token": irriSenseTestToken})
	require.Equal(t, http.StatusOK, w.Code)

	w = doJSON(t, r, http.MethodPut, "/api/irrisense/settings", map[string]any{"clearToken": true})
	require.Equal(t, http.StatusOK, w.Code)

	_, err := db.Get("irrisense.token")
	assert.Error(t, err)
	var resp IrriSenseSettingsResponse
	require.NoError(t, json.Unmarshal(w.Body.Bytes(), &resp))
	assert.False(t, resp.TokenSet)
}

func TestIrriSenseSettings_RejectsInvalidUrlAndThresholds(t *testing.T) {
	r, _, _ := setupIrriSenseRouter(t)

	w := doJSON(t, r, http.MethodPut, "/api/irrisense/settings", map[string]any{"baseUrl": "irrisense-cloud.fly.dev"})
	assert.Equal(t, http.StatusBadRequest, w.Code)
	assert.Contains(t, w.Body.String(), "http")

	w = doJSON(t, r, http.MethodPut, "/api/irrisense/settings", map[string]any{"maxStaleMinutes": 0})
	assert.Equal(t, http.StatusBadRequest, w.Code)

	w = doJSON(t, r, http.MethodPut, "/api/irrisense/settings", map[string]any{"wetDeficitMm": -1})
	assert.Equal(t, http.StatusBadRequest, w.Code)

	w = doJSON(t, r, http.MethodPut, "/api/irrisense/settings", "{not json")
	assert.Equal(t, http.StatusBadRequest, w.Code)
}

func TestIrriSenseStatus_UnknownUntilConfiguredThenWet(t *testing.T) {
	cloud := newFakeIrriSenseCloud(t)
	r, p, _ := setupIrriSenseRouter(t)

	w := doJSON(t, r, http.MethodGet, "/api/irrisense/status", nil)
	require.Equal(t, http.StatusOK, w.Code)
	var status types.SoilStatus
	require.NoError(t, json.Unmarshal(w.Body.Bytes(), &status))
	assert.True(t, status.Unknown)
	assert.False(t, status.Enabled)
	assert.NotNil(t, status.Zones, "zones must serialise as [] not null")

	w = doJSON(t, r, http.MethodPut, "/api/irrisense/settings", map[string]any{
		"enabled": true, "baseUrl": cloud.URL, "token": irriSenseTestToken, "gardenId": "garden-1",
	})
	require.Equal(t, http.StatusOK, w.Code)

	// The poll loop is not running in tests; drive one refresh by hand.
	p.Refresh()

	w = doJSON(t, r, http.MethodGet, "/api/irrisense/status", nil)
	require.Equal(t, http.StatusOK, w.Code)
	require.NoError(t, json.Unmarshal(w.Body.Bytes(), &status))
	assert.True(t, status.Fresh)
	assert.True(t, status.Wet)
	assert.False(t, status.Unknown)
	assert.Equal(t, "Jardin", status.GardenName)
	assert.Contains(t, status.Reason, `zone "Pelouse nord"`)
	assert.NotContains(t, w.Body.String(), irriSenseTestToken)
	require.Len(t, status.Zones, 2)
	assert.False(t, status.Zones[1].Wet, "disabled zone must not count")
}

func TestIrriSenseGardens_ListsWithStoredToken(t *testing.T) {
	cloud := newFakeIrriSenseCloud(t)
	r, _, _ := setupIrriSenseRouter(t)
	w := doJSON(t, r, http.MethodPut, "/api/irrisense/settings", map[string]any{"baseUrl": cloud.URL, "token": irriSenseTestToken})
	require.Equal(t, http.StatusOK, w.Code)

	w = doJSON(t, r, http.MethodGet, "/api/irrisense/gardens", nil)
	require.Equal(t, http.StatusOK, w.Code, w.Body.String())

	var resp IrriSenseGardensResponse
	require.NoError(t, json.Unmarshal(w.Body.Bytes(), &resp))
	require.Len(t, resp.Gardens, 1)
	assert.Equal(t, "garden-1", resp.Gardens[0].ID)
	assert.Equal(t, "Jardin", resp.Gardens[0].Name)
	require.Len(t, resp.Gardens[0].Zones, 2)
	assert.Equal(t, "Potager", resp.Gardens[0].Zones[1].Label)
	assert.False(t, resp.Gardens[0].Zones[1].Enabled)
}

func TestIrriSenseGardens_UnauthorizedIsAClearError(t *testing.T) {
	cloud := newFakeIrriSenseCloud(t)
	r, _, _ := setupIrriSenseRouter(t)
	w := doJSON(t, r, http.MethodPut, "/api/irrisense/settings", map[string]any{"baseUrl": cloud.URL, "token": "a-login-jwt-is-not-an-api-token"})
	require.Equal(t, http.StatusOK, w.Code)

	w = doJSON(t, r, http.MethodGet, "/api/irrisense/gardens", nil)
	assert.Equal(t, http.StatusBadRequest, w.Code)

	var resp IrriSenseErrorResponse
	require.NoError(t, json.Unmarshal(w.Body.Bytes(), &resp))
	assert.Equal(t, "unauthorized", resp.Code)
	assert.Contains(t, resp.Error, "401")
	assert.Contains(t, resp.Error, "read-only API token")
}

func TestIrriSenseGardens_NoTokenIsAClearError(t *testing.T) {
	r, _, _ := setupIrriSenseRouter(t)

	w := doJSON(t, r, http.MethodGet, "/api/irrisense/gardens", nil)
	assert.Equal(t, http.StatusBadRequest, w.Code)
	var resp IrriSenseErrorResponse
	require.NoError(t, json.Unmarshal(w.Body.Bytes(), &resp))
	assert.Equal(t, "no_token", resp.Code)
}

func TestIrriSenseGardens_UpstreamDownIsBadGateway(t *testing.T) {
	down := httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, _ *http.Request) {
		w.WriteHeader(http.StatusServiceUnavailable)
	}))
	t.Cleanup(down.Close)
	r, _, _ := setupIrriSenseRouter(t)
	w := doJSON(t, r, http.MethodPut, "/api/irrisense/settings", map[string]any{"baseUrl": down.URL, "token": irriSenseTestToken})
	require.Equal(t, http.StatusOK, w.Code)

	w = doJSON(t, r, http.MethodGet, "/api/irrisense/gardens", nil)
	assert.Equal(t, http.StatusBadGateway, w.Code)
	var resp IrriSenseErrorResponse
	require.NoError(t, json.Unmarshal(w.Body.Bytes(), &resp))
	assert.Equal(t, "upstream", resp.Code)
}

func TestApplyIrriSenseUpdate_DoesNotMutateInput(t *testing.T) {
	current := providers.DefaultIrriSenseConfig()
	current.ZoneIDs = []string{"a"}
	enabled := true
	zones := []string{"b"}

	next := applyIrriSenseUpdate(current, IrriSenseSettingsUpdate{Enabled: &enabled, ZoneIds: &zones})

	assert.False(t, current.Enabled)
	assert.Equal(t, []string{"a"}, current.ZoneIDs)
	assert.True(t, next.Enabled)
	assert.Equal(t, []string{"b"}, next.ZoneIDs)
}
