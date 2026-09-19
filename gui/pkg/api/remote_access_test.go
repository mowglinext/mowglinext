package api

import (
	"bytes"
	"context"
	"encoding/json"
	"errors"
	"net/http"
	"net/http/httptest"
	"testing"

	"github.com/gin-gonic/gin"
	"github.com/mowglinext/mowglinext/pkg/providers"
	"github.com/mowglinext/mowglinext/pkg/types"
	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
)

// remoteAccessFakeDocker never sees a container: enough for the settings
// round-trip, the status shape and the logout error path.
type remoteAccessFakeDocker struct{}

func (remoteAccessFakeDocker) ImagePull(context.Context, string) error { return nil }
func (remoteAccessFakeDocker) FindContainerByName(context.Context, string) (types.ContainerDetails, bool, error) {
	return types.ContainerDetails{}, false, nil
}
func (remoteAccessFakeDocker) ContainerCreateService(context.Context, types.ServiceContainerSpec) (string, error) {
	return "", errors.New("not expected")
}
func (remoteAccessFakeDocker) ContainerRemove(context.Context, string, bool) error { return nil }
func (remoteAccessFakeDocker) ContainerStart(context.Context, string) error        { return nil }
func (remoteAccessFakeDocker) ContainerStop(context.Context, string) error         { return nil }
func (remoteAccessFakeDocker) ContainerInspect(context.Context, string) (types.ContainerDetails, error) {
	return types.ContainerDetails{}, nil
}
func (remoteAccessFakeDocker) ContainerExec(context.Context, string, types.ContainerExecSpec) (types.ContainerExecResult, error) {
	return types.ContainerExecResult{}, nil
}

func setupRemoteAccessRouter(t *testing.T) (*gin.Engine, *providers.RemoteAccessProvider) {
	t.Helper()
	gin.SetMode(gin.TestMode)
	db := types.NewMockDBProvider()
	p := providers.NewIdleRemoteAccessProvider(db, remoteAccessFakeDocker{})
	r := gin.New()
	RemoteAccessRoutes(r.Group("/api"), p)
	return r, p
}

func remoteAccessRequest(t *testing.T, r *gin.Engine, method, path string, body any) *httptest.ResponseRecorder {
	t.Helper()
	var reader *bytes.Buffer
	if body != nil {
		raw, err := json.Marshal(body)
		require.NoError(t, err)
		reader = bytes.NewBuffer(raw)
	} else {
		reader = bytes.NewBuffer(nil)
	}
	req := httptest.NewRequest(method, path, reader)
	req.Header.Set("Content-Type", "application/json")
	w := httptest.NewRecorder()
	r.ServeHTTP(w, req)
	return w
}

func TestRemoteAccessSettingsDefaultsAndMaskedKey(t *testing.T) {
	r, _ := setupRemoteAccessRouter(t)

	w := remoteAccessRequest(t, r, http.MethodGet, "/api/remote-access/settings", nil)

	require.Equal(t, http.StatusOK, w.Code)
	var res RemoteAccessSettingsResponse
	require.NoError(t, json.Unmarshal(w.Body.Bytes(), &res))
	assert.False(t, res.Enabled)
	assert.Equal(t, providers.DefaultRemoteAccessHostname, res.Hostname)
	assert.False(t, res.AuthKeySet)
	assert.True(t, res.ServeHttps)
	assert.Equal(t, providers.DefaultRemoteAccessImage, res.Image)
	assert.Equal(t, providers.RemoteAccessContainerName, res.ContainerName)
}

func TestRemoteAccessPutIsPartialAndNeverEchoesTheKey(t *testing.T) {
	r, p := setupRemoteAccessRouter(t)

	w := remoteAccessRequest(t, r, http.MethodPut, "/api/remote-access/settings", map[string]any{
		"enabled": true, "hostname": "Lawn-Bot", "authKey": "tskey-auth-kABC123CNTRL-supersecretvalue",
	})

	require.Equal(t, http.StatusOK, w.Code)
	var res RemoteAccessSettingsResponse
	require.NoError(t, json.Unmarshal(w.Body.Bytes(), &res))
	assert.True(t, res.Enabled)
	assert.Equal(t, "lawn-bot", res.Hostname)
	assert.True(t, res.AuthKeySet)
	assert.NotContains(t, w.Body.String(), "supersecretvalue")
	assert.Equal(t, "tskey-auth-kABC123CNTRL-supersecretvalue", p.Config().AuthKey)

	// A second partial update keeps the key and the hostname.
	w = remoteAccessRequest(t, r, http.MethodPut, "/api/remote-access/settings", map[string]any{"serveHttps": false})
	require.Equal(t, http.StatusOK, w.Code)
	require.NoError(t, json.Unmarshal(w.Body.Bytes(), &res))
	assert.False(t, res.ServeHttps)
	assert.True(t, res.AuthKeySet)
	assert.Equal(t, "lawn-bot", res.Hostname)

	// clearAuthKey forgets it; an empty image string falls back to the default.
	w = remoteAccessRequest(t, r, http.MethodPut, "/api/remote-access/settings", map[string]any{"clearAuthKey": true, "image": ""})
	require.Equal(t, http.StatusOK, w.Code)
	require.NoError(t, json.Unmarshal(w.Body.Bytes(), &res))
	assert.False(t, res.AuthKeySet)
	assert.Equal(t, providers.DefaultRemoteAccessImage, res.Image)
}

func TestRemoteAccessPutRejectsAForeignImage(t *testing.T) {
	r, p := setupRemoteAccessRouter(t)

	w := remoteAccessRequest(t, r, http.MethodPut, "/api/remote-access/settings", map[string]any{"enabled": true, "image": "docker.io/attacker/evil:latest"})

	assert.Equal(t, http.StatusBadRequest, w.Code)
	assert.False(t, p.Config().Enabled)
	assert.Equal(t, providers.DefaultRemoteAccessImage, p.Config().Image)
}

func TestRemoteAccessPutRejectsAnInvalidHostname(t *testing.T) {
	r, p := setupRemoteAccessRouter(t)

	w := remoteAccessRequest(t, r, http.MethodPut, "/api/remote-access/settings", map[string]any{"hostname": "no spaces allowed"})

	assert.Equal(t, http.StatusBadRequest, w.Code)
	assert.Equal(t, providers.DefaultRemoteAccessHostname, p.Config().Hostname)
}

func TestRemoteAccessPutRejectsMalformedJSON(t *testing.T) {
	r, _ := setupRemoteAccessRouter(t)
	req := httptest.NewRequest(http.MethodPut, "/api/remote-access/settings", bytes.NewBufferString("{nope"))
	req.Header.Set("Content-Type", "application/json")
	w := httptest.NewRecorder()
	r.ServeHTTP(w, req)
	assert.Equal(t, http.StatusBadRequest, w.Code)
}

func TestRemoteAccessStatusRouteReturnsTheProviderView(t *testing.T) {
	r, _ := setupRemoteAccessRouter(t)

	w := remoteAccessRequest(t, r, http.MethodGet, "/api/remote-access/status", nil)

	require.Equal(t, http.StatusOK, w.Code)
	var st providers.RemoteAccessStatus
	require.NoError(t, json.Unmarshal(w.Body.Bytes(), &st))
	assert.False(t, st.Enabled)
	assert.Equal(t, providers.RemoteAccessDisabled, st.Phase)
	assert.NotNil(t, st.HttpUrls)
}

func TestRemoteAccessApplyAndLogoutRoutes(t *testing.T) {
	r, _ := setupRemoteAccessRouter(t)

	w := remoteAccessRequest(t, r, http.MethodPost, "/api/remote-access/apply", nil)
	assert.Equal(t, http.StatusOK, w.Code)

	// No sidecar exists, so logout must fail loudly rather than pretend.
	w = remoteAccessRequest(t, r, http.MethodPost, "/api/remote-access/logout", nil)
	assert.Equal(t, http.StatusBadRequest, w.Code)
	assert.Contains(t, w.Body.String(), "not running")
}

func TestApplyRemoteAccessUpdateDoesNotMutateItsInput(t *testing.T) {
	current := providers.DefaultRemoteAccessConfig()
	current.AuthKey = "tskey-auth-original"
	enabled := true
	hostname := "New-Name"

	next := applyRemoteAccessUpdate(current, RemoteAccessSettingsUpdate{Enabled: &enabled, Hostname: &hostname})

	assert.False(t, current.Enabled)
	assert.Equal(t, providers.DefaultRemoteAccessHostname, current.Hostname)
	assert.True(t, next.Enabled)
	assert.Equal(t, "new-name", next.Hostname)
	assert.Equal(t, "tskey-auth-original", next.AuthKey, "absent key keeps the stored one")

	blank := "   "
	next = applyRemoteAccessUpdate(current, RemoteAccessSettingsUpdate{AuthKey: &blank})
	assert.Equal(t, "tskey-auth-original", next.AuthKey, "a blank key is not a clear")
}
