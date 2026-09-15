package api

import (
	"encoding/json"
	"io"
	"net/http"
	"net/http/httptest"
	"sync"
	"testing"

	"github.com/gin-gonic/gin"
	"github.com/mowglinext/mowglinext/pkg/providers"
	"github.com/mowglinext/mowglinext/pkg/types"
	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
)

type ntfyCapture struct {
	mu    sync.Mutex
	paths []string
	auth  []string
	body  []string
}

func newFakeNtfy(t *testing.T, status int) (*httptest.Server, *ntfyCapture) {
	t.Helper()
	cap := &ntfyCapture{}
	srv := httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		body, _ := io.ReadAll(r.Body)
		cap.mu.Lock()
		cap.paths = append(cap.paths, r.URL.Path)
		cap.auth = append(cap.auth, r.Header.Get("Authorization"))
		cap.body = append(cap.body, string(body))
		cap.mu.Unlock()
		w.WriteHeader(status)
	}))
	t.Cleanup(srv.Close)
	return srv, cap
}

func newNotificationRouter(t *testing.T) (*gin.Engine, *providers.NotificationProvider) {
	t.Helper()
	gin.SetMode(gin.TestMode)
	p := providers.NewIdleNotificationProvider(types.NewMockDBProvider())
	r := gin.New()
	NotificationRoutes(r.Group("/api"), p)
	return r, p
}

func TestNotificationSettings_DefaultsAndEventKinds(t *testing.T) {
	r, _ := newNotificationRouter(t)
	w := doJSON(t, r, http.MethodGet, "/api/notifications/settings", nil)
	require.Equal(t, http.StatusOK, w.Code)
	var res NotificationSettingsResponse
	require.NoError(t, json.Unmarshal(w.Body.Bytes(), &res))
	assert.False(t, res.Enabled)
	assert.Equal(t, "telegram", res.Channel)
	assert.Equal(t, providers.NotifyChannels, res.Channels)
	assert.Equal(t, "https://ntfy.sh", res.NtfyServer)
	assert.Equal(t, providers.NotifyEventKinds, res.EventKinds)
	assert.True(t, res.Events["blocked"])
	assert.False(t, res.Events["mowStopped"])
}

func TestNotificationSettings_UpdateMasksSecretsAndRejectsHalfConfig(t *testing.T) {
	r, _ := newNotificationRouter(t)

	// Enabling Telegram without a token is refused, nothing is persisted.
	w := doJSON(t, r, http.MethodPut, "/api/notifications/settings", map[string]any{"enabled": true, "telegramChatId": "42"})
	require.Equal(t, http.StatusBadRequest, w.Code)

	w = doJSON(t, r, http.MethodPut, "/api/notifications/settings", map[string]any{
		"enabled": true, "channel": "ntfy", "ntfyTopic": "mowgli-garden", "ntfyToken": "tk_abcdefghijklmnop",
		"pushoverAppToken": "azGDORePK8gMaC0QOYAMyEEuzJnyUi", "pushoverUserKey": "uQiRzpo4DXghDmr9QzzfQu27cmVRsG",
		"title": "Jardin", "language": "fr",
		"events": map[string]bool{"mowStopped": true, "zoneStarted": false, "bogus": true},
	})
	require.Equal(t, http.StatusOK, w.Code, w.Body.String())
	var res NotificationSettingsResponse
	require.NoError(t, json.Unmarshal(w.Body.Bytes(), &res))
	assert.True(t, res.Enabled)
	assert.True(t, res.NtfyTokenSet)
	assert.Equal(t, "tk_a••••••••", res.NtfyTokenMasked)
	assert.NotContains(t, w.Body.String(), "tk_abcdefghijklmnop")
	assert.True(t, res.PushoverAppTokenSet)
	assert.Equal(t, "azGD••••••••", res.PushoverAppTokenMasked)
	assert.NotContains(t, w.Body.String(), "azGDORePK8gMaC0QOYAMyEEuzJnyUi")
	assert.Equal(t, "uQiRzpo4DXghDmr9QzzfQu27cmVRsG", res.PushoverUserKey)
	assert.Equal(t, "fr", res.Language)
	assert.True(t, res.Events["mowStopped"])
	assert.False(t, res.Events["zoneStarted"])
	_, hasBogus := res.Events["bogus"]
	assert.False(t, hasBogus, "unknown event kinds are dropped")

	// Clearing the token while enabled is fine for ntfy (token is optional).
	w = doJSON(t, r, http.MethodPut, "/api/notifications/settings", map[string]any{"clearNtfyToken": true})
	require.Equal(t, http.StatusOK, w.Code)
	require.NoError(t, json.Unmarshal(w.Body.Bytes(), &res))
	assert.False(t, res.NtfyTokenSet)
}

func TestNotificationTest_SendsWithStoredConfig(t *testing.T) {
	srv, cap := newFakeNtfy(t, http.StatusOK)
	r, _ := newNotificationRouter(t)

	w := doJSON(t, r, http.MethodPost, "/api/notifications/test", nil)
	assert.Equal(t, http.StatusBadRequest, w.Code, "unconfigured")

	w = doJSON(t, r, http.MethodPut, "/api/notifications/settings", map[string]any{
		"channel": "ntfy", "ntfyServer": srv.URL, "ntfyTopic": "garden", "ntfyToken": "tk_secret_value_1",
	})
	require.Equal(t, http.StatusOK, w.Code, w.Body.String())

	w = doJSON(t, r, http.MethodPost, "/api/notifications/test", nil)
	require.Equal(t, http.StatusOK, w.Code, w.Body.String())
	cap.mu.Lock()
	defer cap.mu.Unlock()
	require.Len(t, cap.paths, 1)
	assert.Equal(t, "/", cap.paths[0], "ntfy JSON publish: topic travels in the body")
	assert.Contains(t, cap.body[0], `"topic":"garden"`)
	assert.Equal(t, "Bearer tk_secret_value_1", cap.auth[0])
	assert.Contains(t, cap.body[0], "Test notification")

	w = doJSON(t, r, http.MethodGet, "/api/notifications/status", nil)
	require.Equal(t, http.StatusOK, w.Code)
	var status providers.NotifyDeliveryStatus
	require.NoError(t, json.Unmarshal(w.Body.Bytes(), &status))
	assert.Equal(t, 1, status.SentCount)
	assert.True(t, status.Configured)
	assert.False(t, status.Enabled)
}

func TestNotificationTest_UpstreamFailureIs502(t *testing.T) {
	srv, _ := newFakeNtfy(t, http.StatusForbidden)
	r, _ := newNotificationRouter(t)
	w := doJSON(t, r, http.MethodPut, "/api/notifications/settings", map[string]any{"channel": "ntfy", "ntfyServer": srv.URL, "ntfyTopic": "garden"})
	require.Equal(t, http.StatusOK, w.Code)
	w = doJSON(t, r, http.MethodPost, "/api/notifications/test", nil)
	assert.Equal(t, http.StatusBadGateway, w.Code)
	assert.Contains(t, w.Body.String(), "403")
}
