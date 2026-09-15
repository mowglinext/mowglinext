package providers

import (
	"context"
	"encoding/json"
	"io"
	"net/http"
	"net/http/httptest"
	"net/url"
	"strings"
	"sync"
	"testing"
	"time"

	"github.com/mowglinext/mowglinext/pkg/types"
	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
)

type capturedRequest struct {
	Path    string
	Headers http.Header
	Body    string
}

// newCaptureServer records every request and answers with the given status.
func newCaptureServer(t *testing.T, status int) (*httptest.Server, func() []capturedRequest) {
	t.Helper()
	var mu sync.Mutex
	var got []capturedRequest
	srv := httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		body, _ := io.ReadAll(r.Body)
		mu.Lock()
		got = append(got, capturedRequest{Path: r.URL.Path, Headers: r.Header.Clone(), Body: string(body)})
		mu.Unlock()
		w.WriteHeader(status)
		_, _ = w.Write([]byte(`{"error":"nope"}`))
	}))
	t.Cleanup(srv.Close)
	return srv, func() []capturedRequest {
		mu.Lock()
		defer mu.Unlock()
		return append([]capturedRequest(nil), got...)
	}
}

func testMessage() NotifyMessage {
	return NotifyMessage{Title: "Mowgli", Body: "Mowing started.", Kind: NotifyEventMowStarted, Message: NotifyMsgMowStarted, Priority: 3, Tags: []string{"seedling"}, Params: map[string]string{}, At: t0}
}

// redirectTransport rewrites every request to the fake server, so a library
// with a hardcoded API host (Telegram) can be exercised offline.
type redirectTransport struct {
	target *url.URL
	next   http.RoundTripper
}

func (r redirectTransport) RoundTrip(req *http.Request) (*http.Response, error) {
	clone := req.Clone(req.Context())
	clone.URL.Scheme = r.target.Scheme
	clone.URL.Host = r.target.Host
	clone.Host = r.target.Host
	return r.next.RoundTrip(clone)
}

func redirectingClient(t *testing.T, srv *httptest.Server) *http.Client {
	t.Helper()
	target, err := url.Parse(srv.URL)
	require.NoError(t, err)
	return &http.Client{Transport: redirectTransport{target: target, next: srv.Client().Transport}}
}

func ntfySender(t *testing.T, srv *httptest.Server, token string) notifySender {
	t.Helper()
	cfg := DefaultNotificationConfig()
	cfg.Channel = NotifyChannelNtfy
	cfg.NtfyServer = srv.URL
	cfg.NtfyTopic = "mowgli-garden"
	cfg.NtfyToken = token
	s, err := newNotifySender(cfg, srv.Client())
	require.NoError(t, err)
	return s
}

func TestNtfySender_JSONPublishWithToken(t *testing.T) {
	srv, requests := newCaptureServer(t, http.StatusOK)
	s := ntfySender(t, srv, "tk_secret")

	require.NoError(t, s.Send(context.Background(), testMessage()))

	got := requests()
	require.Len(t, got, 1)
	assert.Equal(t, "/", got[0].Path, "ntfy JSON publish goes to the server root, topic in the body")
	assert.Equal(t, "Bearer tk_secret", got[0].Headers.Get("Authorization"))
	var payload map[string]any
	require.NoError(t, json.Unmarshal([]byte(got[0].Body), &payload))
	assert.Equal(t, "mowgli-garden", payload["topic"])
	assert.Equal(t, "Mowgli", payload["title"])
	assert.Equal(t, "Mowing started.", payload["message"])
	assert.Equal(t, float64(3), payload["priority"])
	assert.Equal(t, []any{"seedling"}, payload["tags"])
}

func TestNtfySender_NoTokenNoAuthHeader(t *testing.T) {
	srv, requests := newCaptureServer(t, http.StatusOK)
	s := ntfySender(t, srv, "")
	require.NoError(t, s.Send(context.Background(), testMessage()))
	assert.Empty(t, requests()[0].Headers.Get("Authorization"))
}

// newFakeTelegramAPI answers getMe (the bot handshake tgbotapi performs at
// construction) and records sendMessage form posts.
func newFakeTelegramAPI(t *testing.T) (*httptest.Server, func() []capturedRequest) {
	t.Helper()
	var mu sync.Mutex
	var got []capturedRequest
	srv := httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		w.Header().Set("Content-Type", "application/json")
		if strings.HasSuffix(r.URL.Path, "/getMe") {
			_, _ = w.Write([]byte(`{"ok":true,"result":{"id":1,"is_bot":true,"first_name":"Mowgli","user_name":"mowgli_bot"}}`))
			return
		}
		require.NoError(t, r.ParseForm())
		mu.Lock()
		got = append(got, capturedRequest{Path: r.URL.Path, Headers: r.Header.Clone(), Body: r.Form.Encode()})
		mu.Unlock()
		_, _ = w.Write([]byte(`{"ok":true,"result":{"message_id":1}}`))
	}))
	t.Cleanup(srv.Close)
	return srv, func() []capturedRequest {
		mu.Lock()
		defer mu.Unlock()
		return append([]capturedRequest(nil), got...)
	}
}

func TestTelegramSender_EscapesHTMLAndTargetsChat(t *testing.T) {
	srv, requests := newFakeTelegramAPI(t)
	cfg := DefaultNotificationConfig()
	cfg.Channel = NotifyChannelTelegram
	cfg.TelegramBotToken = "123456:abcdef"
	cfg.TelegramChatID = "-100424242"
	s, err := newNotifySender(cfg, redirectingClient(t, srv))
	require.NoError(t, err)
	msg := testMessage()
	msg.Body = "Stuck <NAV_TO_DOCK_FAILED> & more"

	require.NoError(t, s.Send(context.Background(), msg))
	require.NoError(t, s.Send(context.Background(), msg), "second send reuses the handshaken bot")

	got := requests()
	require.Len(t, got, 2)
	assert.Equal(t, "/bot123456:abcdef/sendMessage", got[0].Path)
	form, err := url.ParseQuery(got[0].Body)
	require.NoError(t, err)
	assert.Equal(t, "-100424242", form.Get("chat_id"))
	assert.Equal(t, "HTML", form.Get("parse_mode"))
	assert.Equal(t, "<b>Mowgli</b>\nStuck &lt;NAV_TO_DOCK_FAILED&gt; &amp; more", form.Get("text"))
}

func TestTelegramSender_BadChatIDFailsBeforeAnyRequest(t *testing.T) {
	srv, requests := newFakeTelegramAPI(t)
	cfg := DefaultNotificationConfig()
	cfg.Channel = NotifyChannelTelegram
	cfg.TelegramBotToken = "123456:abcdef"
	cfg.TelegramChatID = "@mychannel"
	s, err := newNotifySender(cfg, redirectingClient(t, srv))
	require.NoError(t, err)
	err = s.Send(context.Background(), testMessage())
	require.Error(t, err)
	assert.Contains(t, err.Error(), "not a number")
	assert.Empty(t, requests())
}

func TestWebhookSender_PostsWholeMessage(t *testing.T) {
	srv, requests := newCaptureServer(t, http.StatusNoContent)
	cfg := DefaultNotificationConfig()
	cfg.Channel = NotifyChannelWebhook
	cfg.WebhookURL = srv.URL + "/hook"
	s, err := newNotifySender(cfg, srv.Client())
	require.NoError(t, err)
	require.NoError(t, s.Send(context.Background(), testMessage()))
	got := requests()
	require.Len(t, got, 1)
	assert.Equal(t, "/hook", got[0].Path)
	assert.Contains(t, got[0].Headers.Get("Content-Type"), "application/json")
	var payload NotifyMessage
	require.NoError(t, json.Unmarshal([]byte(got[0].Body), &payload))
	assert.Equal(t, NotifyMsgMowStarted, payload.Message)
	assert.Equal(t, "Mowgli", payload.Title)
	assert.Equal(t, "Mowing started.", payload.Body)
	assert.Equal(t, []string{"seedling"}, payload.Tags)
}

func TestPushoverSender_WiresAppTokenAndUserKey(t *testing.T) {
	cfg := DefaultNotificationConfig()
	cfg.Channel = NotifyChannelPushover
	cfg.PushoverAppToken = "azGDORePK8gMaC0QOYAMyEEuzJnyUi"
	cfg.PushoverUserKey = "uQiRzpo4DXghDmr9QzzfQu27cmVRsG"
	s, err := newNotifySender(cfg, nil)
	require.NoError(t, err)
	assert.True(t, cfg.IsConfigured())
	// The Pushover client has a fixed API host and no injection point, so the
	// wire format is the library's; only the routing is ours to check.
	_, isLib := s.(*libSender)
	assert.True(t, isLib)
}

func TestSenders_NonSuccessIsAnError(t *testing.T) {
	srv, _ := newCaptureServer(t, http.StatusForbidden)
	s := ntfySender(t, srv, "")
	err := s.Send(context.Background(), testMessage())
	require.Error(t, err)
	assert.Contains(t, err.Error(), "403")
}

func TestNotificationConfig_RoundTripAndSecrets(t *testing.T) {
	db := types.NewMockDBProvider()
	cfg := DefaultNotificationConfig()
	cfg.Enabled = true
	cfg.Channel = NotifyChannelTelegram
	cfg.TelegramBotToken = "123456:abcdef"
	cfg.TelegramChatID = "42"
	cfg.PushoverAppToken = "azGDORePK8gMaC0QOYAMyEEuzJnyUi"
	cfg.PushoverUserKey = "uQiRzpo4DXghDmr9QzzfQu27cmVRsG"
	cfg.Language = "fr"
	cfg.Events[NotifyEventZoneStarted] = false
	require.NoError(t, cfg.Validate())
	require.NoError(t, SaveNotificationConfig(db, cfg))

	loaded := LoadNotificationConfig(db)
	assert.Equal(t, cfg, loaded)
	assert.Equal(t, "1234••••••••", loaded.MaskedTelegramBotToken())
	assert.Equal(t, "azGD••••••••", loaded.MaskedPushoverAppToken())
	assert.Equal(t, "", loaded.MaskedNtfyToken())

	cfg.TelegramBotToken = ""
	require.Error(t, cfg.Validate(), "enabled telegram without a token")
	cfg.Enabled = false
	require.NoError(t, cfg.Validate(), "disabled config only needs well-formed fields")
	require.NoError(t, SaveNotificationConfig(db, cfg))
	_, err := db.Get(notifyKeyTelegramBotToken)
	assert.Error(t, err, "an empty secret deletes its key")
}

func TestNotificationConfig_Validate(t *testing.T) {
	cases := []struct {
		name   string
		mutate func(*NotificationConfig)
		ok     bool
	}{
		{"default", func(*NotificationConfig) {}, true},
		{"bad channel", func(c *NotificationConfig) { c.Channel = "sms" }, false},
		{"enabled telegram without token", func(c *NotificationConfig) { c.Enabled = true; c.TelegramChatID = "42" }, false},
		{"telegram chat id not numeric", func(c *NotificationConfig) { c.TelegramChatID = "@channel" }, false},
		{"enabled telegram", func(c *NotificationConfig) {
			c.Enabled = true
			c.TelegramBotToken = "123456:abcdef"
			c.TelegramChatID = "-42"
		}, true},
		{"enabled pushover without user key", func(c *NotificationConfig) {
			c.Enabled = true
			c.Channel = NotifyChannelPushover
			c.PushoverAppToken = "app"
		}, false},
		{"bad language", func(c *NotificationConfig) { c.Language = "de" }, false},
		{"empty title", func(c *NotificationConfig) { c.Title = " " }, false},
		{"bad ntfy server", func(c *NotificationConfig) { c.NtfyServer = "ntfy.sh" }, false},
		{"bad ntfy topic", func(c *NotificationConfig) { c.NtfyTopic = "my topic!" }, false},
		{"enabled ntfy without topic", func(c *NotificationConfig) { c.Enabled = true; c.Channel = NotifyChannelNtfy }, false},
		{"enabled ntfy with topic", func(c *NotificationConfig) {
			c.Enabled = true
			c.Channel = NotifyChannelNtfy
			c.NtfyTopic = "mowgli_1"
		}, true},
		{"bad webhook", func(c *NotificationConfig) { c.WebhookURL = "ftp://x" }, false},
		{"enabled webhook", func(c *NotificationConfig) {
			c.Enabled = true
			c.Channel = NotifyChannelWebhook
			c.WebhookURL = "http://homeassistant.local:8123/api/webhook/abc"
		}, true},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			cfg := DefaultNotificationConfig()
			tc.mutate(&cfg)
			err := cfg.Validate()
			if tc.ok {
				assert.NoError(t, err)
			} else {
				assert.Error(t, err)
			}
		})
	}
}

func TestNotificationConfig_EventEnabledFallsBackToDefault(t *testing.T) {
	cfg := DefaultNotificationConfig()
	cfg.Events = map[string]bool{}
	assert.True(t, cfg.EventEnabled(NotifyEventBlocked))
	assert.False(t, cfg.EventEnabled(NotifyEventMowStopped))
}

func statusJSON(state int, name string, area int) []byte {
	b, _ := json.Marshal(map[string]any{"state": state, "state_name": name, "current_area": area, "coverage_percent": 0, "battery_percent": 77, "emergency": false})
	return b
}

func TestNotificationProvider_DeliversEnabledEventsOnly(t *testing.T) {
	srv, requests := newCaptureServer(t, http.StatusOK)
	db := types.NewMockDBProvider()
	cfg := DefaultNotificationConfig()
	cfg.Enabled = true
	cfg.Channel = NotifyChannelNtfy
	cfg.NtfyServer = srv.URL
	cfg.NtfyTopic = "garden"
	cfg.Title = "Robot"
	cfg.Events[NotifyEventZoneStarted] = false
	cfg.Events[NotifyEventZoneFinished] = false
	require.NoError(t, SaveNotificationConfig(db, cfg))

	now := t0
	p := newNotificationProvider(db, srv.Client(), func() time.Time { return now })
	p.HandleMap(mustJSON(t, map[string]any{
		"working_area_indices": []uint32{0, 2},
		"working_area":         []map[string]any{{"name": "Front"}, {"name": "Back"}},
	}))

	p.HandleStatus(statusJSON(2, "MOWING", 0))
	now = now.Add(time.Minute)
	p.HandleStatus(statusJSON(2, "MOWING", 2)) // zone change: both zone kinds off → nothing
	now = now.Add(time.Minute)
	p.HandleStatus(statusJSON(2, "MOWING_COMPLETE", 2))

	got := requests()
	require.Len(t, got, 2)
	assert.Contains(t, got[0].Body, `"title":"Robot"`)
	assert.Contains(t, got[0].Body, "Mowing started.")
	assert.Contains(t, got[1].Body, "Last zone Back")

	status := p.Status()
	assert.True(t, status.Enabled)
	assert.True(t, status.Configured)
	assert.Equal(t, 2, status.SentCount)
	assert.Equal(t, 0, status.FailedCount)
	assert.Empty(t, status.LastError)
}

func TestNotificationProvider_DisabledStillTracksState(t *testing.T) {
	srv, requests := newCaptureServer(t, http.StatusOK)
	db := types.NewMockDBProvider()
	now := t0
	p := newNotificationProvider(db, srv.Client(), func() time.Time { return now })

	p.HandleStatus(statusJSON(2, "MOWING", 0))
	assert.Empty(t, requests())

	cfg := p.Config()
	cfg.Enabled = true
	cfg.Channel = NotifyChannelNtfy
	cfg.NtfyServer = srv.URL
	cfg.NtfyTopic = "garden"
	require.NoError(t, p.UpdateConfig(cfg))

	now = now.Add(time.Minute)
	p.HandleStatus(statusJSON(2, "MOWING", 0))
	assert.Empty(t, requests(), "enabling mid-mow must not replay 'mowing started'")
}

func TestNotificationProvider_RecordsFailures(t *testing.T) {
	srv, _ := newCaptureServer(t, http.StatusUnauthorized)
	db := types.NewMockDBProvider()
	now := t0
	p := newNotificationProvider(db, srv.Client(), func() time.Time { return now })
	cfg := p.Config()
	cfg.Enabled = true
	cfg.Channel = NotifyChannelNtfy
	cfg.NtfyServer = srv.URL
	cfg.NtfyTopic = "garden"
	require.NoError(t, p.UpdateConfig(cfg))

	p.HandleStatus(statusJSON(2, "MOWING", 0)) // mowing started + zone started
	status := p.Status()
	assert.Equal(t, 2, status.FailedCount)
	assert.Equal(t, 0, status.SentCount)
	assert.Contains(t, status.LastError, "401")
}

func TestNotificationProvider_SendTestNeedsConfiguredChannel(t *testing.T) {
	srv, requests := newCaptureServer(t, http.StatusOK)
	db := types.NewMockDBProvider()
	p := newNotificationProvider(db, srv.Client(), time.Now)

	require.Error(t, p.SendTest(context.Background()), "nothing configured yet")

	cfg := p.Config()
	cfg.Channel = NotifyChannelWebhook
	cfg.WebhookURL = srv.URL + "/hook"
	require.NoError(t, p.UpdateConfig(cfg), "a test must work while still disabled")
	require.NoError(t, p.SendTest(context.Background()))
	got := requests()
	require.Len(t, got, 1)
	assert.Contains(t, got[0].Body, "Test notification")
}

func mustJSON(t *testing.T, v any) []byte {
	t.Helper()
	b, err := json.Marshal(v)
	require.NoError(t, err)
	return b
}
