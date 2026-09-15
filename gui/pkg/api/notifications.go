package api

import (
	"context"
	"net/http"
	"strings"
	"time"

	"github.com/gin-gonic/gin"
	"github.com/mowglinext/mowglinext/pkg/providers"
)

// NotificationSettingsResponse is the operator-facing view of the push
// notification settings. Secrets never leave the backend: only whether one
// is stored and a recognisable masked prefix.
type NotificationSettingsResponse struct {
	Enabled                bool            `json:"enabled"`
	Channel                string          `json:"channel"`
	Language               string          `json:"language"`
	Title                  string          `json:"title"`
	NtfyServer             string          `json:"ntfyServer"`
	NtfyTopic              string          `json:"ntfyTopic"`
	NtfyTokenSet           bool            `json:"ntfyTokenSet"`
	NtfyTokenMasked        string          `json:"ntfyTokenMasked"`
	TelegramBotTokenSet    bool            `json:"telegramBotTokenSet"`
	TelegramBotTokenMasked string          `json:"telegramBotTokenMasked"`
	TelegramChatId         string          `json:"telegramChatId"`
	PushoverAppTokenSet    bool            `json:"pushoverAppTokenSet"`
	PushoverAppTokenMasked string          `json:"pushoverAppTokenMasked"`
	PushoverUserKey        string          `json:"pushoverUserKey"`
	WebhookUrl             string          `json:"webhookUrl"`
	Events                 map[string]bool `json:"events"`
	EventKinds             []string        `json:"eventKinds"`
	Channels               []string        `json:"channels"`
}

// NotificationSettingsUpdate is a partial update: absent fields keep their
// current value. Secrets are write-only — send one to replace it, or the
// matching Clear flag to forget it.
type NotificationSettingsUpdate struct {
	Enabled               *bool            `json:"enabled"`
	Channel               *string          `json:"channel"`
	Language              *string          `json:"language"`
	Title                 *string          `json:"title"`
	NtfyServer            *string          `json:"ntfyServer"`
	NtfyTopic             *string          `json:"ntfyTopic"`
	NtfyToken             *string          `json:"ntfyToken"`
	ClearNtfyToken        bool             `json:"clearNtfyToken"`
	TelegramBotToken      *string          `json:"telegramBotToken"`
	ClearTelegramBotToken bool             `json:"clearTelegramBotToken"`
	TelegramChatId        *string          `json:"telegramChatId"`
	PushoverAppToken      *string          `json:"pushoverAppToken"`
	ClearPushoverAppToken bool             `json:"clearPushoverAppToken"`
	PushoverUserKey       *string          `json:"pushoverUserKey"`
	WebhookUrl            *string          `json:"webhookUrl"`
	Events                *map[string]bool `json:"events"`
}

const notificationTestTimeout = 20 * time.Second

// NotificationRoutes registers the push-notification endpoints.
func NotificationRoutes(r *gin.RouterGroup, p *providers.NotificationProvider) {
	group := r.Group("/notifications")
	group.GET("/settings", getNotificationSettings(p))
	group.PUT("/settings", putNotificationSettings(p))
	group.GET("/status", getNotificationStatus(p))
	group.POST("/test", postNotificationTest(p))
}

// getNotificationSettings returns the notification settings (secrets masked)
//
// @Summary notification settings
// @Tags notifications
// @Produce json
// @Success 200 {object} NotificationSettingsResponse
// @Router /notifications/settings [get]
func getNotificationSettings(p *providers.NotificationProvider) gin.HandlerFunc {
	return func(c *gin.Context) {
		c.JSON(http.StatusOK, notificationSettingsView(p.Config()))
	}
}

// putNotificationSettings updates the notification settings
//
// @Summary update notification settings
// @Tags notifications
// @Accept json
// @Produce json
// @Param settings body NotificationSettingsUpdate true "partial settings"
// @Success 200 {object} NotificationSettingsResponse
// @Failure 400 {object} ErrorResponse
// @Router /notifications/settings [put]
func putNotificationSettings(p *providers.NotificationProvider) gin.HandlerFunc {
	return func(c *gin.Context) {
		var update NotificationSettingsUpdate
		if err := c.BindJSON(&update); err != nil {
			c.JSON(http.StatusBadRequest, ErrorResponse{Error: err.Error()})
			return
		}
		cfg := applyNotificationUpdate(p.Config(), update)
		if err := p.UpdateConfig(cfg); err != nil {
			c.JSON(http.StatusBadRequest, ErrorResponse{Error: err.Error()})
			return
		}
		c.JSON(http.StatusOK, notificationSettingsView(p.Config()))
	}
}

// getNotificationStatus returns the delivery summary
//
// @Summary notification delivery status
// @Tags notifications
// @Produce json
// @Success 200 {object} providers.NotifyDeliveryStatus
// @Router /notifications/status [get]
func getNotificationStatus(p *providers.NotificationProvider) gin.HandlerFunc {
	return func(c *gin.Context) {
		c.JSON(http.StatusOK, p.Status())
	}
}

// postNotificationTest sends a test push with the stored settings
//
// @Summary send a test notification
// @Tags notifications
// @Produce json
// @Success 200 {object} OkResponse
// @Failure 400 {object} ErrorResponse
// @Failure 502 {object} ErrorResponse
// @Router /notifications/test [post]
func postNotificationTest(p *providers.NotificationProvider) gin.HandlerFunc {
	return func(c *gin.Context) {
		ctx, cancel := context.WithTimeout(c.Request.Context(), notificationTestTimeout)
		defer cancel()
		if !p.Config().IsConfigured() {
			c.JSON(http.StatusBadRequest, ErrorResponse{Error: "notification channel is not configured"})
			return
		}
		if err := p.SendTest(ctx); err != nil {
			c.JSON(http.StatusBadGateway, ErrorResponse{Error: err.Error()})
			return
		}
		c.JSON(http.StatusOK, OkResponse{Ok: "true"})
	}
}

func notificationSettingsView(cfg providers.NotificationConfig) NotificationSettingsResponse {
	events := make(map[string]bool, len(providers.NotifyEventKinds))
	for _, kind := range providers.NotifyEventKinds {
		events[kind] = cfg.EventEnabled(kind)
	}
	return NotificationSettingsResponse{
		Enabled:                cfg.Enabled,
		Channel:                cfg.Channel,
		Language:               cfg.Language,
		Title:                  cfg.Title,
		NtfyServer:             cfg.NtfyServer,
		NtfyTopic:              cfg.NtfyTopic,
		NtfyTokenSet:           cfg.NtfyToken != "",
		NtfyTokenMasked:        cfg.MaskedNtfyToken(),
		TelegramBotTokenSet:    cfg.TelegramBotToken != "",
		TelegramBotTokenMasked: cfg.MaskedTelegramBotToken(),
		TelegramChatId:         cfg.TelegramChatID,
		PushoverAppTokenSet:    cfg.PushoverAppToken != "",
		PushoverAppTokenMasked: cfg.MaskedPushoverAppToken(),
		PushoverUserKey:        cfg.PushoverUserKey,
		WebhookUrl:             cfg.WebhookURL,
		Events:                 events,
		EventKinds:             append([]string(nil), providers.NotifyEventKinds...),
		Channels:               append([]string(nil), providers.NotifyChannels...),
	}
}

// applyNotificationUpdate returns a NEW config with the update's present
// fields laid over the current one; the input is not mutated.
func applyNotificationUpdate(current providers.NotificationConfig, u NotificationSettingsUpdate) providers.NotificationConfig {
	next := current
	next.Events = make(map[string]bool, len(current.Events))
	for k, v := range current.Events {
		next.Events[k] = v
	}
	setString := func(dst *string, src *string) {
		if src != nil {
			*dst = strings.TrimSpace(*src)
		}
	}
	if u.Enabled != nil {
		next.Enabled = *u.Enabled
	}
	setString(&next.Channel, u.Channel)
	setString(&next.Language, u.Language)
	setString(&next.Title, u.Title)
	setString(&next.NtfyServer, u.NtfyServer)
	setString(&next.NtfyTopic, u.NtfyTopic)
	setString(&next.TelegramChatID, u.TelegramChatId)
	setString(&next.PushoverUserKey, u.PushoverUserKey)
	setString(&next.WebhookURL, u.WebhookUrl)
	next.NtfyToken = applySecret(current.NtfyToken, u.NtfyToken, u.ClearNtfyToken)
	next.TelegramBotToken = applySecret(current.TelegramBotToken, u.TelegramBotToken, u.ClearTelegramBotToken)
	next.PushoverAppToken = applySecret(current.PushoverAppToken, u.PushoverAppToken, u.ClearPushoverAppToken)
	if u.Events != nil {
		for kind, on := range *u.Events {
			if _, known := providers.DefaultNotifyEvents()[kind]; known {
				next.Events[kind] = on
			}
		}
	}
	return next
}

func applySecret(current string, replacement *string, clear bool) string {
	if clear {
		return ""
	}
	if replacement != nil && strings.TrimSpace(*replacement) != "" {
		return strings.TrimSpace(*replacement)
	}
	return current
}
