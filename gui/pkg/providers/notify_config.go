package providers

import (
	"fmt"
	"net/url"
	"strconv"
	"strings"

	"github.com/mowglinext/mowglinext/pkg/types"
)

// Notification settings live in the GUI's key-value DB, NOT in
// mowgli_robot.yaml: the bot/ntfy tokens are secrets, nothing on the ROS2 side
// consumes any of these values, and the events are derived from the same
// /behavior_tree_node/high_level_status stream the session tracker watches.
const (
	notifyKeyEnabled          = "notifications.enabled"
	notifyKeyChannel          = "notifications.channel"
	notifyKeyLanguage         = "notifications.language"
	notifyKeyTitle            = "notifications.title"
	notifyKeyNtfyServer       = "notifications.ntfy.server"
	notifyKeyNtfyTopic        = "notifications.ntfy.topic"
	notifyKeyNtfyToken        = "notifications.ntfy.token"
	notifyKeyTelegramBotToken = "notifications.telegram.botToken"
	notifyKeyTelegramChatID   = "notifications.telegram.chatId"
	notifyKeyPushoverAppToken = "notifications.pushover.appToken"
	notifyKeyPushoverUserKey  = "notifications.pushover.userKey"
	notifyKeyWebhookURL       = "notifications.webhook.url"
	notifyKeyEventPrefix      = "notifications.event."
)

// Delivery channels, all routed through github.com/nikoksr/notify
// (notify_senders.go).
const (
	NotifyChannelTelegram = "telegram"
	NotifyChannelPushover = "pushover"
	NotifyChannelNtfy     = "ntfy"
	NotifyChannelWebhook  = "webhook"
)

// NotifyChannels lists every channel in display order.
var NotifyChannels = []string{NotifyChannelTelegram, NotifyChannelPushover, NotifyChannelNtfy, NotifyChannelWebhook}

const (
	DefaultNotifyChannel    = NotifyChannelTelegram
	DefaultNotifyLanguage   = "en"
	DefaultNotifyTitle      = "MowgliNext"
	DefaultNotifyNtfyServer = "https://ntfy.sh"
)

// NotifyLanguages are the languages the server-side message catalogue speaks.
var NotifyLanguages = []string{"en", "fr"}

// NotificationConfig is the operator's push-notification settings.
type NotificationConfig struct {
	Enabled  bool
	Channel  string
	Language string
	// Title is the notification title / sender name, e.g. the robot's name.
	Title            string
	NtfyServer       string
	NtfyTopic        string
	NtfyToken        string
	TelegramBotToken string
	TelegramChatID   string
	PushoverAppToken string
	PushoverUserKey  string
	WebhookURL       string
	// Events maps a NotifyEventKind to whether it is delivered.
	Events map[string]bool
}

// DefaultNotificationConfig is a fresh install: disabled, Telegram selected,
// ntfy pointed at the public server, every event on except the chatty
// "mowing stopped" catch-all.
func DefaultNotificationConfig() NotificationConfig {
	return NotificationConfig{
		Enabled:    false,
		Channel:    DefaultNotifyChannel,
		Language:   DefaultNotifyLanguage,
		Title:      DefaultNotifyTitle,
		NtfyServer: DefaultNotifyNtfyServer,
		Events:     DefaultNotifyEvents(),
	}
}

// IsConfigured reports whether the selected channel has what it needs to send.
func (c NotificationConfig) IsConfigured() bool {
	switch c.Channel {
	case NotifyChannelNtfy:
		return c.NtfyTopic != "" && validateHTTPURL(c.NtfyServer) == nil
	case NotifyChannelTelegram:
		return c.TelegramBotToken != "" && isTelegramChatID(c.TelegramChatID)
	case NotifyChannelPushover:
		return c.PushoverAppToken != "" && c.PushoverUserKey != ""
	case NotifyChannelWebhook:
		return validateHTTPURL(c.WebhookURL) == nil
	default:
		return false
	}
}

// EventEnabled reports whether an event kind should be delivered; an unknown
// kind (a newer backend than the stored config) falls back to its default.
func (c NotificationConfig) EventEnabled(kind string) bool {
	if v, ok := c.Events[kind]; ok {
		return v
	}
	return DefaultNotifyEvents()[kind]
}

// MaskedNtfyToken / MaskedTelegramBotToken are the only forms of the secrets
// that ever leave the backend.
func (c NotificationConfig) MaskedNtfyToken() string        { return maskSecret(c.NtfyToken) }
func (c NotificationConfig) MaskedTelegramBotToken() string { return maskSecret(c.TelegramBotToken) }
func (c NotificationConfig) MaskedPushoverAppToken() string { return maskSecret(c.PushoverAppToken) }

func maskSecret(s string) string {
	if s == "" {
		return ""
	}
	if len(s) <= 8 {
		return "••••••••"
	}
	return s[:4] + "••••••••"
}

// Validate rejects a config the provider could not act on. A disabled config
// only needs well-formed URLs; an enabled one must be fully configured for
// its channel, so a half-filled form cannot silently arm a dead channel.
func (c NotificationConfig) Validate() error {
	if !isNotifyChannel(c.Channel) {
		return fmt.Errorf("channel must be one of %s", strings.Join(NotifyChannels, ", "))
	}
	if !isNotifyLanguage(c.Language) {
		return fmt.Errorf("language must be one of %s", strings.Join(NotifyLanguages, ", "))
	}
	if strings.TrimSpace(c.Title) == "" {
		return fmt.Errorf("title must not be empty")
	}
	if c.NtfyServer != "" {
		if err := validateHTTPURL(c.NtfyServer); err != nil {
			return fmt.Errorf("ntfy server: %w", err)
		}
	}
	if c.NtfyTopic != "" && !isValidNtfyTopic(c.NtfyTopic) {
		return fmt.Errorf("ntfy topic may only contain letters, digits, '-' and '_'")
	}
	if c.TelegramChatID != "" && !isTelegramChatID(c.TelegramChatID) {
		return fmt.Errorf("telegram chat id must be a whole number (negative for groups)")
	}
	if c.WebhookURL != "" {
		if err := validateHTTPURL(c.WebhookURL); err != nil {
			return fmt.Errorf("webhook url: %w", err)
		}
	}
	if c.Enabled && !c.IsConfigured() {
		return fmt.Errorf("%s channel is not fully configured", c.Channel)
	}
	return nil
}

func isNotifyChannel(channel string) bool {
	for _, c := range NotifyChannels {
		if c == channel {
			return true
		}
	}
	return false
}

// isTelegramChatID accepts what the Bot API takes as chat_id: an int64,
// negative for groups and supergroups.
func isTelegramChatID(id string) bool {
	_, err := strconv.ParseInt(strings.TrimSpace(id), 10, 64)
	return err == nil
}

func isNotifyLanguage(lang string) bool {
	for _, l := range NotifyLanguages {
		if l == lang {
			return true
		}
	}
	return false
}

// isValidNtfyTopic mirrors ntfy's own topic rule (^[-_A-Za-z0-9]{1,64}$).
func isValidNtfyTopic(topic string) bool {
	if len(topic) == 0 || len(topic) > 64 {
		return false
	}
	for _, r := range topic {
		isAlpha := (r >= 'a' && r <= 'z') || (r >= 'A' && r <= 'Z')
		isDigit := r >= '0' && r <= '9'
		if !isAlpha && !isDigit && r != '-' && r != '_' {
			return false
		}
	}
	return true
}

// validateHTTPURL accepts an absolute http(s) URL with a host.
func validateHTTPURL(raw string) error {
	u, err := url.Parse(strings.TrimSpace(raw))
	if err != nil {
		return fmt.Errorf("not a valid URL: %w", err)
	}
	if u.Scheme != "http" && u.Scheme != "https" {
		return fmt.Errorf("must start with http:// or https://")
	}
	if u.Host == "" {
		return fmt.Errorf("must include a host")
	}
	return nil
}

// LoadNotificationConfig reads the settings from the DB, falling back to the
// defaults for every absent or unparseable key.
func LoadNotificationConfig(db types.IDBProvider) NotificationConfig {
	cfg := DefaultNotificationConfig()
	cfg.Enabled = dbBool(db, notifyKeyEnabled, cfg.Enabled)
	cfg.Channel = dbString(db, notifyKeyChannel, cfg.Channel)
	cfg.Language = dbString(db, notifyKeyLanguage, cfg.Language)
	cfg.Title = dbString(db, notifyKeyTitle, cfg.Title)
	cfg.NtfyServer = dbString(db, notifyKeyNtfyServer, cfg.NtfyServer)
	cfg.NtfyTopic = dbString(db, notifyKeyNtfyTopic, "")
	cfg.NtfyToken = dbString(db, notifyKeyNtfyToken, "")
	cfg.TelegramBotToken = dbString(db, notifyKeyTelegramBotToken, "")
	cfg.TelegramChatID = dbString(db, notifyKeyTelegramChatID, "")
	cfg.PushoverAppToken = dbString(db, notifyKeyPushoverAppToken, "")
	cfg.PushoverUserKey = dbString(db, notifyKeyPushoverUserKey, "")
	cfg.WebhookURL = dbString(db, notifyKeyWebhookURL, "")
	for kind, def := range DefaultNotifyEvents() {
		cfg.Events[kind] = dbBool(db, notifyKeyEventPrefix+kind, def)
	}
	return cfg
}

// SaveNotificationConfig persists every key. Secrets are written verbatim to
// the DB and nowhere else; an empty secret deletes its key.
func SaveNotificationConfig(db types.IDBProvider, cfg NotificationConfig) error {
	writes := []struct {
		key   string
		value string
	}{
		{notifyKeyEnabled, strconv.FormatBool(cfg.Enabled)},
		{notifyKeyChannel, cfg.Channel},
		{notifyKeyLanguage, cfg.Language},
		{notifyKeyTitle, strings.TrimSpace(cfg.Title)},
		{notifyKeyNtfyServer, strings.TrimSpace(cfg.NtfyServer)},
		{notifyKeyNtfyTopic, strings.TrimSpace(cfg.NtfyTopic)},
		{notifyKeyTelegramChatID, strings.TrimSpace(cfg.TelegramChatID)},
		{notifyKeyPushoverUserKey, strings.TrimSpace(cfg.PushoverUserKey)},
		{notifyKeyWebhookURL, strings.TrimSpace(cfg.WebhookURL)},
	}
	for kind := range DefaultNotifyEvents() {
		writes = append(writes, struct {
			key   string
			value string
		}{notifyKeyEventPrefix + kind, strconv.FormatBool(cfg.EventEnabled(kind))})
	}
	for _, w := range writes {
		if err := db.Set(w.key, []byte(w.value)); err != nil {
			return fmt.Errorf("persist %s: %w", w.key, err)
		}
	}
	for key, value := range map[string]string{
		notifyKeyNtfyToken:        cfg.NtfyToken,
		notifyKeyTelegramBotToken: cfg.TelegramBotToken,
		notifyKeyPushoverAppToken: cfg.PushoverAppToken,
	} {
		if err := saveSecret(db, key, value); err != nil {
			return err
		}
	}
	return nil
}

func saveSecret(db types.IDBProvider, key, value string) error {
	if value == "" {
		if err := db.Delete(key); err != nil {
			return fmt.Errorf("clear %s: %w", key, err)
		}
		return nil
	}
	if err := db.Set(key, []byte(value)); err != nil {
		return fmt.Errorf("persist %s: %w", key, err)
	}
	return nil
}

// copyNotificationConfig returns a deep copy (the Events map is shared state).
func copyNotificationConfig(c NotificationConfig) NotificationConfig {
	next := c
	next.Events = make(map[string]bool, len(c.Events))
	for k, v := range c.Events {
		next.Events[k] = v
	}
	return next
}
