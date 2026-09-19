package providers

import (
	"context"
	"fmt"
	"html"
	"net/http"
	"strconv"
	"strings"
	"sync"
	"time"

	tgbotapi "github.com/go-telegram-bot-api/telegram-bot-api"
	"github.com/nikoksr/notify"
	notifyhttp "github.com/nikoksr/notify/service/http"
	"github.com/nikoksr/notify/service/pushover"
	"github.com/nikoksr/notify/service/telegram"
)

// notifySender delivers one rendered message over the configured channel.
type notifySender interface {
	Send(ctx context.Context, msg NotifyMessage) error
}

// libSender routes every channel through github.com/nikoksr/notify: Telegram
// and Pushover use the library's own services; ntfy and the generic webhook
// ride its HTTP service with a per-message payload builder (ntfy's JSON
// publish endpoint carries priority and tags, which a static webhook cannot).
type libSender struct {
	cfg NotificationConfig
	// httpClient is shared by every channel, Telegram included (tests inject a
	// transport that redirects api.telegram.org to a fake).
	httpClient *http.Client

	mu sync.Mutex
	// telegram is built lazily: tgbotapi.NewBotAPI performs a getMe round trip,
	// which must not happen at config time (offline robot, typo in the token)
	// and must not be repeated on every push. Dropped after a failed send so a
	// rotated token or a recovered network gets a fresh handshake.
	telegram *telegram.Telegram
}

// newNotifySender builds the sender for the config's channel. It does not
// check IsConfigured: the provider does, so a disabled config never sends.
func newNotifySender(cfg NotificationConfig, client *http.Client) (notifySender, error) {
	switch cfg.Channel {
	case NotifyChannelTelegram, NotifyChannelPushover, NotifyChannelNtfy, NotifyChannelWebhook:
		return &libSender{cfg: copyNotificationConfig(cfg), httpClient: clientOrDefault(client)}, nil
	default:
		return nil, fmt.Errorf("unknown notification channel %q", cfg.Channel)
	}
}

func clientOrDefault(client *http.Client) *http.Client {
	if client != nil {
		return client
	}
	return &http.Client{Timeout: notifyHTTPTimeout}
}

func (s *libSender) Send(ctx context.Context, msg NotifyMessage) error {
	switch s.cfg.Channel {
	case NotifyChannelTelegram:
		return s.sendTelegram(ctx, msg)
	case NotifyChannelPushover:
		svc := pushover.New(s.cfg.PushoverAppToken)
		svc.AddReceivers(s.cfg.PushoverUserKey)
		return wrapNotifyErr("pushover", notify.NewWithServices(svc).Send(ctx, msg.Title, msg.Body))
	case NotifyChannelNtfy:
		return wrapNotifyErr("ntfy", notify.NewWithServices(s.ntfyService(msg)).Send(ctx, msg.Title, msg.Body))
	case NotifyChannelWebhook:
		return wrapNotifyErr("webhook", notify.NewWithServices(s.webhookService(msg)).Send(ctx, msg.Title, msg.Body))
	default:
		return fmt.Errorf("unknown notification channel %q", s.cfg.Channel)
	}
}

// sendTelegram sends "<b>title</b>\nbody": the library's Telegram service
// posts with parse_mode=HTML, so both halves are escaped.
func (s *libSender) sendTelegram(ctx context.Context, msg NotifyMessage) error {
	svc, err := s.telegramService()
	if err != nil {
		return fmt.Errorf("telegram: %w", err)
	}
	subject := "<b>" + html.EscapeString(msg.Title) + "</b>"
	if err := notify.NewWithServices(svc).Send(ctx, subject, html.EscapeString(msg.Body)); err != nil {
		s.mu.Lock()
		s.telegram = nil
		s.mu.Unlock()
		return wrapNotifyErr("telegram", err)
	}
	return nil
}

func (s *libSender) telegramService() (*telegram.Telegram, error) {
	s.mu.Lock()
	defer s.mu.Unlock()
	if s.telegram != nil {
		return s.telegram, nil
	}
	chatID, err := strconv.ParseInt(strings.TrimSpace(s.cfg.TelegramChatID), 10, 64)
	if err != nil {
		return nil, fmt.Errorf("chat id %q is not a number", s.cfg.TelegramChatID)
	}
	bot, err := tgbotapi.NewBotAPIWithClient(s.cfg.TelegramBotToken, s.httpClient)
	if err != nil {
		return nil, fmt.Errorf("bot handshake (getMe): %w", err)
	}
	svc := &telegram.Telegram{}
	svc.SetClient(bot)
	svc.AddReceivers(chatID)
	s.telegram = svc
	return svc, nil
}

// ntfyService publishes through ntfy's JSON endpoint (POST <server>/ with
// the topic in the body, https://docs.ntfy.sh/publish/#publish-as-json).
func (s *libSender) ntfyService(msg NotifyMessage) *notifyhttp.Service {
	svc := notifyhttp.New()
	svc.WithClient(s.httpClient)
	header := http.Header{}
	if s.cfg.NtfyToken != "" {
		header.Set("Authorization", "Bearer "+s.cfg.NtfyToken)
	}
	topic := s.cfg.NtfyTopic
	priority := msg.Priority
	tags := append([]string(nil), msg.Tags...)
	svc.AddReceivers(&notifyhttp.Webhook{
		URL:         strings.TrimRight(s.cfg.NtfyServer, "/") + "/",
		Method:      http.MethodPost,
		ContentType: "application/json; charset=utf-8",
		Header:      header,
		BuildPayload: func(subject, message string) any {
			return map[string]any{
				"topic":    topic,
				"title":    subject,
				"message":  message,
				"priority": priority,
				"tags":     tags,
			}
		},
	})
	return svc
}

// webhookService POSTs the whole rendered message as JSON to an operator URL
// (Home Assistant webhooks, Node-RED, n8n, a custom relay).
func (s *libSender) webhookService(msg NotifyMessage) *notifyhttp.Service {
	svc := notifyhttp.New()
	svc.WithClient(s.httpClient)
	payload := msg
	svc.AddReceivers(&notifyhttp.Webhook{
		URL:         s.cfg.WebhookURL,
		Method:      http.MethodPost,
		ContentType: "application/json; charset=utf-8",
		Header:      http.Header{},
		BuildPayload: func(subject, message string) any {
			payload.Title = subject
			payload.Body = message
			return payload
		},
	})
	return svc
}

func wrapNotifyErr(channel string, err error) error {
	if err == nil {
		return nil
	}
	return fmt.Errorf("%s: %w", channel, err)
}

// notifyHTTPTimeout bounds one delivery attempt inside the library's services.
const notifyHTTPTimeout = 15 * time.Second
