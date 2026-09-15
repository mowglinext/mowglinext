package providers

import (
	"context"
	"encoding/json"
	"fmt"
	"net/http"
	"sync"
	"time"

	"github.com/mowglinext/mowglinext/pkg/msgs/mowgli"
	"github.com/mowglinext/mowglinext/pkg/types"
	"github.com/sirupsen/logrus"
)

const (
	notifySendTimeout = notifyHTTPTimeout
	notifyQueueDepth  = 256
)

// NotifyDeliveryStatus is what the GUI shows under the settings: did the last
// push go out, and what was it.
type NotifyDeliveryStatus struct {
	Enabled     bool   `json:"enabled"`
	Configured  bool   `json:"configured"`
	Channel     string `json:"channel"`
	SentCount   int    `json:"sentCount"`
	FailedCount int    `json:"failedCount"`
	LastMessage string `json:"lastMessage,omitempty"`
	LastSentAt  string `json:"lastSentAt,omitempty"`
	LastError   string `json:"lastError,omitempty"`
	LastErrorAt string `json:"lastErrorAt,omitempty"`
}

// NotificationProvider watches the high-level status stream (fed by the ROS
// provider's fanOut, like the session tracker) and pushes operator
// notifications through the configured channel.
//
// The detector is fed on every tick even while notifications are disabled, so
// enabling them mid-mow does not fire a spurious "mowing started".
type NotificationProvider struct {
	db     types.IDBProvider
	client *http.Client
	now    func() time.Time

	queue    chan []byte
	mapQueue chan []byte

	mu       sync.RWMutex
	cfg      NotificationConfig
	detector *NotifyDetector
	sender   notifySender
	status   NotifyDeliveryStatus
}

// NewNotificationProvider loads the config and starts the consumer goroutine.
func NewNotificationProvider(db types.IDBProvider) *NotificationProvider {
	p := newNotificationProvider(db, &http.Client{Timeout: notifySendTimeout}, time.Now)
	go p.run()
	return p
}

// NewIdleNotificationProvider builds a provider WITHOUT the consumer goroutine;
// tests drive HandleStatus by hand.
func NewIdleNotificationProvider(db types.IDBProvider) *NotificationProvider {
	return newNotificationProvider(db, &http.Client{Timeout: notifySendTimeout}, time.Now)
}

func newNotificationProvider(db types.IDBProvider, client *http.Client, now func() time.Time) *NotificationProvider {
	p := &NotificationProvider{
		db:       db,
		client:   client,
		now:      now,
		queue:    make(chan []byte, notifyQueueDepth),
		mapQueue: make(chan []byte, 4),
		detector: NewNotifyDetector(),
	}
	p.applyConfig(LoadNotificationConfig(db))
	return p
}

func (p *NotificationProvider) run() {
	for {
		select {
		case msg := <-p.queue:
			p.HandleStatus(msg)
		case msg := <-p.mapQueue:
			p.HandleMap(msg)
		}
	}
}

// Enqueue hands a raw highLevelStatus message to the ordered consumer.
// Non-blocking: a full buffer drops the tick rather than stalling fanOut.
func (p *NotificationProvider) Enqueue(msg []byte) {
	select {
	case p.queue <- msg:
	default:
		logrus.Warn("Notifications: status queue full, dropping message")
	}
}

// EnqueueMap hands a raw virtual "map" message over so zone names resolve.
// Only the latest map matters, so a full buffer drops the oldest.
func (p *NotificationProvider) EnqueueMap(msg []byte) {
	select {
	case p.mapQueue <- msg:
	default:
		select {
		case <-p.mapQueue:
		default:
		}
		p.mapQueue <- msg
	}
}

// HandleStatus folds one status message and delivers whatever it produced.
func (p *NotificationProvider) HandleStatus(raw []byte) {
	var st NotifyStatus
	if err := json.Unmarshal(raw, &st); err != nil {
		return
	}
	now := p.now()

	p.mu.Lock()
	cfg := p.cfg
	events := p.detector.OnStatus(st, now, cfg.EventEnabled)
	sender := p.sender
	p.mu.Unlock()

	if !cfg.Enabled || sender == nil {
		return
	}
	for _, ev := range events {
		if !cfg.EventEnabled(ev.Kind) {
			continue
		}
		p.deliver(sender, RenderNotification(cfg.Language, cfg.Title, ev))
	}
}

// HandleMap refreshes the area-name lookup from the GUI's map payload.
func (p *NotificationProvider) HandleMap(raw []byte) {
	var m mowgli.Map
	if err := json.Unmarshal(raw, &m); err != nil {
		return
	}
	names := make(map[int]string, len(m.WorkingArea))
	for i, area := range m.WorkingArea {
		if i < len(m.WorkingAreaIndices) {
			names[int(m.WorkingAreaIndices[i])] = area.Name
		}
	}
	p.mu.Lock()
	p.detector.SetAreaNames(names)
	p.mu.Unlock()
}

func (p *NotificationProvider) deliver(sender notifySender, msg NotifyMessage) {
	ctx, cancel := context.WithTimeout(context.Background(), notifySendTimeout)
	defer cancel()
	err := sender.Send(ctx, msg)
	p.recordDelivery(msg, err)
	if err != nil {
		logrus.WithError(err).WithField("message", msg.Message).Warn("Notifications: delivery failed")
		return
	}
	logrus.WithField("message", msg.Message).Info("Notifications: delivered")
}

func (p *NotificationProvider) recordDelivery(msg NotifyMessage, err error) {
	now := p.now().UTC().Format(time.RFC3339)
	p.mu.Lock()
	defer p.mu.Unlock()
	if err != nil {
		p.status.FailedCount++
		p.status.LastError = err.Error()
		p.status.LastErrorAt = now
		return
	}
	p.status.SentCount++
	p.status.LastMessage = msg.Body
	p.status.LastSentAt = now
	p.status.LastError = ""
	p.status.LastErrorAt = ""
}

// Config returns a copy of the current settings (secrets included — callers
// that expose it must mask them).
func (p *NotificationProvider) Config() NotificationConfig {
	p.mu.RLock()
	defer p.mu.RUnlock()
	return copyNotificationConfig(p.cfg)
}

// Status is the delivery summary for the GUI.
func (p *NotificationProvider) Status() NotifyDeliveryStatus {
	p.mu.RLock()
	defer p.mu.RUnlock()
	s := p.status
	s.Enabled = p.cfg.Enabled
	s.Configured = p.cfg.IsConfigured()
	s.Channel = p.cfg.Channel
	return s
}

// UpdateConfig validates, persists and applies new settings.
func (p *NotificationProvider) UpdateConfig(cfg NotificationConfig) error {
	if err := cfg.Validate(); err != nil {
		return err
	}
	if err := SaveNotificationConfig(p.db, cfg); err != nil {
		return err
	}
	p.applyConfig(cfg)
	return nil
}

func (p *NotificationProvider) applyConfig(cfg NotificationConfig) {
	sender, err := newNotifySender(cfg, p.client)
	if err != nil {
		logrus.WithError(err).Warn("Notifications: no sender for stored config")
	}
	p.mu.Lock()
	defer p.mu.Unlock()
	p.cfg = copyNotificationConfig(cfg)
	if cfg.IsConfigured() {
		p.sender = sender
	} else {
		p.sender = nil
	}
}

// SendTest pushes a test message with the STORED config, whether or not
// notifications are enabled, so the operator can verify a channel before
// arming it.
func (p *NotificationProvider) SendTest(ctx context.Context) error {
	p.mu.RLock()
	cfg := p.cfg
	sender := p.sender
	p.mu.RUnlock()
	if sender == nil {
		return fmt.Errorf("%s channel is not configured", cfg.Channel)
	}
	msg := RenderNotification(cfg.Language, cfg.Title, NotifyEvent{
		Kind:     "test",
		Message:  NotifyMsgTestNotifcation,
		Priority: notifyPriorityDefault,
		Params:   map[string]string{},
		At:       p.now(),
	})
	err := sender.Send(ctx, msg)
	p.recordDelivery(msg, err)
	return err
}
