package api

import (
	"bytes"
	"encoding/json"
	"errors"
	"io"
	"log"
	"mime"
	"net/http"
	"net/url"
	"sync"
	"time"

	"github.com/gin-gonic/gin"
)

const gamepadReportLimit = 16 * 1024

// This history is deliberately independent of the transport ownership mutex:
// operators must be able to inspect a cleanup blocked in a provider call.
type gamepadDiagnosticHistory struct {
	mu      sync.Mutex
	started time.Time
	server  []gamepadServerEvent
	browser []gamepadBrowserReport
	logger  *gamepadDiagnosticLogger
}

// Docker's log pipe may block. A single worker handles that I/O, while control
// paths only attempt a bounded, nonblocking enqueue. Dropped log lines remain
// available in the independent in-memory history.
type gamepadDiagnosticLogger struct {
	once  sync.Once
	queue chan gamepadServerEvent
	write func(gamepadServerEvent)
}

var defaultGamepadDiagnosticLogger = newGamepadDiagnosticLogger(func(event gamepadServerEvent) {
	// JSON quoting prevents a browser close reason from forging log lines.
	raw, _ := json.Marshal(event)
	log.Printf("gamepad %s", raw)
})

func newGamepadDiagnosticLogger(write func(gamepadServerEvent)) *gamepadDiagnosticLogger {
	return &gamepadDiagnosticLogger{queue: make(chan gamepadServerEvent, 64), write: write}
}

func (l *gamepadDiagnosticLogger) enqueue(event gamepadServerEvent) {
	l.once.Do(func() {
		go func() {
			for event := range l.queue {
				l.write(event)
			}
		}()
	})
	select {
	case l.queue <- event:
	default:
	}
}

type gamepadServerEvent struct {
	At          time.Time `json:"at"`
	Event       string    `json:"event"`
	Session     string    `json:"session,omitempty"`
	Client      string    `json:"client,omitempty"`
	Cause       string    `json:"cause,omitempty"`
	Error       string    `json:"error,omitempty"`
	Code        int       `json:"code,omitempty"`
	Reason      string    `json:"reason,omitempty"`
	Frames      uint64    `json:"frames"`
	AgeMS       int64     `json:"ageMs"`
	LastFrameMS int64     `json:"lastFrameMs"`
	MaxGapMS    int64     `json:"maxGapMs"`
	State       uint8     `json:"state"`
	ManualOwned bool      `json:"manualOwned"`
	Skipped     bool      `json:"skipped,omitempty"`
	Success     *bool     `json:"success,omitempty"`
}

type gamepadBrowserReport struct {
	ReceivedAt time.Time       `json:"receivedAt"`
	Report     json.RawMessage `json:"report" swaggertype:"object"`
}

type gamepadBrowserEvent struct {
	At     string          `json:"at"`
	Event  string          `json:"event"`
	Detail json.RawMessage `json:"detail,omitempty" swaggertype:"object"`
}

type gamepadBrowserEnvelope struct {
	Version  int                   `json:"version"`
	ClientID string                `json:"clientId"`
	Events   []gamepadBrowserEvent `json:"events"`
	Snapshot json.RawMessage       `json:"snapshot" swaggertype:"object"`
}

type gamepadDiagnosticResponse struct {
	StartedAt time.Time              `json:"startedAt"`
	Active    bool                   `json:"active"`
	Server    []gamepadServerEvent   `json:"server"`
	Browser   []gamepadBrowserReport `json:"browser"`
}

func (h *gamepadDiagnosticHistory) initialize() {
	h.mu.Lock()
	defer h.mu.Unlock()
	if h.started.IsZero() {
		h.started = time.Now().UTC()
	}
}

func (h *gamepadDiagnosticHistory) record(event gamepadServerEvent) {
	event.At = time.Now().UTC()
	h.mu.Lock()
	if len(h.server) == 64 {
		copy(h.server, h.server[1:])
		h.server = h.server[:63]
	}
	h.server = append(h.server, event)
	h.mu.Unlock()
	logger := h.logger
	if logger == nil {
		logger = defaultGamepadDiagnosticLogger
	}
	logger.enqueue(event)
}

func (h *gamepadDiagnosticHistory) snapshot(active bool) gamepadDiagnosticResponse {
	h.mu.Lock()
	defer h.mu.Unlock()
	return gamepadDiagnosticResponse{
		StartedAt: h.started, Active: active,
		Server:  append([]gamepadServerEvent{}, h.server...),
		Browser: append([]gamepadBrowserReport{}, h.browser...),
	}
}

func gamepadClientLabel(value string) string {
	if len(value) > 80 {
		return ""
	}
	for _, c := range value {
		if !(c >= 'a' && c <= 'z' || c >= 'A' && c <= 'Z' || c >= '0' && c <= '9' || c == '-') {
			return ""
		}
	}
	return value
}

func gamepadDiagnosticOriginAllowed(r *http.Request) bool {
	origin, err := url.Parse(r.Header.Get("Origin"))
	mediaType, _, mediaErr := mime.ParseMediaType(r.Header.Get("Content-Type"))
	return err == nil && origin.Host == r.Host && (origin.Scheme == "http" || origin.Scheme == "https") && mediaErr == nil && mediaType == "application/json"
}

func gamepadJSONObject(raw json.RawMessage) bool {
	raw = bytes.TrimSpace(raw)
	return len(raw) > 0 && raw[0] == '{' && json.Valid(raw)
}

func validateGamepadBrowserReport(raw []byte) error {
	var envelope gamepadBrowserEnvelope
	decoder := json.NewDecoder(bytes.NewReader(raw))
	decoder.DisallowUnknownFields()
	if err := decoder.Decode(&envelope); err != nil {
		return err
	}
	if decoder.Decode(new(any)) != io.EOF || envelope.Version != 1 || len(envelope.ClientID) == 0 || len(envelope.ClientID) > 64 || envelope.Events == nil || len(envelope.Events) > 32 || !gamepadJSONObject(envelope.Snapshot) {
		return errors.New("invalid diagnostic envelope")
	}
	for _, event := range envelope.Events {
		if event.At == "" || event.Event == "" || (event.Detail != nil && !gamepadJSONObject(event.Detail)) {
			return errors.New("invalid diagnostic event")
		}
	}
	return nil
}

// @Summary Read bounded, memory-only gamepad diagnostics
// @Tags gamepad
// @Produce json
// @Success 200 {object} gamepadDiagnosticResponse
// @Router /mowglinext/gamepad/diagnostics [get]
func (c *gamepadCoordinator) getDiagnostics(ctx *gin.Context) {
	ctx.Header("Cache-Control", "no-store")
	ctx.JSON(http.StatusOK, c.diagnostics.snapshot(c.active.Load()))
}

// @Summary Store a bounded browser diagnostic report (same-origin JSON only)
// @Tags gamepad
// @Accept json
// @Param report body gamepadBrowserEnvelope true "Bounded controller diagnostics"
// @Success 204
// @Failure 400 {object} ErrorResponse
// @Failure 403 {object} ErrorResponse
// @Failure 413 {object} ErrorResponse
// @Router /mowglinext/gamepad/diagnostics [post]
func (c *gamepadCoordinator) postDiagnostics(ctx *gin.Context) {
	if !gamepadDiagnosticOriginAllowed(ctx.Request) {
		ctx.AbortWithStatus(http.StatusForbidden)
		return
	}
	raw, err := io.ReadAll(http.MaxBytesReader(ctx.Writer, ctx.Request.Body, gamepadReportLimit))
	if err != nil {
		var tooLarge *http.MaxBytesError
		if errors.As(err, &tooLarge) {
			ctx.AbortWithStatus(http.StatusRequestEntityTooLarge)
		} else {
			ctx.AbortWithStatus(http.StatusBadRequest)
		}
		return
	}
	if validateGamepadBrowserReport(raw) != nil {
		ctx.AbortWithStatus(http.StatusBadRequest)
		return
	}
	h := &c.diagnostics
	h.mu.Lock()
	if len(h.browser) == 8 {
		copy(h.browser, h.browser[1:])
		h.browser = h.browser[:7]
	}
	h.browser = append(h.browser, gamepadBrowserReport{ReceivedAt: time.Now().UTC(), Report: json.RawMessage(raw)})
	h.mu.Unlock()
	ctx.Status(http.StatusNoContent)
}
