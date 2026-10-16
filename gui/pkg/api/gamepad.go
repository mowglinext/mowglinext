package api

import (
	"bytes"
	"context"
	"encoding/json"
	"errors"
	"fmt"
	"io"
	"math"
	"net/http"
	"sync"
	"sync/atomic"
	"time"

	"github.com/gin-gonic/gin"
	"github.com/gorilla/websocket"
	"github.com/mowglinext/mowglinext/pkg/msgs/geometry"
	"github.com/mowglinext/mowglinext/pkg/msgs/mowgli"
	"github.com/mowglinext/mowglinext/pkg/types"
)

const gamepadLease = 300 * time.Millisecond
const gamepadStopTimeout = time.Second

var errGamepadOwner = errors.New("gamepad owns teleop")

// gamepadCoordinator owns only GUI transport arbitration, never robot behavior.
// Ownership remains reserved through cleanup, including a blocked transport
// write, so an old session's delayed stop cannot affect a replacement session.
type gamepadCoordinator struct {
	mu          sync.Mutex
	owner       *gamepadSession
	provider    types.IRosProvider
	epoch       atomic.Uint64
	active      atomic.Bool
	diagnostics gamepadDiagnosticHistory
}

type gamepadSession struct {
	coordinator *gamepadCoordinator
	conn        *websocket.Conn
	id          string
	client      string
	started     time.Time
	mu          sync.Mutex
	ended       bool
	deadman     bool
	state       uint8
	manualOwned bool
	lastValid   time.Time
	frames      uint64
	maxGap      time.Duration
	timer       *time.Timer
}

// Wire: every <=50 ms send {"twist": <TwistStamped>, "deadman": <bool>}.
// There is no reconnect/replay support: 300 ms without a validated frame ends
// the socket permanently. Deadman false carries zero motion and keeps only the
// connection lease alive. Its falling edge also exits MANUAL_MOWING via STOP.
func decodeGamepadFrame(raw []byte) (geometry.TwistStamped, bool, error) {
	var frame struct {
		Twist   *geometry.TwistStamped `json:"twist"`
		Deadman *bool                  `json:"deadman"`
	}
	decoder := json.NewDecoder(bytes.NewReader(raw))
	decoder.DisallowUnknownFields()
	if err := decoder.Decode(&frame); err != nil {
		return geometry.TwistStamped{}, false, err
	}
	if err := decoder.Decode(new(any)); err != io.EOF {
		return geometry.TwistStamped{}, false, errors.New("trailing gamepad JSON")
	}
	if frame.Twist == nil || frame.Deadman == nil {
		return geometry.TwistStamped{}, false, errors.New("twist and deadman are required")
	}
	// Missing/null components must not silently decode to a plausible zero.
	var fields map[string]json.RawMessage
	_ = json.Unmarshal(raw, &fields)
	var stamped map[string]json.RawMessage
	_ = json.Unmarshal(fields["twist"], &stamped)
	var twist map[string]json.RawMessage
	_ = json.Unmarshal(stamped["twist"], &twist)
	for _, component := range []string{"linear", "angular"} {
		var axes map[string]json.RawMessage
		_ = json.Unmarshal(twist[component], &axes)
		for _, axis := range []string{"x", "y", "z"} {
			value, present := axes[axis]
			if !present || bytes.Equal(bytes.TrimSpace(value), []byte("null")) {
				return geometry.TwistStamped{}, false, errors.New("all twist components are required")
			}
		}
	}
	t := frame.Twist.Twist
	values := []float64{t.Linear.X, t.Linear.Y, t.Linear.Z, t.Angular.X, t.Angular.Y, t.Angular.Z}
	for _, value := range values {
		if math.IsNaN(value) || math.IsInf(value, 0) {
			return geometry.TwistStamped{}, false, errors.New("non-finite gamepad velocity")
		}
	}
	if math.Abs(t.Linear.X) > 0.25 || math.Abs(t.Angular.Z) > 0.6 || t.Linear.Y != 0 || t.Linear.Z != 0 || t.Angular.X != 0 || t.Angular.Y != 0 {
		return geometry.TwistStamped{}, false, errors.New("gamepad velocity outside teleop envelope")
	}
	if !*frame.Deadman && (t.Linear.X != 0 || t.Angular.Z != 0) {
		return geometry.TwistStamped{}, false, errors.New("deadman released with nonzero velocity")
	}
	return *frame.Twist, *frame.Deadman, nil
}

func (c *gamepadCoordinator) publishLegacy(msg *geometry.TwistStamped, epoch uint64) error {
	if c.active.Load() || c.epoch.Load() != epoch {
		return errGamepadOwner
	}
	c.mu.Lock()
	defer c.mu.Unlock()
	if c.owner != nil || c.epoch.Load() != epoch {
		return errGamepadOwner
	}
	return c.provider.Publish("/cmd_vel_teleop", "geometry_msgs/msg/TwistStamped", msg)
}

func (c *gamepadCoordinator) handle(ctx *gin.Context) {
	if c.active.Load() {
		c.diagnostics.record(gamepadServerEvent{Event: "upgradeRejected", Cause: "active", Client: gamepadClientLabel(ctx.Query("client"))})
		ctx.AbortWithStatus(http.StatusConflict)
		return
	}
	c.mu.Lock()
	if c.owner != nil {
		c.mu.Unlock()
		c.diagnostics.record(gamepadServerEvent{Event: "upgradeRejected", Cause: "active", Client: gamepadClientLabel(ctx.Query("client"))})
		ctx.AbortWithStatus(http.StatusConflict)
		return
	}
	conn, err := upgrader.Upgrade(ctx.Writer, ctx.Request, nil)
	if err != nil {
		c.mu.Unlock()
		return
	}
	now := time.Now()
	s := &gamepadSession{coordinator: c, conn: conn, id: fmt.Sprintf("gamepad-%p", conn), client: gamepadClientLabel(ctx.Query("client")), started: now, lastValid: now}
	c.owner = s
	c.epoch.Add(1)
	c.active.Store(true)
	c.mu.Unlock()
	s.record(gamepadServerEvent{Event: "sessionAccepted"})
	conn.SetReadLimit(2048)
	if err := c.provider.Subscribe("highLevelStatus", s.id, 0, func(raw []byte) {
		var status mowgli.HighLevelStatus
		if json.Unmarshal(raw, &status) != nil {
			return
		}
		s.mu.Lock()
		s.state = status.State
		// Guard states (including emergency NULL and IDLE) can temporarily
		// mask a still-latched manual command. Retain its cleanup obligation
		// until the operator explicitly enters recording, whose trajectory a
		// STOP would discard.
		if status.State == 4 {
			s.manualOwned = true
		} else if status.State == 3 {
			s.manualOwned = false
		}
		s.mu.Unlock()
	}); err != nil {
		s.record(gamepadServerEvent{Event: "subscribeFailed", Error: err.Error()})
		s.terminate(gamepadServerEvent{Cause: "subscribeFailed", Error: err.Error()})
		return
	}
	// Subscribe may immediately replay cached status. This is a mode hint for
	// best-effort STOP, not proof of fresh physical observation or BT liveness.
	s.mu.Lock()
	s.timer = time.AfterFunc(gamepadLease, s.expire)
	s.mu.Unlock()
	defer s.end()
	// Preempt any already-active legacy touch command before accepting motion.
	if err := s.publish(&geometry.TwistStamped{}); err != nil {
		s.terminate(gamepadServerEvent{Cause: "publishFailed", Error: err.Error()})
		return
	}
	for {
		kind, raw, err := conn.ReadMessage()
		if err != nil {
			var closed *websocket.CloseError
			cause := gamepadServerEvent{Cause: "readError", Error: err.Error()}
			if errors.As(err, &closed) && closed.Code != websocket.CloseAbnormalClosure {
				cause = gamepadServerEvent{Cause: "browserClose", Code: closed.Code, Reason: closed.Text}
			}
			s.terminate(cause)
			return
		}
		if kind != websocket.TextMessage {
			s.terminate(gamepadServerEvent{Cause: "unsupportedFrame"})
			return
		}
		msg, deadman, err := decodeGamepadFrame(raw)
		if err != nil {
			s.terminate(gamepadServerEvent{Cause: "invalidFrame", Error: err.Error()})
			return
		}
		s.mu.Lock()
		if s.ended {
			s.mu.Unlock()
			return
		}
		released := s.deadman && !deadman
		s.deadman = deadman
		now := time.Now()
		if gap := now.Sub(s.lastValid); gap > s.maxGap {
			s.maxGap = gap
		}
		s.frames++
		s.lastValid = now
		s.timer.Reset(gamepadLease)
		state := s.state
		manualOwned := s.manualOwned
		s.mu.Unlock()
		// Manual and recording are the existing teleop modes. A false deadman
		// heartbeat is accepted in any state without initiating robot behavior.
		if deadman && state != 3 && state != 4 {
			s.terminate(gamepadServerEvent{Cause: "modeRejected"})
			return
		}
		if released && manualOwned {
			s.terminate(gamepadServerEvent{Cause: "deadmanReleased"})
			return
		}
		if err := s.publish(&msg); err != nil {
			s.terminate(gamepadServerEvent{Cause: "publishFailed", Error: err.Error()})
			return
		}
	}
}

func (s *gamepadSession) publish(msg *geometry.TwistStamped) error {
	c := s.coordinator
	c.mu.Lock()
	defer c.mu.Unlock()
	s.mu.Lock()
	ended := s.ended
	s.mu.Unlock()
	if ended || c.owner != s {
		return errors.New("gamepad session ended")
	}
	return c.provider.Publish("/cmd_vel_teleop", "geometry_msgs/msg/TwistStamped", msg)
}

func (s *gamepadSession) expire() {
	s.mu.Lock()
	if s.ended {
		s.mu.Unlock()
		return
	}
	remaining := gamepadLease - time.Since(s.lastValid)
	if remaining > 0 {
		s.timer.Reset(remaining)
		s.mu.Unlock()
		return
	}
	s.mu.Unlock()
	s.terminate(gamepadServerEvent{Cause: "leaseExpired"})
}

func (s *gamepadSession) stopManual() {
	s.mu.Lock()
	manual := s.manualOwned
	s.mu.Unlock()
	s.record(gamepadServerEvent{Event: "stopStarted", Skipped: !manual})
	if !manual {
		s.record(gamepadServerEvent{Event: "stopFinished", Skipped: true})
		return
	}
	ctx, cancel := context.WithTimeout(context.Background(), gamepadStopTimeout)
	defer cancel()
	var response mowgli.HighLevelControlRes
	err := s.coordinator.provider.CallService(ctx, "/behavior_tree_node/high_level_control", &mowgli.HighLevelControlReq{Command: 8}, &response, "mowgli_interfaces/srv/HighLevelControl")
	event := gamepadServerEvent{Event: "stopFinished", Success: &response.Success}
	if err != nil {
		event.Error = err.Error()
	}
	s.record(event)
}

func (s *gamepadSession) end() {
	s.terminate(gamepadServerEvent{Cause: "readError"})
}

func (s *gamepadSession) terminate(cause gamepadServerEvent) {
	s.mu.Lock()
	if s.ended {
		s.mu.Unlock()
		return
	}
	s.ended = true
	if gap := time.Since(s.lastValid); gap > s.maxGap {
		s.maxGap = gap
	}
	if s.timer != nil {
		s.timer.Stop()
	}
	s.mu.Unlock()
	_ = s.conn.Close()
	cause.Event = "sessionEnded"
	s.record(cause)
	// STOP must not wait behind a stalled velocity transport. Its own ROS
	// service context is bounded. A stuck zero write keeps ownership reserved
	// until it completes rather than permitting an unsafe delayed takeover.
	go func() {
		stopped := make(chan struct{})
		go func() { defer close(stopped); s.stopManual() }()
		c := s.coordinator
		s.record(gamepadServerEvent{Event: "cleanupZeroStarted"})
		var publishErr error
		c.mu.Lock()
		if c.owner == s {
			publishErr = c.provider.Publish("/cmd_vel_teleop", "geometry_msgs/msg/TwistStamped", &geometry.TwistStamped{})
		}
		c.mu.Unlock()
		zero := gamepadServerEvent{Event: "cleanupZeroFinished"}
		if publishErr != nil {
			zero.Error = publishErr.Error()
		}
		s.record(zero)
		<-stopped
		s.record(gamepadServerEvent{Event: "unsubscribeStarted"})
		c.provider.UnSubscribe("highLevelStatus", s.id)
		s.record(gamepadServerEvent{Event: "unsubscribeFinished"})
		c.mu.Lock()
		if c.owner == s {
			c.owner = nil
			c.active.Store(false)
		}
		c.mu.Unlock()
		s.record(gamepadServerEvent{Event: "ownerReleased"})
	}()
}

func (s *gamepadSession) record(event gamepadServerEvent) {
	s.mu.Lock()
	event.Session, event.Client = s.id, s.client
	event.Frames, event.State, event.ManualOwned = s.frames, s.state, s.manualOwned
	event.AgeMS = time.Since(s.started).Milliseconds()
	event.LastFrameMS = time.Since(s.lastValid).Milliseconds()
	event.MaxGapMS = s.maxGap.Milliseconds()
	s.mu.Unlock()
	s.coordinator.diagnostics.record(event)
}
