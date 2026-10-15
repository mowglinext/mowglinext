package api

import (
	"bytes"
	"context"
	"encoding/json"
	"errors"
	"fmt"
	"io"
	"log"
	"math"
	"net/http"
	"strings"
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
	mu       sync.Mutex
	owner    *gamepadSession
	provider types.IRosProvider
	epoch    atomic.Uint64
	active   atomic.Bool
}

type gamepadSession struct {
	coordinator *gamepadCoordinator
	conn        *websocket.Conn
	id          string
	mu          sync.Mutex
	ended       bool
	deadman     bool
	state       uint8
	manualOwned bool
	lastValid   time.Time
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
		ctx.AbortWithStatus(http.StatusConflict)
		return
	}
	c.mu.Lock()
	if c.owner != nil {
		c.mu.Unlock()
		ctx.AbortWithStatus(http.StatusConflict)
		return
	}
	conn, err := upgrader.Upgrade(ctx.Writer, ctx.Request, nil)
	if err != nil {
		c.mu.Unlock()
		return
	}
	s := &gamepadSession{coordinator: c, conn: conn, id: fmt.Sprintf("gamepad-%p", conn), lastValid: time.Now()}
	c.owner = s
	c.epoch.Add(1)
	c.active.Store(true)
	c.mu.Unlock()
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
		s.end()
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
		return
	}
	for {
		kind, raw, err := conn.ReadMessage()
		if err != nil {
			var closed *websocket.CloseError
			if errors.As(err, &closed) && strings.HasPrefix(closed.Text, "input:") {
				// Client diagnostics only; never renew the lease or change cleanup.
				log.Printf("gamepad browser input loss: %q", closed.Text)
			}
			return
		}
		if kind != websocket.TextMessage {
			return
		}
		msg, deadman, err := decodeGamepadFrame(raw)
		if err != nil {
			return
		}
		s.mu.Lock()
		if s.ended {
			s.mu.Unlock()
			return
		}
		released := s.deadman && !deadman
		s.deadman = deadman
		s.lastValid = time.Now()
		s.timer.Reset(gamepadLease)
		state := s.state
		manualOwned := s.manualOwned
		s.mu.Unlock()
		// Manual and recording are the existing teleop modes. A false deadman
		// heartbeat is accepted in any state without initiating robot behavior.
		if deadman && state != 3 && state != 4 {
			return
		}
		if released && manualOwned {
			return
		}
		if err := s.publish(&msg); err != nil {
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
	s.end()
}

func (s *gamepadSession) stopManual() {
	s.mu.Lock()
	manual := s.manualOwned
	s.mu.Unlock()
	if !manual {
		return
	}
	ctx, cancel := context.WithTimeout(context.Background(), gamepadStopTimeout)
	defer cancel()
	var response mowgli.HighLevelControlRes
	err := s.coordinator.provider.CallService(ctx, "/behavior_tree_node/high_level_control", &mowgli.HighLevelControlReq{Command: 8}, &response, "mowgli_interfaces/srv/HighLevelControl")
	if err != nil || !response.Success {
		log.Printf("gamepad STOP was not acknowledged: error=%v success=%v", err, response.Success)
	}
}

func (s *gamepadSession) end() {
	s.mu.Lock()
	if s.ended {
		s.mu.Unlock()
		return
	}
	s.ended = true
	if s.timer != nil {
		s.timer.Stop()
	}
	s.mu.Unlock()
	_ = s.conn.Close()
	// STOP must not wait behind a stalled velocity transport. Its own ROS
	// service context is bounded. A stuck zero write keeps ownership reserved
	// until it completes rather than permitting an unsafe delayed takeover.
	go func() {
		stopped := make(chan struct{})
		go func() { defer close(stopped); s.stopManual() }()
		c := s.coordinator
		c.mu.Lock()
		if c.owner == s {
			if err := c.provider.Publish("/cmd_vel_teleop", "geometry_msgs/msg/TwistStamped", &geometry.TwistStamped{}); err != nil {
				log.Printf("gamepad stop velocity: %v", err)
			}
		}
		c.mu.Unlock()
		<-stopped
		c.provider.UnSubscribe("highLevelStatus", s.id)
		c.mu.Lock()
		if c.owner == s {
			c.owner = nil
			c.active.Store(false)
		}
		c.mu.Unlock()
	}()
}
