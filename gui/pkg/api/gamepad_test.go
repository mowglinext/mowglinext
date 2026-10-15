package api

import (
	"context"
	"encoding/json"
	"errors"
	"fmt"
	"net/http"
	"net/http/httptest"
	"strings"
	"sync"
	"testing"
	"time"

	"github.com/gin-gonic/gin"
	"github.com/gorilla/websocket"
	"github.com/mowglinext/mowglinext/pkg/msgs/geometry"
	"github.com/mowglinext/mowglinext/pkg/msgs/mowgli"
	"github.com/mowglinext/mowglinext/pkg/types"
	"github.com/stretchr/testify/require"
)

type gamepadTestProvider struct {
	types.IRosProvider
	mu           sync.Mutex
	state        uint8
	cb           func([]byte)
	published    chan geometry.TwistStamped
	stopped      chan struct{}
	publishErr   error
	subscribeErr error
	serviceErr   error
	blockMotion  <-chan struct{}
	blockStop    <-chan struct{}
}

func (p *gamepadTestProvider) Subscribe(_ string, _ string, _ int, cb func([]byte)) error {
	if p.subscribeErr != nil {
		return p.subscribeErr
	}
	p.mu.Lock()
	p.cb = cb
	p.mu.Unlock()
	cb([]byte(`{"state":` + string(rune('0'+p.state)) + `}`))
	return nil
}
func (p *gamepadTestProvider) UnSubscribe(string, string) {}
func (p *gamepadTestProvider) emitState(state uint8) {
	p.mu.Lock()
	cb := p.cb
	p.mu.Unlock()
	raw, _ := json.Marshal(mowgli.HighLevelStatus{State: state})
	cb(raw)
}
func (p *gamepadTestProvider) Publish(_ string, _ string, value interface{}) error {
	msg := *value.(*geometry.TwistStamped)
	p.published <- msg
	if msg.Twist.Linear.X != 0 && p.blockMotion != nil {
		<-p.blockMotion
	}
	return p.publishErr
}
func (p *gamepadTestProvider) CallService(ctx context.Context, service string, req any, res any, _ ...string) error {
	if service != "/behavior_tree_node/high_level_control" || req.(*mowgli.HighLevelControlReq).Command != 8 {
		panic("unexpected gamepad service")
	}
	res.(*mowgli.HighLevelControlRes).Success = p.serviceErr == nil
	p.stopped <- struct{}{}
	if p.blockStop != nil {
		select {
		case <-p.blockStop:
		case <-ctx.Done():
			return ctx.Err()
		}
	}
	return p.serviceErr
}

func gamepadHarness(t *testing.T, state uint8) (*gamepadTestProvider, *httptest.Server, *websocket.Conn) {
	t.Helper()
	p := &gamepadTestProvider{state: state, published: make(chan geometry.TwistStamped, 32), stopped: make(chan struct{}, 8)}
	server := httptest.NewServer(setupMowgliNextRouter(p))
	t.Cleanup(server.Close)
	conn, _, err := websocket.DefaultDialer.Dial(gamepadURL(server), nil)
	require.NoError(t, err)
	t.Cleanup(func() { _ = conn.Close() })
	require.Zero(t, readGamepadVelocity(t, p).Twist.Linear.X) // preemption zero
	return p, server, conn
}
func gamepadURL(server *httptest.Server) string {
	return "ws" + strings.TrimPrefix(server.URL, "http") + "/api/mowglinext/publish/gamepad"
}
func gamepadVelocity(x float64) geometry.TwistStamped {
	var msg geometry.TwistStamped
	msg.Twist.Linear.X = x
	return msg
}
func writeGamepad(t *testing.T, conn *websocket.Conn, x float64, deadman bool) {
	t.Helper()
	require.NoError(t, conn.WriteJSON(map[string]any{"twist": gamepadVelocity(x), "deadman": deadman}))
}
func readGamepadVelocity(t *testing.T, p *gamepadTestProvider) geometry.TwistStamped {
	t.Helper()
	select {
	case value := <-p.published:
		return value
	case <-time.After(2 * time.Second):
		t.Fatal("no gamepad velocity")
		return geometry.TwistStamped{}
	}
}
func awaitGamepadStop(t *testing.T, p *gamepadTestProvider) {
	t.Helper()
	select {
	case <-p.stopped:
	case <-time.After(2 * time.Second):
		t.Fatal("no manual STOP")
	}
}

func TestGamepadManualReleaseStopsAndCloses(t *testing.T) {
	p, _, conn := gamepadHarness(t, 4)
	writeGamepad(t, conn, 0.2, true)
	require.Equal(t, 0.2, readGamepadVelocity(t, p).Twist.Linear.X)
	writeGamepad(t, conn, 0, false)
	require.Zero(t, readGamepadVelocity(t, p).Twist.Linear.X)
	awaitGamepadStop(t, p)
	_ = conn.SetReadDeadline(time.Now().Add(time.Second))
	_, _, err := conn.ReadMessage()
	require.Error(t, err)
}

func TestGamepadRecordingReleasePreservesRecording(t *testing.T) {
	p, _, conn := gamepadHarness(t, 3)
	writeGamepad(t, conn, 0.2, true)
	_ = readGamepadVelocity(t, p)
	writeGamepad(t, conn, 0, false)
	require.Zero(t, readGamepadVelocity(t, p).Twist.Linear.X)
	writeGamepad(t, conn, 0.1, true)
	require.Equal(t, 0.1, readGamepadVelocity(t, p).Twist.Linear.X)
	_ = conn.Close()
	_ = readGamepadVelocity(t, p)
	select {
	case <-p.stopped:
		t.Fatal("recording was discarded by STOP")
	default:
	}
}

func TestGamepadManualGuardDoesNotLoseStopOwnership(t *testing.T) {
	for _, guardedState := range []uint8{0, 1} {
		t.Run(fmt.Sprintf("guard-%d", guardedState), func(t *testing.T) {
			p, _, conn := gamepadHarness(t, 4)
			writeGamepad(t, conn, 0.2, true)
			_ = readGamepadVelocity(t, p)
			p.emitState(guardedState)
			writeGamepad(t, conn, 0, false)
			require.Zero(t, readGamepadVelocity(t, p).Twist.Linear.X)
			awaitGamepadStop(t, p)
			// A reset cannot revive this session, even if the BT reports the
			// old manual branch before its STOP acknowledgement propagates.
			p.emitState(4)
			_ = conn.SetReadDeadline(time.Now().Add(time.Second))
			_, _, err := conn.ReadMessage()
			require.Error(t, err)
		})
	}
}

func TestGamepadManualGuardFaultStillStops(t *testing.T) {
	for _, fault := range []string{"disconnect", "timeout", "malformed"} {
		t.Run(fault, func(t *testing.T) {
			p, _, conn := gamepadHarness(t, 4)
			writeGamepad(t, conn, 0.2, true)
			_ = readGamepadVelocity(t, p)
			p.emitState(0)
			switch fault {
			case "disconnect":
				_ = conn.Close()
			case "malformed":
				require.NoError(t, conn.WriteMessage(websocket.TextMessage, []byte("{")))
			}
			require.Zero(t, readGamepadVelocity(t, p).Twist.Linear.X)
			awaitGamepadStop(t, p)
		})
	}
}

func TestGamepadExplicitRecordingClearsManualOwnership(t *testing.T) {
	p, _, conn := gamepadHarness(t, 4)
	writeGamepad(t, conn, 0.2, true)
	_ = readGamepadVelocity(t, p)
	p.emitState(3)
	writeGamepad(t, conn, 0, false)
	require.Zero(t, readGamepadVelocity(t, p).Twist.Linear.X)
	writeGamepad(t, conn, 0.1, true)
	require.Equal(t, 0.1, readGamepadVelocity(t, p).Twist.Linear.X)
	_ = conn.Close()
	_ = readGamepadVelocity(t, p)
	select {
	case <-p.stopped:
		t.Fatal("explicit recording retained manual STOP ownership")
	default:
	}
}

func TestGamepadTimeoutAndDisconnectStopManual(t *testing.T) {
	for _, disconnect := range []bool{false, true} {
		name := "timeout"
		if disconnect {
			name = "disconnect"
		}
		t.Run(name, func(t *testing.T) {
			p, _, conn := gamepadHarness(t, 4)
			writeGamepad(t, conn, 0.2, true)
			_ = readGamepadVelocity(t, p)
			if disconnect {
				_ = conn.Close()
			}
			require.Zero(t, readGamepadVelocity(t, p).Twist.Linear.X)
			awaitGamepadStop(t, p)
			_ = conn.SetReadDeadline(time.Now().Add(time.Second))
			_, _, err := conn.ReadMessage()
			require.Error(t, err)
		})
	}
}

func TestGamepadOwnershipBlocksOtherGamepadAndLegacy(t *testing.T) {
	p, server, conn := gamepadHarness(t, 3)
	other, response, err := websocket.DefaultDialer.Dial(gamepadURL(server), nil)
	if other != nil {
		_ = other.Close()
	}
	require.Error(t, err)
	require.Equal(t, http.StatusConflict, response.StatusCode)
	legacyURL := strings.TrimSuffix(gamepadURL(server), "gamepad") + "joy"
	legacy, _, err := websocket.DefaultDialer.Dial(legacyURL, nil)
	require.NoError(t, err)
	defer legacy.Close()
	require.NoError(t, legacy.WriteJSON(gamepadVelocity(0.24)))
	// A gamepad frame is the synchronization point: no legacy frame may reach
	// the provider while the gamepad owns it, regardless of handler ordering.
	writeGamepad(t, conn, 0.1, true)
	require.Equal(t, 0.1, readGamepadVelocity(t, p).Twist.Linear.X)
	_ = legacy.Close()
	writeGamepad(t, conn, 0.15, true)
	require.Equal(t, 0.15, readGamepadVelocity(t, p).Twist.Linear.X)
}

func TestGamepadInvalidFrameTerminatesWithoutForwarding(t *testing.T) {
	for name, raw := range map[string]string{
		"missing":            "{}",
		"null":               "{\"twist\":null,\"deadman\":true}",
		"missing components": "{\"twist\":{},\"deadman\":true}",
		"malformed":          "[",
		"oversize":           strings.Repeat("x", 4096),
	} {
		t.Run(name, func(t *testing.T) {
			p, _, conn := gamepadHarness(t, 4)
			require.NoError(t, conn.WriteMessage(websocket.TextMessage, []byte(raw)))
			require.Zero(t, readGamepadVelocity(t, p).Twist.Linear.X)
			awaitGamepadStop(t, p)
		})
	}
}

func TestGamepadEnvelopeValidation(t *testing.T) {
	for _, item := range []struct {
		x       float64
		deadman bool
		valid   bool
	}{
		{0.25, true, true}, {-0.25, true, true}, {0, false, true}, {0.251, true, false}, {0.1, false, false},
	} {
		raw, err := json.Marshal(map[string]any{"twist": gamepadVelocity(item.x), "deadman": item.deadman})
		require.NoError(t, err)
		_, _, err = decodeGamepadFrame(raw)
		require.Equal(t, item.valid, err == nil)
	}
	msg := gamepadVelocity(0)
	msg.Twist.Linear.Y = 0.1
	raw, _ := json.Marshal(map[string]any{"twist": msg, "deadman": true})
	_, _, err := decodeGamepadFrame(raw)
	require.Error(t, err)
}

func TestGamepadPublishAndServiceFailureReleaseOwnership(t *testing.T) {
	p := &gamepadTestProvider{state: 4, published: make(chan geometry.TwistStamped, 32), stopped: make(chan struct{}, 8), publishErr: errors.New("offline"), serviceErr: errors.New("offline")}
	server := httptest.NewServer(setupMowgliNextRouter(p))
	defer server.Close()
	conn, _, err := websocket.DefaultDialer.Dial(gamepadURL(server), nil)
	require.NoError(t, err)
	defer conn.Close()
	_ = readGamepadVelocity(t, p) // failed initial preemption
	_ = readGamepadVelocity(t, p) // cleanup still attempts zero
	awaitGamepadStop(t, p)
	_ = conn.SetReadDeadline(time.Now().Add(time.Second))
	_, _, err = conn.ReadMessage()
	require.Error(t, err)
}

func TestGamepadExpiredSessionCannotRearmOrReviveLegacySocket(t *testing.T) {
	p := &gamepadTestProvider{published: make(chan geometry.TwistStamped, 8)}
	c := &gamepadCoordinator{provider: p}
	oldEpoch := c.epoch.Load()
	c.epoch.Add(1) // a gamepad session came and went
	require.ErrorIs(t, c.publishLegacy(&geometry.TwistStamped{}, oldEpoch), errGamepadOwner)
	s := &gamepadSession{coordinator: c, ended: true}
	c.owner = s
	require.Error(t, s.publish(&geometry.TwistStamped{}))
	select {
	case <-p.published:
		t.Fatal("an expired producer published")
	default:
	}
}

func TestGamepadWatchdogStopsWhileVelocityWriteIsBlocked(t *testing.T) {
	release := make(chan struct{})
	p := &gamepadTestProvider{state: 4, published: make(chan geometry.TwistStamped, 8), stopped: make(chan struct{}, 8), blockMotion: release}
	server := httptest.NewServer(setupMowgliNextRouter(p))
	defer server.Close()
	conn, _, err := websocket.DefaultDialer.Dial(gamepadURL(server), nil)
	require.NoError(t, err)
	defer conn.Close()
	defer close(release)
	_ = readGamepadVelocity(t, p)
	writeGamepad(t, conn, 0.2, true)
	_ = readGamepadVelocity(t, p)
	awaitGamepadStop(t, p)
	_, response, err := websocket.DefaultDialer.Dial(gamepadURL(server), nil)
	require.Error(t, err)
	require.Equal(t, http.StatusConflict, response.StatusCode)
}

func TestGamepadFreshLegacySocketWaitsThroughCleanup(t *testing.T) {
	release := make(chan struct{})
	p := &gamepadTestProvider{state: 4, published: make(chan geometry.TwistStamped, 8), stopped: make(chan struct{}, 8), blockStop: release}
	coordinator := &gamepadCoordinator{provider: p}
	router := gin.New()
	registerPublisherRoutes(router.Group("/api/mowglinext"), coordinator)
	server := httptest.NewServer(router)
	defer server.Close()
	legacyURL := strings.TrimSuffix(gamepadURL(server), "gamepad") + "joy"
	stale, _, err := websocket.DefaultDialer.Dial(legacyURL, nil)
	require.NoError(t, err)
	defer stale.Close()
	gamepad, _, err := websocket.DefaultDialer.Dial(gamepadURL(server), nil)
	require.NoError(t, err)
	_ = readGamepadVelocity(t, p)
	_ = gamepad.Close()
	awaitGamepadStop(t, p) // STOP is deliberately still in flight
	require.Zero(t, readGamepadVelocity(t, p).Twist.Linear.X)

	fresh, _, err := websocket.DefaultDialer.Dial(legacyURL, nil)
	require.NoError(t, err)
	defer fresh.Close()
	// A pong proves the server handled (and dropped) the preceding movement
	// before cleanup is released, without sleeps or timing assumptions.
	pong := make(chan struct{})
	fresh.SetPongHandler(func(string) error { close(pong); return nil })
	go func() { _, _, _ = fresh.ReadMessage() }()
	require.NoError(t, fresh.WriteJSON(gamepadVelocity(0.2)))
	require.NoError(t, fresh.WriteControl(websocket.PingMessage, nil, time.Now().Add(time.Second)))
	select {
	case <-pong:
	case <-time.After(time.Second):
		t.Fatal("fresh legacy socket closed or failed to process its ignored frame")
	}
	select {
	case <-p.published:
		t.Fatal("legacy frame was forwarded during controller cleanup")
	default:
	}
	close(release)
	require.Eventually(t, func() bool { return !coordinator.active.Load() }, time.Second, time.Millisecond)
	require.NoError(t, fresh.WriteJSON(gamepadVelocity(0.15)))
	require.Equal(t, 0.15, readGamepadVelocity(t, p).Twist.Linear.X)

	// The pre-controller socket stays invalid after cleanup, even though the
	// fresh socket can now move. Its buffered input may never take over.
	require.NoError(t, stale.WriteJSON(gamepadVelocity(0.24)))
	_ = stale.SetReadDeadline(time.Now().Add(time.Second))
	_, _, err = stale.ReadMessage()
	require.Error(t, err)
	select {
	case <-p.published:
		t.Fatal("stale legacy socket regained ownership")
	default:
	}
}
