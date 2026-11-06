package api

import (
	"context"
	"encoding/json"
	"errors"
	"fmt"
	"net"
	"net/http"
	"net/http/httptest"
	"strings"
	"sync"
	"sync/atomic"
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
	mu              sync.Mutex
	state           uint8
	cb              func([]byte)
	published       chan geometry.TwistStamped
	stopped         chan struct{}
	publishErr      error
	subscribeErr    error
	serviceErr      error
	blockMotion     <-chan struct{}
	blockStop       <-chan struct{}
	beforePublish   func(context.Context) error
	subscriptions   atomic.Int64
	unsubscriptions atomic.Int64
}

func (p *gamepadTestProvider) Subscribe(_ string, _ string, _ int, cb func([]byte)) error {
	p.subscriptions.Add(1)
	if p.subscribeErr != nil {
		return p.subscribeErr
	}
	p.mu.Lock()
	p.cb = cb
	p.mu.Unlock()
	cb([]byte(`{"state":` + string(rune('0'+p.state)) + `}`))
	return nil
}
func (p *gamepadTestProvider) UnSubscribe(string, string) { p.unsubscriptions.Add(1) }
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
func (p *gamepadTestProvider) PublishContext(ctx context.Context, topic string, msgType string, value interface{}) error {
	if p.beforePublish != nil {
		if err := p.beforePublish(ctx); err != nil {
			return err
		}
	}
	if err := ctx.Err(); err != nil {
		return err
	}
	return p.Publish(topic, msgType, value)
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

var gamepadTestOwners sync.Map // transport fixtures echo the current control snapshot

func gamepadHarness(t *testing.T, state uint8) (*gamepadTestProvider, *httptest.Server, *websocket.Conn) {
	t.Helper()
	p := &gamepadTestProvider{state: state, published: make(chan geometry.TwistStamped, 32), stopped: make(chan struct{}, 8)}
	coordinator := &gamepadCoordinator{provider: p}
	router := gin.New()
	group := router.Group("/api/mowglinext")
	registerPublisherRoutes(group, coordinator)
	registerServiceRoutes(group, p, coordinator)
	server := httptest.NewServer(router)
	t.Cleanup(server.Close)
	conn, _, err := websocket.DefaultDialer.Dial(gamepadURL(server), nil)
	require.NoError(t, err)
	t.Cleanup(func() { _ = conn.Close() })
	require.NoError(t, completeGamepadHandshake(conn))
	coordinator.mu.Lock()
	gamepadTestOwners.Store(conn, coordinator.owner)
	coordinator.mu.Unlock()
	t.Cleanup(func() { gamepadTestOwners.Delete(conn) })
	require.Zero(t, readGamepadVelocity(t, p).Twist.Linear.X) // preemption zero
	return p, server, conn
}
func gamepadURL(server *httptest.Server) string {
	return "ws" + strings.TrimPrefix(server.URL, "http") + "/api/mowglinext/publish/gamepad"
}

func sendGamepadClaim(conn *websocket.Conn) error {
	var hello struct {
		Type      string `json:"type"`
		Challenge string `json:"challenge"`
		Version   int    `json:"version"`
	}
	if err := conn.ReadJSON(&hello); err != nil {
		return err
	}
	if hello.Type != "gamepadHello" || hello.Version != gamepadProtocolVersion || len(hello.Challenge) != 32 {
		return errors.New("invalid gamepad hello")
	}
	return conn.WriteJSON(map[string]any{"type": "gamepadClaim", "challenge": hello.Challenge, "version": gamepadProtocolVersion})
}

func completeGamepadHandshake(conn *websocket.Conn) error {
	if err := sendGamepadClaim(conn); err != nil {
		return err
	}
	var ready struct {
		Type string `json:"type"`
	}
	if err := conn.ReadJSON(&ready); err != nil {
		return err
	}
	if ready.Type != "gamepadReady" {
		return errors.New("invalid gamepad ready")
	}
	return nil
}
func gamepadVelocity(x float64) geometry.TwistStamped {
	var msg geometry.TwistStamped
	msg.Twist.Linear.X = x
	return msg
}
func gamepadWire(msg geometry.TwistStamped, deadman bool) map[string]any {
	return map[string]any{"twist": msg, "deadman": deadman, "physicalDeadman": deadman, "neutral": msg.Twist.Linear.X == 0 && msg.Twist.Angular.Z == 0, "epoch": uint64(0)}
}
func writeGamepad(t *testing.T, conn *websocket.Conn, x float64, deadman bool) {
	t.Helper()
	frame := gamepadWire(gamepadVelocity(x), deadman)
	if value, ok := gamepadTestOwners.Load(conn); ok {
		s := value.(*gamepadSession)
		s.mu.Lock()
		frame["epoch"] = s.controlEpoch
		s.mu.Unlock()
	}
	require.NoError(t, conn.WriteJSON(frame))
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

func TestGamepadManualReleaseStopsAndKeepsConnection(t *testing.T) {
	p, _, conn := gamepadHarness(t, 4)
	writeGamepad(t, conn, 0.2, true)
	require.Equal(t, 0.2, readGamepadVelocity(t, p).Twist.Linear.X)
	writeGamepad(t, conn, 0, false)
	require.Zero(t, readGamepadVelocity(t, p).Twist.Linear.X)
	awaitGamepadStop(t, p)
	writeGamepad(t, conn, 0, false)
	require.Zero(t, readGamepadVelocity(t, p).Twist.Linear.X)
	require.Zero(t, p.unsubscriptions.Load())
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
			writeGamepad(t, conn, 0.2, true)
			require.Zero(t, readGamepadVelocity(t, p).Twist.Linear.X)
			require.Zero(t, p.unsubscriptions.Load())
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
			for {
				_, _, err := conn.ReadMessage()
				if err != nil {
					break
				}
			}
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
		raw, err := json.Marshal(gamepadWire(gamepadVelocity(item.x), item.deadman))
		require.NoError(t, err)
		_, _, err = decodeGamepadFrame(raw)
		require.Equal(t, item.valid, err == nil)
	}
	msg := gamepadVelocity(0)
	msg.Twist.Linear.Y = 0.1
	raw, _ := json.Marshal(gamepadWire(msg, true))
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
	require.NoError(t, sendGamepadClaim(conn))
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
	require.NoError(t, completeGamepadHandshake(conn))
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
	require.NoError(t, completeGamepadHandshake(gamepad))
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

func readGamepadHello(t *testing.T, conn *websocket.Conn) string {
	t.Helper()
	var hello struct {
		Type      string `json:"type"`
		Challenge string `json:"challenge"`
		Version   int    `json:"version"`
	}
	require.NoError(t, conn.ReadJSON(&hello))
	require.Equal(t, "gamepadHello", hello.Type)
	require.Equal(t, gamepadProtocolVersion, hello.Version)
	require.Regexp(t, "^[0-9a-f]{32}$", hello.Challenge)
	return hello.Challenge
}

func assertNoGamepadROS(t *testing.T, c *gamepadCoordinator, p *gamepadTestProvider) {
	t.Helper()
	require.False(t, c.active.Load())
	require.Zero(t, c.epoch.Load())
	require.Zero(t, p.subscriptions.Load())
	require.Zero(t, p.unsubscriptions.Load())
	select {
	case <-p.published:
		t.Fatal("unclaimed connection published velocity")
	default:
	}
	select {
	case <-p.stopped:
		t.Fatal("unclaimed connection issued STOP")
	default:
	}
}

func awaitGamepadHandshakeRejection(t *testing.T, c *gamepadCoordinator) {
	t.Helper()
	require.Eventually(t, func() bool {
		for _, event := range c.diagnostics.snapshot(false).Server {
			if event.Event == "handshakeRejected" {
				return true
			}
		}
		return false
	}, time.Second, time.Millisecond)
}

func TestGamepadUnclaimedOrInvalidHandshakeNeverTouchesROS(t *testing.T) {
	for _, kind := range []string{"cancel", "expired", "wrongChallenge", "wrongType", "unknownField", "trailingJSON", "binary", "oldVelocityProtocol", "unversionedClaim", "wrongVersion"} {
		t.Run(kind, func(t *testing.T) {
			c, p := diagnosticTestCoordinator()
			p.state = 4 // A later operator manual command must not acquire an old connection's STOP.
			server := httptest.NewServer(diagnosticRouter(c))
			defer server.Close()
			conn, _, err := websocket.DefaultDialer.Dial(gamepadURL(server), nil)
			require.NoError(t, err)
			defer conn.Close()
			challenge := readGamepadHello(t, conn)
			assertNoGamepadROS(t, c, p)
			switch kind {
			case "cancel":
				require.NoError(t, conn.Close())
			case "expired":
				// Read waits for the server deadline. An eventual echo cannot revive it.
				_ = conn.SetReadDeadline(time.Now().Add(time.Second))
				_, _, err := conn.ReadMessage()
				require.Error(t, err)
				_ = conn.WriteJSON(map[string]any{"type": "gamepadClaim", "challenge": challenge, "version": gamepadProtocolVersion})
			default:
				raw := fmt.Sprintf(`{"type":"gamepadClaim","challenge":%q,"version":3}`, challenge)
				messageType := websocket.TextMessage
				switch kind {
				case "unversionedClaim":
					raw = strings.Replace(raw, `,"version":3`, "", 1)
				case "wrongVersion":
					raw = strings.Replace(raw, `"version":3`, `"version":1`, 1)
				case "wrongChallenge":
					raw = `{"type":"gamepadClaim","challenge":"wrong","version":3}`
				case "wrongType":
					raw = strings.Replace(raw, "gamepadClaim", "gamepadReady", 1)
				case "unknownField":
					raw = strings.TrimSuffix(raw, "}") + `,"extra":true}`
				case "trailingJSON":
					raw += `{}`
				case "binary":
					messageType = websocket.BinaryMessage
				case "oldVelocityProtocol":
					raw = `{"twist":{},"deadman":false}`
				}
				require.NoError(t, conn.WriteMessage(messageType, []byte(raw)))
			}
			awaitGamepadHandshakeRejection(t, c)
			assertNoGamepadROS(t, c, p)
		})
	}
}

func TestGamepadCanceledDelayedUpgradeCannotStopLaterManualCommand(t *testing.T) {
	c, p := diagnosticTestCoordinator()
	entered, release, handled := make(chan struct{}), make(chan struct{}), make(chan struct{})
	router := diagnosticRouter(c)
	server := httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		close(entered)
		<-release // model a request buffered upstream until after client cancellation
		router.ServeHTTP(w, r)
		close(handled)
	}))
	defer server.Close()
	address := strings.TrimPrefix(server.URL, "http://")
	conn, err := net.Dial("tcp", address)
	require.NoError(t, err)
	defer conn.Close()
	_, err = fmt.Fprintf(conn, "GET /api/mowglinext/publish/gamepad HTTP/1.1\r\nHost: %s\r\nUpgrade: websocket\r\nConnection: Upgrade\r\nSec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==\r\nSec-WebSocket-Version: 13\r\n\r\n", address)
	require.NoError(t, err)
	<-entered
	require.NoError(t, conn.Close())
	p.state = 4 // operator starts Manual Mowing after abandoning the old attempt
	close(release)
	select {
	case <-handled:
	case <-time.After(time.Second):
		t.Fatal("canceled upgrade did not finish")
	}
	assertNoGamepadROS(t, c, p)
}

func TestGamepadClaimCannotReuseAnotherConnectionsChallenge(t *testing.T) {
	c, p := diagnosticTestCoordinator()
	server := httptest.NewServer(diagnosticRouter(c))
	defer server.Close()
	first, _, err := websocket.DefaultDialer.Dial(gamepadURL(server), nil)
	require.NoError(t, err)
	defer first.Close()
	old := readGamepadHello(t, first)
	second, _, err := websocket.DefaultDialer.Dial(gamepadURL(server), nil)
	require.NoError(t, err)
	defer second.Close()
	require.NotEqual(t, old, readGamepadHello(t, second))
	require.NoError(t, second.WriteJSON(map[string]any{"type": "gamepadClaim", "challenge": old, "version": gamepadProtocolVersion}))
	awaitGamepadHandshakeRejection(t, c)
	assertNoGamepadROS(t, c, p)
}

func TestGamepadPendingClaimsHaveOnlyOneROSOwner(t *testing.T) {
	c, p := diagnosticTestCoordinator()
	server := httptest.NewServer(diagnosticRouter(c))
	defer server.Close()
	first, _, err := websocket.DefaultDialer.Dial(gamepadURL(server), nil)
	require.NoError(t, err)
	defer first.Close()
	firstChallenge := readGamepadHello(t, first)
	second, _, err := websocket.DefaultDialer.Dial(gamepadURL(server), nil)
	require.NoError(t, err)
	defer second.Close()
	secondChallenge := readGamepadHello(t, second)
	assertNoGamepadROS(t, c, p)
	require.NoError(t, first.WriteJSON(map[string]any{"type": "gamepadClaim", "challenge": firstChallenge, "version": gamepadProtocolVersion}))
	var ready map[string]string
	require.NoError(t, first.ReadJSON(&ready))
	require.Equal(t, "gamepadReady", ready["type"])
	_ = readGamepadVelocity(t, p)
	require.NoError(t, second.WriteJSON(map[string]any{"type": "gamepadClaim", "challenge": secondChallenge, "version": gamepadProtocolVersion}))
	awaitGamepadHandshakeRejection(t, c)
	require.True(t, c.active.Load())
	require.EqualValues(t, 1, c.epoch.Load())
	require.EqualValues(t, 1, p.subscriptions.Load())
	require.Zero(t, p.unsubscriptions.Load())
	history, err := json.Marshal(c.diagnostics.snapshot(true))
	require.NoError(t, err)
	require.NotContains(t, string(history), firstChallenge)
	require.NotContains(t, string(history), secondChallenge)
	select {
	case <-p.published:
		t.Fatal("losing claim changed velocity")
	default:
	}
	select {
	case <-p.stopped:
		t.Fatal("losing claim issued STOP")
	default:
	}
}

func TestGamepadLateFrameCannotRenewBeforeDelayedTimerRuns(t *testing.T) {
	for _, concurrentExpiry := range []bool{false, true} {
		t.Run(fmt.Sprint(concurrentExpiry), func(t *testing.T) {
			c, p := diagnosticTestCoordinator()
			server := httptest.NewServer(diagnosticRouter(c))
			defer server.Close()
			conn, _, err := websocket.DefaultDialer.Dial(gamepadURL(server), nil)
			require.NoError(t, err)
			defer conn.Close()
			require.NoError(t, completeGamepadHandshake(conn))
			_ = readGamepadVelocity(t, p)
			c.mu.Lock()
			s := c.owner
			c.mu.Unlock()
			s.mu.Lock()
			s.timer.Stop() // model a timer whose callback has not received CPU yet
			s.lastValid = time.Now().Add(-gamepadLease - time.Millisecond)
			s.mu.Unlock()
			var expiry sync.WaitGroup
			if concurrentExpiry {
				expiry.Add(1)
				go func() { defer expiry.Done(); s.expire() }()
			}
			_, accepted := s.acceptFrame(true)
			require.False(t, accepted)
			expiry.Wait()
			events := awaitGamepadReleased(t, c)
			ended := 0
			for _, event := range events {
				if event.Event == "sessionEnded" {
					ended++
					require.Equal(t, "leaseExpired", event.Cause)
					require.Zero(t, event.Frames)
				}
			}
			require.Equal(t, 1, ended)
			require.Zero(t, readGamepadVelocity(t, p).Twist.Linear.X)
		})
	}
}

type blockedGamepadSubscriber struct {
	*gamepadTestProvider
	entered chan struct{}
	release chan struct{}
	block   atomic.Bool
}

func (p *blockedGamepadSubscriber) Subscribe(topic, id string, interval int, cb func([]byte)) error {
	if p.block.Swap(false) {
		close(p.entered)
		<-p.release
	}
	return p.gamepadTestProvider.Subscribe(topic, id, interval, cb)
}

func TestGamepadLeaseRunsDuringSubscribeAndIgnoresLateManualStatus(t *testing.T) {
	c, base := diagnosticTestCoordinator()
	p := &blockedGamepadSubscriber{gamepadTestProvider: base, entered: make(chan struct{}), release: make(chan struct{})}
	p.block.Store(true)
	c.provider = p
	var unblock sync.Once
	server := httptest.NewServer(diagnosticRouter(c))
	defer server.Close()
	defer unblock.Do(func() { close(p.release) })
	conn, _, err := websocket.DefaultDialer.Dial(gamepadURL(server), nil)
	require.NoError(t, err)
	defer conn.Close()
	require.NoError(t, sendGamepadClaim(conn))
	select {
	case <-p.entered:
	case <-time.After(time.Second):
		t.Fatal("Subscribe did not start")
	}
	// The lease must expire and publish cleanup zero before Subscribe returns.
	require.Zero(t, readGamepadVelocity(t, base).Twist.Linear.X)
	c.mu.Lock()
	s := c.owner
	c.mu.Unlock()
	require.NotNil(t, s)
	s.mu.Lock()
	ended := s.ended
	s.mu.Unlock()
	require.True(t, ended)
	require.True(t, c.active.Load(), "owner stays reserved through late subscription cleanup")
	require.Zero(t, p.unsubscriptions.Load())
	select {
	case <-p.stopped:
		t.Fatal("no manual mode was observed before expiry")
	default:
	}
	// An operator's subsequent manual command must not become this expired
	// session's STOP obligation when a cached callback eventually replays it.
	p.state = 4
	unblock.Do(func() { close(p.release) })
	awaitGamepadReleased(t, c)
	require.EqualValues(t, 1, p.subscriptions.Load())
	require.EqualValues(t, 1, p.unsubscriptions.Load())
	s.mu.Lock()
	manualOwned, state := s.manualOwned, s.state
	s.mu.Unlock()
	require.False(t, manualOwned)
	require.Zero(t, state)
	select {
	case <-p.stopped:
		t.Fatal("late callback issued STOP against subsequent manual mode")
	default:
	}
	next, _, err := websocket.DefaultDialer.Dial(gamepadURL(server), nil)
	require.NoError(t, err)
	defer next.Close()
	require.NoError(t, completeGamepadHandshake(next))
	require.Zero(t, readGamepadVelocity(t, base).Twist.Linear.X)
}

func TestGamepadClaimExpiresWhileWaitingForCoordinator(t *testing.T) {
	c, p := diagnosticTestCoordinator()
	server := httptest.NewServer(diagnosticRouter(c))
	defer server.Close()
	conn, _, err := websocket.DefaultDialer.Dial(gamepadURL(server), nil)
	require.NoError(t, err)
	defer conn.Close()
	challenge := readGamepadHello(t, conn)
	c.mu.Lock()
	locked := true
	defer func() {
		if locked {
			c.mu.Unlock()
		}
	}()
	require.NoError(t, conn.WriteJSON(map[string]any{"type": "gamepadClaim", "challenge": challenge, "version": gamepadProtocolVersion}))
	// Model another publisher occupying the coordinator across the complete
	// claim window. A valid echo must not gain ownership after that wait.
	<-time.After(gamepadLease + 25*time.Millisecond)
	c.mu.Unlock()
	locked = false
	awaitGamepadHandshakeRejection(t, c)
	assertNoGamepadROS(t, c, p)
	var cause string
	for _, event := range c.diagnostics.snapshot(false).Server {
		if event.Event == "handshakeRejected" {
			cause = event.Cause
		}
	}
	require.Equal(t, "claimExpired", cause)
}

func TestGamepadPublishRejectsExpiredFrameBeforeDelayedTimerRuns(t *testing.T) {
	c, p := diagnosticTestCoordinator()
	p.state = 4
	server := httptest.NewServer(diagnosticRouter(c))
	defer server.Close()
	conn, _, err := websocket.DefaultDialer.Dial(gamepadURL(server), nil)
	require.NoError(t, err)
	defer conn.Close()
	require.NoError(t, completeGamepadHandshake(conn))
	_ = readGamepadVelocity(t, p)
	c.mu.Lock()
	s := c.owner
	c.mu.Unlock()
	_, accepted := s.acceptFrame(true)
	require.True(t, accepted)
	s.mu.Lock()
	s.timer.Stop()
	s.lastValid = time.Now().Add(-gamepadLease - time.Millisecond)
	s.mu.Unlock()
	motion := gamepadVelocity(0.2)
	require.Error(t, s.publish(&motion))
	awaitGamepadStop(t, p)
	require.Zero(t, readGamepadVelocity(t, p).Twist.Linear.X)
	awaitGamepadReleased(t, c)
	select {
	case frame := <-p.published:
		t.Fatalf("expired frame forwarded: %v", frame)
	default:
	}
}

// A provider with only the old contract must never get leased motion through
// an implicit, uncancellable Publish fallback.
func TestGamepadRejectsProviderWithoutContextPublishing(t *testing.T) {
	p := &gamepadTestProvider{published: make(chan geometry.TwistStamped, 8)}
	legacyOnly := struct{ types.IRosProvider }{p}
	c := &gamepadCoordinator{provider: legacyOnly}
	server := httptest.NewServer(diagnosticRouter(c))
	defer server.Close()
	conn, response, err := websocket.DefaultDialer.Dial(gamepadURL(server), nil)
	if conn != nil {
		defer conn.Close()
	}
	require.Error(t, err)
	require.Equal(t, http.StatusServiceUnavailable, response.StatusCode)
	assertNoGamepadROS(t, c, p)
}

func TestGamepadQueuedPublishIsCanceledWithoutCancelingCleanupZero(t *testing.T) {
	for _, end := range []string{"lease", "termination"} {
		t.Run(end, func(t *testing.T) {
			c, p := diagnosticTestCoordinator()
			p.state = 4
			entered := make(chan context.Context, 1)
			release := make(chan struct{})
			var once sync.Once
			unblock := func() { once.Do(func() { close(release) }) }
			defer unblock()
			// Block only the motion publish after the initial preemption zero.
			var calls atomic.Int64
			p.beforePublish = func(ctx context.Context) error {
				if calls.Add(1) == 1 {
					return nil
				}
				entered <- ctx
				<-release
				return ctx.Err()
			}
			server := httptest.NewServer(diagnosticRouter(c))
			defer server.Close()
			conn, _, err := websocket.DefaultDialer.Dial(gamepadURL(server), nil)
			require.NoError(t, err)
			defer conn.Close()
			require.NoError(t, completeGamepadHandshake(conn))
			require.Zero(t, readGamepadVelocity(t, p).Twist.Linear.X)
			c.mu.Lock()
			s := c.owner
			c.mu.Unlock()
			writeGamepad(t, conn, 0.2, true)
			var publishCtx context.Context
			select {
			case publishCtx = <-entered:
			case <-time.After(time.Second):
				t.Fatal("motion did not enter provider")
			}
			s.mu.Lock()
			leaseEnd := s.lastValid.Add(gamepadLease)
			if end == "lease" {
				// The context must expire even when the watchdog has not run.
				s.timer.Stop()
			}
			s.mu.Unlock()
			deadline, ok := publishCtx.Deadline()
			require.True(t, ok)
			require.True(t, deadline.Equal(leaseEnd), "publish extended the accepted frame's lease")
			if end == "termination" {
				s.terminate(gamepadServerEvent{Cause: "testTermination"})
			}
			select {
			case <-publishCtx.Done():
			case <-time.After(time.Second):
				t.Fatal("queued motion was not canceled")
			}
			if end == "lease" {
				require.ErrorIs(t, publishCtx.Err(), context.DeadlineExceeded)
			} else {
				require.ErrorIs(t, publishCtx.Err(), context.Canceled)
			}
			unblock()
			awaitGamepadStop(t, p)
			require.Zero(t, readGamepadVelocity(t, p).Twist.Linear.X, "cleanup must send zero, never the queued motion")
			events := awaitGamepadReleased(t, c)
			ended := 0
			for _, event := range events {
				if event.Event == "sessionEnded" {
					ended++
					if end == "lease" {
						require.Equal(t, "leaseExpired", event.Cause)
					} else {
						require.Equal(t, "testTermination", event.Cause)
					}
				}
			}
			require.Equal(t, 1, ended)
			select {
			case msg := <-p.published:
				t.Fatalf("unexpected replay: %+v", msg)
			default:
			}
		})
	}
}

func TestGamepadHelloFailsClosedForCachedControllerParser(t *testing.T) {
	c, p := diagnosticTestCoordinator()
	server := httptest.NewServer(diagnosticRouter(c))
	defer server.Close()
	conn, _, err := websocket.DefaultDialer.Dial(gamepadURL(server), nil)
	require.NoError(t, err)
	defer conn.Close()
	var hello map[string]any
	require.NoError(t, conn.ReadJSON(&hello))
	// The previous browser build required exactly type/challenge. It must not
	// adopt a session capable of issuing its retired untagged STOP chain.
	legacyAccepts := hello["type"] == "gamepadHello" && len(hello) == 2
	require.False(t, legacyAccepts)
	require.EqualValues(t, gamepadProtocolVersion, hello["version"])
	assertNoGamepadROS(t, c, p)
	require.NoError(t, conn.Close())
	awaitGamepadHandshakeRejection(t, c)
	assertNoGamepadROS(t, c, p)
}
