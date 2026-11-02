package api

import (
	"bufio"
	"bytes"
	"context"
	"encoding/json"
	"errors"
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
	"github.com/stretchr/testify/require"
)

type actionTestProvider struct {
	*gamepadTestProvider
	modeEntered      chan context.Context
	releaseMode      chan struct{}
	releaseFinalStop chan struct{}
	stops            chan int
	stopCount        atomic.Int32
	modeErr          error
	failFirstStop    bool
}

func (p *actionTestProvider) CallService(ctx context.Context, service string, req, res any, _ ...string) error {
	if service == "/hardware_bridge/emergency_stop" {
		res.(*mowgli.EmergencyStopRes).Success = true
		return nil
	}
	if control, ok := req.(*mowgli.HighLevelControlReq); ok && control.Command == 8 {
		n := int(p.stopCount.Add(1))
		p.stops <- n
		if n == 1 && p.failFirstStop {
			return errors.New("STOP transport failed")
		}
		if n == 2 {
			select {
			case <-p.releaseFinalStop:
			case <-ctx.Done():
				return ctx.Err()
			}
		}
	} else {
		p.modeEntered <- ctx
		select {
		case <-p.releaseMode:
		case <-ctx.Done():
			return ctx.Err()
		}
		if control, ok := req.(*mowgli.HighLevelControlReq); p.modeErr != nil &&
			((ok && control.Command == 3) || service == "/behavior_tree_node/clear_coverage_resume") {
			return p.modeErr
		}
	}
	return json.Unmarshal([]byte(`{"success":true}`), res)
}
func actionHarness(t *testing.T) (*actionTestProvider, *gamepadCoordinator, *gin.Engine, *httptest.Server) {
	t.Helper()
	p := &actionTestProvider{
		gamepadTestProvider: &gamepadTestProvider{state: 1, published: make(chan geometry.TwistStamped, 1024)},
		modeEntered:         make(chan context.Context, 32), releaseMode: make(chan struct{}),
		releaseFinalStop: make(chan struct{}), stops: make(chan int, 32),
	}
	c := &gamepadCoordinator{provider: p}
	router := gin.New()
	group := router.Group("/api/mowglinext")
	registerServiceRoutes(group, p, c)
	registerPublisherRoutes(group, c)
	server := httptest.NewServer(router)
	t.Cleanup(server.Close)
	return p, c, router, server
}
func actionOwner(t *testing.T, server *httptest.Server) (*websocket.Conn, string) {
	t.Helper()
	conn, _, err := websocket.DefaultDialer.Dial(gamepadURL(server), nil)
	require.NoError(t, err)
	t.Cleanup(func() { _ = conn.Close() })
	var hello struct {
		Challenge string `json:"challenge"`
	}
	require.NoError(t, conn.ReadJSON(&hello))
	require.NoError(t, conn.WriteJSON(map[string]any{"type": "gamepadClaim", "challenge": hello.Challenge, "version": gamepadProtocolVersion}))
	var ready struct {
		Type string `json:"type"`
	}
	require.NoError(t, conn.ReadJSON(&ready))
	require.Equal(t, "gamepadReady", ready.Type)
	done := make(chan struct{})
	t.Cleanup(func() { close(done) })
	go func() {
		ticker := time.NewTicker(25 * time.Millisecond)
		defer ticker.Stop()
		for {
			select {
			case <-done:
				return
			case <-ticker.C:
				if conn.WriteJSON(gamepadWire(gamepadVelocity(0), false)) != nil {
					return
				}
			}
		}
	}()
	return conn, hello.Challenge
}
func actionRequest(ctx context.Context, router http.Handler, command, body, token string) *httptest.ResponseRecorder {
	req := httptest.NewRequest(http.MethodPost, "/api/mowglinext/call/"+command, strings.NewReader(body)).WithContext(ctx)
	req.Header.Set("Content-Type", "application/json")
	if token != "" {
		req.Header.Set(gamepadActionHeader, token)
	}
	w := httptest.NewRecorder()
	router.ServeHTTP(w, req)
	return w
}
func actionAsync(ctx context.Context, router http.Handler, command, body, token string) <-chan *httptest.ResponseRecorder {
	done := make(chan *httptest.ResponseRecorder, 1)
	go func() { done <- actionRequest(ctx, router, command, body, token) }()
	return done
}
func actionReceive[T any](t *testing.T, ch <-chan T) T {
	t.Helper()
	select {
	case value := <-ch:
		return value
	case <-time.After(2 * time.Second):
		t.Fatal("action did not settle")
		var zero T
		return zero
	}
}
func actionRejected(t *testing.T, router http.Handler, command, body, token string) {
	t.Helper()
	require.Equal(t, http.StatusConflict, actionRequest(context.Background(), router, command, body, token).Code)
}
func actionOwnerBlocked(t *testing.T, server *httptest.Server) {
	t.Helper()
	conn, res, err := websocket.DefaultDialer.Dial(gamepadURL(server), nil)
	if conn != nil {
		_ = conn.Close()
	}
	require.Error(t, err)
	require.Equal(t, http.StatusConflict, res.StatusCode)
}

func TestGamepadActionLifetimeSurvivesCancellationAndReplacement(t *testing.T) {
	for _, failure := range []bool{false, true} {
		t.Run(map[bool]string{false: "late success", true: "failed request"}[failure], func(t *testing.T) {
			p, c, router, server := actionHarness(t)
			p.failFirstStop = true
			if failure {
				p.modeErr = errors.New("record response failed")
			}
			var modeOnce, stopOnce sync.Once
			releaseMode := func() { modeOnce.Do(func() { close(p.releaseMode) }) }
			releaseStop := func() { stopOnce.Do(func() { close(p.releaseFinalStop) }) }
			defer releaseStop()
			defer releaseMode()
			conn, token := actionOwner(t, server)
			requestCtx, cancel := context.WithCancel(context.Background())
			defer cancel()
			start := actionAsync(requestCtx, router, "high_level_control", `{"Command":3}`, token)
			serviceCtx := actionReceive(t, p.modeEntered)
			cancel()
			require.NoError(t, serviceCtx.Err(), "browser cancellation must not end action ownership")
			for _, body := range []string{`{"Command":7}`, `{"Command":1}`, `{"Command":3}`} {
				actionRejected(t, router, "high_level_control", body, "")
			}
			actionRejected(t, router, "start_in_area", `{"id":1}`, "")
			actionRejected(t, router, "coverage_clear_resume", `{}`, "")
			actionRejected(t, router, "high_level_control", `{"Command":5}`, token)
			require.Equal(t, http.StatusOK, actionRequest(context.Background(), router, "emergency", `{ "Emergency":1 }`, "").Code)
			stop := actionAsync(context.Background(), router, "high_level_control", `{"Command":8}`, token)
			require.Equal(t, 1, actionReceive(t, p.stops), "STOP must bypass pending recordStart")
			// Repeated STOP and socket cleanup join the same server-owned chain.
			duplicate, err := c.requestStop(token)
			require.NoError(t, err)
			_ = conn.Close()
			require.Eventually(t, func() bool {
				c.actionMu.Lock()
				defer c.actionMu.Unlock()
				c.actionOwner.mu.Lock()
				defer c.actionOwner.mu.Unlock()
				return c.actionOwner.ended
			}, time.Second, time.Millisecond)
			actionOwnerBlocked(t, server)
			actionRejected(t, router, "high_level_control", `{"Command":3}`, token)
			actionRejected(t, router, "high_level_control", `{"Command":8}`, token)
			releaseMode()
			expected := http.StatusOK
			if failure {
				expected = http.StatusInternalServerError
			}
			require.Equal(t, expected, actionReceive(t, start).Code)
			require.Equal(t, 2, actionReceive(t, p.stops), "late settlement must be followed by STOP despite first STOP failure")
			actionOwnerBlocked(t, server)
			actionRejected(t, router, "high_level_control", `{"Command":7}`, "")
			select {
			case <-duplicate.done:
				t.Fatal("STOP completed before compensation")
			default:
			}
			releaseStop()
			require.Equal(t, http.StatusOK, actionReceive(t, stop).Code)
			actionReceive(t, duplicate.done)
			require.EqualValues(t, 2, p.stopCount.Load(), "duplicate STOP must coalesce")
			require.Eventually(t, func() bool { return !c.active.Load() }, time.Second, time.Millisecond)
			// A remounted/full-page/new-tab hook gets a new challenge; retired requests
			// cannot affect its ordinary Manual Mowing mode.
			replacement, newToken := actionOwner(t, server)
			require.NotEqual(t, token, newToken)
			actionRejected(t, router, "high_level_control", `{"Command":8}`, token)
			manual := actionAsync(context.Background(), router, "high_level_control", `{"Command":7}`, "")
			actionReceive(t, p.modeEntered)
			require.Equal(t, http.StatusOK, actionReceive(t, manual).Code)
			_ = replacement.Close()
			require.Equal(t, 3, actionReceive(t, p.stops), "manual7 must retain cleanup even before mode status arrives")
		})
	}
}

func TestGamepadFailedActionCompensatesWithoutBrowserStop(t *testing.T) {
	p, c, router, server := actionHarness(t)
	p.modeErr = errors.New("response lost after dispatch")
	close(p.releaseFinalStop)
	conn, token := actionOwner(t, server)
	request := actionAsync(context.Background(), router, "high_level_control", `{"Command":3}`, token)
	actionReceive(t, p.modeEntered)
	close(p.releaseMode)
	require.Equal(t, http.StatusInternalServerError, actionReceive(t, request).Code)
	require.Equal(t, 1, actionReceive(t, p.stops))
	require.Equal(t, 2, actionReceive(t, p.stops))
	require.True(t, c.active.Load(), "compensation does not need browser cancellation")
	_ = conn.Close()
}

func TestGamepadLegacyModeInFlightBlocksNewOwner(t *testing.T) {
	p, _, router, server := actionHarness(t)
	close(p.releaseFinalStop)
	request := actionAsync(context.Background(), router, "high_level_control", `{"Command":7}`, "")
	actionReceive(t, p.modeEntered)
	// No socket owns the path, but admitting a new owner before this RPC settles
	// would let an old GUI mode switch land in the new session.
	conn, _, err := websocket.DefaultDialer.Dial(gamepadURL(server), nil)
	require.NoError(t, err)
	require.Error(t, completeGamepadHandshake(conn))
	_ = conn.Close()
	close(p.releaseMode)
	require.Equal(t, http.StatusOK, actionReceive(t, request).Code)
	_, _ = actionOwner(t, server)
}

func TestGamepadModeAdmissionSerializesCheckAndDispatch(t *testing.T) {
	c := &gamepadCoordinator{}
	var winners atomic.Int32
	var wg sync.WaitGroup
	finishers := make(chan func(error), 16)
	for range 16 {
		wg.Add(1)
		go func() {
			defer wg.Done()
			finish, _, err := c.beginMode("", 7)
			if err == nil {
				winners.Add(1)
				finishers <- finish
			}
		}()
	}
	wg.Wait()
	require.EqualValues(t, 1, winners.Load())
	(<-finishers)(nil)
	finish, _, err := c.beginMode("", 7)
	require.NoError(t, err)
	finish(nil)
}

func TestGamepadNoOwnerModeRetainsRequestCancellation(t *testing.T) {
	p, c, router, _ := actionHarness(t)
	requestCtx, cancel := context.WithCancel(context.Background())
	defer cancel()
	request := actionAsync(requestCtx, router, "high_level_control", `{"Command":7}`, "")
	serviceCtx := actionReceive(t, p.modeEntered)
	cancel()
	require.Equal(t, http.StatusInternalServerError, actionReceive(t, request).Code)
	require.ErrorIs(t, serviceCtx.Err(), context.Canceled)
	c.actionMu.Lock()
	require.Nil(t, c.modeAction)
	require.Nil(t, c.stopAction)
	c.actionMu.Unlock()
	require.Zero(t, p.stopCount.Load())
}

func TestGamepadStopBypassesVelocityWriterDuringPendingAction(t *testing.T) {
	p, c, router, server := actionHarness(t)
	close(p.releaseFinalStop)
	_, token := actionOwner(t, server)
	action := actionAsync(context.Background(), router, "high_level_control", `{"Command":3}`, token)
	actionReceive(t, p.modeEntered)
	// A stalled velocity writer holds coordinator.mu. STOP admission and its
	// prompt service call must remain independent of that transport lock.
	c.mu.Lock()
	stop, err := c.requestStop(token)
	require.NoError(t, err)
	require.Equal(t, 1, actionReceive(t, p.stops))
	c.mu.Unlock()
	close(p.releaseMode)
	require.Equal(t, http.StatusOK, actionReceive(t, action).Code)
	require.Equal(t, 2, actionReceive(t, p.stops))
	actionReceive(t, stop.done)
}

// Pause after Ready reaches the client but before the server's WriteJSON
// returns. HTTP actions must already see the same ready state as the client.
type readyBarrierWriter struct {
	http.ResponseWriter
	release <-chan struct{}
}

func (w readyBarrierWriter) Hijack() (net.Conn, *bufio.ReadWriter, error) {
	conn, rw, err := w.ResponseWriter.(http.Hijacker).Hijack()
	if err != nil {
		return conn, rw, err
	}
	return &readyBarrierConn{Conn: conn, release: w.release}, rw, nil
}

type readyBarrierConn struct {
	net.Conn
	release <-chan struct{}
}

func (c *readyBarrierConn) Write(data []byte) (int, error) {
	n, err := c.Conn.Write(data)
	if bytes.Contains(data, []byte("gamepadReady")) {
		<-c.release
	}
	return n, err
}

func TestGamepadReadyAdmitsActionBeforeHandshakeWriteReturns(t *testing.T) {
	p, _, router, _ := actionHarness(t)
	release := make(chan struct{})
	var once sync.Once
	unblock := func() { once.Do(func() { close(release) }) }
	defer unblock()
	defer close(p.releaseMode)
	defer close(p.releaseFinalStop)
	server := httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		router.ServeHTTP(readyBarrierWriter{ResponseWriter: w, release: release}, r)
	}))
	defer server.Close()
	_, token := actionOwner(t, server)
	result := actionAsync(context.Background(), router, "high_level_control", `{"Command":3}`, token)
	select {
	case <-p.modeEntered:
		// The request was admitted while Ready's writer was still blocked.
	case response := <-result:
		t.Fatalf("first action rejected after Ready: status %d", response.Code)
	case <-time.After(time.Second):
		t.Fatal("first action did not reach provider after Ready")
	}
	unblock()
}

func TestGamepadManualAdmissionSurvivesUncorrelatedRecordingStatus(t *testing.T) {
	p, c, router, server := actionHarness(t)
	p.state = 3
	close(p.releaseFinalStop)
	conn, _ := actionOwner(t, server)
	request := actionAsync(context.Background(), router, "high_level_control", `{"Command":7}`, "")
	actionReceive(t, p.modeEntered)
	p.emitState(3) // Previous recording still describes the robot during dispatch.
	close(p.releaseMode)
	require.Equal(t, http.StatusOK, actionReceive(t, request).Code)
	p.emitState(3) // Response success also does not identify this cached status.
	_ = conn.Close()
	require.Eventually(t, func() bool { return !c.active.Load() }, time.Second, time.Millisecond)
	require.EqualValues(t, 1, p.stopCount.Load(), "admitted manual7 must retain cleanup before manual status arrives")
}

func TestGamepadExplicitRecordingSupersedesManualAdmission(t *testing.T) {
	p, c, router, server := actionHarness(t)
	p.state = 3
	close(p.releaseMode)
	close(p.releaseFinalStop)
	conn, token := actionOwner(t, server)
	require.Equal(t, http.StatusOK, actionRequest(context.Background(), router, "high_level_control", `{"Command":7}`, "").Code)
	require.Equal(t, http.StatusOK, actionRequest(context.Background(), router, "high_level_control", `{"Command":3}`, token).Code)
	p.emitState(3)
	_ = conn.Close()
	require.Eventually(t, func() bool { return !c.active.Load() }, time.Second, time.Millisecond)
	require.Zero(t, p.stopCount.Load(), "explicit completed recording must survive disconnect")
}

func TestGamepadFailedStopRetainsAdmittedManualCleanup(t *testing.T) {
	p, c, router, server := actionHarness(t)
	p.state = 3
	p.failFirstStop = true
	close(p.releaseMode)
	close(p.releaseFinalStop)
	conn, token := actionOwner(t, server)
	require.Equal(t, http.StatusOK, actionRequest(context.Background(), router, "high_level_control", `{"Command":7}`, "").Code)
	require.Equal(t, http.StatusInternalServerError, actionRequest(context.Background(), router, "high_level_control", `{"Command":8}`, token).Code)
	p.emitState(3)
	_ = conn.Close()
	require.Eventually(t, func() bool { return !c.active.Load() }, time.Second, time.Millisecond)
	require.EqualValues(t, 2, p.stopCount.Load(), "failed STOP must not erase the disconnect cleanup obligation")
}

func TestGamepadManualCleanupSurvivesCoverageClearOnly(t *testing.T) {
	p, c, router, server := actionHarness(t)
	p.state = 4
	close(p.releaseMode)
	close(p.releaseFinalStop)
	conn, _ := actionOwner(t, server)
	require.Equal(t, http.StatusOK, actionRequest(context.Background(), router, "high_level_control", `{"Command":7}`, "").Code)
	require.Equal(t, http.StatusOK, actionRequest(context.Background(), router, "coverage_clear_resume", `{}`, "").Code)
	_ = conn.Close()
	require.Eventually(t, func() bool { return !c.active.Load() }, time.Second, time.Millisecond)
	require.EqualValues(t, 1, p.stopCount.Load(), "clear_coverage_resume does not supersede the manual command")
}

func TestGamepadCompletedRecordingSurvivesLateManualStatus(t *testing.T) {
	p, c, router, server := actionHarness(t)
	close(p.releaseMode)
	close(p.releaseFinalStop)
	conn, token := actionOwner(t, server)
	require.Equal(t, http.StatusOK, actionRequest(context.Background(), router, "high_level_control", `{"Command":7}`, "").Code)
	// Model RosSubscriber's asynchronous delivery: receive manual4 before the
	// new recording RPC, but allow its callback only after the RPC succeeds.
	p.mu.Lock()
	original := p.cb
	entered, deliver, delivered := make(chan struct{}), make(chan struct{}), make(chan struct{})
	p.cb = func(raw []byte) { close(entered); <-deliver; original(raw); close(delivered) }
	p.mu.Unlock()
	go p.emitState(4)
	actionReceive(t, entered)
	require.Equal(t, http.StatusOK, actionRequest(context.Background(), router, "high_level_control", `{"Command":3}`, token).Code)
	close(deliver)
	actionReceive(t, delivered)
	_ = conn.Close()
	require.Eventually(t, func() bool { return !c.active.Load() }, time.Second, time.Millisecond)
	require.Zero(t, p.stopCount.Load(), "completed recording should survive disconnect before its first status")
}

func TestGamepadPendingCoverageClearPreservesRecordingOnDisconnect(t *testing.T) {
	for _, fail := range []bool{false, true} {
		t.Run(map[bool]string{false: "success", true: "failure"}[fail], func(t *testing.T) {
			p, c, router, server := actionHarness(t)
			p.state = 3
			if fail {
				p.modeErr = errors.New("progress clear failed")
			}
			close(p.releaseFinalStop)
			conn, _ := actionOwner(t, server)
			request := actionAsync(context.Background(), router, "coverage_clear_resume", `{}`, "")
			actionReceive(t, p.modeEntered)
			_ = conn.Close()
			require.Eventually(t, func() bool {
				c.actionMu.Lock()
				defer c.actionMu.Unlock()
				c.actionOwner.mu.Lock()
				defer c.actionOwner.mu.Unlock()
				return c.actionOwner.ended
			}, time.Second, time.Millisecond)
			actionOwnerBlocked(t, server)
			close(p.releaseMode)
			expected := http.StatusOK
			if fail {
				expected = http.StatusInternalServerError
			}
			require.Equal(t, expected, actionReceive(t, request).Code)
			require.Eventually(t, func() bool { return !c.active.Load() }, time.Second, time.Millisecond)
			require.Zero(t, p.stopCount.Load(), "progress clearing cannot require discarding a recording")
		})
	}
}
