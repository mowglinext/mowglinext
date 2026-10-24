package api

import (
	"context"
	"encoding/json"
	"errors"
	"net/http"
	"net/http/httptest"
	"strings"
	"sync/atomic"
	"testing"
	"time"

	"github.com/gin-gonic/gin"
	"github.com/gorilla/websocket"
	"github.com/mowglinext/mowglinext/pkg/msgs/geometry"
	"github.com/mowglinext/mowglinext/pkg/msgs/mowgli"
	"github.com/stretchr/testify/require"
)

type manualTestProvider struct {
	*gamepadTestProvider
	commands chan uint8
	modeGate <-chan struct{}
	stopGate <-chan struct{}
	modeErr  error
	stopErr  error
}

func (p *manualTestProvider) CallService(ctx context.Context, service string, req, res any, _ ...string) error {
	if service == "/hardware_bridge/emergency_stop" {
		res.(*mowgli.EmergencyStopRes).Success = true
		return nil
	}
	cmd := req.(*mowgli.HighLevelControlReq).Command
	p.commands <- cmd
	gate, err := p.modeGate, p.modeErr
	if cmd == 8 {
		gate, err = p.stopGate, p.stopErr
	}
	if gate != nil {
		select {
		case <-gate:
		case <-ctx.Done():
			return ctx.Err()
		}
	}
	res.(*mowgli.HighLevelControlRes).Success = err == nil
	return err
}

type manualHarness struct {
	t       *testing.T
	p       *manualTestProvider
	c       *gamepadCoordinator
	s       *gamepadSession
	conn    *websocket.Conn
	router  *gin.Engine
	epoch   uint64
	resumed bool
}

func newManualHarness(t *testing.T, stopGate, modeGate <-chan struct{}, stopErr, modeErr error) *manualHarness {
	t.Helper()
	p := &manualTestProvider{gamepadTestProvider: &gamepadTestProvider{state: 4, published: make(chan geometry.TwistStamped, 256)}, commands: make(chan uint8, 32), stopGate: stopGate, modeGate: modeGate, stopErr: stopErr, modeErr: modeErr}
	c := &gamepadCoordinator{provider: p}
	router := gin.New()
	group := router.Group("/api/mowglinext")
	registerServiceRoutes(group, p, c)
	registerPublisherRoutes(group, c)
	server := httptest.NewServer(router)
	t.Cleanup(server.Close)
	conn, _, err := websocket.DefaultDialer.Dial(gamepadURL(server), nil)
	require.NoError(t, err)
	t.Cleanup(func() { _ = conn.Close() })
	require.NoError(t, completeGamepadHandshake(conn))
	require.Zero(t, readGamepadVelocity(t, p.gamepadTestProvider).Twist.Linear.X)
	c.mu.Lock()
	s := c.owner
	c.mu.Unlock()
	return &manualHarness{t: t, p: p, c: c, s: s, conn: conn, router: router}
}
func (h *manualHarness) input(physical, neutral, effective bool, x float64) {
	h.t.Helper()
	frame := gamepadWire(gamepadVelocity(x), effective)
	frame["physicalDeadman"], frame["neutral"], frame["epoch"] = physical, neutral, h.epoch
	require.NoError(h.t, h.conn.WriteJSON(frame))
}
func (h *manualHarness) zero() {
	h.t.Helper()
	require.Zero(h.t, readGamepadVelocity(h.t, h.p.gamepadTestProvider).Twist.Linear.X)
}
func (h *manualHarness) notice(phase string) {
	h.t.Helper()
	require.NoError(h.t, h.conn.SetReadDeadline(time.Now().Add(time.Second)))
	var notice struct {
		Type    string `json:"type"`
		Phase   string `json:"phase"`
		Epoch   uint64 `json:"epoch"`
		Resumed bool   `json:"resumed"`
	}
	require.NoError(h.t, h.conn.ReadJSON(&notice))
	require.Equal(h.t, "gamepadControl", notice.Type)
	require.Equal(h.t, phase, notice.Phase)
	require.Greater(h.t, notice.Epoch, h.epoch)
	h.epoch = notice.Epoch
	h.resumed = notice.Resumed
	require.NoError(h.t, h.conn.SetReadDeadline(time.Time{}))
}
func (h *manualHarness) command(want uint8) {
	h.t.Helper()
	select {
	case got := <-h.p.commands:
		require.Equal(h.t, want, got)
	case <-time.After(time.Second):
		h.t.Fatalf("missing command %d", want)
	}
}
func (h *manualHarness) noCommand() {
	h.t.Helper()
	select {
	case got := <-h.p.commands:
		h.t.Fatalf("unexpected command %d", got)
	case <-time.After(20 * time.Millisecond):
	}
}
func (h *manualHarness) wait(fn func() bool) {
	h.t.Helper()
	require.Eventually(h.t, fn, 200*time.Millisecond, time.Millisecond)
}
func (h *manualHarness) status(state uint8) {
	h.p.mu.Lock()
	cb := h.p.cb
	h.p.mu.Unlock()
	name := map[uint8]string{1: "IDLE", 2: "MOWING", 3: "RECORDING", 4: "MANUAL_MOWING"}[state]
	raw := []byte(`{"state":` + string(rune('0'+state)) + `,"state_name":"` + name + `"}`)
	cb(raw)
}
func (h *manualHarness) active() {
	h.input(false, true, false, 0)
	h.zero()
	h.input(true, true, true, 0)
	h.zero()
	h.notice("active")
	require.False(h.t, h.resumed)
	h.input(true, false, true, .2)
	require.Equal(h.t, .2, readGamepadVelocity(h.t, h.p.gamepadTestProvider).Twist.Linear.X)
}
func (h *manualHarness) release() {
	h.input(false, false, false, 0)
	h.zero()
	h.command(8)
	h.notice("pausing")
}
func (h *manualHarness) paused() {
	h.wait(func() bool { h.s.mu.Lock(); defer h.s.mu.Unlock(); return h.s.pauseStopSucceeded })
	h.status(1)
	h.notice("paused")
}
func (h *manualHarness) resume() {
	h.input(false, true, false, 0)
	h.zero()
	h.input(true, true, false, 0)
	h.zero()
	h.command(7)
	h.notice("resuming")
}
func (h *manualHarness) settledResume() {
	h.wait(func() bool { h.s.mu.Lock(); defer h.s.mu.Unlock(); return h.s.resumeSucceeded })
}
func (h *manualHarness) http(command, body string) int {
	req := httptest.NewRequest(http.MethodPost, "/api/mowglinext/call/"+command, strings.NewReader(body))
	req.Header.Set("Content-Type", "application/json")
	w := httptest.NewRecorder()
	h.router.ServeHTTP(w, req)
	return w.Code
}

func TestManualPauseResumeRepeatWithNeutralAndEpochFences(t *testing.T) {
	h := newManualHarness(t, nil, nil, nil, nil)
	h.active()
	for cycle := 0; cycle < 3; cycle++ {
		h.release()
		h.paused()
		// A stale epoch cannot arm or start the next manual session.
		epoch := h.epoch
		h.epoch--
		h.input(false, true, false, 0)
		h.zero()
		h.input(true, true, false, 0)
		h.zero()
		h.noCommand()
		h.epoch = epoch
		// Releasing with deflected axes and pressing while deflected never starts.
		h.input(false, false, false, 0)
		h.zero()
		h.input(true, false, false, 0)
		h.zero()
		h.noCommand()
		h.resume()
		h.settledResume()
		// Success alone never enables velocity; require a new manual status.
		h.input(true, false, true, .2)
		h.zero()
		h.input(true, true, false, 0)
		h.zero()
		h.status(4)
		h.notice("active")
		require.True(t, h.resumed)
		// An old active epoch from the prior cycle cannot carry velocity either.
		epoch = h.epoch
		h.epoch--
		h.input(true, false, true, .2)
		h.zero()
		h.epoch = epoch
		h.input(true, false, true, .2)
		require.Equal(t, .2, readGamepadVelocity(t, h.p.gamepadTestProvider).Twist.Linear.X)
	}
}

func TestManualPauseSlowStopDoesNotBlockInputOrQueueResume(t *testing.T) {
	gate := make(chan struct{})
	defer func() {
		select {
		case <-gate:
		default:
			close(gate)
		}
	}()
	h := newManualHarness(t, gate, nil, nil, nil)
	h.active()
	h.release()
	h.status(1) // Before STOP completion cannot grant a paused capability.
	for i := 0; i < 9; i++ {
		h.input(true, true, false, 0)
		h.zero()
		time.Sleep(40 * time.Millisecond)
	}
	h.s.mu.Lock()
	ended := h.s.ended
	h.s.mu.Unlock()
	require.False(t, ended)
	h.noCommand()
	close(gate)
	h.paused()
	h.input(true, true, false, 0)
	h.zero()
	h.noCommand() // held before ready has no edge
	h.resume()
	h.settledResume()
	h.status(4)
	h.notice("active")
}

func TestManualPauseStopFailureRevokesResume(t *testing.T) {
	gate := make(chan struct{})
	h := newManualHarness(t, gate, nil, errors.New("STOP failed"), nil)
	h.active()
	h.release()
	close(gate)
	h.notice("inactive")
	h.status(1)
	h.input(false, true, false, 0)
	h.zero()
	h.input(true, true, false, 0)
	h.zero()
	h.noCommand()
	require.Zero(t, h.p.unsubscriptions.Load())
}

func TestManualReleasePendingResumeStopsAndCompensatesLateStart(t *testing.T) {
	gate := make(chan struct{})
	defer func() {
		select {
		case <-gate:
		default:
			close(gate)
		}
	}()
	h := newManualHarness(t, nil, gate, nil, nil)
	h.active()
	h.release()
	h.paused()
	h.resume()
	// Effective deadman remains false while waiting; physical release still STOPs.
	h.input(false, true, false, 0)
	h.zero()
	h.command(8)
	h.notice("inactive")
	close(gate)
	h.command(8)
	h.status(4)
	h.input(true, false, true, .2)
	h.zero()
	h.noCommand()
	require.Zero(t, h.p.unsubscriptions.Load())
}

func TestManualResumeFailureCannotRetryOnHeldOrStatus(t *testing.T) {
	gate := make(chan struct{})
	h := newManualHarness(t, nil, gate, nil, errors.New("manual rejected"))
	h.active()
	h.release()
	h.paused()
	h.resume()
	close(gate)
	h.notice("inactive")
	h.command(8)
	h.command(8)
	h.status(4)
	h.input(true, true, false, 0)
	h.zero()
	h.noCommand()
	h.input(false, true, false, 0)
	h.zero()
	h.input(true, true, false, 0)
	h.zero()
	// The latter release may best-effort STOP an uncertain failed start, but never retries7.
	for {
		select {
		case command := <-h.p.commands:
			require.Equal(t, uint8(8), command)
		default:
			return
		}
	}
}

func TestManualExplicitIntentRevokesPausedPermission(t *testing.T) {
	for _, intent := range []string{"stop", "record", "emergency"} {
		t.Run(intent, func(t *testing.T) {
			h := newManualHarness(t, nil, nil, nil, nil)
			h.active()
			h.release()
			h.paused()
			switch intent {
			case "stop":
				require.Equal(t, http.StatusOK, h.http("high_level_control", `{"command":8}`))
				h.command(8)
			case "record":
				require.Equal(t, http.StatusOK, h.http("high_level_control", `{"command":3}`))
				h.command(3)
			case "emergency":
				require.Equal(t, http.StatusOK, h.http("emergency", `{"emergency":1}`))
			}
			h.notice("inactive")
			h.status(4) // Late old MANUAL status cannot revive motion or resume.
			h.input(false, true, false, 0)
			h.zero()
			h.input(true, true, true, 0)
			h.zero()
			h.input(true, false, true, .2)
			h.zero()
			h.noCommand()
		})
	}
}

func TestManualConfirmationRequiresHeldNeutralAndFreshStatus(t *testing.T) {
	h := newManualHarness(t, nil, nil, nil, nil)
	h.active()
	h.release()
	h.paused()
	h.resume()
	h.settledResume()
	h.input(true, false, false, 0)
	h.zero()
	h.status(4)
	h.notice("inactive")
	h.command(8)
	h.input(true, true, false, 0)
	h.zero()
	h.status(4)
	h.noCommand()
}

func TestManualStaleStatusEndsPausedSession(t *testing.T) {
	h := newManualHarness(t, nil, nil, nil, nil)
	h.active()
	h.release()
	h.paused()
	h.s.mu.Lock()
	h.s.statusAt = time.Now().Add(-3 * time.Second)
	h.s.mu.Unlock()
	h.input(false, true, false, 0)
	h.zero() // Fault cleanup zero, not permission renewal.
	h.wait(func() bool { h.s.mu.Lock(); defer h.s.mu.Unlock(); return h.s.ended })
}

func TestManualStopCancelsQueuedMotionWithoutDisconnecting(t *testing.T) {
	h := newManualHarness(t, nil, nil, nil, nil)
	h.active()
	entered := make(chan context.Context, 1)
	var first atomic.Bool
	h.p.beforePublish = func(ctx context.Context) error {
		if !first.CompareAndSwap(false, true) {
			return nil
		}
		entered <- ctx
		<-ctx.Done()
		return ctx.Err()
	}
	h.input(true, false, true, .23)
	var queued context.Context
	select {
	case queued = <-entered:
	case <-time.After(time.Second):
		t.Fatal("motion not queued")
	}
	stop, err := h.c.requestStop("")
	require.NoError(t, err)
	h.command(8)
	select {
	case <-queued.Done():
		require.ErrorIs(t, queued.Err(), context.Canceled)
	case <-time.After(100 * time.Millisecond):
		t.Fatal("STOP did not cancel queued velocity")
	}
	<-stop.done
	h.notice("inactive")
	h.input(true, false, true, .2)
	h.zero()
	h.s.mu.Lock()
	ended := h.s.ended
	h.s.mu.Unlock()
	require.False(t, ended)
	require.Zero(t, h.p.unsubscriptions.Load())
	h.noCommand()
}

func TestManualStatusBeforeResumeRPCSuccessDoesNotGrantMotion(t *testing.T) {
	gate := make(chan struct{})
	defer func() {
		select {
		case <-gate:
		default:
			close(gate)
		}
	}()
	h := newManualHarness(t, nil, gate, nil, nil)
	h.active()
	h.release()
	h.paused()
	h.resume()
	h.status(4)
	h.input(true, false, true, .2)
	h.zero()
	h.s.mu.Lock()
	phase := h.s.manualPhase
	h.s.mu.Unlock()
	require.Equal(t, "resuming", phase)
	h.input(true, true, false, 0)
	h.zero()
	close(gate)
	h.settledResume()
	h.input(true, false, true, .2)
	h.zero()
	h.input(true, true, false, 0)
	h.zero()
	h.status(4)
	h.notice("active")
	h.input(true, false, true, .2)
	require.Equal(t, .2, readGamepadVelocity(t, h.p.gamepadTestProvider).Twist.Linear.X)
}

func TestManualNewConnectionNeverInheritsPausedEpochOrToken(t *testing.T) {
	h := newManualHarness(t, nil, nil, nil, nil)
	h.active()
	h.release()
	h.paused()
	oldEpoch, oldToken := h.epoch, h.s.challenge
	require.NoError(t, h.conn.Close())
	h.wait(func() bool { return !h.c.active.Load() })
	server := httptest.NewServer(h.router)
	defer server.Close()
	conn, _, err := websocket.DefaultDialer.Dial(gamepadURL(server), nil)
	require.NoError(t, err)
	defer conn.Close()
	require.NoError(t, completeGamepadHandshake(conn))
	h.zero()
	// A replay from the retired socket can renew neither manual permission nor motion.
	frame := gamepadWire(gamepadVelocity(0), false)
	frame["epoch"] = oldEpoch
	require.NoError(t, conn.WriteJSON(frame))
	h.zero()
	frame["physicalDeadman"] = true
	require.NoError(t, conn.WriteJSON(frame))
	h.zero()
	frame["twist"], frame["deadman"], frame["neutral"] = gamepadVelocity(.2), true, false
	require.NoError(t, conn.WriteJSON(frame))
	h.zero()
	h.noCommand()
	req := httptest.NewRequest(http.MethodPost, "/api/mowglinext/call/high_level_control", strings.NewReader(`{"command":8}`))
	req.Header.Set("Content-Type", "application/json")
	req.Header.Set(gamepadActionHeader, oldToken)
	w := httptest.NewRecorder()
	h.router.ServeHTTP(w, req)
	require.Equal(t, http.StatusConflict, w.Code)
	h.c.mu.Lock()
	s := h.c.owner
	h.c.mu.Unlock()
	s.mu.Lock()
	phase, epoch := s.manualPhase, s.controlEpoch
	s.mu.Unlock()
	require.Equal(t, "inactive", phase)
	require.Zero(t, epoch)
}

func TestGamepadV3RequiresPhysicalIntentAndRejectsV2Claim(t *testing.T) {
	valid := gamepadWire(gamepadVelocity(0), false)
	for _, field := range []string{"physicalDeadman", "neutral", "epoch"} {
		t.Run(field, func(t *testing.T) {
			frame := make(map[string]any)
			for k, v := range valid {
				if k != field {
					frame[k] = v
				}
			}
			raw, err := json.Marshal(frame)
			require.NoError(t, err)
			_, _, err = decodeGamepadFrame(raw)
			require.Error(t, err)
		})
	}
	frame := gamepadWire(gamepadVelocity(.1), true)
	frame["physicalDeadman"] = false
	raw, err := json.Marshal(frame)
	require.NoError(t, err)
	_, _, err = decodeGamepadFrame(raw)
	require.Error(t, err)
	frame = gamepadWire(gamepadVelocity(.1), true)
	frame["neutral"] = true
	raw, err = json.Marshal(frame)
	require.NoError(t, err)
	_, _, err = decodeGamepadFrame(raw)
	require.Error(t, err)
	require.False(t, validGamepadClaim([]byte(`{"type":"gamepadClaim","challenge":"test","version":2}`), "test"))
}

func TestManualAcceptedFrameIsFencedAgainAtWriterAfterStatusTransition(t *testing.T) {
	h := newManualHarness(t, nil, nil, nil, nil)
	h.active()
	msg := gamepadVelocity(.23)
	epoch, accepted := h.s.processInput(&msg, true, gamepadPhysicalInput{PhysicalDeadman: true, Neutral: false, Epoch: h.epoch})
	require.True(t, accepted)
	require.Equal(t, .23, msg.Twist.Linear.X)
	h.status(3)
	h.notice("inactive")
	require.NoError(t, h.s.publish(&msg, epoch))
	h.zero()
	h.noCommand() // A late manual frame must neither drive nor discard recording.
}

func TestManualLateGenericStartCannotUndoEmergencyOrCompetingIntent(t *testing.T) {
	for _, intent := range []string{"emergency", "record"} {
		t.Run(intent, func(t *testing.T) {
			gate := make(chan struct{})
			defer func() {
				select {
				case <-gate:
				default:
					close(gate)
				}
			}()
			h := newManualHarness(t, nil, gate, nil, nil)
			result := make(chan int, 1)
			go func() { result <- h.http("high_level_control", `{"command":7}`) }()
			h.command(7)
			h.notice("inactive")
			switch intent {
			case "emergency":
				require.Equal(t, http.StatusOK, h.http("emergency", `{"emergency":1}`))
			case "record":
				require.Equal(t, http.StatusConflict, h.http("high_level_control", `{"command":3}`))
			}
			h.notice("inactive")
			h.command(8)
			close(gate)
			h.command(8)
			require.Equal(t, http.StatusOK, <-result)
			h.status(4)
			h.input(false, true, false, 0)
			h.zero()
			h.input(true, true, true, 0)
			h.zero()
			h.input(true, false, true, .2)
			h.zero()
			h.noCommand()
			h.s.mu.Lock()
			pending, phase := h.s.genericManualPending, h.s.manualPhase
			h.s.mu.Unlock()
			require.False(t, pending)
			require.Equal(t, "inactive", phase)
		})
	}
}

func TestManualGenericStartAwaitingStatusIsRevokedByEmergencyStatus(t *testing.T) {
	h := newManualHarness(t, nil, nil, nil, nil)
	require.Equal(t, http.StatusOK, h.http("high_level_control", `{"command":7}`))
	h.command(7)
	h.notice("inactive")
	h.s.observeStatus(mowgli.HighLevelStatus{State: 0, Emergency: true})
	h.notice("inactive")
	h.command(8)
	h.status(4)
	h.input(false, true, false, 0)
	h.zero()
	h.input(true, true, true, 0)
	h.zero()
	h.input(true, false, true, .2)
	h.zero()
	h.noCommand()
	h.s.mu.Lock()
	pending, phase := h.s.genericManualPending, h.s.manualPhase
	h.s.mu.Unlock()
	require.False(t, pending)
	require.Equal(t, "inactive", phase)
}

func TestManualStopKeepsConnectionForOldAndCurrentEpochHeldInputAcrossIdle(t *testing.T) {
	h := newManualHarness(t, nil, nil, nil, nil)
	h.active()
	oldEpoch := h.epoch
	stop, err := h.c.requestStop("")
	require.NoError(t, err)
	h.command(8)
	<-stop.done
	h.status(1)
	// This poll was prepared before the STOP notice reached the browser.
	h.epoch = oldEpoch
	h.input(true, false, true, .2)
	h.zero()
	h.notice("inactive")
	// Nor does a current epoch override a lagging MANUAL browser status.
	h.input(true, false, true, .2)
	h.zero()
	h.s.mu.Lock()
	ended := h.s.ended
	h.s.mu.Unlock()
	require.False(t, ended)
	h.noCommand()
	require.Zero(t, h.p.unsubscriptions.Load())
}

func TestManualHeldBeforeEnableCannotMintPausePermission(t *testing.T) {
	h := newManualHarness(t, nil, nil, nil, nil)
	h.input(true, true, false, 0)
	h.zero() // interlocked old physical hold
	h.input(false, true, false, 0)
	h.zero()
	h.command(8)
	h.notice("inactive")
	h.status(1)
	h.input(false, true, false, 0)
	h.zero()
	h.input(true, true, false, 0)
	h.zero()
	h.noCommand()
}

func TestManualFreshInvalidModeStillTerminates(t *testing.T) {
	h := newManualHarness(t, nil, nil, nil, nil)
	h.status(2)
	h.input(true, false, true, .2)
	h.zero()
	h.wait(func() bool { h.s.mu.Lock(); defer h.s.mu.Unlock(); return h.s.ended })
}
