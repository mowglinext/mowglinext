package api

import (
	"context"
	"encoding/json"
	"errors"
	"fmt"
	"net/http"
	"net/http/httptest"
	"strings"
	"sync/atomic"
	"testing"
	"time"

	"github.com/gin-gonic/gin"
	"github.com/gorilla/websocket"
	"github.com/mowglinext/mowglinext/pkg/msgs/geometry"
	"github.com/stretchr/testify/require"
)

func diagnosticRouter(c *gamepadCoordinator) *gin.Engine {
	router := gin.New()
	registerPublisherRoutes(router.Group("/api/mowglinext"), c)
	return router
}

func postGamepadReport(router http.Handler, body, origin string) *httptest.ResponseRecorder {
	request := httptest.NewRequest(http.MethodPost, "http://mower.local/api/mowglinext/gamepad/diagnostics", strings.NewReader(body))
	request.Header.Set("Origin", origin)
	request.Header.Set("Content-Type", "application/json; charset=utf-8")
	response := httptest.NewRecorder()
	router.ServeHTTP(response, request)
	return response
}

func TestGamepadDiagnosticReportValidationAndBounds(t *testing.T) {
	c := &gamepadCoordinator{}
	router := diagnosticRouter(c)
	valid := `{"version":1,"clientId":"test","events":[{"at":"now","event":"inputLost","detail":{"reason":"focusLost"}}],"snapshot":{}}`
	require.Equal(t, http.StatusNoContent, postGamepadReport(router, valid, "http://mower.local").Code)
	for _, origin := range []string{"", "null", "http://mower.local.evil.com", "ftp://mower.local"} {
		require.Equal(t, http.StatusForbidden, postGamepadReport(router, valid, origin).Code)
	}
	for _, body := range []string{
		`{`,
		strings.Replace(valid, `"version":1`, `"version":2`, 1),
		strings.Replace(valid, `"test"`, `"`+strings.Repeat("a", 65)+`"`, 1),
		strings.Replace(valid, `"snapshot":{}`, `"snapshot":null`, 1),
		strings.Replace(valid, `"snapshot":{}`, `"snapshot":[]`, 1),
		strings.Replace(valid, `"snapshot":{}`, `"snapshot":{},"unknown":1`, 1),
		strings.Replace(valid, `"events":[`, `"events":null,"unused":[`, 1),
		strings.Replace(valid, `"detail":{"reason":"focusLost"}`, `"detail":[]`, 1),
		strings.Replace(valid, `"event":"inputLost"`, `"event":"inputLost","unknown":1`, 1),
		valid + `{}`,
		fmt.Sprintf(`{"version":1,"clientId":"test","events":[%s],"snapshot":{}}`, strings.TrimSuffix(strings.Repeat(`{"at":"now","event":"test"},`, 33), ",")),
	} {
		require.Equal(t, http.StatusBadRequest, postGamepadReport(router, body, "http://mower.local").Code, body)
	}
	oversized := strings.Replace(valid, `"snapshot":{}`, `"snapshot":{"padding":"`+strings.Repeat("x", gamepadReportLimit)+`"}`, 1)
	require.Equal(t, http.StatusRequestEntityTooLarge, postGamepadReport(router, oversized, "http://mower.local").Code)
	wrongType := httptest.NewRequest(http.MethodPost, "http://mower.local/api/mowglinext/gamepad/diagnostics", strings.NewReader(valid))
	wrongType.Header.Set("Origin", "http://mower.local")
	wrongType.Header.Set("Content-Type", "text/plain")
	wrongTypeResponse := httptest.NewRecorder()
	router.ServeHTTP(wrongTypeResponse, wrongType)
	require.Equal(t, http.StatusForbidden, wrongTypeResponse.Code)
	require.Len(t, c.diagnostics.snapshot(false).Browser, 1)
	for i := range 10 {
		report := fmt.Sprintf(`{"version":1,"clientId":"client%d","events":[],"snapshot":{}}`, i)
		require.Equal(t, http.StatusNoContent, postGamepadReport(router, report, "http://mower.local").Code)
	}
	history := c.diagnostics.snapshot(false)
	require.Len(t, history.Browser, 8)
	require.Contains(t, string(history.Browser[0].Report), "client2")
	require.Contains(t, string(history.Browser[7].Report), "client9")
	for i := range 70 {
		c.diagnostics.record(gamepadServerEvent{Event: fmt.Sprintf("test%d", i)})
	}
	history = c.diagnostics.snapshot(false)
	require.Len(t, history.Server, 64)
	require.Equal(t, "test6", history.Server[0].Event)
	require.Equal(t, "test69", history.Server[63].Event)
}

func TestGamepadDiagnosticsDoNotAcquireOwnershipLock(t *testing.T) {
	c := &gamepadCoordinator{}
	router := diagnosticRouter(c)
	c.active.Store(true)
	c.mu.Lock()
	defer c.mu.Unlock()
	done := make(chan *httptest.ResponseRecorder, 1)
	go func() {
		report := `{"version":1,"clientId":"test","events":[],"snapshot":{}}`
		post := postGamepadReport(router, report, "http://mower.local")
		if post.Code != http.StatusNoContent {
			done <- post
			return
		}
		response := httptest.NewRecorder()
		router.ServeHTTP(response, httptest.NewRequest(http.MethodGet, "/api/mowglinext/gamepad/diagnostics", nil))
		done <- response
	}()
	select {
	case response := <-done:
		require.Equal(t, http.StatusOK, response.Code)
		require.Equal(t, "no-store", response.Header().Get("Cache-Control"))
		var result gamepadDiagnosticResponse
		require.NoError(t, json.Unmarshal(response.Body.Bytes(), &result))
		require.True(t, result.Active)
		require.False(t, result.StartedAt.IsZero())
		require.NotNil(t, result.Server)
		require.NotNil(t, result.Browser)
	case <-time.After(time.Second):
		t.Fatal("diagnostics blocked behind ownership mutex")
	}
}

func diagnosticTestCoordinator() (*gamepadCoordinator, *gamepadTestProvider) {
	p := &gamepadTestProvider{state: 3, published: make(chan geometry.TwistStamped, 32), stopped: make(chan struct{}, 8)}
	return &gamepadCoordinator{provider: p}, p
}

func awaitGamepadReleased(t *testing.T, c *gamepadCoordinator) []gamepadServerEvent {
	t.Helper()
	require.Eventually(t, func() bool {
		for _, event := range c.diagnostics.snapshot(c.active.Load()).Server {
			if event.Event == "ownerReleased" {
				return true
			}
		}
		return false
	}, 2*time.Second, time.Millisecond)
	require.False(t, c.active.Load())
	return c.diagnostics.snapshot(false).Server
}

func TestGamepadDiagnosticLeaseCauseAndRecovery(t *testing.T) {
	c, p := diagnosticTestCoordinator()
	server := httptest.NewServer(diagnosticRouter(c))
	defer server.Close()
	conn, _, err := websocket.DefaultDialer.Dial(gamepadURL(server)+"?client=browser-1", nil)
	require.NoError(t, err)
	defer conn.Close()
	require.NoError(t, completeGamepadHandshake(conn))
	_ = readGamepadVelocity(t, p)
	writeGamepad(t, conn, 0, false)
	_ = readGamepadVelocity(t, p)
	_ = readGamepadVelocity(t, p) // lease-expiry cleanup zero
	events := awaitGamepadReleased(t, c)
	ended := 0
	for _, event := range events {
		if event.Event == "sessionEnded" {
			ended++
			require.Equal(t, "leaseExpired", event.Cause)
			require.Equal(t, "browser-1", event.Client)
			require.EqualValues(t, 1, event.Frames)
			require.GreaterOrEqual(t, event.LastFrameMS, gamepadLease.Milliseconds())
		}
	}
	require.Equal(t, 1, ended) // ReadMessage's close error cannot overwrite it.
	next, _, err := websocket.DefaultDialer.Dial(gamepadURL(server), nil)
	require.NoError(t, err)
	defer next.Close()
	require.NoError(t, completeGamepadHandshake(next))
	require.Zero(t, readGamepadVelocity(t, p).Twist.Linear.X)
}

type failFirstGamepadPublish struct {
	*gamepadTestProvider
	fail atomic.Bool
}

func (p *failFirstGamepadPublish) PublishContext(ctx context.Context, topic, kind string, value interface{}) error {
	if err := p.gamepadTestProvider.PublishContext(ctx, topic, kind, value); err != nil {
		return err
	}
	if p.fail.Swap(false) {
		return errors.New("test transport write failed")
	}
	return nil
}

func TestGamepadDiagnosticPublishFailureReleasesForNextSession(t *testing.T) {
	c, base := diagnosticTestCoordinator()
	p := &failFirstGamepadPublish{gamepadTestProvider: base}
	p.fail.Store(true)
	c.provider = p
	server := httptest.NewServer(diagnosticRouter(c))
	defer server.Close()
	conn, _, err := websocket.DefaultDialer.Dial(gamepadURL(server), nil)
	require.NoError(t, err)
	defer conn.Close()
	require.NoError(t, sendGamepadClaim(conn))
	events := awaitGamepadReleased(t, c)
	found := false
	for _, event := range events {
		if event.Event == "sessionEnded" {
			found = true
			require.Equal(t, "publishFailed", event.Cause)
			require.Equal(t, "test transport write failed", event.Error)
		}
	}
	require.True(t, found)
	next, _, err := websocket.DefaultDialer.Dial(gamepadURL(server), nil)
	require.NoError(t, err)
	defer next.Close()
	require.NoError(t, completeGamepadHandshake(next))
}

func TestGamepadDiagnosticClientLabel(t *testing.T) {
	require.Equal(t, "abc-123", gamepadClientLabel("abc-123"))
	for _, value := range []string{"with space", "line\nbreak", "é", strings.Repeat("a", 81)} {
		require.Empty(t, gamepadClientLabel(value))
	}
}

func TestGamepadBlockedDiagnosticLoggerDoesNotDelayCleanup(t *testing.T) {
	blocked := make(chan struct{})
	started := make(chan struct{})
	var writes atomic.Int64
	logger := newGamepadDiagnosticLogger(func(gamepadServerEvent) {
		if writes.Add(1) == 1 {
			close(started)
		}
		<-blocked
	})
	// Release the sink even if an assertion fails; the queue remains open just
	// like the process-lifetime production logger, including for late cleanup.
	defer close(blocked)
	c, p := diagnosticTestCoordinator()
	p.state = 4
	c.diagnostics.logger = logger
	server := httptest.NewServer(diagnosticRouter(c))
	defer server.Close()
	conn, _, err := websocket.DefaultDialer.Dial(gamepadURL(server), nil)
	require.NoError(t, err)
	defer conn.Close()
	require.NoError(t, completeGamepadHandshake(conn))
	_ = readGamepadVelocity(t, p)
	select {
	case <-started:
	case <-time.After(time.Second):
		t.Fatal("logger did not start")
	}
	filled := make(chan struct{})
	go func() {
		for range 128 {
			c.diagnostics.record(gamepadServerEvent{Event: "testFill"})
		}
		close(filled)
	}()
	select {
	case <-filled:
	case <-time.After(time.Second):
		t.Fatal("full log queue blocked history recording")
	}
	require.Len(t, logger.queue, 64)
	require.Len(t, c.diagnostics.snapshot(true).Server, 64)
	require.NoError(t, conn.Close())
	awaitGamepadStop(t, p)
	require.Zero(t, readGamepadVelocity(t, p).Twist.Linear.X)
	events := awaitGamepadReleased(t, c)
	require.Equal(t, "ownerReleased", events[len(events)-1].Event)
	require.EqualValues(t, 1, writes.Load(), "cleanup completed while Docker logging remained blocked")
}
