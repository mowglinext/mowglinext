package api

import (
	"encoding/json"
	"math"
	"net/http"
	"net/http/httptest"
	"sync"
	"testing"
	"time"

	"github.com/gin-gonic/gin"
	"github.com/gorilla/websocket"
	"github.com/mowglinext/mowglinext/pkg/msgs/geometry"
	"github.com/stretchr/testify/require"
)

func TestGamepadValidatedFramesRenewLeaseThenSilenceExpires(t *testing.T) {
	p, _, conn := gamepadHarness(t, 3)
	ticker := time.NewTicker(gamepadLease / 4)
	defer ticker.Stop()
	// Stay connected for two original lease windows. Each publish proves the
	// server processed a validated frame and renewed the lease before silence.
	for range 8 {
		<-ticker.C
		writeGamepad(t, conn, 0.1, true)
		require.Equal(t, 0.1, readGamepadVelocity(t, p).Twist.Linear.X)
	}
	require.Zero(t, readGamepadVelocity(t, p).Twist.Linear.X)
	_ = conn.SetReadDeadline(time.Now().Add(time.Second))
	_, _, err := conn.ReadMessage()
	require.Error(t, err)
	select {
	case <-p.stopped:
		t.Fatal("silent recording session was discarded")
	default:
	}
}

func TestGamepadWebSocketPingsDoNotRenewCommandLease(t *testing.T) {
	p, _, conn := gamepadHarness(t, 4)
	// Control frames establish a live transport, but carry no operator input.
	// Keep receiving pongs while the command lease independently expires.
	readDone := make(chan struct{})
	go func() {
		defer close(readDone)
		_, _, _ = conn.ReadMessage()
	}()
	ticker := time.NewTicker(gamepadLease / 6)
	defer ticker.Stop()
	deadline := time.NewTimer(3 * gamepadLease)
	defer deadline.Stop()
	for {
		select {
		case <-readDone:
			require.Zero(t, readGamepadVelocity(t, p).Twist.Linear.X)
			awaitGamepadStop(t, p)
			return
		case <-ticker.C:
			_ = conn.WriteControl(websocket.PingMessage, []byte("alive"), time.Now().Add(time.Second))
		case <-deadline.C:
			t.Fatal("transport pings extended the command lease")
		}
	}
}

func TestGamepadConcurrentClaimsHaveOneOwner(t *testing.T) {
	p := &gamepadTestProvider{state: 3, published: make(chan geometry.TwistStamped, 32), stopped: make(chan struct{}, 8)}
	coordinator := &gamepadCoordinator{provider: p}
	router := gin.New()
	registerPublisherRoutes(router.Group("/api/mowglinext"), coordinator)
	server := httptest.NewServer(router)
	defer server.Close()
	const contenders = 8
	type result struct {
		conn   *websocket.Conn
		status int
		err    error
	}
	results := make(chan result, contenders)
	start := make(chan struct{})
	keepaliveDone := make(chan struct{})
	var keepalive sync.WaitGroup
	defer func() { close(keepaliveDone); keepalive.Wait() }()
	for range contenders {
		go func() {
			<-start
			conn, response, err := websocket.DefaultDialer.Dial(gamepadURL(server), nil)
			status := 0
			if response != nil {
				status = response.StatusCode
			}
			if conn != nil {
				if claimErr := completeGamepadHandshake(conn); claimErr != nil {
					_ = conn.Close()
					conn = nil
					err = claimErr
				}
			}
			if conn != nil {
				keepalive.Add(1)
				go func() {
					defer keepalive.Done()
					ticker := time.NewTicker(gamepadLease / 4)
					defer ticker.Stop()
					for {
						select {
						case <-keepaliveDone:
							return
						case <-ticker.C:
							if conn.WriteJSON(gamepadWire(gamepadVelocity(0), false)) != nil {
								return
							}
						}
					}
				}()
			}
			results <- result{conn: conn, status: status, err: err}
		}()
	}
	close(start)
	owners := 0
	for range contenders {
		select {
		case result := <-results:
			if result.conn != nil {
				owners++
				defer result.conn.Close()
				require.NoError(t, result.err)
			} else {
				require.Error(t, result.err)
				require.Contains(t, []int{http.StatusConflict, http.StatusSwitchingProtocols}, result.status)
			}
		case <-time.After(2 * time.Second):
			t.Fatal("concurrent ownership claim hung")
		}
	}
	require.Equal(t, 1, owners)
	coordinator.mu.Lock()
	owner := coordinator.owner
	coordinator.mu.Unlock()
	require.NotNil(t, owner)
}

func TestGamepadBinaryVelocityFrameIsTerminal(t *testing.T) {
	p, _, conn := gamepadHarness(t, 4)
	raw, err := json.Marshal(gamepadWire(gamepadVelocity(0.1), true))
	require.NoError(t, err)
	require.NoError(t, conn.WriteMessage(websocket.BinaryMessage, raw))
	require.Zero(t, readGamepadVelocity(t, p).Twist.Linear.X)
	awaitGamepadStop(t, p)
}

func FuzzDecodeGamepadFrame(f *testing.F) {
	for _, x := range []float64{-0.25, -0.0, 0, 0.25, 0.250001} {
		raw, _ := json.Marshal(gamepadWire(gamepadVelocity(x), true))
		f.Add(raw)
	}
	zero, _ := json.Marshal(gamepadWire(gamepadVelocity(0), false))
	f.Add(zero)
	for _, raw := range []string{"{}", "null", "[]", "{", `{"twist":{},"deadman":true}`, `{"twist":null,"deadman":false}`, `{"twist":{"twist":{"linear":{"x":1e309,"y":0,"z":0},"angular":{"x":0,"y":0,"z":0}}},"deadman":true}`} {
		f.Add([]byte(raw))
	}
	f.Fuzz(func(t *testing.T, raw []byte) {
		// The WebSocket route bounds frames before calling this decoder.
		if len(raw) > 2048 {
			t.Skip()
		}
		msg, deadman, err := decodeGamepadFrame(raw)
		if err != nil {
			return
		}
		if !json.Valid(raw) {
			t.Fatal("accepted malformed JSON")
		}
		twist := msg.Twist
		for _, value := range []float64{twist.Linear.X, twist.Linear.Y, twist.Linear.Z, twist.Angular.X, twist.Angular.Y, twist.Angular.Z} {
			if math.IsNaN(value) || math.IsInf(value, 0) {
				t.Fatal("accepted non-finite velocity")
			}
		}
		if math.Abs(twist.Linear.X) > 0.25 || math.Abs(twist.Angular.Z) > 0.6 || twist.Linear.Y != 0 || twist.Linear.Z != 0 || twist.Angular.X != 0 || twist.Angular.Y != 0 {
			t.Fatal("accepted velocity outside the gamepad envelope")
		}
		if !deadman && (twist.Linear.X != 0 || twist.Angular.Z != 0) {
			t.Fatal("accepted motion without deadman")
		}
		canonical, err := json.Marshal(gamepadWire(msg, deadman))
		if err != nil {
			t.Fatal(err)
		}
		reparsed, reparsedDeadman, err := decodeGamepadFrame(canonical)
		if err != nil || reparsedDeadman != deadman || reparsed != msg {
			t.Fatal("accepted frame changed under canonical round trip")
		}
	})
}
