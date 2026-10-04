package foxglove

import (
	"context"
	"encoding/json"
	"net"
	"net/http"
	"net/http/httptest"
	"os"
	"strings"
	"sync"
	"sync/atomic"
	"testing"
	"time"

	"github.com/gorilla/websocket"
	"github.com/stretchr/testify/require"
)

// Simulate a full socket send buffer without relying on OS buffer sizes. The
// real WebSocket handshake/read pump still run over TCP, and a failed write
// behaves like net.Conn's deadline expiry.
type stalledWriteConn struct {
	net.Conn
	stall    atomic.Bool
	mu       sync.Mutex
	deadline time.Time
	closed   chan struct{}
	once     sync.Once
}

func (c *stalledWriteConn) SetWriteDeadline(deadline time.Time) error {
	c.mu.Lock()
	c.deadline = deadline
	c.mu.Unlock()
	return c.Conn.SetWriteDeadline(deadline)
}

func (c *stalledWriteConn) Write(data []byte) (int, error) {
	if !c.stall.Load() {
		return c.Conn.Write(data)
	}
	c.mu.Lock()
	deadline := c.deadline
	c.mu.Unlock()
	var expired <-chan time.Time
	var timer *time.Timer
	if !deadline.IsZero() {
		timer = time.NewTimer(time.Until(deadline))
		defer timer.Stop()
		expired = timer.C
	}
	select {
	case <-expired:
		return 0, os.ErrDeadlineExceeded
	case <-c.closed:
		return 0, net.ErrClosed
	}
}

func (c *stalledWriteConn) Close() error {
	c.once.Do(func() { close(c.closed) })
	return c.Conn.Close()
}

func TestSubscriptionWriteTimeoutReconnectsAndRestoresDesiredTopics(t *testing.T) {
	serverSubscribed := make(chan uint32, 4)
	server := httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		conn, err := (&websocket.Upgrader{Subprotocols: []string{"foxglove.sdk.v1"}}).Upgrade(w, r, nil)
		if err != nil {
			return
		}
		defer conn.Close()
		if err := conn.WriteJSON(map[string]any{"op": "advertise", "channels": []map[string]any{
			{"id": 1, "topic": "/status", "encoding": "cdr", "schemaName": "std_msgs/msg/Bool", "schema": "bool data"},
		}}); err != nil {
			return
		}
		for {
			var op clientSubscribe
			if err := conn.ReadJSON(&op); err != nil {
				return
			}
			if op.Op == "subscribe" {
				for _, sub := range op.Subscriptions {
					serverSubscribed <- sub.ChannelID
				}
			}
		}
	}))
	defer server.Close()
	client := NewClient("ws" + strings.TrimPrefix(server.URL, "http"))
	client.writeTimeout = 30 * time.Millisecond
	var first *stalledWriteConn
	var dials atomic.Int32
	client.dialer.NetDialContext = func(ctx context.Context, network, address string) (net.Conn, error) {
		conn, err := (&net.Dialer{}).DialContext(ctx, network, address)
		if err != nil {
			return nil, err
		}
		wrapped := &stalledWriteConn{Conn: conn, closed: make(chan struct{})}
		if dials.Add(1) == 1 {
			first = wrapped
		}
		return wrapped, nil
	}
	ctx, cancel := context.WithCancel(context.Background())
	defer cancel()
	require.NoError(t, client.Connect(ctx))
	defer client.Close()
	require.Eventually(t, func() bool {
		client.chanMu.RLock()
		defer client.chanMu.RUnlock()
		return client.channels["/status"] != nil
	}, time.Second, time.Millisecond)
	first.stall.Store(true)
	done := make(chan struct{})
	go func() { defer close(done); _ = client.Subscribe("/status", "", "active", func(json.RawMessage) {}) }()
	select {
	case <-done:
	case <-time.After(time.Second):
		t.Fatal("subscription held the socket lock past the write deadline")
	}
	select {
	case channel := <-serverSubscribed:
		require.Equal(t, uint32(1), channel)
	case <-time.After(3 * time.Second):
		t.Fatal("desired subscription was not restored after the failed write")
	}
	require.GreaterOrEqual(t, dials.Load(), int32(2))
	client.Unsubscribe("/status", "active")
	client.subMu.RLock()
	require.Empty(t, client.subscribers)
	client.subMu.RUnlock()
}

func TestUnsubscribeBeforeAdvertiseDoesNotRestoreRemovedListener(t *testing.T) {
	client := NewClient("unused")
	require.NoError(t, client.Subscribe("/status", "", "gone", func(json.RawMessage) {}))
	client.Unsubscribe("/status", "gone")
	require.Empty(t, client.pendingTopics)
	client.handleAdvertise(serverAdvertise{Channels: []channelDef{{ID: 1, Topic: "/status", Schema: "bool data"}}})
	require.Zero(t, client.channels["/status"].subscriptionID)
	require.Empty(t, client.subscribers)
}

func TestDisconnectCompletesOldCallsBeforeAllowingReconnect(t *testing.T) {
	drop := make(chan struct{})
	var accepted atomic.Int32
	server := httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		conn, err := (&websocket.Upgrader{Subprotocols: []string{"foxglove.sdk.v1"}}).Upgrade(w, r, nil)
		if err != nil {
			return
		}
		defer conn.Close()
		if accepted.Add(1) == 1 {
			<-drop
			return
		}
		_, _, _ = conn.ReadMessage()
	}))
	defer server.Close()
	client := NewClient("ws" + strings.TrimPrefix(server.URL, "http"))
	ctx, cancel := context.WithCancel(context.Background())
	defer cancel()
	require.NoError(t, client.Connect(ctx))
	defer client.Close()

	// Hold completion of the old service call. A new connection must not be
	// published until that generation's pending calls have been failed.
	response := make(chan serviceCallResult, 1)
	client.svcMu.Lock()
	client.pendingSvc[1] = &pendingServiceCall{ch: response}
	var released sync.Once
	defer released.Do(func() { client.svcMu.Unlock() })
	close(drop)
	require.Eventually(t, func() bool {
		if !client.Connected() {
			return true // The old ordering published disconnection too early.
		}
		if !client.connMu.TryLock() {
			return true // The read pump is waiting to complete the old call.
		}
		client.connMu.Unlock()
		return false
	}, time.Second, time.Millisecond)
	require.True(t, client.Connected(), "reconnect became visible before retiring old service calls")
	require.Equal(t, int32(1), accepted.Load())
	released.Do(func() { client.svcMu.Unlock() })
	select {
	case result := <-response:
		require.False(t, result.success)
	case <-time.After(time.Second):
		t.Fatal("old call was not failed on disconnect")
	}
	require.Eventually(t, func() bool { return accepted.Load() == 2 && client.Connected() }, time.Second, time.Millisecond)
}
