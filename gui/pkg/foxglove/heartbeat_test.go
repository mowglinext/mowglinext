package foxglove

import (
	"context"
	"encoding/binary"
	"encoding/json"
	"net"
	"net/http"
	"net/http/httptest"
	"strings"
	"sync"
	"sync/atomic"
	"testing"
	"time"

	"github.com/gorilla/websocket"
	"github.com/stretchr/testify/require"
)

// Pause user argument serialization after CallService has captured discovery.
type heldServiceArgs struct {
	started chan struct{}
	release chan struct{}
}

func (a heldServiceArgs) MarshalJSON() ([]byte, error) {
	close(a.started)
	<-a.release
	return []byte(`{}`), nil
}

func fastHeartbeatClient(server *httptest.Server) *Client {
	c := NewClient("ws" + strings.TrimPrefix(server.URL, "http"))
	c.pingInterval = 10 * time.Millisecond
	c.pongTimeout = 80 * time.Millisecond
	c.reconnectDelay = 20 * time.Millisecond
	c.maxReconnect = 80 * time.Millisecond
	return c
}

func TestSilentFoxgloveSocketReconnects(t *testing.T) {
	var accepted atomic.Int32
	stop := make(chan struct{})
	server := httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		conn, err := (&websocket.Upgrader{Subprotocols: []string{"foxglove.sdk.v1"}}).Upgrade(w, r, nil)
		if err != nil {
			return
		}
		defer conn.Close()
		accepted.Add(1)
		<-stop
	}))
	defer server.Close()
	defer close(stop)
	client := fastHeartbeatClient(server)
	ctx, cancel := context.WithCancel(context.Background())
	defer cancel()
	require.NoError(t, client.Connect(ctx))
	defer client.Close()
	require.Eventually(t, func() bool { return accepted.Load() >= 2 }, time.Second, time.Millisecond,
		"silent established connection must be retired and reconnected")
}

func TestHealthyIdleFoxgloveStaysConnected(t *testing.T) {
	var accepted atomic.Int32
	server := httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		conn, err := (&websocket.Upgrader{Subprotocols: []string{"foxglove.sdk.v1"}}).Upgrade(w, r, nil)
		if err != nil {
			return
		}
		accepted.Add(1)
		defer conn.Close()
		// ReadMessage processes ping/pong even with no topic traffic at all.
		_, _, _ = conn.ReadMessage()
	}))
	defer server.Close()
	client := fastHeartbeatClient(server)
	ctx, cancel := context.WithCancel(context.Background())
	defer cancel()
	require.NoError(t, client.Connect(ctx))
	defer client.Close()
	require.Eventually(t, func() bool { return client.Health().HeartbeatOK }, time.Second, time.Millisecond)
	require.Never(t, func() bool { return !client.Connected() || !client.Health().HeartbeatOK || accepted.Load() != 1 },
		4*client.pongTimeout, time.Millisecond)
	require.NoError(t, client.Close())
	select {
	case <-client.stopped:
	case <-time.After(time.Second):
		t.Fatal("connection workers did not exit")
	}
	require.False(t, client.Health().Connected)
	require.False(t, client.Health().HeartbeatOK)
}

func TestTopicTrafficCannotReplaceHeartbeatEvidence(t *testing.T) {
	var accepted atomic.Int32
	server := httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		conn, err := (&websocket.Upgrader{Subprotocols: []string{"foxglove.sdk.v1"}}).Upgrade(w, r, nil)
		if err != nil {
			return
		}
		accepted.Add(1)
		defer conn.Close()
		// Deliberately write topic-like frames without reading/responding to pings.
		ticker := time.NewTicker(5 * time.Millisecond)
		defer ticker.Stop()
		for range ticker.C {
			if err := conn.WriteMessage(websocket.BinaryMessage, []byte{serverBinTime}); err != nil {
				return
			}
		}
	}))
	defer server.Close()
	client := fastHeartbeatClient(server)
	ctx, cancel := context.WithCancel(context.Background())
	defer cancel()
	require.NoError(t, client.Connect(ctx))
	defer client.Close()
	require.Eventually(t, func() bool { return accepted.Load() >= 2 }, time.Second, time.Millisecond)
	require.False(t, client.Health().HeartbeatOK)
}

func TestRepeatedSilentConnectionsBackOffAndStop(t *testing.T) {
	attempts := make(chan time.Time, 10)
	server := httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		conn, err := (&websocket.Upgrader{Subprotocols: []string{"foxglove.sdk.v1"}}).Upgrade(w, r, nil)
		if err != nil {
			return
		}
		defer conn.Close()
		attempts <- time.Now()
		// Read so closure is observed, but suppress automatic pong responses.
		conn.SetPingHandler(func(string) error { return nil })
		_, _, _ = conn.ReadMessage()
	}))
	defer server.Close()
	client := fastHeartbeatClient(server)
	client.reconnectDelay = 40 * time.Millisecond
	client.maxReconnect = 160 * time.Millisecond
	ctx, cancel := context.WithCancel(context.Background())
	defer cancel()
	require.NoError(t, client.Connect(ctx))
	defer client.Close()
	var previous time.Time
	for i := 0; i < 5; i++ {
		select {
		case at := <-attempts:
			if i > 0 {
				gap := at.Sub(previous)
				expected := client.reconnectDelay * time.Duration(1<<uint(i-1))
				if expected > client.maxReconnect {
					expected = client.maxReconnect
				}
				require.GreaterOrEqual(t, gap, client.pongTimeout+expected-5*time.Millisecond)
				require.Less(t, gap, client.pongTimeout+expected+500*time.Millisecond)
			}
			previous = at
		case <-time.After(2 * time.Second):
			t.Fatal("reconnect did not occur within capped backoff")
		}
	}
	cancel()
	select {
	case <-client.stopped:
	case <-time.After(time.Second):
		t.Fatal("cancellation leaked a connection/probe")
	}
	require.False(t, client.Connected())
	require.False(t, client.Health().HeartbeatOK)
}

func TestSilentReconnectRestoresTopicsAndServicesAndRetiresOldCalls(t *testing.T) {
	var accepted atomic.Int32
	subscribed := make(chan [2]uint32, 8)
	oldRequest := make(chan struct{})
	oldClosed := make(chan struct{})
	responseCDR := mustSerialize(t, map[string]any{"success": true}, mustParseSchema(t, "bool success"))
	topicCDR := mustSerialize(t, map[string]any{"data": true}, mustParseSchema(t, "bool data"))
	server := httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		conn, err := (&websocket.Upgrader{Subprotocols: []string{"foxglove.sdk.v1"}}).Upgrade(w, r, nil)
		if err != nil {
			return
		}
		defer conn.Close()
		generation := uint32(accepted.Add(1))
		if generation == 1 {
			defer close(oldClosed)
			conn.SetPingHandler(func(string) error { return nil })
		}
		base := generation * 10
		channels := []channelDef{
			{ID: base + 1, Topic: "/one", Encoding: "cdr", Schema: "bool data"},
			{ID: base + 2, Topic: "/two", Encoding: "cdr", Schema: "bool data"},
		}
		if generation == 1 {
			channels = append(channels, channelDef{ID: 99, Topic: "/old-only", Schema: "bool data"})
		}
		if err := conn.WriteJSON(serverAdvertise{Op: "advertise", Channels: channels}); err != nil {
			return
		}
		services := []serviceDef{{ID: base + 3, Name: "/check", Request: serviceEndpointDef{Schema: ""}, Response: serviceEndpointDef{Schema: "bool success"}}}
		if generation == 1 {
			services = append(services, serviceDef{ID: 99, Name: "/old-service"})
		}
		if err := conn.WriteJSON(serverAdvertiseServices{Op: "advertiseServices", Services: services}); err != nil {
			return
		}
		for {
			kind, data, err := conn.ReadMessage()
			if err != nil {
				return
			}
			if kind == websocket.TextMessage {
				var op clientSubscribe
				if json.Unmarshal(data, &op) != nil || op.Op != "subscribe" {
					continue
				}
				for _, sub := range op.Subscriptions {
					subscribed <- [2]uint32{generation, sub.ChannelID}
					if generation > 1 {
						frame := make([]byte, 13)
						frame[0] = serverBinMessageData
						binary.LittleEndian.PutUint32(frame[1:5], sub.ID)
						frame = append(frame, topicCDR...)
						if conn.WriteMessage(websocket.BinaryMessage, frame) != nil {
							return
						}
					}
				}
			} else if kind == websocket.BinaryMessage && len(data) >= 16 && data[0] == clientBinServiceCallRequest {
				if generation == 1 {
					close(oldRequest) // Request accepted, but its result never arrives.
					continue
				}
				if got := binary.LittleEndian.Uint32(data[1:5]); got != base+3 {
					t.Errorf("service ID = %d, want replacement ID %d", got, base+3)
					return
				}
				response := append([]byte{}, data[:16]...)
				response[0] = serverBinServiceCallResponse
				response = append(response, responseCDR...)
				if conn.WriteMessage(websocket.BinaryMessage, response) != nil {
					return
				}
			}
		}
	}))
	defer server.Close()
	client := fastHeartbeatClient(server)
	// Plenty of time to register the in-flight call even under the race detector.
	client.pongTimeout = 300 * time.Millisecond
	callbacks := make(chan string, 8)
	require.NoError(t, client.Subscribe("/one", "", "one", func(json.RawMessage) { callbacks <- "/one" }))
	require.NoError(t, client.Subscribe("/two", "", "two", func(json.RawMessage) { callbacks <- "/two" }))
	ctx, cancel := context.WithTimeout(context.Background(), 5*time.Second)
	defer cancel()
	require.NoError(t, client.Connect(ctx))
	defer client.Close()
	require.Eventually(t, func() bool {
		client.svcDefMu.RLock()
		defer client.svcDefMu.RUnlock()
		return client.services["/check"] != nil
	}, time.Second, time.Millisecond)
	client.connMu.Lock()
	oldConn := client.conn
	client.connMu.Unlock()
	oldHeartbeat := client.heartbeat.Load()
	args := heldServiceArgs{started: make(chan struct{}), release: make(chan struct{})}
	var releaseOnce sync.Once
	defer releaseOnce.Do(func() { close(args.release) })
	preparedResult := make(chan error, 1)
	go func() { _, err := client.CallService(ctx, "/check", args); preparedResult <- err }()
	select {
	case <-args.started:
	case <-ctx.Done():
		t.Fatal("old request serialization not reached")
	}
	oldResult := make(chan error, 1)
	go func() { _, err := client.CallService(ctx, "/check", struct{}{}); oldResult <- err }()
	select {
	case <-oldRequest:
	case <-ctx.Done():
		t.Fatal("old request not sent")
	}
	select {
	case err := <-oldResult:
		require.ErrorContains(t, err, "connection to foxglove_bridge lost")
	case <-ctx.Done():
		t.Fatal("old call did not fail on heartbeat expiry")
	}
	seen := map[[2]uint32]bool{}
	for len(seen) < 4 {
		select {
		case pair := <-subscribed:
			seen[pair] = true
		case <-ctx.Done():
			t.Fatalf("subscriptions not replayed: %v", seen)
		}
	}
	require.True(t, seen[[2]uint32{1, 11}])
	require.True(t, seen[[2]uint32{1, 12}])
	require.True(t, seen[[2]uint32{2, 21}])
	require.True(t, seen[[2]uint32{2, 22}])
	select {
	case <-oldClosed:
	case <-ctx.Done():
		t.Fatal("old socket not closed")
	}
	require.False(t, oldHeartbeat.connected.Load())
	require.NotSame(t, oldHeartbeat, client.heartbeat.Load())
	require.Eventually(t, func() bool { return client.Health().HeartbeatOK }, time.Second, time.Millisecond)
	client.chanMu.RLock()
	_, oldChannelStillPresent := client.channels["/old-only"]
	client.chanMu.RUnlock()
	require.False(t, oldChannelStillPresent)
	client.svcDefMu.RLock()
	_, oldServiceStillPresent := client.services["/old-service"]
	client.svcDefMu.RUnlock()
	require.False(t, oldServiceStillPresent)
	for i := 0; i < 2; i++ {
		select {
		case <-callbacks:
		case <-ctx.Done():
			t.Fatal("restored topic did not dispatch")
		}
	}
	releaseOnce.Do(func() { close(args.release) })
	select {
	case err := <-preparedResult:
		require.ErrorContains(t, err, "connection unavailable or replaced")
	case <-ctx.Done():
		t.Fatal("old prepared call remained pending")
	}
	result, err := client.CallService(ctx, "/check", struct{}{})
	require.NoError(t, err)
	require.JSONEq(t, `{"success":true}`, string(result))
	require.Error(t, client.writeMessageOn(oldConn, websocket.TextMessage, []byte(`{"op":"old"}`)), "old generation cannot write on replacement")
	client.svcMu.Lock()
	require.Empty(t, client.pendingSvc)
	client.svcMu.Unlock()
	require.NoError(t, client.Close())
	select {
	case <-client.stopped:
	case <-ctx.Done():
		t.Fatal("workers not stopped")
	}
}

func TestCloseCancelsHandshakeAndCannotInstallLateConnection(t *testing.T) {
	client := NewClient("ws://unused")
	dialStarted := make(chan struct{})
	client.dialer.NetDialContext = func(ctx context.Context, _, _ string) (net.Conn, error) {
		close(dialStarted)
		<-ctx.Done()
		return nil, ctx.Err()
	}
	connected := make(chan struct{})
	go func() { defer close(connected); _ = client.Connect(context.Background()) }()
	select {
	case <-dialStarted:
	case <-time.After(time.Second):
		t.Fatal("dial not started")
	}
	require.NoError(t, client.Close())
	require.NoError(t, client.Close())
	select {
	case <-connected:
	case <-time.After(time.Second):
		t.Fatal("Close did not cancel handshake")
	}
	select {
	case <-client.stopped:
	case <-time.After(time.Second):
		t.Fatal("reconnect worker not stopped")
	}
	require.False(t, client.Connected())
	require.False(t, client.Health().Connected)
}

func TestHeartbeatCoexistsWithConcurrentDataWriters(t *testing.T) {
	var accepted atomic.Int32
	server := httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		conn, err := (&websocket.Upgrader{Subprotocols: []string{"foxglove.sdk.v1"}}).Upgrade(w, r, nil)
		if err != nil {
			return
		}
		accepted.Add(1)
		defer conn.Close()
		for {
			if _, _, err := conn.ReadMessage(); err != nil {
				return
			}
		}
	}))
	defer server.Close()
	client := fastHeartbeatClient(server)
	client.pingInterval = time.Millisecond
	client.pongTimeout = time.Second
	ctx, cancel := context.WithCancel(context.Background())
	defer cancel()
	require.NoError(t, client.Connect(ctx))
	defer client.Close()
	// Starting again must not create a second reader or reconnect owner.
	require.NoError(t, client.Connect(ctx))
	require.Eventually(t, func() bool { return client.Health().HeartbeatOK }, time.Second, time.Millisecond)
	var writers sync.WaitGroup
	for i := 0; i < 6; i++ {
		writers.Add(1)
		go func() {
			defer writers.Done()
			for j := 0; j < 50; j++ {
				if err := client.Publish("/concurrent", map[string]bool{"data": true}, "std_msgs/msg/Bool"); err != nil {
					t.Error(err)
					return
				}
			}
		}()
	}
	writers.Wait()
	require.True(t, client.Connected())
	require.True(t, client.Health().HeartbeatOK)
	require.Equal(t, int32(1), accepted.Load())
}
