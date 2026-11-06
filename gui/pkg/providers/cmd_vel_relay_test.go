package providers

import (
	"bufio"
	"context"
	"crypto/sha1"
	"encoding/base64"
	"fmt"
	"net"
	"net/http"
	"net/http/httptest"
	"net/url"
	"strings"
	"testing"
	"time"

	"github.com/gorilla/websocket"
	"github.com/stretchr/testify/require"
)

func TestCmdVelRelaySendBoundsAnUnresponsiveWriter(t *testing.T) {
	client, peer := net.Pipe()
	defer peer.Close()
	// Complete only the WebSocket handshake, then deliberately never read
	// command bytes. net.Pipe has no buffering, so the write must hit its
	// deadline regardless of OS socket-buffer size or host scheduling.
	go func() {
		request, err := http.ReadRequest(bufio.NewReader(peer))
		if err != nil {
			return
		}
		digest := sha1.Sum([]byte(request.Header.Get("Sec-WebSocket-Key") + "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"))
		_, _ = fmt.Fprintf(peer, "HTTP/1.1 101 Switching Protocols\r\nUpgrade: websocket\r\nConnection: Upgrade\r\nSec-WebSocket-Accept: %s\r\n\r\n", base64.StdEncoding.EncodeToString(digest[:]))
	}()
	conn, _, err := websocket.NewClient(client, &url.URL{Scheme: "ws", Host: "relay.test"}, nil, 1024, 1024)
	require.NoError(t, err)
	defer conn.Close()
	relay := &cmdVelRelayClient{conn: conn}
	started := time.Now()
	err = relay.Send(map[string]int{"command": 1})
	require.Error(t, err)
	require.Less(t, time.Since(started), time.Second)
	// A timed-out Gorilla writer is terminal. Closing it allows the relay's
	// existing read loop to observe failure and reconnect.
	_, err = peer.Read(make([]byte, 1))
	require.Error(t, err)
}

func TestCmdVelRelayDiscardsQueuedMotionAndStillSendsZero(t *testing.T) {
	for _, expire := range []bool{false, true} {
		t.Run(fmt.Sprintf("deadline=%t", expire), func(t *testing.T) {
			frames := make(chan []byte, 8)
			server := httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
				conn, err := (&websocket.Upgrader{}).Upgrade(w, r, nil)
				if err != nil {
					t.Error(err)
					return
				}
				defer conn.Close()
				for {
					_, data, err := conn.ReadMessage()
					if err != nil {
						return
					}
					frames <- data
				}
			}))
			defer server.Close()
			conn, _, err := websocket.DefaultDialer.Dial("ws"+strings.TrimPrefix(server.URL, "http"), nil)
			require.NoError(t, err)
			defer conn.Close()
			relay := &cmdVelRelayClient{conn: conn}
			relay.mu.Lock()
			locked := true
			defer func() {
				if locked {
					relay.mu.Unlock()
				}
			}()
			ctx, cancel := context.WithCancel(context.Background())
			if expire {
				cancel()
				ctx, cancel = context.WithTimeout(context.Background(), 50*time.Millisecond)
			}
			defer cancel()
			entered := make(chan struct{})
			result := make(chan error, 1)
			go func() { result <- relay.SendContext(ctx, relayTestMessage{entered}) }()
			select {
			case <-entered:
			case <-time.After(time.Second):
				t.Fatal("motion never reached relay serialization")
			}
			if expire {
				<-ctx.Done()
			} else {
				cancel()
			}
			relay.mu.Unlock()
			locked = false
			select {
			case err := <-result:
				require.ErrorIs(t, err, ctx.Err())
			case <-time.After(time.Second):
				t.Fatal("stale publish did not finish after writer unlock")
			}
			require.NoError(t, relay.Send(map[string]float64{"velocity": 0}))
			select {
			case raw := <-frames:
				require.JSONEq(t, `{"velocity":0}`, string(raw))
			case <-time.After(time.Second):
				t.Fatal("cleanup zero missing")
			}
			// Discarding stale input must not close the healthy relay.
			require.True(t, relay.Connected())
			healthy, stop := context.WithTimeout(context.Background(), time.Second)
			defer stop()
			require.NoError(t, relay.SendContext(healthy, map[string]float64{"velocity": 0.1}))
			select {
			case raw := <-frames:
				require.JSONEq(t, `{"velocity":0.1}`, string(raw))
			case <-time.After(time.Second):
				t.Fatal("healthy command missing")
			}
		})
	}
}

type relayTestMessage struct{ entered chan struct{} }

func (m relayTestMessage) MarshalJSON() ([]byte, error) {
	close(m.entered)
	return []byte(`{"velocity":0.2}`), nil
}
