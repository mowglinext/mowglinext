package providers

import (
	"context"
	"encoding/json"
	"fmt"
	"net/http"
	"net/http/httptest"
	"strings"
	"sync"
	"testing"
	"time"

	"github.com/gorilla/websocket"
	"github.com/mowglinext/mowglinext/pkg/foxglove"
	"github.com/mowglinext/mowglinext/pkg/msgs/geometry"
	"github.com/mowglinext/mowglinext/pkg/types"
	"github.com/stretchr/testify/require"
)

var _ types.ContextRosPublisher = (*RosProvider)(nil)

// Use the production provider and both real WebSocket transports. Holding the
// relay's selection lock models a busy writer before transport selection, so
// the same input can become stale on either the relay or the fallback route.
func TestRosProviderCarriesLeaseThroughTransportSelection(t *testing.T) {
	for _, useRelay := range []bool{false, true} {
		for _, expire := range []bool{false, true} {
			t.Run(fmt.Sprintf("relay=%t/deadline=%t", useRelay, expire), func(t *testing.T) {
				relayWire := make(chan geometry.TwistStamped, 8)
				foxgloveWire := make(chan geometry.TwistStamped, 8)
				relayServer := teleopWireServer(t, false, relayWire)
				defer relayServer.Close()
				bridge := teleopWireServer(t, true, foxgloveWire)
				defer bridge.Close()
				client := foxglove.NewClient("ws" + strings.TrimPrefix(bridge.URL, "http"))
				clientCtx, closeClient := context.WithCancel(context.Background())
				defer closeClient()
				require.NoError(t, client.Connect(clientCtx))
				defer client.Close()
				relay := &cmdVelRelayClient{}
				if useRelay {
					conn, _, err := websocket.DefaultDialer.Dial("ws"+strings.TrimPrefix(relayServer.URL, "http"), nil)
					require.NoError(t, err)
					defer conn.Close()
					relay.conn = conn
				}
				provider := &RosProvider{client: client, cmdVelRelay: relay}
				ctx, cancel := context.WithCancel(context.Background())
				if expire {
					cancel()
					ctx, cancel = context.WithTimeout(context.Background(), 50*time.Millisecond)
				}
				defer cancel()
				observed := &publishObservedContext{Context: ctx, entered: make(chan struct{})}
				relay.mu.Lock()
				locked := true
				defer func() {
					if locked {
						relay.mu.Unlock()
					}
				}()
				motion := &geometry.TwistStamped{}
				motion.Twist.Linear.X = 0.2
				result := make(chan error, 1)
				go func() {
					result <- provider.PublishContext(observed, "/cmd_vel_teleop", "geometry_msgs/msg/TwistStamped", motion)
				}()
				select {
				case <-observed.entered:
				case <-time.After(time.Second):
					t.Fatal("publish did not enter provider")
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
				// Old callers, including cleanup, retain the background context
				// path. This zero must be the first motion frame on either wire.
				require.NoError(t, provider.Publish("/cmd_vel_teleop", "geometry_msgs/msg/TwistStamped", &geometry.TwistStamped{}))
				wire, unused := foxgloveWire, relayWire
				if useRelay {
					wire, unused = relayWire, foxgloveWire
				}
				select {
				case msg := <-wire:
					require.Zero(t, msg.Twist.Linear.X, "stale motion preceded cleanup zero")
				case <-time.After(time.Second):
					t.Fatal("cleanup zero missing")
				}
				healthy, stop := context.WithTimeout(context.Background(), time.Second)
				defer stop()
				require.NoError(t, provider.PublishContext(healthy, "/cmd_vel_teleop", "geometry_msgs/msg/TwistStamped", motion))
				select {
				case msg := <-wire:
					require.Equal(t, 0.2, msg.Twist.Linear.X)
				case <-time.After(time.Second):
					t.Fatal("healthy leased motion missing")
				}
				select {
				case msg := <-unused:
					t.Fatalf("motion also went through unused transport: %+v", msg)
				default:
				}
			})
		}
	}
}

// Snapshot the entry Err result before signaling so cancellation happens after
// the provider's initial check and must still survive transport selection.
type publishObservedContext struct {
	context.Context
	entered chan struct{}
	once    sync.Once
}

func (c *publishObservedContext) Err() error {
	err := c.Context.Err()
	c.once.Do(func() { close(c.entered) })
	return err
}

func teleopWireServer(t *testing.T, foxgloveProtocol bool, wire chan<- geometry.TwistStamped) *httptest.Server {
	t.Helper()
	return httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		upgrader := websocket.Upgrader{Subprotocols: []string{"foxglove.sdk.v1"}}
		conn, err := upgrader.Upgrade(w, r, nil)
		if err != nil {
			t.Error(err)
			return
		}
		defer conn.Close()
		for {
			kind, raw, err := conn.ReadMessage()
			if err != nil {
				return
			}
			if foxgloveProtocol {
				if kind != websocket.BinaryMessage || len(raw) < 5 || raw[0] != 1 {
					continue
				}
				raw = raw[5:]
			}
			var msg geometry.TwistStamped
			if err := json.Unmarshal(raw, &msg); err != nil {
				t.Error(err)
				return
			}
			wire <- msg
		}
	}))
}
