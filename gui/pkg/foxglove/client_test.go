package foxglove

import (
	"context"
	"encoding/binary"
	"encoding/json"
	"errors"
	"fmt"
	"math"
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

func TestCallServicePreservesSnapshotWithUnknownMeasurement(t *testing.T) {
	// Exercise the actual binary service-response path, not only the sanitizer.
	schema := "float64 accuracy\nbool success\nuint64 observations"
	cdr := mustSerialize(t, map[string]any{"accuracy": 0.0, "success": true, "observations": 42}, mustParseSchema(t, schema))
	binary.LittleEndian.PutUint64(cdr[4:12], math.Float64bits(math.NaN()))
	server := httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		upgrader := websocket.Upgrader{Subprotocols: []string{"foxglove.sdk.v1"}}
		conn, err := upgrader.Upgrade(w, r, nil)
		if err != nil {
			t.Error(err)
			return
		}
		defer conn.Close()
		_, request, err := conn.ReadMessage()
		if err != nil {
			t.Error(err)
			return
		}
		if len(request) < 13 || request[0] != clientBinServiceCallRequest {
			t.Error("invalid service request")
			return
		}
		response := append([]byte{}, request[:16]...)
		response[0] = serverBinServiceCallResponse
		response = append(response, cdr...)
		if err = conn.WriteMessage(websocket.BinaryMessage, response); err != nil {
			t.Error(err)
		}
		_, _, _ = conn.ReadMessage() // Wait for the client's clean close.
	}))
	defer server.Close()
	client := NewClient("ws" + strings.TrimPrefix(server.URL, "http"))
	var service serviceDef
	if err := json.Unmarshal([]byte(`{"id":1,"name":"/snapshot","request":{"schema":""},"response":{"schema":"float64 accuracy\nbool success\nuint64 observations"}}`), &service); err != nil {
		t.Fatal(err)
	}
	client.services["/snapshot"] = &serviceState{def: service}
	ctx, cancel := context.WithTimeout(context.Background(), 3*time.Second)
	defer cancel()
	if err := client.Connect(ctx); err != nil {
		t.Fatal(err)
	}
	defer client.Close()
	result, err := client.CallService(ctx, "/snapshot", struct{}{})
	if err != nil {
		t.Fatal(err)
	}
	var values map[string]any
	if err = json.Unmarshal(result, &values); err != nil {
		t.Fatal(err)
	}
	if values["accuracy"] != nil || values["success"] != true || values["observations"] != float64(42) {
		t.Fatalf("snapshot damaged: %s", result)
	}
}

func TestSanitizeJSONValueReplacesNonFiniteFloats(t *testing.T) {
	raw := map[string]interface{}{
		"finite": 1.5,
		"nan":    math.NaN(),
		"nested": map[string]interface{}{
			"inf": math.Inf(1),
		},
		"array": []float64{2.0, math.NaN(), 4.0},
	}

	sanitized := sanitizeJSONValue(raw)
	data, err := json.Marshal(sanitized)
	if err != nil {
		t.Fatalf("json.Marshal(sanitizeJSONValue(...)) = %v", err)
	}

	var decoded map[string]interface{}
	if err := json.Unmarshal(data, &decoded); err != nil {
		t.Fatalf("json.Unmarshal = %v", err)
	}

	if decoded["nan"] != nil {
		t.Fatalf("nan field = %#v, want nil", decoded["nan"])
	}

	nested, ok := decoded["nested"].(map[string]interface{})
	if !ok {
		t.Fatalf("nested field type = %T, want map[string]interface{}", decoded["nested"])
	}
	if nested["inf"] != nil {
		t.Fatalf("nested.inf = %#v, want nil", nested["inf"])
	}

	array, ok := decoded["array"].([]interface{})
	if !ok {
		t.Fatalf("array field type = %T, want []interface{}", decoded["array"])
	}
	if len(array) != 3 {
		t.Fatalf("array len = %d, want 3", len(array))
	}
	if array[1] != nil {
		t.Fatalf("array[1] = %#v, want nil", array[1])
	}
}

func TestCallServiceFailsFastWhenTheConnectionDropsMidCall(t *testing.T) {
	// Arrange: a bridge that takes the request and dies before answering.
	server := httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		upgrader := websocket.Upgrader{Subprotocols: []string{"foxglove.sdk.v1"}}
		conn, err := upgrader.Upgrade(w, r, nil)
		if err != nil {
			t.Error(err)
			return
		}
		_, _, _ = conn.ReadMessage()
		_ = conn.Close()
	}))
	defer server.Close()
	client := NewClient("ws" + strings.TrimPrefix(server.URL, "http"))
	var service serviceDef
	if err := json.Unmarshal([]byte(`{"id":1,"name":"/add_area","request":{"schema":""},"response":{"schema":"bool success"}}`), &service); err != nil {
		t.Fatal(err)
	}
	client.services["/add_area"] = &serviceState{def: service}
	ctx, cancel := context.WithTimeout(context.Background(), 20*time.Second)
	defer cancel()
	if err := client.Connect(ctx); err != nil {
		t.Fatal(err)
	}
	defer client.Close()

	// Act
	started := time.Now()
	_, err := client.CallService(ctx, "/add_area", struct{}{})

	// Assert: an error that names the cause, long before the caller's deadline.
	if err == nil {
		t.Fatal("a call whose connection dropped must fail")
	}
	if !strings.Contains(err.Error(), "connection to foxglove_bridge lost") {
		t.Fatalf("error does not name the cause: %v", err)
	}
	if elapsed := time.Since(started); elapsed > 5*time.Second {
		t.Fatalf("call waited %s for its deadline instead of failing on the drop", elapsed)
	}
}

func TestCallServiceDoesNotWriteAfterContextExpiresBehindWriter(t *testing.T) {
	type wireFrame struct {
		kind int
		data []byte
		err  error
	}
	frames := make(chan wireFrame, 1)
	server := httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		conn, err := (&websocket.Upgrader{}).Upgrade(w, r, nil)
		if err != nil {
			frames <- wireFrame{err: err}
			return
		}
		defer conn.Close()
		kind, data, err := conn.ReadMessage()
		frames <- wireFrame{kind: kind, data: data, err: err}
	}))
	defer server.Close()
	conn, _, err := websocket.DefaultDialer.Dial("ws"+strings.TrimPrefix(server.URL, "http"), nil)
	if err != nil {
		t.Fatal(err)
	}
	defer conn.Close()
	client := NewClient("")
	client.conn = conn
	client.connected.Store(true)
	var service serviceDef
	if err := json.Unmarshal([]byte(`{"id":1,"name":"/stop","request":{"schema":""},"response":{"schema":"bool success"}}`), &service); err != nil {
		t.Fatal(err)
	}
	client.services["/stop"] = &serviceState{def: service}
	client.connMu.Lock()
	locked := true
	defer func() {
		if locked {
			client.connMu.Unlock()
		}
	}()
	ctx, cancel := context.WithTimeout(context.Background(), 200*time.Millisecond)
	defer cancel()
	result := make(chan error, 1)
	go func() { _, err := client.CallService(ctx, "/stop", struct{}{}); result <- err }()
	// Observe registration to establish that CallService has reached its send
	// path before allowing the context to expire behind the held writer lock.
	deadline := time.Now().Add(time.Second)
	for {
		client.svcMu.Lock()
		pending := len(client.pendingSvc)
		client.svcMu.Unlock()
		if pending == 1 {
			break
		}
		if time.Now().After(deadline) {
			t.Fatal("service never reached pending send")
		}
		time.Sleep(time.Millisecond)
	}
	<-ctx.Done()
	client.connMu.Unlock()
	locked = false
	select {
	case err := <-result:
		if !errors.Is(err, context.DeadlineExceeded) {
			t.Fatalf("expired call returned %v", err)
		}
	case <-time.After(time.Second):
		t.Fatal("expired call did not finish after writer unlock")
	}
	client.svcMu.Lock()
	pending := len(client.pendingSvc)
	client.svcMu.Unlock()
	if pending != 0 {
		t.Fatalf("expired call left %d pending service requests", pending)
	}
	// A successful sentinel is positive proof that expiry did not close the
	// healthy connection and that no service frame preceded it on the wire.
	if err := client.writeMessage(websocket.TextMessage, []byte("still-connected")); err != nil {
		t.Fatal(err)
	}
	select {
	case frame := <-frames:
		if frame.err != nil || frame.kind != websocket.TextMessage || string(frame.data) != "still-connected" {
			t.Fatalf("unexpected first wire frame: kind=%d data=%q error=%v", frame.kind, frame.data, frame.err)
		}
	case <-time.After(time.Second):
		t.Fatal("sentinel did not reach bridge")
	}
}

// Publish must preserve its context through both the first advertisement and
// subsequent binary messages. A skipped stale command must leave the healthy
// socket usable for the independent cleanup zero.
func TestPublishContextDiscardsQueuedMotionAndStillSendsZero(t *testing.T) {
	for _, advertised := range []bool{false, true} {
		for _, expire := range []bool{false, true} {
			t.Run(fmt.Sprintf("advertised=%t/deadline=%t", advertised, expire), func(t *testing.T) {
				type frame struct {
					kind int
					data []byte
				}
				frames := make(chan frame, 8)
				server := httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
					conn, err := (&websocket.Upgrader{}).Upgrade(w, r, nil)
					if err != nil {
						t.Error(err)
						return
					}
					defer conn.Close()
					for {
						kind, data, err := conn.ReadMessage()
						if err != nil {
							return
						}
						frames <- frame{kind, data}
					}
				}))
				defer server.Close()
				conn, _, err := websocket.DefaultDialer.Dial("ws"+strings.TrimPrefix(server.URL, "http"), nil)
				require.NoError(t, err)
				defer conn.Close()
				client := NewClient("")
				client.conn = conn
				client.connected.Store(true)
				const topic = "/cmd_vel_teleop"
				if advertised {
					client.advertised[topic] = 1
				}
				client.connMu.Lock()
				locked := true
				defer func() {
					if locked {
						client.connMu.Unlock()
					}
				}()
				ctx, cancel := context.WithCancel(context.Background())
				if expire {
					cancel()
					ctx, cancel = context.WithTimeout(context.Background(), 50*time.Millisecond)
				}
				defer cancel()
				publishCtx := ctx
				var barrier *publishContextBarrier
				if advertised {
					// Pause after the last pre-writer cancellation check. This
					// makes the binary-path regression independent of scheduling:
					// only the check at the transport writer can discard it.
					barrier = &publishContextBarrier{Context: ctx, checked: make(chan error, 1), proceed: make(chan struct{})}
					defer barrier.release()
					publishCtx = barrier
				}
				entered := make(chan struct{})
				motion := publishTestMessage{entered: entered, payload: `{"velocity":0.2}`}
				result := make(chan error, 1)
				go func() { result <- client.PublishContext(publishCtx, topic, motion) }()
				select {
				case <-entered:
				case <-time.After(time.Second):
					t.Fatal("publish never reached serialization")
				}
				if !advertised {
					// First publication holds advMu while waiting on connMu.
					require.Eventually(t, func() bool {
						if client.advMu.TryLock() {
							client.advMu.Unlock()
							return false
						}
						return true
					}, time.Second, time.Millisecond)
				} else {
					select {
					case err := <-barrier.checked:
						require.NoError(t, err, "context expired before reaching binary publish path")
					case <-time.After(time.Second):
						t.Fatal("publish did not reach binary path")
					}
				}
				if expire {
					<-ctx.Done()
				} else {
					cancel()
				}
				if barrier != nil {
					barrier.release()
				}
				client.connMu.Unlock()
				locked = false
				select {
				case err := <-result:
					require.ErrorIs(t, err, ctx.Err())
				case <-time.After(time.Second):
					t.Fatal("stale publish did not finish after writer unlock")
				}
				require.NoError(t, client.Publish(topic, map[string]float64{"velocity": 0}))
				if !advertised {
					select {
					case f := <-frames:
						require.Equal(t, websocket.TextMessage, f.kind)
						require.Contains(t, string(f.data), `"op":"advertise"`)
					case <-time.After(time.Second):
						t.Fatal("zero advertisement missing")
					}
				}
				select {
				case f := <-frames:
					require.Equal(t, websocket.BinaryMessage, f.kind)
					require.GreaterOrEqual(t, len(f.data), 5)
					require.Equal(t, byte(clientBinMessageData), f.data[0])
					require.JSONEq(t, `{"velocity":0}`, string(f.data[5:]))
				case <-time.After(time.Second):
					t.Fatal("cleanup zero missing")
				}
				// A healthy leased command must still work on this connection.
				healthy, stop := context.WithTimeout(context.Background(), time.Second)
				defer stop()
				require.NoError(t, client.PublishContext(healthy, topic, map[string]float64{"velocity": 0.1}))
				select {
				case f := <-frames:
					require.Equal(t, websocket.BinaryMessage, f.kind)
					require.JSONEq(t, `{"velocity":0.1}`, string(f.data[5:]))
				case <-time.After(time.Second):
					t.Fatal("healthy command missing")
				}
			})
		}
	}
}

type publishTestMessage struct {
	entered chan struct{}
	payload string
}

func (m publishTestMessage) MarshalJSON() ([]byte, error) {
	close(m.entered)
	return []byte(m.payload), nil
}

// Err returns a snapshot, as it can when cancellation races immediately after
// a context check. Delay that return only at PublishContext's second check.
type publishContextBarrier struct {
	context.Context
	checks  atomic.Int64
	once    sync.Once
	checked chan error
	proceed chan struct{}
}

func (c *publishContextBarrier) release() {
	c.once.Do(func() { close(c.proceed) })
}

func (c *publishContextBarrier) Err() error {
	err := c.Context.Err()
	if c.checks.Add(1) == 2 {
		c.checked <- err
		<-c.proceed
	}
	return err
}
