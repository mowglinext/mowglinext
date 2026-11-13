package foxglove

import (
	"bytes"
	"context"
	"crypto/sha256"
	"encoding/binary"
	"encoding/json"
	"fmt"
	"math"
	"math/rand"
	"net/http"
	"net/http/httptest"
	"strings"
	"sync/atomic"
	"testing"
	"time"

	"github.com/gorilla/websocket"
)

// This test uses an actual TCP/WebSocket stream and a loss-free reference from
// the exact same frozen CDR source. It can also run inside a private network
// namespace with delay/jitter/packet loss; no impairment is installed by this
// test. TCP retransmission may change delivery time, never the expected bytes.
func TestCDRJSONRealSocketSourceReplay(t *testing.T) {
	const frameCount = 8
	const replayTopic = "/network_lab/grid"
	schema := mustParseSchema(t, cdrJSONTestSchema)
	var source, reference [frameCount][]byte
	sourceHash := sha256.New()
	for i := range source {
		cells := make([]byte, []int{300000, 16, 8192}[i%3])
		rng := rand.New(rand.NewSource(97531 + int64(i)))
		_, _ = rng.Read(cells)
		source[i] = cdrJSONFixture([]byte{0, 1, 6, 7}[i%4], cells, 6113+int64(i))
		var err error
		reference[i], err = cdrJSONReference(source[i], schema)
		if err != nil {
			t.Fatal(err)
		}
		var length [8]byte
		binary.LittleEndian.PutUint64(length[:], uint64(len(source[i])))
		_, _ = sourceHash.Write(length[:])
		_, _ = sourceHash.Write(source[i])
	}
	t.Logf("same-source frozen CDR: frames=%d sha256=%x", frameCount, sourceHash.Sum(nil))

	serverResult := make(chan error, 1)
	var accepted atomic.Int32
	server := httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		if accepted.Add(1) != 1 {
			http.Error(w, "source is replayed once", http.StatusGone)
			return
		}
		conn, err := (&websocket.Upgrader{Subprotocols: []string{"foxglove.sdk.v1"}}).Upgrade(w, r, nil)
		if err != nil {
			serverResult <- err
			return
		}
		defer conn.Close()
		_ = conn.SetReadDeadline(time.Now().Add(20 * time.Second))
		_ = conn.SetWriteDeadline(time.Now().Add(20 * time.Second))
		if err := conn.WriteJSON(serverAdvertise{Op: "advertise", Channels: []channelDef{{
			ID: 37, Topic: replayTopic, Encoding: "cdr", SchemaName: "example/msg/Replay", Schema: cdrJSONTestSchema,
		}}}); err != nil {
			serverResult <- err
			return
		}
		var subscriptionID uint32
		for subscriptionID == 0 {
			var request clientSubscribe
			if err := conn.ReadJSON(&request); err != nil {
				serverResult <- err
				return
			}
			if request.Op != "subscribe" {
				continue
			}
			for _, subscription := range request.Subscriptions {
				if subscription.ChannelID == 37 {
					subscriptionID = subscription.ID
				}
			}
		}

		// Keep processing ping/pong while large writes are delayed. Gorilla's
		// default control handler uses WriteControl, safe alongside data writes.
		readerDone := make(chan struct{})
		go func() {
			defer close(readerDone)
			for {
				if _, _, err := conn.ReadMessage(); err != nil {
					return
				}
			}
		}()
		for i, payload := range source {
			frame := make([]byte, 13, 13+len(payload))
			frame[0] = serverBinMessageData
			binary.LittleEndian.PutUint32(frame[1:], subscriptionID)
			binary.LittleEndian.PutUint64(frame[5:], 1700000000123456789+uint64(i))
			if err := conn.WriteMessage(websocket.BinaryMessage, append(frame, payload...)); err != nil {
				serverResult <- fmt.Errorf("source frame %d: %w", i, err)
				return
			}
		}
		serverResult <- nil
		<-readerDone
	}))
	defer server.Close()

	ctx, cancel := context.WithTimeout(context.Background(), 20*time.Second)
	defer cancel()
	client := NewClient("ws" + strings.TrimPrefix(server.URL, "http"))
	defer client.Close()
	var received [2]chan []byte
	for i := range received {
		i := i
		received[i] = make(chan []byte, frameCount+1)
		// Register both listeners BEFORE Connect so the single wire
		// subscription cannot race the second listener's registration.
		if err := client.Subscribe(replayTopic, "", fmt.Sprintf("listener-%d", i), func(raw json.RawMessage) {
			received[i] <- append([]byte(nil), raw...)
		}); err != nil {
			t.Fatal(err)
		}
	}
	if err := client.Connect(ctx); err != nil {
		t.Fatal(err)
	}
	for subscriber, stream := range received {
		for i, want := range reference {
			select {
			case got := <-stream:
				if !bytes.Equal(got, want) {
					t.Fatalf("subscriber %d frame %d differs: bytes=%d want=%d", subscriber, i, len(got), len(want))
				}
			case <-ctx.Done():
				t.Fatalf("subscriber %d received only %d/%d frames: %v", subscriber, i, frameCount, ctx.Err())
			}
		}
	}
	select {
	case err := <-serverResult:
		if err != nil {
			t.Fatal(err)
		}
	case <-ctx.Done():
		t.Fatal(ctx.Err())
	}
	_ = client.Close()
	select {
	case <-client.stopped:
	case <-ctx.Done():
		t.Fatal("client did not stop after replay")
	}
	for subscriber, stream := range received {
		select {
		case <-stream:
			t.Fatalf("subscriber %d received an extra frame", subscriber)
		default:
		}
	}
}

func cdrJSONFloatOnlyFixture(enc byte) ([]byte, *msgSchema, error) {
	return cdrJSONFloatFixture(enc, false)
}

func cdrJSONFloatFixture(enc byte, withByte bool) ([]byte, *msgSchema, error) {
	schemaText := "string frame_id\nuint64 sequence\nfloat64[] values\nfloat32[4] metadata\nstring source_id"
	if withByte {
		schemaText += "\nbyte[] tag"
	}
	schema, err := ParseSchema(schemaText)
	if err != nil {
		return nil, nil, err
	}
	w := newCDRJSONWire(enc)
	w.str("held-out/<>&\u2028\xff")
	w.u64(math.MaxUint64 - 19)
	w.u32(2048)
	rng := rand.New(rand.NewSource(719151))
	for i := 0; i < 2048; i++ {
		bits := rng.Uint64()
		if i%13 == 0 {
			bits = []uint64{0, 1 << 63, 1, 0x7ff8000000001234, 0xfff0000000000000}[i%5]
		}
		w.u64(bits)
	}
	for _, bits := range []uint32{0, 1 << 31, 0x7f800000, 0x7fc01234} {
		w.u32(bits)
	}
	w.str("receiver/session-held-out")
	if withByte {
		w.sequence([]byte{255})
	}
	return w.wire, schema, nil
}

func TestCDRJSONHeldOutLargeFloatOnlyExact(t *testing.T) {
	for _, enc := range []byte{0, 1, 6, 7} {
		for _, withByte := range []bool{false, true} {
			wire, schema, err := cdrJSONFloatFixture(enc, withByte)
			if err != nil {
				t.Fatal(err)
			}
			if len(wire) < 4096 {
				t.Fatal("fixture did not exercise large no-marker path")
			}
			requireCDRJSONExact(t, wire, schema)
		}
	}
}

func BenchmarkCDRJSONMixedFloats(b *testing.B) {
	wire, schema, err := cdrJSONFloatFixture(7, true)
	if err != nil {
		b.Fatal(err)
	}
	for _, arm := range []struct {
		name string
		fn   func([]byte, *msgSchema) ([]byte, error)
	}{{"baseline", cdrJSONReference}, {"candidate", deserializeCDRJSON}} {
		b.Run(arm.name, func(b *testing.B) {
			b.ReportAllocs()
			for b.Loop() {
				cdrJSONBenchmarkOutput, err = arm.fn(wire, schema)
				if err != nil {
					b.Fatal(err)
				}
			}
		})
	}
}

func BenchmarkCDRJSONFloatOnly(b *testing.B) {
	wire, schema, err := cdrJSONFloatOnlyFixture(7)
	if err != nil {
		b.Fatal(err)
	}
	for _, arm := range []struct {
		name string
		fn   func([]byte, *msgSchema) ([]byte, error)
	}{{"baseline", cdrJSONReference}, {"candidate", deserializeCDRJSON}} {
		b.Run(arm.name, func(b *testing.B) {
			b.SetBytes(int64(len(wire)))
			b.ReportAllocs()
			for b.Loop() {
				cdrJSONBenchmarkOutput, err = arm.fn(wire, schema)
				if err != nil {
					b.Fatal(err)
				}
			}
		})
	}
}
