package api

import (
	"encoding/json"
	"net"
	"net/http/httptest"
	"strings"
	"sync/atomic"
	"testing"
	"time"

	"github.com/gorilla/websocket"
	"github.com/stretchr/testify/require"
	"github.com/vmihailenco/msgpack/v5"
)

// countingConn counts the bytes that actually crossed the socket (compressed, on the wire).
type countingConn struct {
	net.Conn
	read *int64
}

func (c countingConn) Read(b []byte) (int, error) {
	n, err := c.Conn.Read(b)
	atomic.AddInt64(c.read, int64(n))
	return n, err
}

type deflateResult struct {
	extensions string
	wireBytes  int64
	cells      []int
}

// gridOverTheWire sends one occupancy-grid frame through the multiplex route and
// reports how many bytes the browser side really received.
func gridOverTheWire(t *testing.T, offerCompression bool, topic string, cells []int) deflateResult {
	t.Helper()
	p := compactFrameProvider{subscribed: make(chan func([]byte), 1)}
	server := httptest.NewServer(setupMowgliNextRouter(p))
	defer server.Close()

	var wire int64
	dialer := websocket.Dialer{
		EnableCompression: offerCompression,
		NetDial: func(network, addr string) (net.Conn, error) {
			conn, err := net.Dial(network, addr)
			return countingConn{Conn: conn, read: &wire}, err
		},
	}
	wsURL := "ws" + strings.TrimPrefix(server.URL, "http") + "/api/mowglinext/multiplex"
	conn, resp, err := dialer.Dial(wsURL, nil)
	require.NoError(t, err)
	defer conn.Close()

	require.NoError(t, conn.WriteJSON(map[string]string{"op": "subscribe", "topic": topic}))
	var cb func([]byte)
	select {
	case cb = <-p.subscribed:
	case <-time.After(2 * time.Second):
		t.Fatal("subscription was not registered")
	}
	data, err := json.Marshal(map[string]any{"data": cells, "info": map[string]any{"width": 600, "height": 500, "resolution": 0.05}})
	require.NoError(t, err)

	before := atomic.LoadInt64(&wire)
	go cb(data)
	require.NoError(t, conn.SetReadDeadline(time.Now().Add(3*time.Second)))
	_, raw, err := conn.ReadMessage()
	require.NoError(t, err)

	var frame struct {
		Topic string `msgpack:"topic"`
		Data  struct {
			Data []int `msgpack:"data"`
		} `msgpack:"data"`
	}
	require.NoError(t, msgpack.Unmarshal(raw, &frame))
	require.Equal(t, topic, frame.Topic)
	return deflateResult{
		extensions: resp.Header.Get("Sec-WebSocket-Extensions"),
		wireBytes:  atomic.LoadInt64(&wire) - before,
		cells:      frame.Data.Data,
	}
}

// A mow-progress grid is almost all one value (a few cut stripes), so it must travel in a
// small fraction of its size when the browser offers permessage-deflate — and arrive intact.
func TestMultiplexCompressesLargeGridFrames(t *testing.T) {
	cells := make([]int, 300000)
	for i := range cells {
		if i%1000 < 40 {
			cells[i] = 100
		}
	}

	plain := gridOverTheWire(t, false, "mowProgress", cells)
	require.Empty(t, plain.extensions, "a client that does not offer deflate must not get it")
	require.Equal(t, cells, plain.cells)
	require.GreaterOrEqual(t, plain.wireBytes, int64(len(cells)), "without compression the grid is sent at full size")

	packed := gridOverTheWire(t, true, "mowProgress", cells)
	require.Contains(t, packed.extensions, "permessage-deflate")
	require.Equal(t, cells, packed.cells, "decompressed grid must equal what was sent")
	require.Less(t, packed.wireBytes, int64(len(cells))/20,
		"a nearly empty grid should shrink by well over 20x, got %d bytes", packed.wireBytes)
	t.Logf("300000-cell grid on the wire: %d bytes uncompressed, %d bytes with permessage-deflate", plain.wireBytes, packed.wireBytes)
}

// Small, frequent frames (pose, status) are not worth compressing.
func TestMultiplexDoesNotCompressSmallFrames(t *testing.T) {
	require.Equal(t, 4096, wsCompressMinBytes)
	small := make([]int, 100)
	r := gridOverTheWire(t, true, "mowProgress", small)
	require.Contains(t, r.extensions, "permessage-deflate", "negotiated for the connection")
	require.Equal(t, small, r.cells)
	require.Less(t, r.wireBytes, int64(wsCompressMinBytes))
}
