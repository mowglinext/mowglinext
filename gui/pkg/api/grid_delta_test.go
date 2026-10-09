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

func gridMessage(t *testing.T, width, height int, origin float64, stamp int, cells []int) []byte {
	t.Helper()
	data, err := json.Marshal(map[string]any{
		"header": map[string]any{"stamp": map[string]any{"sec": stamp, "nanosec": 0}, "frame_id": "map"},
		"info": map[string]any{
			"map_load_time": map[string]any{"sec": stamp, "nanosec": 7}, // changes every publish
			"resolution":    0.05, "width": width, "height": height,
			"origin": map[string]any{"position": map[string]any{"x": origin, "y": 0, "z": 0}, "orientation": map[string]any{"x": 0, "y": 0, "z": 0, "w": 1}},
		},
		"data": cells,
	})
	require.NoError(t, err)
	return data
}

// applyPatch is what the browser does with a patch.
func applyPatch(cells []int, f gridFrame) []int {
	out := append([]int(nil), cells...)
	at := 0
	for i := range f.Gaps {
		at += f.Gaps[i]
		out[at] = f.Vals[i]
	}
	return out
}

func zeros(n int) []int { return make([]int, n) }

func TestGridDeltaFirstFrameIsFullThenPatches(t *testing.T) {
	d := &gridDelta{}
	a := zeros(1000)
	f, ok := d.next(gridMessage(t, 40, 25, 0, 1, a))
	require.True(t, ok)
	require.True(t, f.Full)
	require.EqualValues(t, 1, f.Seq)

	b := append([]int(nil), a...)
	b[3], b[500], b[999] = 100, 100, 50
	f, ok = d.next(gridMessage(t, 40, 25, 0, 2, b)) // new stamp and map_load_time, same cells otherwise
	require.True(t, ok)
	require.False(t, f.Full, "a few changed cells must be a patch")
	require.EqualValues(t, 1, f.Base)
	require.EqualValues(t, 2, f.Seq)
	require.Equal(t, []int{3, 497, 499}, f.Gaps)
	require.Equal(t, []int{100, 100, 50}, f.Vals)
	require.Equal(t, b, applyPatch(a, f))
	require.Contains(t, string(f.Header), `"sec":2`, "the patch carries the new header stamp")
}

func TestGridDeltaUnchangedGridIsAnEmptyPatch(t *testing.T) {
	d := &gridDelta{}
	a := zeros(100)
	d.next(gridMessage(t, 10, 10, 0, 1, a))
	f, ok := d.next(gridMessage(t, 10, 10, 0, 2, a))
	require.True(t, ok)
	require.False(t, f.Full)
	require.Empty(t, f.Vals)
}

func TestGridDeltaSendsFullWhenTheGridMoves(t *testing.T) {
	d := &gridDelta{}
	a := zeros(100)
	d.next(gridMessage(t, 10, 10, 0, 1, a))
	f, _ := d.next(gridMessage(t, 10, 10, 5, 2, a)) // origin moved (the LiDAR map's window)
	require.True(t, f.Full)
	f, _ = d.next(gridMessage(t, 20, 5, 5, 3, a)) // same cell count, other shape
	require.True(t, f.Full)
	f, _ = d.next(gridMessage(t, 20, 5, 5, 4, a))
	require.False(t, f.Full, "after the full frame the next identical-shape grid is a patch again")
}

func TestGridDeltaSendsFullWhenMostOfTheGridChanged(t *testing.T) {
	d := &gridDelta{}
	a := zeros(1000)
	d.next(gridMessage(t, 40, 25, 0, 1, a))
	b := zeros(1000)
	for i := 0; i < 300; i++ {
		b[i] = 100 // 30% changed: not worth a patch
	}
	f, _ := d.next(gridMessage(t, 40, 25, 0, 2, b))
	require.True(t, f.Full)
}

func TestGridDeltaSendsAFullFrameEveryNPatches(t *testing.T) {
	d := &gridDelta{}
	cells := zeros(100)
	f, _ := d.next(gridMessage(t, 10, 10, 0, 0, cells))
	require.True(t, f.Full)
	fulls := 0
	for i := 1; i <= gridDeltaFullEvery+1; i++ {
		cells[i%100] = i % 100
		f, _ = d.next(gridMessage(t, 10, 10, 0, i, cells))
		if f.Full {
			fulls++
		}
	}
	require.Equal(t, 1, fulls, "one periodic full frame within %d updates", gridDeltaFullEvery+1)
}

func TestGridDeltaResyncReplaysTheLastGridInFull(t *testing.T) {
	d := &gridDelta{}
	_, ok := d.resync()
	require.False(t, ok, "nothing to resync before the first grid")
	a := zeros(100)
	d.next(gridMessage(t, 10, 10, 0, 1, a))
	b := append([]int(nil), a...)
	b[7] = 100
	last := gridMessage(t, 10, 10, 0, 2, b)
	d.next(last)
	f, ok := d.resync()
	require.True(t, ok)
	require.True(t, f.Full)
	require.EqualValues(t, 3, f.Seq)
	require.Equal(t, last, f.Raw)
}

func TestGridDeltaIgnoresWhatIsNotAGrid(t *testing.T) {
	d := &gridDelta{}
	_, ok := d.next([]byte(`{"hello":"world"}`))
	require.False(t, ok)
	_, ok = d.next([]byte(`not json`))
	require.False(t, ok)
}

// --- the whole route, over a real WebSocket -------------------------------------------

// deltaCountingConn counts the bytes that actually crossed the socket.
type deltaCountingConn struct {
	net.Conn
	read *int64
}

func (c deltaCountingConn) Read(b []byte) (int, error) {
	n, err := c.Conn.Read(b)
	atomic.AddInt64(c.read, int64(n))
	return n, err
}

type deltaFrame struct {
	Topic string `msgpack:"topic"`
	Seq   uint64 `msgpack:"seq"`
	Data  *struct {
		Data []int `msgpack:"data"`
	} `msgpack:"data"`
	Patch *struct {
		Base uint64 `msgpack:"base"`
		Seq  uint64 `msgpack:"seq"`
		Gaps []int  `msgpack:"gaps"`
		Vals []int  `msgpack:"vals"`
	} `msgpack:"patch"`
}

type deltaClient struct {
	t    *testing.T
	conn *websocket.Conn
	cb   func([]byte)
	wire *int64
}

func openDeltaClient(t *testing.T, topic string, delta bool) *deltaClient {
	t.Helper()
	p := compactFrameProvider{subscribed: make(chan func([]byte), 1)}
	server := httptest.NewServer(setupMowgliNextRouter(p))
	t.Cleanup(server.Close)
	var wire int64
	dialer := websocket.Dialer{NetDial: func(network, addr string) (net.Conn, error) {
		c, err := net.Dial(network, addr)
		return deltaCountingConn{Conn: c, read: &wire}, err
	}}
	conn, _, err := dialer.Dial("ws"+strings.TrimPrefix(server.URL, "http")+"/api/mowglinext/multiplex", nil)
	require.NoError(t, err)
	t.Cleanup(func() { _ = conn.Close() })
	require.NoError(t, conn.WriteJSON(map[string]any{"op": "subscribe", "topic": topic, "delta": delta}))
	var cb func([]byte)
	select {
	case cb = <-p.subscribed:
	case <-time.After(2 * time.Second):
		t.Fatal("subscription was not registered")
	}
	return &deltaClient{t: t, conn: conn, cb: cb, wire: &wire}
}

func (c *deltaClient) deliver(msg []byte) (deltaFrame, int64) {
	c.t.Helper()
	before := atomic.LoadInt64(c.wire)
	go c.cb(msg)
	return c.read(), atomic.LoadInt64(c.wire) - before
}

func (c *deltaClient) read() deltaFrame {
	c.t.Helper()
	require.NoError(c.t, c.conn.SetReadDeadline(time.Now().Add(3*time.Second)))
	_, raw, err := c.conn.ReadMessage()
	require.NoError(c.t, err)
	var f deltaFrame
	require.NoError(c.t, msgpack.Unmarshal(raw, &f))
	return f
}

func TestMultiplexSendsGridPatchesToADeltaSubscriber(t *testing.T) {
	const n = 300000
	c := openDeltaClient(t, "mowProgress", true)
	a := zeros(n)
	for i := 0; i < n; i += 1000 {
		a[i] = 100
	}

	first, fullBytes := c.deliver(gridMessage(t, 600, 500, 0, 1, a))
	require.NotNil(t, first.Data, "first frame is the full grid")
	require.EqualValues(t, 1, first.Seq)
	require.Equal(t, a, first.Data.Data)

	b := append([]int(nil), a...)
	b[10], b[12], b[200001] = 100, 100, 100
	second, patchBytes := c.deliver(gridMessage(t, 600, 500, 0, 2, b))
	require.Nil(t, second.Data, "an update with three changed cells must not resend the grid")
	require.NotNil(t, second.Patch)
	require.EqualValues(t, 1, second.Patch.Base)
	require.EqualValues(t, 2, second.Patch.Seq)
	got := append([]int(nil), a...)
	at := 0
	for i := range second.Patch.Gaps {
		at += second.Patch.Gaps[i]
		got[at] = second.Patch.Vals[i]
	}
	require.Equal(t, b, got, "the patch applied to the first grid is the second grid")
	require.Less(t, patchBytes, int64(200), "a three-cell patch is a few dozen bytes on the wire, got %d", patchBytes)
	t.Logf("300000-cell grid: full frame %d bytes, 3-cell patch %d bytes", fullBytes, patchBytes)

	// The browser reports a gap: the grid comes again in full, under a new seq.
	require.NoError(t, c.conn.WriteJSON(map[string]any{"op": "resync", "topic": "mowProgress"}))
	again := c.read()
	require.NotNil(t, again.Data)
	require.EqualValues(t, 3, again.Seq)
	require.Equal(t, b, again.Data.Data)
}

func TestMultiplexKeepsSendingWholeGridsToASubscriberThatDidNotAskForDeltas(t *testing.T) {
	c := openDeltaClient(t, "mowProgress", false)
	a := zeros(1000)
	first, _ := c.deliver(gridMessage(t, 40, 25, 0, 1, a))
	require.NotNil(t, first.Data)
	require.Zero(t, first.Seq, "no seq without delta")
	b := append([]int(nil), a...)
	b[1] = 100
	second, _ := c.deliver(gridMessage(t, 40, 25, 0, 2, b))
	require.NotNil(t, second.Data, "without delta every update is the whole grid")
	require.Nil(t, second.Patch)
}

func TestMultiplexDeltaIsOnlyForGridTopics(t *testing.T) {
	c := openDeltaClient(t, "pose", true)
	first, _ := c.deliver([]byte(`{"pose":{"x":1}}`))
	require.Nil(t, first.Patch)
	require.Zero(t, first.Seq)
}
