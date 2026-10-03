package api

import (
	"encoding/json"
	"math"
	"net/http/httptest"
	"testing"
	"time"

	"github.com/gorilla/websocket"
	"github.com/mowglinext/mowglinext/pkg/types"
	"github.com/stretchr/testify/require"
	"github.com/vmihailenco/msgpack/v5"
)

type compactFrameProvider struct {
	types.IRosProvider
	subscribed chan func([]byte)
}

func (p compactFrameProvider) Subscribe(_ string, _ string, _ int, cb func([]byte)) error {
	p.subscribed <- cb
	return nil
}

func (p compactFrameProvider) UnSubscribe(string, string) {}

func compactWireFrame(t *testing.T, topic string, data []byte) []byte {
	t.Helper()
	p := compactFrameProvider{subscribed: make(chan func([]byte), 1)}
	server := httptest.NewServer(setupMowgliNextRouter(p))
	defer server.Close()
	conn := dialMultiplex(t, server)
	defer conn.Close()
	require.NoError(t, conn.WriteJSON(map[string]string{"op": "subscribe", "topic": topic}))
	var cb func([]byte)
	select {
	case cb = <-p.subscribed:
	case <-time.After(2 * time.Second):
		t.Fatal("subscription was not registered")
	}
	delivered := make(chan struct{})
	go func() { cb(data); close(delivered) }()
	require.NoError(t, conn.SetReadDeadline(time.Now().Add(2*time.Second)))
	kind, raw, err := conn.ReadMessage()
	require.NoError(t, err)
	require.Equal(t, websocket.BinaryMessage, kind)
	select {
	case <-delivered:
	case <-time.After(2 * time.Second):
		t.Fatal("frame callback did not return")
	}
	return raw
}

func TestMultiplexGridUsesCompactNumericCells(t *testing.T) {
	cells := make([]int, 300000)
	for i := range cells {
		cells[i] = []int{-1, 0, 1, 100}[i%4]
	}
	data, err := json.Marshal(map[string]any{"data": cells, "info": map[string]any{"width": 600, "height": 500, "resolution": 0.05}})
	require.NoError(t, err)
	raw := compactWireFrame(t, "mowProgress", data)
	// Every valid signed occupancy value fits one MessagePack byte. Allow
	// envelope/metadata overhead, but reject the old nine-byte float64 cells.
	require.Less(t, len(raw), len(cells)+1024)
	var frame struct {
		Topic string `msgpack:"topic"`
		Data  struct {
			Data []int `msgpack:"data"`
			Info struct {
				Resolution float64 `msgpack:"resolution"`
			} `msgpack:"info"`
		} `msgpack:"data"`
	}
	require.NoError(t, msgpack.Unmarshal(raw, &frame))
	require.Equal(t, "mowProgress", frame.Topic)
	require.Equal(t, cells, frame.Data.Data)
	require.Equal(t, 0.05, frame.Data.Info.Resolution)
	t.Logf("300000-cell multiplex frame: %d bytes", len(raw))
}

func TestMultiplexCompactEncodingPreservesJSONValues(t *testing.T) {
	data := []byte(`{"position":{"x":59.123456789,"y":-18.987654321,"z":0},"fraction":0.125,"negative":-129,"timestamp":1791040000,"values":[-1,0,100,256,1.0000000000000002,1e30],"ok":true,"missing":null,"name":"lawn"}`)
	raw := compactWireFrame(t, "pose", data)
	var frame struct {
		Topic string `msgpack:"topic"`
		Data  any    `msgpack:"data"`
	}
	require.NoError(t, msgpack.Unmarshal(raw, &frame))
	out, err := json.Marshal(frame.Data)
	require.NoError(t, err)
	require.Equal(t, "pose", frame.Topic)
	require.JSONEq(t, string(data), string(out))
}

func TestMultiplexCompactEncodingPreservesBrowserNumberTypes(t *testing.T) {
	data := []byte(`{"values":[4294967295,-2147483648,4294967296,-2147483649,9007199254740992,1791040000000000000,-0,1.0000000000000002]}`)
	raw := compactWireFrame(t, "gnssStatus", data)
	var frame struct {
		Data struct {
			Values []any `msgpack:"values"`
		} `msgpack:"data"`
	}
	require.NoError(t, msgpack.Unmarshal(raw, &frame))
	require.IsType(t, uint32(0), frame.Data.Values[0])
	require.IsType(t, int32(0), frame.Data.Values[1])
	// msgpackr decodes 64-bit MessagePack integers as BigInt, unlike the
	// float64 wire type. Require floats here instead of only comparing JSON.
	for _, value := range frame.Data.Values[2:] {
		require.IsType(t, float64(0), value)
	}
	require.True(t, math.Signbit(frame.Data.Values[6].(float64)))
	require.Equal(t, 1.0000000000000002, frame.Data.Values[7])
}
