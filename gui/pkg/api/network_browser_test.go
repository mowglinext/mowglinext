package api

import (
	"encoding/base64"
	"encoding/json"
	"net/http/httptest"
	"os"
	"testing"
	"time"

	"github.com/stretchr/testify/require"
	"github.com/vmihailenco/msgpack/v5"
)

// Regenerate the browser fixture through the actual full-frame/patch route.
// The default test still checks the complete route without writing any files.
func TestNetworkBrowserFrames(t *testing.T) {
	first := labGrid(4096, "lidar", 971)
	var obj map[string]any
	require.NoError(t, json.Unmarshal(first, &obj))
	info := obj["info"].(map[string]any)
	info["width"], info["height"] = 64, 64
	first, err := json.Marshal(obj)
	require.NoError(t, err)
	cells := obj["data"].([]any)
	cells[1], cells[4095] = -1, 100
	obj["header"].(map[string]any)["stamp"].(map[string]any)["nanosec"] = 123456790
	last, err := json.Marshal(obj)
	require.NoError(t, err)
	p := compactFrameProvider{subscribed: make(chan func([]byte), 1)}
	server := httptest.NewServer(setupMowgliNextRouter(p))
	defer server.Close()
	conn := dialMultiplex(t, server)
	defer conn.Close()
	require.NoError(t, conn.WriteJSON(map[string]any{"op": "subscribe", "topic": "lidarMap", "delta": true}))
	var cb func([]byte)
	select {
	case cb = <-p.subscribed:
	case <-time.After(2 * time.Second):
		t.Fatal("subscribe timeout")
	}
	var frames []string
	for _, source := range [][]byte{first, last} {
		done := make(chan struct{})
		go func() { cb(source); close(done) }()
		require.NoError(t, conn.SetReadDeadline(time.Now().Add(3*time.Second)))
		_, wire, err := conn.ReadMessage()
		require.NoError(t, err)
		frames = append(frames, base64.StdEncoding.EncodeToString(wire))
		<-done
	}
	var full map[string]msgpack.RawMessage
	raw, _ := base64.StdEncoding.DecodeString(frames[0])
	require.NoError(t, msgpack.Unmarshal(raw, &full))
	require.Contains(t, full, "seq")
	require.Contains(t, full, "data")
	var patch map[string]msgpack.RawMessage
	raw, _ = base64.StdEncoding.DecodeString(frames[1])
	require.NoError(t, msgpack.Unmarshal(raw, &patch))
	require.Contains(t, patch, "patch")
	if path := os.Getenv("MOWGLI_NETWORK_BROWSER_FIXTURE"); path != "" {
		fixture, err := json.Marshal(struct {
			Frames   []string          `json:"frames"`
			Expected []json.RawMessage `json:"expected"`
		}{frames, []json.RawMessage{first, last}})
		require.NoError(t, err)
		require.NoError(t, os.WriteFile(path, append(fixture, '\n'), 0600))
	}
}
