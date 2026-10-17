package providers

import (
	"bufio"
	"crypto/sha1"
	"encoding/base64"
	"fmt"
	"net"
	"net/http"
	"net/url"
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
