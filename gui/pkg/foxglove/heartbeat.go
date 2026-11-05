package foxglove

import (
	"context"
	"sync/atomic"
	"time"

	"github.com/gorilla/websocket"
)

const heartbeatPayload = "mowgli-gui"

// TransportHealth describes only the Foxglove wire transport. HeartbeatOK
// proves a recent pong, never fresh ROS observations or mower readiness.
type TransportHealth struct {
	Connected   bool       `json:"connected"`
	HeartbeatOK bool       `json:"heartbeat_ok"`
	LastPongAt  *time.Time `json:"last_pong_at,omitempty"`
}

type connectionHeartbeat struct {
	connected atomic.Bool
	lastPong  atomic.Pointer[time.Time]
}

// Health is a local, non-blocking snapshot, independent of topic rates and I/O.
func (c *Client) Health() TransportHealth {
	heartbeat := c.heartbeat.Load()
	if heartbeat == nil {
		return TransportHealth{}
	}
	pong := heartbeat.lastPong.Load()
	if pong != nil {
		snapshot := *pong
		pong = &snapshot
	}
	connected := heartbeat.connected.Load()
	return TransportHealth{
		Connected:   connected,
		HeartbeatOK: connected && pong != nil && time.Since(*pong) < c.pongTimeout,
		LastPongAt:  pong,
	}
}

// WriteControl is safe alongside the serialized data writer. A separate probe
// belongs to exactly one socket; it never writes through c.conn after reconnect.
func (c *Client) probeConnection(ctx context.Context, conn *websocket.Conn, stop <-chan struct{}) {
	ticker := time.NewTicker(c.pingInterval)
	defer ticker.Stop()
	for {
		select {
		case <-stop:
			return
		case <-c.done:
			_ = conn.Close()
			return
		case <-ctx.Done():
			_ = conn.Close()
			return
		default:
		}
		if err := conn.WriteControl(websocket.PingMessage, []byte(heartbeatPayload), time.Now().Add(c.writeTimeout)); err != nil {
			_ = conn.Close()
			return
		}
		select {
		case <-stop:
			return
		case <-c.done:
			_ = conn.Close()
			return
		case <-ctx.Done():
			_ = conn.Close()
			return
		case <-ticker.C:
		}
	}
}
