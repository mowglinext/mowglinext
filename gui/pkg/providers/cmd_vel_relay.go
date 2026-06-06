package providers

import (
	"encoding/json"
	"fmt"
	"sync"
	"time"

	"github.com/gorilla/websocket"
	"github.com/sirupsen/logrus"
)

const (
	cmdVelRelayReconnectDelay = 1 * time.Second
	cmdVelRelayMaxReconnect   = 10 * time.Second
)

// cmdVelRelayClient maintains a persistent WebSocket connection to the
// cmd_vel_ws_relay rclpy node (port 8766). It publishes TwistStamped JSON
// directly to /cmd_vel_teleop without going through foxglove_bridge,
// eliminating the JSON→CDR conversion overhead and the shared-connection
// head-of-line blocking with subscription data.
//
// All exported methods are safe for concurrent use.
type cmdVelRelayClient struct {
	url string

	mu   sync.Mutex
	conn *websocket.Conn

	connected bool
	done      chan struct{}
}

func newCmdVelRelayClient(url string) *cmdVelRelayClient {
	c := &cmdVelRelayClient{
		url:  url,
		done: make(chan struct{}),
	}
	go c.reconnectLoop()
	return c
}

// Send JSON-encodes msg and writes it over the relay WebSocket.
// Returns a non-nil error when the relay is not connected; callers
// should fall back to foxglove_bridge in that case.
func (c *cmdVelRelayClient) Send(msg interface{}) error {
	data, err := json.Marshal(msg)
	if err != nil {
		return fmt.Errorf("cmd_vel relay: marshal: %w", err)
	}

	c.mu.Lock()
	defer c.mu.Unlock()
	if c.conn == nil {
		return fmt.Errorf("cmd_vel relay: not connected")
	}
	return c.conn.WriteMessage(websocket.TextMessage, data)
}

// Connected reports whether the relay WebSocket is currently up.
func (c *cmdVelRelayClient) Connected() bool {
	c.mu.Lock()
	defer c.mu.Unlock()
	return c.conn != nil
}

// Close shuts down the reconnect loop and closes the connection.
func (c *cmdVelRelayClient) Close() {
	select {
	case <-c.done:
	default:
		close(c.done)
	}
	c.mu.Lock()
	if c.conn != nil {
		_ = c.conn.Close()
		c.conn = nil
	}
	c.mu.Unlock()
}

func (c *cmdVelRelayClient) reconnectLoop() {
	dialer := websocket.Dialer{HandshakeTimeout: 2 * time.Second}
	delay := cmdVelRelayReconnectDelay

	for {
		select {
		case <-c.done:
			return
		default:
		}

		conn, _, err := dialer.Dial(c.url, nil)
		if err != nil {
			logrus.WithField("retry_in", delay).
				Debug("cmd_vel relay: connect failed, will retry")
			select {
			case <-c.done:
				return
			case <-time.After(delay):
			}
			if delay < cmdVelRelayMaxReconnect {
				delay *= 2
				if delay > cmdVelRelayMaxReconnect {
					delay = cmdVelRelayMaxReconnect
				}
			}
			continue
		}

		logrus.WithField("url", c.url).Info("cmd_vel relay: connected")
		delay = cmdVelRelayReconnectDelay

		c.mu.Lock()
		c.conn = conn
		c.mu.Unlock()

		// Drain server messages to detect disconnection. The relay never
		// sends data to us, so this will block until the connection closes.
		for {
			if _, _, err := conn.ReadMessage(); err != nil {
				break
			}
		}

		c.mu.Lock()
		c.conn = nil
		c.mu.Unlock()
		logrus.Debug("cmd_vel relay: disconnected, will reconnect")
	}
}
