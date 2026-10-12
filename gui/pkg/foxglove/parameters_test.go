package foxglove

import (
	"context"
	"errors"
	"net/http"
	"net/http/httptest"
	"strings"
	"testing"
	"time"

	"github.com/gorilla/websocket"
)

func TestSetParametersAcknowledgement(t *testing.T) {
	for _, mode := range []string{"echo", "no reply", "late reply", "disconnect", "rejection"} {
		t.Run(mode, func(t *testing.T) {
			requestSeen := make(chan struct{})
			releaseReply := make(chan struct{})
			server := httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
				upgrader := websocket.Upgrader{Subprotocols: []string{"foxglove.sdk.v1"}}
				conn, err := upgrader.Upgrade(w, r, nil)
				if err != nil {
					t.Error(err)
					return
				}
				defer conn.Close()
				var request struct {
					Op         string      `json:"op"`
					ID         string      `json:"id"`
					Parameters []Parameter `json:"parameters"`
				}
				if err := conn.ReadJSON(&request); err != nil {
					t.Error(err)
					return
				}
				if request.Op != "setParameters" || request.ID == "" {
					t.Errorf("invalid request: %+v", request)
				}
				close(requestSeen)
				switch mode {
				case "echo":
					// Return a different value to prove we use the actual response.
					request.Parameters[0].Value = 0.1
					_ = conn.WriteJSON(map[string]any{"op": "parameterValues", "id": request.ID, "parameters": request.Parameters})
				case "late reply":
					<-releaseReply
					_ = conn.WriteJSON(map[string]any{"op": "parameterValues", "id": request.ID, "parameters": request.Parameters})
				case "disconnect":
					return
				case "rejection":
					// Protocol status is not a correlated parameter acknowledgement.
					_ = conn.WriteJSON(map[string]any{"op": "status", "level": 2, "message": "parameter is read-only"})
				}
				_, _, _ = conn.ReadMessage()
			}))
			defer server.Close()
			client := NewClient("ws" + strings.TrimPrefix(server.URL, "http"))
			connectCtx, cancelConnect := context.WithCancel(context.Background())
			defer cancelConnect()
			if err := client.Connect(connectCtx); err != nil {
				t.Fatal(err)
			}
			defer client.Close()
			// Cancel only once the peer has received the request, avoiding timing
			// assumptions about how quickly a local WebSocket is scheduled.
			ctx, cancel := context.WithCancel(context.Background())
			defer cancel()
			if mode != "echo" {
				go func() { <-requestSeen; cancel() }()
			}
			got, err := client.SetParameters(ctx, []Parameter{{Name: "mowgli.speed", Value: 0.2, Type: "float64"}})
			if mode == "late reply" {
				close(releaseReply)
			}
			if mode == "echo" {
				if err != nil || len(got) != 1 || got[0].Value != 0.1 {
					t.Fatalf("reply=%+v err=%v", got, err)
				}
			} else if got != nil || !errors.Is(err, context.Canceled) || !strings.Contains(err.Error(), "update unconfirmed") {
				t.Fatalf("unacknowledged update returned values=%+v err=%v", got, err)
			}
			client.paramMu.Lock()
			pending := len(client.pendingParam)
			client.paramMu.Unlock()
			if pending != 0 {
				t.Fatalf("leaked %d requests", pending)
			}
		})
	}
}

func TestSetParametersDeadlineIsUnconfirmed(t *testing.T) {
	// An expired deadline exercises the same select branch as the client's
	// 10-second deadline without adding a wall-clock wait to the regression.
	ctx, cancel := context.WithDeadline(context.Background(), time.Now().Add(-time.Second))
	defer cancel()
	client := NewClient("")
	client.connected.Store(true)
	// Supply an actual socket so the request is successfully written first.
	server := httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		upgrader := websocket.Upgrader{}
		conn, err := upgrader.Upgrade(w, r, nil)
		if err != nil {
			t.Error(err)
			return
		}
		defer conn.Close()
		for {
			if _, _, err := conn.ReadMessage(); err != nil {
				return
			}
		}
	}))
	defer server.Close()
	conn, _, err := websocket.DefaultDialer.Dial("ws"+strings.TrimPrefix(server.URL, "http"), nil)
	if err != nil {
		t.Fatal(err)
	}
	client.conn = conn
	defer client.Close()
	got, err := client.SetParameters(ctx, []Parameter{{Name: "mowgli.speed", Value: 0.2}})
	if got != nil || !errors.Is(err, context.DeadlineExceeded) || !strings.Contains(err.Error(), "update unconfirmed") {
		t.Fatalf("values=%+v err=%v", got, err)
	}
}
