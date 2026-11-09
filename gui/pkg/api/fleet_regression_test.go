package api

import (
	"encoding/json"
	"errors"
	"net"
	"net/http"
	"net/http/httptest"
	"strconv"
	"sync/atomic"
	"testing"
	"time"

	"github.com/gorilla/websocket"
	"github.com/mowglinext/mowglinext/pkg/providers"
	"github.com/mowglinext/mowglinext/pkg/types"
	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
	"github.com/vmihailenco/msgpack/v5"
)

func TestFleetHighLevelAcknowledgementParity(t *testing.T) {
	for _, tc := range []struct {
		name      string
		accepted  bool
		transport error
		code      int
	}{
		{"success", true, nil, http.StatusOK},
		{"refused", false, nil, http.StatusInternalServerError},
		{"transport", false, errors.New("ROS unavailable"), http.StatusInternalServerError},
	} {
		t.Run(tc.name, func(t *testing.T) {
			setup := func(ros *types.MockRosProvider) {
				ros.ServiceErr = tc.transport
				ros.ServiceResponder = func(_ string, _ any, res any) {
					body, _ := json.Marshal(map[string]bool{"success": tc.accepted})
					_ = json.Unmarshal(body, res)
				}
			}
			a := newFleetRobot(t, "alpha", setup)
			b := newFleetRobot(t, "bravo", setup)
			res, body := a.post(t, "/api/fleet/peers", map[string]string{"address": b.addr})
			require.Equal(t, http.StatusOK, res.StatusCode, string(body))
			for _, path := range []string{"/api/mowglinext/call/high_level_control", "/api/fleet/robots/" + a.id + "/call/high_level_control", "/api/fleet/robots/" + b.id + "/call/high_level_control"} {
				res, body = a.post(t, path, map[string]int{"Command": 1})
				assert.Equal(t, tc.code, res.StatusCode, path+": "+string(body))
				if tc.code != http.StatusOK {
					assert.Contains(t, string(body), "error")
				}
			}
		})
	}
}

func TestFleetResetReportsEveryMemberOutcome(t *testing.T) {
	for _, tc := range []struct {
		name, outcome              string
		remoteStatus, globalStatus int
	}{
		{"success", "success", 200, 200}, {"refused", "refused", 500, 500},
		{"transport", "transport_error", 0, 500}, {"offline", "offline", 0, 200},
	} {
		t.Run(tc.name, func(t *testing.T) {
			a := newFleetRobot(t, "alpha", func(ros *types.MockRosProvider) {
				ros.ServiceResponder = func(_ string, _ any, res any) { _ = json.Unmarshal([]byte(`{"success":true}`), res) }
			})
			var calls atomic.Int32
			subscribed := make(chan struct{})
			peer := httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
				switch r.URL.Path {
				case "/api/fleet/identity":
					_ = json.NewEncoder(w).Encode(providers.RobotIdentity{ID: "peer", Name: "bravo", APIVersion: providers.FleetAPIVersion})
				case "/api/mowglinext/multiplex":
					u := websocket.Upgrader{}
					conn, err := u.Upgrade(w, r, nil)
					if err != nil {
						return
					}
					defer conn.Close()
					for range providers.FleetTopics {
						if _, _, err := conn.ReadMessage(); err != nil {
							return
						}
					}
					if tc.name != "offline" {
						frame, _ := msgpack.Marshal(map[string]any{"topic": "highLevelStatus", "data": map[string]any{"state": 1}})
						if conn.WriteMessage(websocket.BinaryMessage, frame) != nil {
							return
						}
					}
					close(subscribed)
					for {
						if _, _, err := conn.ReadMessage(); err != nil {
							return
						}
					}
				case "/api/mowglinext/call/coverage_clear_resume":
					calls.Add(1)
					if tc.name == "transport" {
						conn, _, err := w.(http.Hijacker).Hijack()
						if err == nil {
							_ = conn.Close()
						}
						return
					}
					w.WriteHeader(tc.remoteStatus)
					_, _ = w.Write([]byte(`{"error":"clear refused"}`))
				default:
					w.WriteHeader(http.StatusNotFound)
				}
			}))
			defer peer.Close()
			host, portString, err := net.SplitHostPort(peer.Listener.Addr().String())
			require.NoError(t, err)
			port, err := strconv.Atoi(portString)
			require.NoError(t, err)
			_, err = a.fleet.RegisterPeer("peer", "bravo", host, port, providers.FleetAPIVersion)
			require.NoError(t, err)
			select {
			case <-subscribed:
			case <-time.After(3 * time.Second):
				t.Fatal("peer did not subscribe")
			}
			require.Eventually(t, func() bool {
				rows, err := a.fleet.Robots()
				return err == nil && len(rows) == 2 && rows[1].Online == (tc.name != "offline")
			}, 3*time.Second, 5*time.Millisecond)
			res, body := a.post(t, "/api/fleet/coordination/reset", map[string]any{})
			require.Equal(t, tc.globalStatus, res.StatusCode, string(body))
			var result struct {
				Error   string `json:"error"`
				Results []struct {
					ID         string `json:"id"`
					Status     string `json:"status"`
					HTTPStatus int    `json:"http_status"`
				} `json:"results"`
			}
			require.NoError(t, json.Unmarshal(body, &result))
			require.Len(t, result.Results, 2)
			assert.Equal(t, "success", result.Results[0].Status)
			assert.Equal(t, "peer", result.Results[1].ID)
			assert.Equal(t, tc.outcome, result.Results[1].Status)
			if tc.name == "offline" {
				assert.Zero(t, calls.Load())
			} else {
				assert.Equal(t, int32(1), calls.Load())
			}
			if tc.globalStatus != 200 {
				assert.Contains(t, result.Error, "bravo")
			}
			if tc.name == "refused" {
				assert.Equal(t, 500, result.Results[1].HTTPStatus)
				assert.Contains(t, result.Error, "HTTP 500")
			}
		})
	}
}

func TestFleetResetLocalRejectionIsNotSuccess(t *testing.T) {
	for _, detail := range []string{"", "maintenance active"} {
		t.Run(detail, func(t *testing.T) {
			a := newFleetRobot(t, "alpha", func(ros *types.MockRosProvider) {
				ros.ServiceResponder = func(service string, _ any, res any) {
					accepted := service != "/behavior_tree_node/clear_coverage_resume"
					body, _ := json.Marshal(map[string]any{"success": accepted, "message": detail})
					_ = json.Unmarshal(body, res)
				}
			})
			res, body := a.post(t, "/api/fleet/coordination/reset", map[string]any{})
			require.Equal(t, http.StatusInternalServerError, res.StatusCode, string(body))
			var result struct {
				Error   string `json:"error"`
				Results []struct {
					Status string `json:"status"`
					Error  string `json:"error"`
				} `json:"results"`
			}
			require.NoError(t, json.Unmarshal(body, &result))
			require.Len(t, result.Results, 1)
			assert.Equal(t, "refused", result.Results[0].Status)
			assert.NotEmpty(t, result.Results[0].Error)
			assert.NotEmpty(t, result.Error)
		})
	}
}
