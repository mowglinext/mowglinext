package providers

import (
	"bytes"
	"context"
	"encoding/binary"
	"encoding/json"
	"net/http"
	"net/http/httptest"
	"strings"
	"testing"
	"time"

	"github.com/gorilla/websocket"
	"github.com/mowglinext/mowglinext/pkg/foxglove"
	"github.com/mowglinext/mowglinext/pkg/msgs/mowgli"
)

const mapAreaResponseSchema = `mowgli_interfaces/MapArea area
bool success
================================================================================
MSG: mowgli_interfaces/MapArea
string name
geometry_msgs/Polygon area
geometry_msgs/Polygon[] obstacles
bool is_navigation_area
mowgli_interfaces/MapObstacleInfo[] obstacle_info
geometry_msgs/Polygon[] proposed_obstacles
mowgli_interfaces/MapObstacleInfo[] proposed_obstacle_info
uint32 id
================================================================================
MSG: geometry_msgs/Polygon
geometry_msgs/Point32[] points
================================================================================
MSG: geometry_msgs/Point32
float32 x
float32 y
float32 z
================================================================================
MSG: mowgli_interfaces/MapObstacleInfo
string name
uint8 source
bool pending
uint32 id`

// Use the production schemas and CDR codec, including real polygon geometry.
func mapAreaResponse(t *testing.T, name string, id uint32, navigation bool) []byte {
	t.Helper()
	polygon := map[string]any{"points": []any{
		map[string]any{"x": float64(id), "y": 0, "z": 0},
		map[string]any{"x": float64(id) + 1, "y": 0, "z": 0},
		map[string]any{"x": float64(id), "y": 1, "z": 0},
	}}
	data, err := json.Marshal(map[string]any{"success": name != "", "area": map[string]any{
		"name": name, "id": id, "area": polygon, "is_navigation_area": navigation,
		"obstacles": []any{polygon}, "proposed_obstacles": []any{polygon},
		"obstacle_info":          []any{map[string]any{"name": "tree", "source": 1, "pending": false, "id": 4}},
		"proposed_obstacle_info": []any{map[string]any{"name": "proposal", "source": 2, "pending": true, "id": 5}},
	}})
	if err != nil {
		t.Fatal(err)
	}
	schema, err := foxglove.ParseSchema(mapAreaResponseSchema)
	if err != nil {
		t.Fatal(err)
	}
	data, err = foxglove.SerializeCDR(data, schema)
	if err != nil {
		t.Fatal(err)
	}
	return data
}

func TestPollMapCommitsOnlyCompleteEnumeration(t *testing.T) {
	for _, scenario := range []string{"failure at zero", "failure in middle", "connection drop", "deadline", "enumeration guard", "shorter", "empty", "unchanged"} {
		t.Run(scenario, func(t *testing.T) {
			front := mapAreaResponse(t, "front", 101, false)
			back := mapAreaResponse(t, "back", 102, true)
			end := mapAreaResponse(t, "", 0, false)
			server := httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, req *http.Request) {
				conn, err := (&websocket.Upgrader{Subprotocols: []string{"foxglove.sdk.v1"}}).Upgrade(w, req, nil)
				if err != nil {
					t.Error(err)
					return
				}
				defer conn.Close()
				if err := conn.WriteJSON(map[string]any{"op": "advertiseServices", "services": []any{
					map[string]any{"id": 7, "name": "/map_server_node/get_mowing_area", "request": map[string]any{"schema": "uint32 index"}, "response": map[string]any{"schema": mapAreaResponseSchema}},
					map[string]any{"id": 8, "name": "/ready", "request": map[string]any{"schema": ""}, "response": map[string]any{"schema": mapAreaResponseSchema}},
				}}); err != nil {
					t.Error(err)
					return
				}
				poll := 0
				for {
					kind, request, err := conn.ReadMessage()
					if err != nil {
						return
					}
					if kind != websocket.BinaryMessage || len(request) < 16 || request[0] != 2 {
						continue
					}
					body := end
					if binary.LittleEndian.Uint32(request[1:5]) == 7 {
						// Request framing: opcode/service/call/encoding length, "cdr",
						// then the 4-byte CDR header and uint32 index.
						index := binary.LittleEndian.Uint32(request[20:24])
						if index == 0 {
							poll++
						}
						if index == 0 {
							body = front
						} else if index == 1 {
							body = back
						}
						if poll == 2 {
							switch {
							case scenario == "failure at zero" && index == 0, scenario == "failure in middle" && index == 1:
								_ = conn.WriteJSON(map[string]any{"op": "serviceCallFailure", "callId": binary.LittleEndian.Uint32(request[5:9]), "message": "refresh failed"})
								continue
							case scenario == "connection drop" && index == 1:
								return
							case scenario == "deadline" && index == 1:
								continue // No reply: exercise the actual shared poll deadline.
							case scenario == "enumeration guard":
								body = front
							case scenario == "shorter" && index == 1, scenario == "empty":
								body = end
							}
						}
					}
					frame := append([]byte(nil), request[:16]...)
					frame[0] = 3
					frame = append(frame, body...)
					if err := conn.WriteMessage(websocket.BinaryMessage, frame); err != nil {
						return
					}
				}
			}))
			defer server.Close()
			client := foxglove.NewClient("ws" + strings.TrimPrefix(server.URL, "http"))
			ctx, cancel := context.WithTimeout(context.Background(), 3*time.Second)
			defer cancel()
			if err := client.Connect(ctx); err != nil {
				t.Fatal(err)
			}
			defer client.Close()
			// A successful call proves that the read pump processed advertisement;
			// avoid assuming it completed after a fixed sleep.
			for {
				_, err := client.CallService(ctx, "/ready", struct{}{})
				if err == nil {
					break
				}
				if ctx.Err() != nil {
					t.Fatal(err)
				}
				time.Sleep(time.Millisecond)
			}
			messages := make(chan []byte, 4)
			sub := NewRosSubscriber("map", "test", 0, func(data []byte) { messages <- data })
			defer sub.Close()
			r := &RosProvider{client: client, lastMessage: map[string][]byte{}, subscribers: map[string]map[string]*RosSubscriber{"map": {"test": sub}}}
			r.pollMap()
			first := append([]byte(nil), r.lastMessage["map"]...)
			var initial mowgli.Map
			if err := json.Unmarshal(first, &initial); err != nil {
				t.Fatal(err)
			}
			if len(initial.WorkingArea) != 1 || len(initial.NavigationAreas) != 1 || initial.WorkingArea[0].Id != 101 || initial.NavigationAreas[0].Id != 102 || len(initial.WorkingArea[0].Area.Points) != 3 || len(initial.WorkingArea[0].Obstacles) != 1 || len(initial.WorkingArea[0].ProposedObstacles) != 1 {
				t.Fatalf("healthy control lost full geometry: %s", first)
			}
			select {
			case <-messages:
			case <-time.After(time.Second):
				t.Fatal("healthy map was not published")
			}
			r.pollMap()
			if scenario == "shorter" || scenario == "empty" {
				var updated mowgli.Map
				if err := json.Unmarshal(r.lastMessage["map"], &updated); err != nil {
					t.Fatal(err)
				}
				want := 1
				if scenario == "empty" {
					want = 0
				}
				if len(updated.WorkingArea) != want || len(updated.NavigationAreas) != 0 {
					t.Fatalf("successful replacement not committed: %s", r.lastMessage["map"])
				}
				select {
				case <-messages:
				case <-time.After(time.Second):
					t.Fatal("successful replacement was not published")
				}
			} else {
				if !bytes.Equal(first, r.lastMessage["map"]) {
					t.Fatal("incomplete or unchanged refresh replaced the complete cached snapshot")
				}
				select {
				case <-messages:
					t.Fatal("incomplete or unchanged refresh was published")
				case <-time.After(20 * time.Millisecond):
				}
			}
		})
	}
}
