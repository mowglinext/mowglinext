package api

import (
	"context"
	"encoding/base64"
	"encoding/json"
	"errors"
	"fmt"
	"log"
	"net/http"
	"net/url"
	"sync"
	"time"

	"github.com/mowglinext/mowglinext/pkg/msgs/geometry"
	"github.com/mowglinext/mowglinext/pkg/msgs/mowgli"
	"github.com/mowglinext/mowglinext/pkg/types"
	"github.com/docker/distribution/uuid"
	"github.com/gin-gonic/gin"
	"github.com/gorilla/websocket"
	"github.com/vmihailenco/msgpack/v5"
)

// wsWriteTimeout bounds a single WebSocket write. A frozen/slow client must not
// block a delivery goroutine forever; on timeout the connection is closed.
const wsWriteTimeout = 5 * time.Second

var upgrader = websocket.Upgrader{
	ReadBufferSize: 1024,
	// Larger write buffer so big frames (OccupancyGrid, /scan) aren't chopped
	// into many tiny TCP writes.
	WriteBufferSize: 32 * 1024,
	CheckOrigin: func(r *http.Request) bool {
		origin := r.Header.Get("Origin")
		if origin == "" {
			return true // non-browser clients
		}
		// Compare the parsed Origin HOST to the request Host exactly.
		// strings.Contains was exploitable: a page served from e.g.
		// "http://mower.local.evil.com" contains the host substring
		// "mower.local" and would pass, enabling cross-site WebSocket
		// hijacking against an API that has no auth layer.
		u, err := url.Parse(origin)
		if err != nil {
			return false
		}
		return u.Host == r.Host
	},
}

func MowgliNextRoutes(r *gin.RouterGroup, provider types.IRosProvider) {
	group := r.Group("/mowglinext")
	ServiceRoute(group, provider)
	AddMapAreaRoute(group, provider)
	SetDockingPointRoute(group, provider)
	ClearMapRoute(group, provider)
	ReplaceMapRoute(group, provider)
	SubscriberRoute(group, provider)
	MultiplexRoute(group, provider)
	PublisherRoute(group, provider)
}

// topicSubscribeInterval returns the throttle interval (ms, -1 = unthrottled)
// for a known logical topic. Mirrors the per-topic intervals used by
// SubscriberRoute. The bool flag is false for unknown topics so the
// multiplex path can ignore subscribe ops for them instead of leaking
// goroutines on bad input.
func topicSubscribeInterval(topic string) (int, bool) {
	switch topic {
	case "gps", "gnssStatus", "pose", "imu", "ticks", "wheelOdom", "lidar":
		return 100, true
	case "fusionRaw", "cogHeading", "magYaw", "obstacles", "icpOdom":
		return 200, true
	case "mowProgress":
		return 500, true // large OccupancyGrid — throttle hard
	case "diagnostics", "status", "highLevelStatus", "btLog", "map",
		"path", "plan", "power", "emergency", "dockingSensor",
		"robotDescription", "recordingTrajectory",
		"coverageResumeAvailable",
		"fusionDiag":
		return -1, true
	default:
		return -1, false
	}
}

// AddMapAreaRoute add a map area
//
// @Summary add a map area
// @Description add a map area
// @Tags mowglinext
// @Accept  json
// @Produce  json
// @Param CallReq body mowgli.AddMowingAreaReq true "request body"
// @Success 200 {object} OkResponse
// @Failure 500 {object} ErrorResponse
// @Router /mowglinext/map/area/add [post]
func AddMapAreaRoute(group *gin.RouterGroup, provider types.IRosProvider) {
	group.POST("/map/area/add", func(c *gin.Context) {
		ctx, cancel := context.WithTimeout(c.Request.Context(), 10*time.Second)
		defer cancel()

		var CallReq mowgli.AddMowingAreaReq
		err := unmarshalROSMessage[*mowgli.AddMowingAreaReq](c.Request.Body, &CallReq)
		if err != nil {
			// Return 400 (was a bare return → silent HTTP 200 with empty body,
			// so the GUI believed the area was added when it was dropped).
			c.JSON(400, ErrorResponse{Error: err.Error()})
			return
		}
		if CallReq.Area.Obstacles == nil {
			CallReq.Area.Obstacles = []geometry.Polygon{}
		}
		err = provider.CallService(ctx, "/map_server_node/add_area", &CallReq, &mowgli.AddMowingAreaRes{}, "mowgli_interfaces/srv/AddMowingArea")
		if err != nil {
			c.JSON(500, ErrorResponse{Error: err.Error()})
		} else {
			c.JSON(200, OkResponse{})
		}
	})
}

// ClearMapRoute delete a map area
//
// @Summary clear the map
// @Description clear the map
// @Tags mowglinext
// @Accept  json
// @Produce  json
// @Success 200 {object} OkResponse
// @Failure 500 {object} ErrorResponse
// @Router /mowglinext/map [delete]
func ClearMapRoute(group *gin.RouterGroup, provider types.IRosProvider) {
	group.DELETE("/map", func(c *gin.Context) {
		ctx, cancel := context.WithTimeout(c.Request.Context(), 10*time.Second)
		defer cancel()

		err := provider.CallService(ctx, "/map_server_node/clear_map", &mowgli.ClearMapReq{}, &mowgli.ClearMapRes{}, "std_srvs/srv/Trigger")
		if err != nil {
			c.JSON(500, ErrorResponse{Error: err.Error()})
		} else {
			c.JSON(200, OkResponse{})
		}
	})
}

// mapWriteBudget returns the context timeout for a clear_map → add_area×N →
// save_areas sequence. clear_map + save_areas are fixed-cost, but each add_area
// RASTERISES its polygon into the map_server grid, which on a large area takes
// seconds — and on a slow SBC (RPi4) a multi-area map easily blows a fixed
// budget, leaving the map half-written (issue #341: "can't save a big map,
// ~30 s timeout"). Scale it: a 60 s base plus per-area headroom, capped at
// 6 min. Map writes are rare and operator-driven, so a generous ceiling beats a
// false timeout. Shared by ReplaceMapRoute (map editor "Save Map") and the
// OpenMower importer so the two never drift.
func mapWriteBudget(nAreas int) time.Duration {
	budget := 60*time.Second + time.Duration(nAreas)*5*time.Second
	if budget > 6*time.Minute {
		budget = 6 * time.Minute
	}
	return budget
}

// replaceMapInternal is the ROS-side flow shared by the public PUT
// handler and the OpenMower importer. It does clear_map → add_area×N →
// save_areas; the wrapping (HTTP body decode / response codes) is the
// caller's job.
//
// The save_areas error is annotated so callers can distinguish a
// partial-success (areas live but not persisted to disk) from a hard
// failure earlier in the sequence.
func replaceMapInternal(ctx context.Context, provider types.IRosProvider, req *mowgli.ReplaceMapReq) error {
	if req == nil {
		return errors.New("replaceMapInternal: nil request")
	}
	if err := provider.CallService(ctx, "/map_server_node/clear_map", &mowgli.ClearMapReq{}, &mowgli.ClearMapRes{}, "std_srvs/srv/Trigger"); err != nil {
		return err
	}
	for _, element := range req.Areas {
		// Ensure Obstacles is an empty slice, not nil — the bridge rejects
		// null for repeated fields ("msg is not a list type").
		if element.Area.Obstacles == nil {
			element.Area.Obstacles = []geometry.Polygon{}
		}
		areaReq := mowgli.AddMowingAreaReq{
			Area:             element.Area,
			IsNavigationArea: element.IsNavigationArea,
		}
		if err := provider.CallService(ctx, "/map_server_node/add_area", &areaReq, &mowgli.AddMowingAreaRes{}, "mowgli_interfaces/srv/AddMowingArea"); err != nil {
			return err
		}
	}
	if err := provider.CallService(ctx, "/map_server_node/save_areas", &mowgli.ClearMapReq{}, &mowgli.ClearMapRes{}, "std_srvs/srv/Trigger"); err != nil {
		return fmt.Errorf("areas added but save_areas failed: %w", err)
	}
	return nil
}

// ReplaceMapRoute clear the map and insert areas
//
// @Summary Delete the current map and replace all areas
// @Description clear the map and insert all provided areas in a single transaction
// @Tags mowglinext
// @Accept  json
// @Produce  json
// @Param CallReq body mowgli.ReplaceMapReq true "replace map request body"
// @Success 200 {object} OkResponse
// @Failure 500 {object} ErrorResponse
// @Router /mowglinext/map [put]
func ReplaceMapRoute(group *gin.RouterGroup, provider types.IRosProvider) {
	group.PUT("/map", func(c *gin.Context) {
		// Decode BEFORE choosing the timeout so the budget can scale with the
		// area count (each add_area rasterises — a fixed 30 s timed out saving a
		// big edited map on RPi4, issue #341).
		var CallReq mowgli.ReplaceMapReq
		if err := unmarshalROSMessage[*mowgli.ReplaceMapReq](c.Request.Body, &CallReq); err != nil {
			c.JSON(500, ErrorResponse{Error: err.Error()})
			return
		}
		ctx, cancel := context.WithTimeout(c.Request.Context(), mapWriteBudget(len(CallReq.Areas)))
		defer cancel()

		if err := replaceMapInternal(ctx, provider, &CallReq); err != nil {
			c.JSON(500, ErrorResponse{Error: err.Error()})
			return
		}
		c.JSON(200, OkResponse{})
	})
}

// setDockingPointInternal is the ROS-side call shared by the public
// POST handler and the OpenMower importer. Single service round-trip,
// no wrapping logic.
func setDockingPointInternal(ctx context.Context, provider types.IRosProvider, req *mowgli.SetDockingPointReq) error {
	if req == nil {
		return errors.New("setDockingPointInternal: nil request")
	}
	return provider.CallService(ctx, "/map_server_node/set_docking_point", req, &mowgli.SetDockingPointRes{}, "mowgli_interfaces/srv/SetDockingPoint")
}

// SetDockingPointRoute set the docking point
//
// @Summary set the docking point
// @Description set the docking point
// @Tags mowglinext
// @Accept  json
// @Produce  json
// @Param CallReq body mowgli.SetDockingPointReq true "request body"
// @Success 200 {object} OkResponse
// @Failure 500 {object} ErrorResponse
// @Router /mowglinext/map/docking [post]
func SetDockingPointRoute(group *gin.RouterGroup, provider types.IRosProvider) {
	group.POST("/map/docking", func(c *gin.Context) {
		ctx, cancel := context.WithTimeout(c.Request.Context(), 10*time.Second)
		defer cancel()

		var CallReq mowgli.SetDockingPointReq
		if err := unmarshalROSMessage[*mowgli.SetDockingPointReq](c.Request.Body, &CallReq); err != nil {
			c.JSON(500, ErrorResponse{Error: err.Error()})
			return
		}
		if err := setDockingPointInternal(ctx, provider, &CallReq); err != nil {
			c.JSON(500, ErrorResponse{Error: err.Error()})
			return
		}
		c.JSON(200, OkResponse{})
	})
}

// SubscriberRoute subscribe to a topic
//
// @Summary subscribe to a topic
// @Description subscribe to a topic
// @Tags mowglinext
// @Param topic path string true "logical topic key: diagnostics, status, highLevelStatus, gps, gnssStatus, pose, imu, ticks, map, path, plan, mowingPath, power, emergency, dockingSensor, lidar"
// @Router /mowglinext/subscribe/{topic} [get]
func SubscriberRoute(group *gin.RouterGroup, provider types.IRosProvider) {
	group.GET("/subscribe/:topic", func(c *gin.Context) {
		topic := c.Param("topic")
		conn, err := upgrader.Upgrade(c.Writer, c.Request, nil)
		if err != nil {
			return
		}
		defer conn.Close()

		// Single-sourced from topicSubscribeInterval so this dedicated-connection
		// path and the MultiplexRoute path can never drift on a topic's throttle
		// interval or its set of known topics (see TestTopicSubscribeInterval_*).
		interval, known := topicSubscribeInterval(topic)
		if !known {
			log.Printf("SubscriberRoute: unknown topic %q", topic)
			return
		}
		def, err := subscribe(provider, c, conn, topic, interval)
		if err != nil {
			log.Println(err.Error())
			return
		}
		defer def()

		_, _, err = conn.ReadMessage()
		if err != nil {
			c.Error(err)
			return
		}
	})
}

// PublisherRoute publish to a topic
//
// @Summary publish to a topic
// @Description publish to a topic
// @Tags mowglinext
// @Param topic path string true "topic to publish to, could be: joy"
// @Router /mowglinext/publish/{topic} [get]
func PublisherRoute(group *gin.RouterGroup, provider types.IRosProvider) {
	group.GET("/publish/:topic", func(c *gin.Context) {
		var err error
		conn, err := upgrader.Upgrade(c.Writer, c.Request, nil)
		if err != nil {
			return
		}
		defer conn.Close()
		for {
			_, msg, err := conn.ReadMessage()
			if err != nil {
				c.Error(err)
				break
			}
			var msgObj geometry.TwistStamped
			err = json.Unmarshal(msg, &msgObj)
			if err != nil {
				log.Printf("PublisherRoute: unmarshal error: %v", err)
				continue
			}
			err = provider.Publish("/cmd_vel_teleop", "geometry_msgs/msg/TwistStamped", &msgObj)
			if err != nil {
				log.Printf("PublisherRoute: publish error: %v", err)
				// Don't break — foxglove may reconnect; keep the browser WebSocket alive
				continue
			}
		}
	})
}

// MultiplexRoute multiplexes any number of topic subscriptions over one
// WebSocket so a single browser tab does not need ~25 simultaneous TCP
// connections. Wire format:
//
//	client → server: {"op": "subscribe"|"unsubscribe", "topic": "<key>"}
//	server → client: {"topic": "<key>", "data": "<base64>"}
//
// Per-topic throttling reuses topicSubscribeInterval. Unknown topics are
// ignored. On disconnect, all live subscriptions are released.
//
// @Summary multiplexed topic subscription
// @Description multiplexed topic subscription
// @Tags mowglinext
// @Router /mowglinext/multiplex [get]
func MultiplexRoute(group *gin.RouterGroup, provider types.IRosProvider) {
	group.GET("/multiplex", func(c *gin.Context) {
		conn, err := upgrader.Upgrade(c.Writer, c.Request, nil)
		if err != nil {
			return
		}
		defer conn.Close()

		type subState struct {
			id string
		}
		var stateMu sync.Mutex
		state := map[string]*subState{}

		var writeMu sync.Mutex
		writeFrame := func(topic string, data []byte) {
			// Re-encode the frame as MessagePack and send it as a BINARY frame.
			// `data` is the per-message snake_case JSON produced upstream; we
			// decode it to a generic value and msgpack-encode {topic, data:obj}
			// so the browser does ONE fast msgpack decode instead of
			// JSON.parse(envelope) → atob → JSON.parse(payload) on the main
			// thread. Field names (snake_case) are preserved, so the frontend
			// TS interfaces are unchanged. The JSON→obj cost moves to Go (fast,
			// off the browser's single thread).
			var obj interface{}
			if err := json.Unmarshal(data, &obj); err != nil {
				return
			}
			payload, err := msgpack.Marshal(map[string]interface{}{
				"topic": topic,
				"data":  obj,
			})
			if err != nil {
				return
			}
			writeMu.Lock()
			defer writeMu.Unlock()
			// Bound every write: a frozen browser tab must NOT block this
			// goroutine indefinitely, because all topics share one conn + one
			// writeMu — one stuck write would otherwise freeze every
			// subscription on this tab (the "stale components" symptom). On
			// timeout/error, close the conn so the read loop unblocks and the
			// deferred cleanup releases all subscriptions.
			_ = conn.SetWriteDeadline(time.Now().Add(wsWriteTimeout))
			if err := conn.WriteMessage(websocket.BinaryMessage, payload); err != nil {
				_ = conn.Close()
			}
		}

		subscribeTopic := func(topic string) {
			interval, known := topicSubscribeInterval(topic)
			if !known {
				log.Printf("MultiplexRoute: ignoring unknown topic %q", topic)
				return
			}
			stateMu.Lock()
			if _, exists := state[topic]; exists {
				stateMu.Unlock()
				return
			}
			id := uuid.Generate().String()
			state[topic] = &subState{id: id}
			stateMu.Unlock()

			// Throttling is enforced inside the RosSubscriber (coalescing,
			// non-blocking) — NOT with a time.Sleep here, which used to block
			// the per-topic delivery goroutine.
			err := provider.Subscribe(topic, id, interval, func(msg []byte) {
				writeFrame(topic, msg)
			})
			if err != nil {
				log.Printf("MultiplexRoute: subscribe %s: %v", topic, err)
				stateMu.Lock()
				delete(state, topic)
				stateMu.Unlock()
			}
		}

		unsubscribeTopic := func(topic string) {
			stateMu.Lock()
			s, ok := state[topic]
			delete(state, topic)
			stateMu.Unlock()
			if ok {
				provider.UnSubscribe(topic, s.id)
			}
		}

		// Drain all subscriptions when the connection closes.
		defer func() {
			stateMu.Lock()
			snapshot := make([]struct {
				topic string
				id    string
			}, 0, len(state))
			for topic, s := range state {
				snapshot = append(snapshot, struct {
					topic string
					id    string
				}{topic, s.id})
			}
			state = map[string]*subState{}
			stateMu.Unlock()
			for _, s := range snapshot {
				provider.UnSubscribe(s.topic, s.id)
			}
		}()

		type clientMsg struct {
			Op    string `json:"op"`
			Topic string `json:"topic"`
		}
		for {
			_, payload, err := conn.ReadMessage()
			if err != nil {
				return
			}
			var m clientMsg
			if err := json.Unmarshal(payload, &m); err != nil {
				continue
			}
			switch m.Op {
			case "subscribe":
				subscribeTopic(m.Topic)
			case "unsubscribe":
				unsubscribeTopic(m.Topic)
			}
		}
	})
}

func subscribe(provider types.IRosProvider, c *gin.Context, conn *websocket.Conn, topic string, interval int) (func(), error) {
	id := uuid.Generate()
	uidString := id.String()
	var writeMu sync.Mutex
	// Throttle is enforced inside the RosSubscriber (coalescing, non-blocking).
	err := provider.Subscribe(topic, uidString, interval, func(msg []byte) {
		writeMu.Lock()
		defer writeMu.Unlock()
		// Bound the write so a slow client can't wedge the delivery goroutine.
		_ = conn.SetWriteDeadline(time.Now().Add(wsWriteTimeout))
		writer, err := conn.NextWriter(websocket.TextMessage)
		if err != nil {
			c.Error(err)
			_ = conn.Close()
			return
		}
		_, err = writer.Write([]byte(base64.StdEncoding.EncodeToString(msg)))
		if err != nil {
			c.Error(err)
			_ = conn.Close()
			return
		}
		err = writer.Close()
		if err != nil {
			c.Error(err)
			_ = conn.Close()
			return
		}
	},
	)
	if err != nil {
		return nil, err
	}
	return func() {
		provider.UnSubscribe(topic, uidString)
	}, nil
}

// ServiceRoute call a service
//
// @Summary call a service
// @Description call a service
// @Tags mowglinext
// @Accept  json
// @Produce  json
// @Param command path string true "command to call, could be: high_level_control, emergency, mow_enabled, start_in_area"
// @Param CallReq body map[string]interface{} true "request body"
// @Success 200 {object} OkResponse
// @Failure 500 {object} ErrorResponse
// @Router /mowglinext/call/{command} [post]
func ServiceRoute(group *gin.RouterGroup, provider types.IRosProvider) {
	group.POST("/call/:command", func(c *gin.Context) {
		command := c.Param("command")
		// Bound every ROS service call: foxglove's CallService waits on
		// ctx.Done(), and ctx only cancels when the browser
		// drops the HTTP connection — a hung ROS node (behavior_tree /
		// hardware_bridge / fusion_graph down) would otherwise pin this
		// handler goroutine and a pendingSvc slot indefinitely. Every other
		// route in this file already wraps with WithTimeout; this one was the
		// exception.
		ctx, cancel := context.WithTimeout(c.Request.Context(), 10*time.Second)
		defer cancel()
		var err error
		switch command {
		case "high_level_control":
			var CallReq mowgli.HighLevelControlReq
			err = c.BindJSON(&CallReq)
			if err != nil {
				// Explicit JSON body: gin's BindJSON aborts with a bare 400, and
				// the frontend's useMowerAction reads res.error from JSON.
				c.JSON(400, ErrorResponse{Error: err.Error()})
				return
			}
			err = provider.CallService(ctx, "/behavior_tree_node/high_level_control", &CallReq, &mowgli.HighLevelControlRes{}, "mowgli_interfaces/srv/HighLevelControl")
		case "emergency":
			var CallReq mowgli.EmergencyStopReq
			err = c.BindJSON(&CallReq)
			if err != nil {
				// Explicit JSON body: gin's BindJSON aborts with a bare 400, and
				// the frontend's useMowerAction reads res.error from JSON.
				c.JSON(400, ErrorResponse{Error: err.Error()})
				return
			}
			err = provider.CallService(ctx, "/hardware_bridge/emergency_stop", &CallReq, &mowgli.EmergencyStopRes{}, "mowgli_interfaces/srv/EmergencyStop")
		case "mow_enabled":
			var CallReq mowgli.MowerControlReq
			err = c.BindJSON(&CallReq)
			if err != nil {
				// Explicit JSON body: gin's BindJSON aborts with a bare 400, and
				// the frontend's useMowerAction reads res.error from JSON.
				c.JSON(400, ErrorResponse{Error: err.Error()})
				return
			}
			err = provider.CallService(ctx, "/hardware_bridge/mower_control", &CallReq, &mowgli.MowerControlRes{}, "mowgli_interfaces/srv/MowerControl")
		case "start_in_area":
			var CallReq mowgli.StartInAreaReq
			err = c.BindJSON(&CallReq)
			if err != nil {
				// Explicit JSON body: gin's BindJSON aborts with a bare 400, and
				// the frontend's useMowerAction reads res.error from JSON.
				c.JSON(400, ErrorResponse{Error: err.Error()})
				return
			}
			err = provider.CallService(ctx, "/behavior_tree_node/start_in_area", &CallReq, &mowgli.StartInAreaRes{}, "mowgli_interfaces/srv/StartInArea")
		case "set_datum":
			type TriggerRes struct {
				Success bool   `json:"success"`
				Message string `json:"message"`
			}
			var res TriggerRes
			err = provider.CallService(ctx, "/navsat_to_absolute_pose/set_datum", &struct{}{}, &res, "std_srvs/srv/Trigger")
			if err == nil && !res.Success {
				err = errors.New(res.Message)
			}
			if err == nil {
				c.JSON(200, map[string]interface{}{"message": res.Message})
				return
			}
		case "promote_obstacle":
			// Convert a transient /obstacle_tracker/obstacles observation
			// (or a free-form polygon) into a persistent keepout for one
			// of the mowing areas. After the obstacle-tracker decouple
			// (#6), this is the only path that mutates obstacle_polygons_;
			// auto-promotion is gone.
			var CallReq mowgli.PromoteObstacleReq
			err = c.BindJSON(&CallReq)
			if err != nil {
				// Explicit JSON body: gin's BindJSON aborts with a bare 400, and
				// the frontend's useMowerAction reads res.error from JSON.
				c.JSON(400, ErrorResponse{Error: err.Error()})
				return
			}
			var promoteRes mowgli.PromoteObstacleRes
			err = provider.CallService(ctx,
				"/map_server_node/promote_obstacle",
				&CallReq,
				&promoteRes,
				"mowgli_interfaces/srv/PromoteObstacle")
			if err == nil && !promoteRes.Success {
				err = errors.New(promoteRes.Message)
			}
			if err == nil {
				c.JSON(200, map[string]interface{}{"message": promoteRes.Message})
				return
			}
		case "fusion_graph_save", "fusion_graph_clear":
			// Both target std_srvs/Trigger services on fusion_graph_node.
			type TriggerRes struct {
				Success bool   `json:"success"`
				Message string `json:"message"`
			}
			service := "/fusion_graph_node/save_graph"
			if command == "fusion_graph_clear" {
				service = "/fusion_graph_node/clear_graph"
			}
			var res TriggerRes
			err = provider.CallService(ctx, service, &struct{}{}, &res, "std_srvs/srv/Trigger")
			if err == nil && !res.Success {
				err = errors.New(res.Message)
			}
			if err == nil {
				c.JSON(200, map[string]interface{}{"message": res.Message})
				return
			}
		case "coverage_clear_resume":
			// "Start fresh": discard persisted mowing progress so the next
			// COMMAND_START begins at the first line instead of resuming mid-path
			// (the "starts at 2nd/3rd line" report). The frontend calls this before
			// sending Command:1 when coverageResumeAvailable is true.
			type TriggerRes struct {
				Success bool   `json:"success"`
				Message string `json:"message"`
			}
			var res TriggerRes
			err = provider.CallService(ctx, "/behavior_tree_node/clear_coverage_resume", &struct{}{}, &res, "std_srvs/srv/Trigger")
			if err == nil && !res.Success {
				err = errors.New(res.Message)
			}
			if err == nil {
				c.JSON(200, map[string]interface{}{"message": res.Message})
				return
			}
		case "reboot_board":
			// Reboot the STM32 board (NVIC_SystemReset) — recovers a wedged
			// firmware state (e.g. the IMU emitting NaN) without a power-cycle.
			type TriggerRes struct {
				Success bool   `json:"success"`
				Message string `json:"message"`
			}
			var res TriggerRes
			err = provider.CallService(ctx, "/hardware_bridge/reboot_board", &struct{}{}, &res, "std_srvs/srv/Trigger")
			if err == nil && !res.Success {
				err = errors.New(res.Message)
			}
			if err == nil {
				c.JSON(200, map[string]interface{}{"message": res.Message})
				return
			}
		default:
			err = errors.New("unknown command")
		}
		if err != nil {
			c.JSON(500, ErrorResponse{Error: err.Error()})
		} else {
			c.JSON(200, OkResponse{})
		}
	})
}
