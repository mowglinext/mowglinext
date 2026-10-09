package api

import (
	"bytes"
	"context"
	"encoding/base64"
	"encoding/json"
	"errors"
	"fmt"
	"log"
	"math"
	"net/http"
	"net/url"
	"sync"
	"time"

	"github.com/docker/distribution/uuid"
	"github.com/gin-gonic/gin"
	"github.com/gorilla/websocket"
	"github.com/mowglinext/mowglinext/pkg/msgs/geometry"
	"github.com/mowglinext/mowglinext/pkg/msgs/mowgli"
	"github.com/mowglinext/mowglinext/pkg/types"
	"github.com/vmihailenco/msgpack/v5"
)

// wsWriteTimeout bounds a single WebSocket write. A frozen/slow client must not
// block a delivery goroutine forever; on timeout the connection is closed.
const wsWriteTimeout = 5 * time.Second

// Frames at least this big are sent with permessage-deflate when the browser offers it.
// The big ones are occupancy grids (mow progress, LiDAR map) that are almost all one
// value, so they shrink by two orders of magnitude; small, frequent frames (pose, status)
// are not worth the CPU. 4 KiB is well below any grid and above every status message.
const wsCompressMinBytes = 4 * 1024

// wsCompressionLevel favours speed: the mower's CPU is small and the win comes from
// the redundancy of the data, not from squeezing the last byte.
const wsCompressionLevel = 1

func compactCoveragePreview(obj interface{}) {
	message, ok := obj.(map[string]interface{})
	if !ok {
		return
	}
	coordinates, ok := message["xy"].([]interface{})
	if !ok {
		return
	}
	compact := make([]float32, len(coordinates))
	for i, coordinate := range coordinates {
		value, ok := coordinate.(float64)
		if !ok {
			return
		}
		compact[i] = float32(value)
	}
	message["xy"] = compact
}

// multiplexUpgrader is the upgrader the multiplex route uses. Identical to `upgrader`
// except it negotiates permessage-deflate (RFC 7692) when the browser offers it, which
// every current browser does; a client that does not simply gets uncompressed frames.
// Only this route is opted in: it carries the large grids.
var multiplexUpgrader = func() websocket.Upgrader {
	u := upgrader
	u.EnableCompression = true
	return u
}()

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

// fusionGraphTriggerServices maps the /mowglinext/call/:command names for
// fusion_graph_node's std_srvs/Trigger services to the ROS service names.
var fusionGraphTriggerServices = map[string]string{
	"fusion_graph_save":            "/fusion_graph_node/save_graph",
	"fusion_graph_clear":           "/fusion_graph_node/clear_graph",
	"fusion_graph_clear_lidar_map": "/fusion_graph_node/clear_lidar_map",
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
	case "fusionRaw", "cogHeading", "magYaw", "obstacles":
		return 200, true
	case "mowProgress", "lidarMap":
		return 500, true // large OccupancyGrid — throttle hard
	case "diagnostics", "status", "highLevelStatus", "btLog", "map",
		"path", "plan", "power", "emergency", "dockingSensor",
		"robotDescription", "recordingTrajectory",
		"coverageResumeAvailable", "coverageSession",
		"fusionDiag", "dockCalibrationStatus", "firmwareParams":
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
		// area count (issue #341). The flow itself lives in map_replace.go.
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
// POST handler and the OpenMower importer. Single service round-trip.
//
// CallService only fails on transport/serialization errors. map_server
// answers a GATED update (not on the dock, GPS not accurate enough, yaw not
// converged…) with {success:false, message}: the dock pose was NOT
// committed, so that is an error too, carrying map_server's reason (#703).
func setDockingPointInternal(ctx context.Context, provider types.IRosProvider, req *mowgli.SetDockingPointReq) error {
	if req == nil {
		return errors.New("setDockingPointInternal: nil request")
	}
	var res mowgli.SetDockingPointRes
	if err := provider.CallService(ctx, "/map_server_node/set_docking_point", req, &res, "mowgli_interfaces/srv/SetDockingPoint"); err != nil {
		return err
	}
	if !res.Success {
		return fmt.Errorf("dock pose rejected by map_server: %s", res.Message)
	}
	return nil
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

// compactMultiplexNumbers preserves JavaScript Number semantics: the browser's
// MessagePack decoder returns BigInt for int64/uint64, so leave larger numbers
// and signed zero as float64. json.Unmarshal owns these maps and slices.
func compactMultiplexNumbers(value any) any {
	switch v := value.(type) {
	case float64:
		if v == 0 && math.Signbit(v) {
			return v
		}
		if v >= math.MinInt32 && v <= math.MaxUint32 && math.Trunc(v) == v {
			if v < 0 {
				return int32(v)
			}
			return uint32(v)
		}
	case []any:
		for i := range v {
			v[i] = compactMultiplexNumbers(v[i])
		}
	case map[string]any:
		for key, item := range v {
			v[key] = compactMultiplexNumbers(item)
		}
	}
	return value
}

// MultiplexRoute multiplexes any number of topic subscriptions over one
// WebSocket so a single browser tab does not need ~25 simultaneous TCP
// connections. Wire format:
//
//	client → server: JSON {"op": "subscribe"|"unsubscribe"|"resync", "topic": "<key>", "delta": <bool>}
//	server → client: MessagePack {"topic": "<key>", "data": <decoded object>[, "seq": n]}
//	                 or, for a delta subscription, {"topic": "<key>", "patch": {...}} (grid_delta.go)
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
		conn, err := multiplexUpgrader.Upgrade(c.Writer, c.Request, nil)
		if err != nil {
			return
		}
		defer conn.Close()
		// A no-op unless the browser negotiated permessage-deflate.
		_ = conn.SetCompressionLevel(wsCompressionLevel)

		type subState struct {
			id string
			// delta is set for the occupancy-grid topics the browser asked to receive as
			// patches (see grid_delta.go); nil for every other subscription.
			delta *gridDelta
		}
		var stateMu sync.Mutex
		state := map[string]*subState{}

		var writeMu sync.Mutex
		// seq is set (one value) only on the full frames of a delta subscription.
		writeFrame := func(topic string, data []byte, seq ...uint64) {
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
			// JSON numbers arrive as float64, including each OccupancyGrid
			// cell. Only compact numbers that the browser decodes as Number:
			// MessagePack int64/uint64 decode as BigInt in msgpackr.
			// Keep the compact plan's coordinate array as MessagePack float32
			// (generic JSON decoding otherwise widens every number to float64).
			if topic == "path" {
				compactCoveragePreview(obj)
			}
			var payload bytes.Buffer
			encoder := msgpack.NewEncoder(&payload)
			encoder.UseCompactInts(true)
			frame := map[string]interface{}{
				"topic": topic,
				"data":  compactMultiplexNumbers(obj),
			}
			if len(seq) > 0 {
				frame["seq"] = seq[0]
			}
			err := encoder.Encode(frame)
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
			conn.EnableWriteCompression(payload.Len() >= wsCompressMinBytes)
			if err := conn.WriteMessage(websocket.BinaryMessage, payload.Bytes()); err != nil {
				_ = conn.Close()
			}
		}

		writePatch := func(topic string, f gridFrame) {
			var header interface{}
			if len(f.Header) > 0 {
				_ = json.Unmarshal(f.Header, &header)
			}
			var payload bytes.Buffer
			encoder := msgpack.NewEncoder(&payload)
			encoder.UseCompactInts(true)
			if err := encoder.Encode(map[string]interface{}{
				"topic": topic,
				"patch": map[string]interface{}{
					"base":   f.Base,
					"seq":    f.Seq,
					"header": compactMultiplexNumbers(header),
					"gaps":   f.Gaps,
					"vals":   f.Vals,
				},
			}); err != nil {
				return
			}
			writeMu.Lock()
			defer writeMu.Unlock()
			_ = conn.SetWriteDeadline(time.Now().Add(wsWriteTimeout))
			if err := conn.WriteMessage(websocket.BinaryMessage, payload.Bytes()); err != nil {
				_ = conn.Close()
			}
		}

		// sendGrid sends one grid message of a delta subscription as a patch or in full.
		sendGrid := func(topic string, d *gridDelta, msg []byte) {
			f, ok := d.next(msg)
			switch {
			case !ok:
				writeFrame(topic, msg)
			case f.Full:
				writeFrame(topic, f.Raw, f.Seq)
			default:
				writePatch(topic, f)
			}
		}

		subscribeTopic := func(topic string, delta bool) {
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
			sub := &subState{id: id}
			if delta && gridDeltaTopics[topic] {
				sub.delta = &gridDelta{}
			}
			state[topic] = sub
			stateMu.Unlock()

			// Throttling is enforced inside the RosSubscriber (coalescing,
			// non-blocking) — NOT with a time.Sleep here, which used to block
			// the per-topic delivery goroutine.
			err := provider.Subscribe(topic, id, interval, func(msg []byte) {
				if sub.delta != nil {
					sendGrid(topic, sub.delta, msg)
					return
				}
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
			// Delta asks for the occupancy-grid topics as patches (grid_delta.go).
			Delta bool `json:"delta"`
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
				subscribeTopic(m.Topic, m.Delta)
			case "unsubscribe":
				unsubscribeTopic(m.Topic)
			case "resync":
				// The browser saw a patch it could not apply: send the grid again in full.
				stateMu.Lock()
				sub := state[m.Topic]
				stateMu.Unlock()
				if sub != nil && sub.delta != nil {
					if f, ok := sub.delta.resync(); ok {
						writeFrame(m.Topic, f.Raw, f.Seq)
					}
				}
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
			var res mowgli.HighLevelControlRes
			err = provider.CallService(ctx, "/behavior_tree_node/high_level_control", &CallReq, &res, "mowgli_interfaces/srv/HighLevelControl")
			if err == nil && !res.Success {
				err = errors.New("high_level_control rejected the command")
			}
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
		case "blade_control":
			handleBladeControl(c, provider)
			return
		case "coverage_orientation":
			var req mowgli.CoverageOrientationReq
			if err = c.BindJSON(&req); err != nil {
				c.JSON(400, ErrorResponse{Error: err.Error()})
				return
			}
			var res mowgli.CoverageOrientationRes
			err = provider.CallService(ctx, "/behavior_tree_node/coverage_orientation", &req, &res, "mowgli_interfaces/srv/CoverageOrientation")
			if err == nil && !res.Success {
				err = errors.New(res.Message)
			}
			if err == nil {
				c.JSON(200, res)
				return
			}
		case "start_in_area":
			var CallReq mowgli.StartInAreaReq
			err = c.BindJSON(&CallReq)
			if err != nil {
				// Explicit JSON body: gin's BindJSON aborts with a bare 400, and
				// the frontend's useMowerAction reads res.error from JSON.
				c.JSON(400, ErrorResponse{Error: err.Error()})
				return
			}
			var res mowgli.StartInAreaRes
			err = provider.CallService(ctx, "/behavior_tree_node/start_in_area", &CallReq, &res, "mowgli_interfaces/srv/StartInArea")
			if err == nil && !res.Success {
				err = errors.New("start_in_area rejected the command")
			}
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
			// Convert a transient /obstacle_tracker/obstacles observation,
			// a free-form polygon, or a PENDING dig proposal (pending_id,
			// see MapObstacleInfo) into a persistent keepout for one of the
			// mowing areas. After the obstacle-tracker decouple (#6), this
			// is the only path that mutates obstacle_polygons_;
			// auto-promotion is gone — and since #502 a detected dig is a
			// proposal that lands here too, instead of writing itself into
			// areas.dat.
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
		case "set_area_coverage_lines":
			// Set or clear ONE mowing area's own swath angle and perimeter
			// winding (opt-in overrides of the robot-wide mow_angle_deg /
			// mow_direction). Addressed by the stable MapArea.id, never by
			// index: the map save rebuilds the whole list. It is a plain map edit
			// that only changes that area's NEXT plan; the Map page only offers
			// it while the robot is not mowing, because a resume re-plans the
			// area and its cursor would point into a different plan.
			var linesReq mowgli.SetAreaCoverageLinesReq
			if err = c.BindJSON(&linesReq); err != nil {
				c.JSON(400, ErrorResponse{Error: err.Error()})
				return
			}
			if linesReq.Id == 0 {
				c.JSON(400, ErrorResponse{Error: "id is required: a new, never-saved area has no id yet"})
				return
			}
			var linesRes mowgli.SetAreaCoverageLinesRes
			err = provider.CallService(ctx,
				"/map_server_node/set_area_coverage_lines",
				&linesReq,
				&linesRes,
				"mowgli_interfaces/srv/SetAreaCoverageLines")
			if err == nil && !linesRes.Success {
				err = errors.New(linesRes.Message)
			}
			if err == nil {
				c.JSON(200, map[string]interface{}{"message": linesRes.Message})
				return
			}
		case "preview_obstacle_clearance":
			// Read-only: buffers each obstacle polygon outward by the LIVE
			// obstacle_margin coverage_server is actually planning with
			// (bufferRingOutward, reused server-side — never reimplemented
			// here), for the Map page's toggleable clearance-preview overlay.
			var previewReq struct {
				Obstacles []geometry.Polygon `json:"obstacles"`
			}
			if err = c.BindJSON(&previewReq); err != nil {
				c.JSON(400, ErrorResponse{Error: err.Error()})
				return
			}
			if previewReq.Obstacles == nil {
				previewReq.Obstacles = []geometry.Polygon{}
			}
			var previewRes mowgli.PreviewObstacleClearanceRes
			err = provider.CallService(ctx,
				"/coverage_server/preview_obstacle_clearance",
				&mowgli.PreviewObstacleClearanceReq{Obstacles: previewReq.Obstacles},
				&previewRes,
				"mowgli_interfaces/srv/PreviewObstacleClearance")
			if err == nil {
				if previewRes.Buffered == nil {
					previewRes.Buffered = []geometry.Polygon{}
				}
				c.JSON(200, previewRes)
				return
			}
		case "preview_coverage":
			// Read-only dry run of coverage_server's planner for the Map page's
			// "mowing lines" overlay: the same planBoustrophedon call a real
			// PlanCoverage goal makes, but with the swath angle and perimeter
			// winding taken from the request so the operator can try a value
			// before saving it. Omitted angle/direction mean "auto" / "the live
			// ring_direction parameter" — NOT 0, which is a real choice (0 deg,
			// planner-default winding) — hence the pointers.
			var previewReq struct {
				OuterBoundary geometry.Polygon   `json:"outer_boundary"`
				Obstacles     []geometry.Polygon `json:"obstacles"`
				MowAngleDeg   *float64           `json:"mow_angle_deg"`
				Perpendicular bool               `json:"perpendicular"`
				RingDirection *int32             `json:"ring_direction"`
				// Where the route starts: snapped onto the OUTERMOST headland ring.
				// Omitted = the planner's own start.
				HasStartPoint bool    `json:"has_start_point"`
				StartX        float64 `json:"start_x"`
				StartY        float64 `json:"start_y"`
			}
			if err = c.BindJSON(&previewReq); err != nil {
				c.JSON(400, ErrorResponse{Error: err.Error()})
				return
			}
			if len(previewReq.OuterBoundary.Points) < 3 {
				c.JSON(400, ErrorResponse{Error: "outer_boundary needs at least 3 points"})
				return
			}
			if previewReq.Obstacles == nil {
				previewReq.Obstacles = []geometry.Polygon{}
			}
			previewCall := mowgli.PreviewCoverageReq{
				OuterBoundary: previewReq.OuterBoundary,
				Obstacles:     previewReq.Obstacles,
				MowAngleDeg:   -1,
				Perpendicular: previewReq.Perpendicular,
				RingDirection: -1,
				HasStartPoint: previewReq.HasStartPoint,
				StartX:        previewReq.StartX,
				StartY:        previewReq.StartY,
			}
			if previewReq.MowAngleDeg != nil {
				previewCall.MowAngleDeg = *previewReq.MowAngleDeg
			}
			if previewReq.RingDirection != nil {
				previewCall.RingDirection = *previewReq.RingDirection
			}
			var previewRes mowgli.PreviewCoverageRes
			err = provider.CallService(ctx,
				"/coverage_server/preview_coverage",
				&previewCall,
				&previewRes,
				"mowgli_interfaces/srv/PreviewCoverage")
			if err == nil {
				if previewRes.Rings == nil {
					previewRes.Rings = []geometry.Polygon{}
				}
				if previewRes.Swaths == nil {
					previewRes.Swaths = []geometry.Polygon{}
				}
				// A planner refusal (field too small, bad direction) is a normal
				// answer, not a transport error: the overlay shows its message.
				c.JSON(200, previewRes)
				return
			}
		case "correct_recorded_obstacle":
			// One-shot: shrinks a polygon recorded by driving the chassis edge
			// around an object by the raw chassis half-width (coverage_server's
			// erodeRingInward — never reimplemented here). The Map page calls it
			// once, when the operator converts a just-recorded area into an
			// obstacle; the BT's RecordArea only records mowing areas, so this
			// is the only point where that correction can happen.
			var correctReq struct {
				Polygon geometry.Polygon `json:"polygon"`
			}
			if err = c.BindJSON(&correctReq); err != nil {
				c.JSON(400, ErrorResponse{Error: err.Error()})
				return
			}
			if correctReq.Polygon.Points == nil {
				correctReq.Polygon.Points = []geometry.Point32{}
			}
			var correctRes mowgli.CorrectRecordedObstacleRes
			err = provider.CallService(ctx,
				"/coverage_server/correct_recorded_obstacle",
				&mowgli.CorrectRecordedObstacleReq{Polygon: correctReq.Polygon},
				&correctRes,
				"mowgli_interfaces/srv/CorrectRecordedObstacle")
			if err == nil {
				if correctRes.Corrected.Points == nil {
					correctRes.Corrected.Points = []geometry.Point32{}
				}
				c.JSON(200, correctRes)
				return
			}
		case "get_lidar_ignore_corridors":
			var res mowgli.GetLidarIgnoreCorridorsRes
			err = provider.CallService(ctx,
				"/map_server_node/get_lidar_ignore_corridors",
				&struct{}{},
				&res,
				"mowgli_interfaces/srv/GetLidarIgnoreCorridors")
			if err == nil {
				if res.Corridors == nil {
					res.Corridors = []mowgli.LidarIgnoreCorridor{}
				}
				c.JSON(200, res)
				return
			}
		case "set_lidar_ignore_corridors":
			// Replace the whole LiDAR-ignore corridor list (clear + add each),
			// the same rebuild shape the map save uses for areas. map_server
			// clamps width_m and persists into areas.dat on every add.
			var setReq struct {
				Corridors []mowgli.LidarIgnoreCorridor `json:"corridors"`
			}
			if err = c.BindJSON(&setReq); err != nil {
				c.JSON(400, ErrorResponse{Error: err.Error()})
				return
			}
			var clearRes mowgli.ClearLidarIgnoreCorridorsRes
			err = provider.CallService(ctx,
				"/map_server_node/clear_lidar_ignore_corridors",
				&mowgli.ClearLidarIgnoreCorridorsReq{},
				&clearRes,
				"mowgli_interfaces/srv/ClearLidarIgnoreCorridors")
			if err == nil && !clearRes.Success {
				err = errors.New("clear_lidar_ignore_corridors failed")
			}
			for i := 0; err == nil && i < len(setReq.Corridors); i++ {
				corridor := setReq.Corridors[i]
				if corridor.Polyline.Points == nil {
					corridor.Polyline.Points = []geometry.Point32{}
				}
				var addRes mowgli.AddLidarIgnoreCorridorRes
				err = provider.CallService(ctx,
					"/map_server_node/add_lidar_ignore_corridor",
					&mowgli.AddLidarIgnoreCorridorReq{Corridor: corridor},
					&addRes,
					"mowgli_interfaces/srv/AddLidarIgnoreCorridor")
				if err == nil && !addRes.Success {
					err = errors.New("add_lidar_ignore_corridor rejected a corridor (needs at least 2 points)")
				}
			}
			if err == nil {
				c.JSON(200, OkResponse{})
				return
			}
		case "ignore_obstacle", "discard_obstacle":
			// Reject a PENDING obstacle proposal (currently: wheel-slip dig
			// keepouts) by its MapObstacleInfo.id. Nothing was persisted, so
			// this only drops it from the live keepout mask.
			// Tracker IDs belong to a separate namespace: Ignore must target
			// the tracker, never the map server's pending dig proposals.
			service := "/map_server_node/discard_obstacle"
			if command == "ignore_obstacle" {
				service = "/obstacle_tracker/clear_obstacle"
			}
			var CallReq mowgli.ClearObstacleReq
			err = c.BindJSON(&CallReq)
			if err != nil {
				c.JSON(400, ErrorResponse{Error: err.Error()})
				return
			}
			var discardRes mowgli.ClearObstacleRes
			err = provider.CallService(ctx,
				service,
				&CallReq,
				&discardRes,
				"mowgli_interfaces/srv/ClearObstacle")
			if err == nil && !discardRes.Success {
				err = errors.New(discardRes.Message)
			}
			if err == nil {
				c.JSON(200, map[string]interface{}{"message": discardRes.Message})
				return
			}
		case "fusion_graph_save", "fusion_graph_clear", "fusion_graph_clear_lidar_map":
			// All three target std_srvs/Trigger services on fusion_graph_node.
			// clear_lidar_map drops only the LiDAR map-anchor occupancy grid
			// (use_lidar_map_anchor); the graph itself is untouched.
			type TriggerRes struct {
				Success bool   `json:"success"`
				Message string `json:"message"`
			}
			service := fusionGraphTriggerServices[command]
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
		case "reset_firmware_param_store":
			// This action only arms an explicit, request-id-correlated reset
			// marker. The GUI waits for the matching firmware status before it
			// offers the separate reboot action.
			type TriggerRes struct {
				Success bool   `json:"success"`
				Message string `json:"message"`
			}
			var res TriggerRes
			err = provider.CallService(ctx, "/hardware_bridge/reset_firmware_param_store", &struct{}{}, &res, "std_srvs/srv/Trigger")
			if err == nil && !res.Success {
				err = errors.New(res.Message)
			}
			if err == nil {
				c.JSON(200, map[string]interface{}{"message": res.Message})
				return
			}
		case "clear_dig_escalation":
			// Operator override for a latched repeat-dig escalation
			// (Status.dig_escalated) — see dig_escalation.hpp. Distance-gated
			// server-side: this fails (res.Success=false, a human-readable
			// reason in Message) until the chassis has moved far enough past
			// the obstruction, so the frontend surfaces that reason rather
			// than treating a refusal as a transport error.
			type TriggerRes struct {
				Success bool   `json:"success"`
				Message string `json:"message"`
			}
			var res TriggerRes
			err = provider.CallService(ctx, "/hardware_bridge/clear_dig_escalation", &struct{}{}, &res, "std_srvs/srv/Trigger")
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
