package api

import (
	"context"
	"encoding/base64"
	"encoding/json"
	"errors"
	"fmt"
	"log"
	"net/http"
	"strings"
	"sync"
	"time"

	"github.com/cedbossneo/mowglinext/pkg/msgs/geometry"
	"github.com/cedbossneo/mowglinext/pkg/msgs/mowgli"
	"github.com/cedbossneo/mowglinext/pkg/types"
	"github.com/docker/distribution/uuid"
	"github.com/gin-gonic/gin"
	"github.com/gorilla/websocket"
)

var upgrader = websocket.Upgrader{
	ReadBufferSize:  1024,
	WriteBufferSize: 1024,
	CheckOrigin: func(r *http.Request) bool {
		origin := r.Header.Get("Origin")
		if origin == "" {
			return true // non-browser clients
		}
		// Allow same-host connections
		host := r.Host
		return strings.Contains(origin, host)
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
	case "gps", "pose", "imu", "ticks", "wheelOdom", "lidar":
		return 100, true
	case "fusionRaw", "cogHeading", "magYaw":
		return 200, true
	case "diagnostics", "status", "highLevelStatus", "btLog", "map",
		"path", "plan", "power", "emergency", "dockingSensor",
		"robotDescription", "coverageCells", "recordingTrajectory",
		"obstacles", "fusionDiag":
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
		ctx, cancel := context.WithTimeout(c.Request.Context(), 30*time.Second)
		defer cancel()

		err := provider.CallService(ctx, "/map_server_node/clear_map", &mowgli.ClearMapReq{}, &mowgli.ClearMapRes{}, "std_srvs/srv/Trigger")
		if err != nil {
			c.JSON(500, ErrorResponse{Error: err.Error()})
			return
		}

		var CallReq mowgli.ReplaceMapReq
		err = unmarshalROSMessage[*mowgli.ReplaceMapReq](c.Request.Body, &CallReq)
		if err != nil {
			c.JSON(500, ErrorResponse{Error: err.Error()})
			return
		}
		for _, element := range CallReq.Areas {
			// Ensure Obstacles is an empty slice, not nil — the bridge
			// rejects null for repeated fields ("msg is not a list type").
			if element.Area.Obstacles == nil {
				element.Area.Obstacles = []geometry.Polygon{}
			}
			areaReq := mowgli.AddMowingAreaReq{
				Area:             element.Area,
				IsNavigationArea: element.IsNavigationArea,
			}
			err = provider.CallService(ctx, "/map_server_node/add_area", &areaReq, &mowgli.AddMowingAreaRes{}, "mowgli_interfaces/srv/AddMowingArea")
			if err != nil {
				c.JSON(500, ErrorResponse{Error: err.Error()})
				return
			}
		}

		// Persist areas to disk so they survive container restarts
		err = provider.CallService(ctx, "/map_server_node/save_areas", &mowgli.ClearMapReq{}, &mowgli.ClearMapRes{}, "std_srvs/srv/Trigger")
		if err != nil {
			c.JSON(500, ErrorResponse{Error: fmt.Sprintf("areas added but save_areas failed: %v", err)})
			return
		}

		c.JSON(200, OkResponse{})
	})
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
		err := unmarshalROSMessage[*mowgli.SetDockingPointReq](c.Request.Body, &CallReq)
		if err != nil {
			return
		}
		err = provider.CallService(ctx, "/map_server_node/set_docking_point", &CallReq, &mowgli.SetDockingPointRes{}, "mowgli_interfaces/srv/SetDockingPoint")
		if err != nil {
			c.JSON(500, ErrorResponse{Error: err.Error()})
		} else {
			c.JSON(200, OkResponse{})
		}
	})
}

// SubscriberRoute subscribe to a topic
//
// @Summary subscribe to a topic
// @Description subscribe to a topic
// @Tags mowglinext
// @Param topic path string true "logical topic key: diagnostics, status, highLevelStatus, gps, pose, imu, ticks, map, path, plan, mowingPath, power, emergency, dockingSensor, lidar"
// @Router /mowglinext/subscribe/{topic} [get]
func SubscriberRoute(group *gin.RouterGroup, provider types.IRosProvider) {
	group.GET("/subscribe/:topic", func(c *gin.Context) {
		var err error
		topic := c.Param("topic")
		conn, err := upgrader.Upgrade(c.Writer, c.Request, nil)
		if err != nil {
			return
		}
		defer conn.Close()

		var def func()
		switch topic {
		case "diagnostics":
			def, err = subscribe(provider, c, conn, "diagnostics", -1)
		case "status":
			def, err = subscribe(provider, c, conn, "status", -1)
		case "highLevelStatus":
			def, err = subscribe(provider, c, conn, "highLevelStatus", -1)
		case "gps":
			def, err = subscribe(provider, c, conn, "gps", 100)
		case "pose":
			def, err = subscribe(provider, c, conn, "pose", 100)
		case "fusionRaw":
			def, err = subscribe(provider, c, conn, "fusionRaw", 200)
		case "btLog":
			def, err = subscribe(provider, c, conn, "btLog", -1)
		case "imu":
			def, err = subscribe(provider, c, conn, "imu", 100)
		case "ticks":
			def, err = subscribe(provider, c, conn, "ticks", 100)
		case "wheelOdom":
			def, err = subscribe(provider, c, conn, "wheelOdom", 100)
		case "map":
			def, err = subscribe(provider, c, conn, "map", -1)
		case "path":
			def, err = subscribe(provider, c, conn, "path", -1)
		case "plan":
			def, err = subscribe(provider, c, conn, "plan", -1)
		case "power":
			def, err = subscribe(provider, c, conn, "power", -1)
		case "emergency":
			def, err = subscribe(provider, c, conn, "emergency", -1)
		case "dockingSensor":
			def, err = subscribe(provider, c, conn, "dockingSensor", -1)
		case "lidar":
			def, err = subscribe(provider, c, conn, "lidar", 100)
		case "robotDescription":
			def, err = subscribe(provider, c, conn, "robotDescription", -1)
		case "coverageCells":
			def, err = subscribe(provider, c, conn, "coverageCells", -1)
		case "recordingTrajectory":
			def, err = subscribe(provider, c, conn, "recordingTrajectory", -1)
		case "obstacles":
			def, err = subscribe(provider, c, conn, "obstacles", -1)
		case "cogHeading":
			def, err = subscribe(provider, c, conn, "cogHeading", 200)
		case "magYaw":
			def, err = subscribe(provider, c, conn, "magYaw", 200)
		case "fusionDiag":
			def, err = subscribe(provider, c, conn, "fusionDiag", -1)
		default:
			log.Printf("SubscriberRoute: unknown topic %q", topic)
			return
		}
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
			frame := map[string]string{
				"topic": topic,
				"data":  base64.StdEncoding.EncodeToString(data),
			}
			payload, err := json.Marshal(frame)
			if err != nil {
				return
			}
			writeMu.Lock()
			defer writeMu.Unlock()
			_ = conn.WriteMessage(websocket.TextMessage, payload)
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

			err := provider.Subscribe(topic, id, func(msg []byte) {
				if interval > 0 {
					time.Sleep(time.Duration(interval) * time.Millisecond)
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
	err := provider.Subscribe(topic, uidString, func(msg []byte) {
		if interval > 0 {
			time.Sleep(time.Duration(interval) * time.Millisecond)
		}
		writeMu.Lock()
		defer writeMu.Unlock()
		writer, err := conn.NextWriter(websocket.TextMessage)
		if err != nil {
			c.Error(err)
			return
		}
		_, err = writer.Write([]byte(base64.StdEncoding.EncodeToString(msg)))
		if err != nil {
			c.Error(err)
			return
		}
		err = writer.Close()
		if err != nil {
			c.Error(err)
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
		var err error
		switch command {
		case "high_level_control":
			var CallReq mowgli.HighLevelControlReq
			err = c.BindJSON(&CallReq)
			if err != nil {
				return
			}
			err = provider.CallService(c.Request.Context(), "/behavior_tree_node/high_level_control", &CallReq, &mowgli.HighLevelControlRes{}, "mowgli_interfaces/srv/HighLevelControl")
		case "emergency":
			var CallReq mowgli.EmergencyStopReq
			err = c.BindJSON(&CallReq)
			if err != nil {
				return
			}
			err = provider.CallService(c.Request.Context(), "/hardware_bridge/emergency_stop", &CallReq, &mowgli.EmergencyStopRes{}, "mowgli_interfaces/srv/EmergencyStop")
		case "mow_enabled":
			var CallReq mowgli.MowerControlReq
			err = c.BindJSON(&CallReq)
			if err != nil {
				return
			}
			err = provider.CallService(c.Request.Context(), "/hardware_bridge/mower_control", &CallReq, &mowgli.MowerControlRes{}, "mowgli_interfaces/srv/MowerControl")
		case "start_in_area":
			var CallReq mowgli.StartInAreaReq
			err = c.BindJSON(&CallReq)
			if err != nil {
				return
			}
			err = provider.CallService(c.Request.Context(), "/behavior_tree_node/start_in_area", &CallReq, &mowgli.StartInAreaRes{}, "mowgli_interfaces/srv/StartInArea")
		case "set_datum":
			type TriggerRes struct {
				Success bool   `json:"success"`
				Message string `json:"message"`
			}
			var res TriggerRes
			err = provider.CallService(c.Request.Context(), "/navsat_to_absolute_pose/set_datum", &struct{}{}, &res, "std_srvs/srv/Trigger")
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
				return
			}
			var promoteRes mowgli.PromoteObstacleRes
			err = provider.CallService(c.Request.Context(),
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
			err = provider.CallService(c.Request.Context(), service, &struct{}{}, &res, "std_srvs/srv/Trigger")
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
