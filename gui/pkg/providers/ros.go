package providers

import (
	"context"
	"encoding/json"
	"math"
	"sync"
	"time"

	"github.com/cedbossneo/mowglinext/pkg/foxglove"
	"github.com/cedbossneo/mowglinext/pkg/msgs/mowgli"
	types2 "github.com/cedbossneo/mowglinext/pkg/types"
	"github.com/sirupsen/logrus"
)

// topicDef maps a logical subscribe key to a ROS2 topic name and message type.
// The frontend and internal routes always use logical keys; the ROS2 topic name
// is only used when sending the foxglove subscribe op.
type topicDef struct {
	ROS2Topic string
	MsgType   string
}

// topicMap maps logical keys (used by SubscriberRoute and internal code) to
// their corresponding ROS2 topics and message types.
// Virtual topics (map) have an empty MsgType and are never sent to
// foxglove_bridge; they are populated by internal logic instead.
var topicMap = map[string]topicDef{
	"status":          {"/hardware_bridge/status", "mowgli_interfaces/msg/Status"},
	"highLevelStatus": {"/behavior_tree_node/high_level_status", "mowgli_interfaces/msg/HighLevelStatus"},
	"gps":             {"/gps/fix", "sensor_msgs/msg/NavSatFix"},
	"gnssStatus":      {"/gps/status", "mowgli_interfaces/msg/GnssStatus"},
	// The robot's global pose comes from fusion_graph_node, the sole
	// map-frame localizer. "pose" and "fusionRaw" both point at
	// /odometry/filtered_map; the duplicate key is kept for backwards
	// compatibility with older GUI components that subscribed to the
	// "fusionRaw" channel by name.
	"pose":                {"/odometry/filtered_map", "nav_msgs/msg/Odometry"},
	"fusionRaw":           {"/odometry/filtered_map", "nav_msgs/msg/Odometry"},
	"btLog":               {"/behavior_tree_log", "nav2_msgs/msg/BehaviorTreeLog"},
	"imu":                 {"/imu/data", "sensor_msgs/msg/Imu"},
	"ticks":               {"/wheel_ticks", "mowgli_interfaces/msg/WheelTick"},
	"wheelOdom":           {"/wheel_odom", "nav_msgs/msg/Odometry"},
	"map":                 {"", ""},                                                                   // virtual – populated via map_server services
	"path":                {"/controller_server/FollowCoveragePath/global_plan", "nav_msgs/msg/Path"}, // coverage plan
	"plan":                {"/plan", "nav_msgs/msg/Path"},                                             // infrequent event
	"coverageCells":       {"/map_server_node/coverage_cells", "nav_msgs/msg/OccupancyGrid"},          // large message
	"power":               {"/hardware_bridge/power", "mowgli_interfaces/msg/Power"},
	"emergency":           {"/hardware_bridge/emergency", "mowgli_interfaces/msg/Emergency"}, // safety-critical
	"lidar":               {"/scan", "sensor_msgs/msg/LaserScan"},                            // large message
	"diagnostics":         {"/diagnostics", "diagnostic_msgs/msg/DiagnosticArray"},
	"fusionDiag":          {"/fusion_graph/diagnostics", "diagnostic_msgs/msg/DiagnosticArray"},
	"obstacles":           {"/obstacle_tracker/obstacles", "mowgli_interfaces/msg/ObstacleArray"},
	"robotDescription":    {"/robot_description", "std_msgs/msg/String"},                     // published once
	"recordingTrajectory": {"/behavior_tree_node/recording_trajectory", "nav_msgs/msg/Path"}, // area recording preview
	// Synthetic heading sources fused by fusion_graph_node as yaw unary
	// factors. Both carry sensor_msgs/Imu with only `orientation` and
	// `orientation_covariance[8]` populated — see cog_to_imu.py and
	// mag_yaw_publisher.py in mowgli_localization.
	"cogHeading": {"/imu/cog_heading", "sensor_msgs/msg/Imu"},
	"magYaw":     {"/imu/mag_yaw", "sensor_msgs/msg/Imu"},
}

// ---------------------------------------------------------------------------
// RosSubscriber – single fan-out worker for one (topic, id) pair
// ---------------------------------------------------------------------------

// RosSubscriber delivers messages from a ROS2 topic to one registered callback.
// It runs a background goroutine that drains a single-slot mailbox so that a
// slow consumer cannot stall the foxglove read pump or other subscribers.
type RosSubscriber struct {
	Topic string
	Id    string

	mtx         sync.Mutex
	cb          func(msg []byte)
	nextMessage []byte
	interval    time.Duration // min spacing between deliveries; 0 = unthrottled
	close       chan struct{}
	wake        chan struct{} // buffered(1) new-message signal
}

// NewRosSubscriber creates and starts a RosSubscriber. interval is the minimum
// time between deliveries to cb; messages arriving faster are coalesced so cb
// always receives the latest value. The caller must eventually call Close to
// release the background goroutine.
func NewRosSubscriber(topic, id string, interval time.Duration, cb func(msg []byte)) *RosSubscriber {
	r := &RosSubscriber{
		Topic:    topic,
		Id:       id,
		cb:       cb,
		interval: interval,
		close:    make(chan struct{}),
		wake:     make(chan struct{}, 1),
	}
	go r.run()
	return r
}

// Publish stores msg as the next message to be delivered and signals the
// delivery loop. If a previous message has not yet been consumed it is silently
// overwritten (coalesce to latest). Never blocks.
func (r *RosSubscriber) Publish(msg []byte) {
	r.mtx.Lock()
	r.nextMessage = msg
	r.mtx.Unlock()
	select {
	case r.wake <- struct{}{}:
	default:
	}
}

// Close stops the background goroutine. It is safe to call from any goroutine.
func (r *RosSubscriber) Close() {
	close(r.close)
}

// run is the delivery loop. It is event-driven (no idle polling) and enforces
// the throttle interval between deliveries WITHOUT blocking the publisher: the
// wait happens here, on the subscriber's own goroutine, and the mailbox keeps
// only the latest message so the delivered value is always fresh.
func (r *RosSubscriber) run() {
	var lastDeliver time.Time
	for {
		select {
		case <-r.close:
			return
		case <-r.wake:
		}

		if r.interval > 0 {
			if wait := r.interval - time.Since(lastDeliver); wait > 0 {
				select {
				case <-r.close:
					return
				case <-time.After(wait):
				}
			}
		}

		r.mtx.Lock()
		msg := r.nextMessage
		r.nextMessage = nil
		r.mtx.Unlock()

		if msg != nil {
			r.cb(msg)
			lastDeliver = time.Now()
		}
	}
}

// ---------------------------------------------------------------------------
// RosProvider – IRosProvider implementation backed by foxglove WebSocket
// ---------------------------------------------------------------------------

// RosProvider implements types2.IRosProvider using a foxglove WebSocket
// client. All topic access uses logical keys defined in topicMap; the actual
// ROS2 topic names are an internal concern.
//
// Upstream foxglove_bridge subscriptions are managed lazily: a logical key is
// subscribed on the bridge only when at least one downstream listener is
// registered for it, and unsubscribed when the last listener leaves. This
// keeps the per-message CDR→JSON deserialization (in gui/pkg/foxglove) off
// the hot path for topics nobody is using right now — e.g. /scan, /imu/data,
// and /wheel_odom no longer chew CPU when the browser is closed and the
// optional MQTT/HomeKit providers are disabled.
type RosProvider struct {
	client       *foxglove.Client
	cmdVelRelay  *cmdVelRelayClient

	mtx                sync.Mutex
	subscribers        map[string]map[string]*RosSubscriber // logicalKey -> id -> subscriber
	lastMessage        map[string][]byte                    // logicalKey -> last JSON bytes
	foxgloveSubscribed map[string]bool                      // logicalKey -> upstream-subscribed?

	// Cached docking pose from map_server_node (guarded by mtx)
	dockPoseSet bool
	dockX       float64
	dockY       float64
	dockHeading float64

	// Charging state from hardware_bridge/status (guarded by mtx)

	dbProvider     types2.IDBProvider
	sessionTracker *SessionTracker
}

// foxgloveAdapters maps logicalKey to a per-message transform applied between
// the foxglove client and the fanOut. Topics absent from this map are
// forwarded as-is (snake_case JSON from CDR deserialization).
var foxgloveAdapters = map[string]func([]byte) ([]byte, error){
	"gps":        adaptGPS,
	"gnssStatus": adaptGnssStatus,
	"pose":       adaptPose,
}

// upstreamDecimationMs caps the rate at which high-frequency topics are
// deserialized from the foxglove bridge. The GUI throttles these to ~10 Hz
// downstream anyway (topicSubscribeInterval), so deserializing /imu at 100 Hz
// or /scan at 40 Hz on the read pump is wasted CPU. 80 ms (~12.5 Hz) stays just
// above the 100 ms downstream throttle so no visible frame is lost. Topics not
// listed here are deserialized at their native rate.
var upstreamDecimationMs = map[string]int{
	"imu":        80,
	"lidar":      80,
	"pose":       80,
	"fusionRaw":  80,
	"wheelOdom":  80,
	"ticks":      80,
	"gps":        80,
	"gnssStatus": 80,
	"cogHeading": 150,
	"magYaw":     150,
}

// NewRosProvider constructs a RosProvider, reads the foxglove URL from the
// database (falling back to ws://localhost:8765), and connects asynchronously.
// The returned value is ready to use immediately; Subscribe calls made before
// the connection is established will be fulfilled once the connection comes up
// via the foxglove client's reconnect loop.
func NewRosProvider(dbProvider types2.IDBProvider) types2.IRosProvider {
	foxgloveURL := "ws://localhost:8765"
	if url, err := dbProvider.Get("system.ros.foxgloveUrl"); err == nil && len(url) > 0 {
		foxgloveURL = string(url)
	}

	cmdVelRelayURL := "ws://localhost:8766"

	r := &RosProvider{
		client:             foxglove.NewClient(foxgloveURL),
		cmdVelRelay:        newCmdVelRelayClient(cmdVelRelayURL),
		subscribers:        make(map[string]map[string]*RosSubscriber),
		lastMessage:        make(map[string][]byte),
		foxgloveSubscribed: make(map[string]bool),
		dbProvider:         dbProvider,
		sessionTracker:     NewSessionTracker(dbProvider),
	}

	go func() {
		if err := r.client.Connect(context.Background()); err != nil {
			logrus.Errorf("RosProvider: foxglove initial connect failed: %v", err)
		}
		r.initDockPoseSubscription()
		r.initMapPolling()
	}()

	return r
}

// ensureFoxgloveSubscribed subscribes the foxglove client to the ROS2 topic
// backing logicalKey if it isn't already. No-op for virtual keys (empty
// MsgType) or unknown keys. Caller must hold r.mtx.
func (r *RosProvider) ensureFoxgloveSubscribed(logicalKey string) {
	if r.foxgloveSubscribed[logicalKey] {
		return
	}
	def, ok := topicMap[logicalKey]
	if !ok || def.MsgType == "" {
		return
	}

	key := logicalKey // capture for closure
	var cb func(json.RawMessage)
	if adapt, ok := foxgloveAdapters[key]; ok {
		fn := adapt
		cb = func(msg json.RawMessage) {
			adapted, err := fn([]byte(msg))
			if err != nil {
				logrus.Errorf("RosProvider: adapt %s: %v", key, err)
				return
			}
			r.fanOut(key, adapted)
		}
	} else {
		cb = func(msg json.RawMessage) {
			r.fanOut(key, []byte(msg))
		}
	}

	var subOpts []int
	if dec, ok := upstreamDecimationMs[key]; ok {
		subOpts = append(subOpts, dec)
	}
	if err := r.client.Subscribe(def.ROS2Topic, def.MsgType, "gui-"+key, cb, subOpts...); err != nil {
		logrus.Errorf("RosProvider: subscribe %s (%s): %v", def.ROS2Topic, key, err)
		return
	}
	r.foxgloveSubscribed[key] = true
	logrus.Infof("RosProvider: subscribed to %s as '%s'", def.ROS2Topic, key)
}

// maybeUnsubscribeFoxglove drops the upstream foxglove subscription for
// logicalKey if no downstream listeners remain. Caller must hold r.mtx.
func (r *RosProvider) maybeUnsubscribeFoxglove(logicalKey string) {
	if !r.foxgloveSubscribed[logicalKey] {
		return
	}
	if subs := r.subscribers[logicalKey]; len(subs) > 0 {
		return
	}
	def, ok := topicMap[logicalKey]
	if !ok || def.MsgType == "" {
		return
	}
	r.client.Unsubscribe(def.ROS2Topic, "gui-"+logicalKey)
	delete(r.foxgloveSubscribed, logicalKey)
	// Drop the cached last-message — stale once we stop receiving updates.
	delete(r.lastMessage, logicalKey)
	logrus.Infof("RosProvider: unsubscribed from %s (no listeners)", def.ROS2Topic)
}

// fanOut stores msg as the latest value for logicalKey and delivers it to all
// registered RosSubscribers for that key. The caller must not hold mtx.
func (r *RosProvider) fanOut(logicalKey string, msg []byte) {
	r.mtx.Lock()
	defer r.mtx.Unlock()
	r.lastMessage[logicalKey] = msg
	for _, sub := range r.subscribers[logicalKey] {
		sub.Publish(msg)
	}
	// Track mowing sessions from high-level status transitions
	if logicalKey == "highLevelStatus" && r.sessionTracker != nil {
		msgCopy := append([]byte(nil), msg...)
		go r.sessionTracker.OnHighLevelStatus(msgCopy)
	}
}

// initDockPoseSubscription subscribes to the map_server_node's docking_pose
// topic (transient_local QoS) and caches the latest dock position/heading.
// The cached values are included in the virtual "map" topic by pollMap().
func (r *RosProvider) initDockPoseSubscription() {
	type rawPoseStamped struct {
		Pose struct {
			Position    struct{ X, Y, Z float64 }    `json:"position"`
			Orientation struct{ X, Y, Z, W float64 } `json:"orientation"`
		} `json:"pose"`
	}

	err := r.client.Subscribe(
		"/map_server_node/docking_pose",
		"geometry_msgs/msg/PoseStamped",
		"gui-docking-pose",
		func(msg json.RawMessage) {
			var ps rawPoseStamped
			if err := json.Unmarshal([]byte(msg), &ps); err != nil {
				logrus.Errorf("RosProvider: unmarshal docking_pose: %v", err)
				return
			}
			q := ps.Pose.Orientation
			heading := math.Atan2(2*(q.W*q.Z+q.X*q.Y), 1-2*(q.Y*q.Y+q.Z*q.Z))

			r.mtx.Lock()
			r.dockPoseSet = true
			r.dockX = ps.Pose.Position.X
			r.dockY = ps.Pose.Position.Y
			r.dockHeading = heading
			r.mtx.Unlock()

			logrus.Infof("RosProvider: docking pose updated (%.3f, %.3f) heading=%.3f",
				ps.Pose.Position.X, ps.Pose.Position.Y, heading)
		},
	)
	if err != nil {
		logrus.Errorf("RosProvider: subscribe docking_pose: %v", err)
	} else {
		logrus.Info("RosProvider: subscribed to /map_server_node/docking_pose")
	}
}

// initMapPolling periodically fetches mowing areas from the map_server_node
// and publishes the result to the virtual "map" topic for the GUI.
func (r *RosProvider) initMapPolling() {
	go func() {
		// Wait for foxglove_bridge to be ready
		time.Sleep(5 * time.Second)

		ticker := time.NewTicker(5 * time.Second)
		defer ticker.Stop()

		for range ticker.C {
			r.pollMap()
		}
	}()
}

func (r *RosProvider) pollMap() {
	ctx, cancel := context.WithTimeout(context.Background(), 10*time.Second)
	defer cancel()

	var workingAreas []mowgli.MapArea
	var navAreas []mowgli.MapArea

	// Fetch all areas (index 0..N until success=false)
	for i := uint32(0); i < 100; i++ {
		req := mowgli.GetMowingAreaReq{Index: i}
		var res mowgli.GetMowingAreaRes
		err := r.CallService(ctx, "/map_server_node/get_mowing_area", &req, &res, "mowgli_interfaces/srv/GetMowingArea")
		if err != nil {
			if i == 0 {
				logrus.WithError(err).WithField("index", i).Warn("pollMap: get_mowing_area failed — map_server_node may not be ready")
			} else {
				logrus.WithError(err).WithField("index", i).Warn("pollMap: get_mowing_area failed mid-iteration")
			}
			break
		}
		if !res.Success {
			break
		}
		if res.Area.IsNavigationArea {
			navAreas = append(navAreas, res.Area)
		} else {
			workingAreas = append(workingAreas, res.Area)
		}
	}

	if workingAreas == nil {
		workingAreas = []mowgli.MapArea{}
	}
	if navAreas == nil {
		navAreas = []mowgli.MapArea{}
	}

	// Read cached docking pose (written by initDockPoseSubscription)
	r.mtx.Lock()
	dockX := r.dockX
	dockY := r.dockY
	dockHeading := r.dockHeading
	r.mtx.Unlock()

	mapData := mowgli.Map{
		MapWidth:        20.0,
		MapHeight:       20.0,
		MapCenterX:      0.0,
		MapCenterY:      0.0,
		NavigationAreas: navAreas,
		WorkingArea:     workingAreas,
		DockX:           dockX,
		DockY:           dockY,
		DockHeading:     dockHeading,
	}

	data, err := json.Marshal(mapData)
	if err != nil {
		return
	}

	r.fanOut("map", data)
}

// ---------------------------------------------------------------------------
// IRosProvider implementation
// ---------------------------------------------------------------------------

// CallService calls a ROS2 service via foxglove_bridge and unmarshals the
// response into res (ignored when res is nil). An optional serviceType
// (e.g. "mowgli_interfaces/srv/GetMowingArea") can be passed for service
// type resolution.
func (r *RosProvider) CallService(ctx context.Context, service string, req any, res any, serviceType ...string) error {
	result, err := r.client.CallService(ctx, service, req, serviceType...)
	if err != nil {
		return err
	}
	if res != nil {
		if err := json.Unmarshal(result, res); err != nil {
			return err
		}
	}
	return nil
}

// Subscribe registers cb to receive JSON messages on the given logical topic
// key. If a message was already received for this key, cb is invoked
// immediately with the cached value. The first listener for a non-virtual
// topic also triggers the upstream foxglove_bridge subscription.
func (r *RosProvider) Subscribe(topic string, id string, intervalMs int, cb func(msg []byte)) error {
	r.mtx.Lock()
	defer r.mtx.Unlock()

	if r.subscribers[topic] == nil {
		r.subscribers[topic] = make(map[string]*RosSubscriber)
	}
	if _, exists := r.subscribers[topic][id]; !exists {
		interval := time.Duration(intervalMs) * time.Millisecond
		r.subscribers[topic][id] = NewRosSubscriber(topic, id, interval, cb)
	}

	// Subscribe upstream on first listener for this logical key. Safe to call
	// repeatedly — ensureFoxgloveSubscribed short-circuits on the second hit.
	r.ensureFoxgloveSubscribed(topic)

	// Replay the most recent message so the subscriber is immediately usable.
	if last, ok := r.lastMessage[topic]; ok {
		r.subscribers[topic][id].Publish(last)
	}
	return nil
}

// UnSubscribe stops and removes the subscriber identified by (topic, id).
// When the last subscriber for a logical key is removed, the upstream
// foxglove_bridge subscription is dropped too.
func (r *RosProvider) UnSubscribe(topic string, id string) {
	r.mtx.Lock()
	defer r.mtx.Unlock()

	subs, ok := r.subscribers[topic]
	if !ok {
		return
	}
	sub, exists := subs[id]
	if !exists {
		return
	}
	sub.Close()
	delete(subs, id)
	if len(subs) == 0 {
		delete(r.subscribers, topic)
		r.maybeUnsubscribeFoxglove(topic)
	}
}

// Publish sends msg to the named ROS2 topic. For /cmd_vel_teleop the relay
// client (port 8766) is preferred over foxglove_bridge: it delivers JSON
// directly to rclpy without JSON→CDR conversion overhead or the shared-
// connection head-of-line blocking that causes manual mowing lag. If the
// relay is not yet connected (e.g., early startup), it falls back to
// foxglove_bridge so the first manual commands still reach the robot.
func (r *RosProvider) Publish(topic string, msgType string, msg interface{}) error {
	if topic == "/cmd_vel_teleop" && r.cmdVelRelay.Connected() {
		return r.cmdVelRelay.Send(msg)
	}
	return r.client.Publish(topic, msg, msgType)
}
