package providers

import (
	"bytes"
	"context"
	"encoding/json"
	"math"
	"sync"
	"time"

	"github.com/mowglinext/mowglinext/pkg/foxglove"
	"github.com/mowglinext/mowglinext/pkg/msgs/mowgli"
	types2 "github.com/mowglinext/mowglinext/pkg/types"
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
	// One-click dock calibration live status (the GUI's foxglove-friendly
	// window into the CalibrateDock action — foxglove_bridge has no action op).
	"dockCalibrationStatus": {"/calibrate_imu_yaw_node/dock_calibration/status", "mowgli_interfaces/msg/DockCalibrationStatus"},
	"gps":                   {"/gps/fix", "sensor_msgs/msg/NavSatFix"},
	"gnssStatus":            {"/gps/status", "mowgli_interfaces/msg/GnssStatus"},
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
	"map":                 {"", ""}, // virtual – populated via map_server services
	"path":                {"/coverage/plan_preview", "mowgli_interfaces/msg/CoveragePlanPreview"},
	"plan":                {"/plan", "nav_msgs/msg/Path"}, // infrequent event
	"power":               {"/hardware_bridge/power", "mowgli_interfaces/msg/Power"},
	"emergency":           {"/hardware_bridge/emergency", "mowgli_interfaces/msg/Emergency"}, // safety-critical
	"lidar":               {"/scan", "sensor_msgs/msg/LaserScan"},                            // large message
	"mowProgress":         {"/map_server_node/mow_progress", "nav_msgs/msg/OccupancyGrid"},   // mowed-area overlay (large)
	"lidarMap":            {"/fusion_graph/lidar_map", "nav_msgs/msg/OccupancyGrid"},         // fusion_graph LiDAR anchor map (large, latched)
	"diagnostics":         {"/diagnostics", "diagnostic_msgs/msg/DiagnosticArray"},
	"fusionDiag":          {"/fusion_graph/diagnostics", "diagnostic_msgs/msg/DiagnosticArray"},
	"obstacles":           {"/obstacle_tracker/obstacles", "mowgli_interfaces/msg/ObstacleArray"},
	"robotDescription":    {"/robot_description", "std_msgs/msg/String"},                     // published once
	"recordingTrajectory": {"/behavior_tree_node/recording_trajectory", "nav_msgs/msg/Path"}, // area recording preview
	// Latched std_msgs/Bool: true when a prior interrupted mow can be resumed, so
	// the GUI offers "Resume" vs "Start fresh" instead of silently resuming (the
	// "starts at 2nd/3rd line" report).
	"coverageResumeAvailable": {"/behavior_tree_node/coverage_resume_available", "std_msgs/msg/Bool"},
	// Synthetic heading sources fused by fusion_graph_node as yaw unary
	// factors. Both carry sensor_msgs/Imu with only `orientation` and
	// `orientation_covariance[8]` populated — see cog_to_imu.py and
	// mag_yaw_publisher.py in mowgli_localization.
	"cogHeading": {"/imu/cog_heading", "sensor_msgs/msg/Imu"},
	"magYaw":     {"/imu/mag_yaw", "sensor_msgs/msg/Imu"},
	// 1 Hz per-session coverage sets (completed / attempted / excluded areas)
	// for the fleet coordinator on every member's GUI (docs/MULTI_ROBOT.md).
	"coverageSession": {"/behavior_tree_node/coverage_session", "mowgli_interfaces/msg/CoverageSession"},
	// Latched: what the STM32 actually runs for every runtime parameter, its
	// envelope and whether it is persisted in the board's flash (protocol v7).
	"firmwareParams": {"/hardware_bridge/firmware_params", "mowgli_interfaces/msg/FirmwareParams"},
	// Native MAVROS inventory surfaces. VehicleInfo is exposed by the
	// sys_status plugin through a service rather than a topic, so the provider
	// republishes that unmodified service response as a virtual GUI stream.
	"mavrosState":       {"/mavros/state", "mavros_msgs/msg/State"},
	"mavrosVehicleInfo": {"", ""},
}

// TopicKeys returns every logical topic key the provider can subscribe to, so
// the API layer can check that each one is also routable.
func TopicKeys() []string {
	keys := make([]string, 0, len(topicMap))
	for key := range topicMap {
		keys = append(keys, key)
	}
	return keys
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

type rosClient interface {
	Connect(context.Context) error
	Subscribe(string, string, string, func(json.RawMessage), ...int) error
	Unsubscribe(string, string)
	CallService(context.Context, string, interface{}, ...string) (json.RawMessage, error)
	Publish(string, interface{}, ...string) error
	GetParameters(context.Context, []string) ([]foxglove.Parameter, error)
	SetParameters(context.Context, []foxglove.Parameter) ([]foxglove.Parameter, error)
}

type foxgloveSubscription struct {
	// Serializes reconciliation for one key, never while holding RosProvider.mtx.
	mtx        sync.Mutex
	subscribed bool
}

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
	client      rosClient
	cmdVelRelay *cmdVelRelayClient

	mtx                   sync.Mutex
	subscribers           map[string]map[string]*RosSubscriber // logicalKey -> id -> subscriber
	lastMessage           map[string][]byte                    // logicalKey -> last JSON bytes
	foxgloveSubscriptions map[string]*foxgloveSubscription     // logicalKey -> reconciliation state

	// Cached docking pose from map_server_node (guarded by mtx)
	dockPoseSet bool
	dockX       float64
	dockY       float64
	dockHeading float64

	// Charging state from hardware_bridge/status (guarded by mtx)

	dbProvider     types2.IDBProvider
	sessionTracker *SessionTracker
	// notifier receives highLevelStatus + map payloads for push notifications;
	// nil until AttachNotifier (guarded by mtx).
	notifier *NotificationProvider

	// The MAVROS vehicle-info service is polled only while a browser is
	// listening. This keeps the established Mowgli backend completely idle on
	// MAVROS-only ROS surfaces.
	mavrosVehicleInfoPollCancel context.CancelFunc
}

// AttachNotifier routes highLevelStatus and map payloads to the notification
// provider from now on. The provider is built after the ROS provider in
// main.go, hence a setter rather than a constructor argument.
func (r *RosProvider) AttachNotifier(n *NotificationProvider) {
	r.mtx.Lock()
	defer r.mtx.Unlock()
	r.notifier = n
	if last, ok := r.lastMessage["map"]; ok && n != nil {
		n.EnqueueMap(append([]byte(nil), last...))
	}
}

// foxgloveAdapters maps logicalKey to a per-message transform applied between
// the foxglove client and the fanOut. Topics absent from this map are
// forwarded as-is (snake_case JSON from CDR deserialization).
var foxgloveAdapters = map[string]func([]byte) ([]byte, error){
	"gps":        adaptGPS,
	"gnssStatus": adaptGnssStatus,
	"pose":       adaptPose,
	"lidar":      adaptLidar,
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
		client:                foxglove.NewClient(foxgloveURL),
		cmdVelRelay:           newCmdVelRelayClient(cmdVelRelayURL),
		subscribers:           make(map[string]map[string]*RosSubscriber),
		lastMessage:           make(map[string][]byte),
		foxgloveSubscriptions: make(map[string]*foxgloveSubscription),
		dbProvider:            dbProvider,
		sessionTracker:        NewSessionTracker(dbProvider),
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

// reconcileFoxgloveSubscription applies the latest downstream listener state.
// Bridge I/O must never hold the provider lock: the read pump also needs it to
// fan out messages. Recheck after each operation so an unsubscribe racing a
// subscribe cannot leave a live listener detached (or an unused topic active).
func (r *RosProvider) reconcileFoxgloveSubscription(logicalKey string) {
	def, ok := topicMap[logicalKey]
	if !ok || def.MsgType == "" {
		return
	}
	r.mtx.Lock()
	if r.foxgloveSubscriptions == nil {
		r.foxgloveSubscriptions = make(map[string]*foxgloveSubscription)
	}
	state := r.foxgloveSubscriptions[logicalKey]
	if state == nil {
		state = &foxgloveSubscription{}
		r.foxgloveSubscriptions[logicalKey] = state
	}
	r.mtx.Unlock()
	state.mtx.Lock()
	defer state.mtx.Unlock()
	for {
		r.mtx.Lock()
		wanted := len(r.subscribers[logicalKey]) > 0
		r.mtx.Unlock()
		if wanted == state.subscribed {
			return
		}
		if wanted {
			if err := r.subscribeFoxglove(logicalKey, def); err != nil {
				logrus.Errorf("RosProvider: subscribe %s (%s): %v", def.ROS2Topic, logicalKey, err)
				return
			}
		} else {
			r.client.Unsubscribe(def.ROS2Topic, "gui-"+logicalKey)
		}
		state.subscribed = wanted
	}
}

func (r *RosProvider) subscribeFoxglove(logicalKey string, def topicDef) error {
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
	return r.client.Subscribe(def.ROS2Topic, def.MsgType, "gui-"+key, cb, subOpts...)
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
	// Track mowing sessions from high-level status transitions. Enqueue to the
	// tracker's single-consumer goroutine so transitions are applied in arrival
	// order (a goroutine-per-message could reorder rapid MOWING->CHARGING->IDLE
	// bursts and corrupt the session state machine).
	if logicalKey == "highLevelStatus" && r.sessionTracker != nil {
		msgCopy := append([]byte(nil), msg...)
		r.sessionTracker.Enqueue(msgCopy)
	}
	// Feed wheel odometry to the session odometer (total distance / blade-wear
	// proxy). Kept alive by the MQTT persistent "wheelOdom" subscription, same as
	// highLevelStatus above.
	if logicalKey == "wheelOdom" && r.sessionTracker != nil {
		msgCopy := append([]byte(nil), msg...)
		r.sessionTracker.EnqueueOdometry(msgCopy)
	}
	// Push notifications: same ordered-queue contract as the session tracker;
	// the map payload keeps its zone-name lookup current.
	if r.notifier != nil {
		switch logicalKey {
		case "highLevelStatus":
			r.notifier.Enqueue(append([]byte(nil), msg...))
		case "map":
			r.notifier.EnqueueMap(append([]byte(nil), msg...))
		}
	}
}

func (r *RosProvider) startMavrosVehicleInfoPolling() {
	r.mtx.Lock()
	if r.mavrosVehicleInfoPollCancel != nil || len(r.subscribers["mavrosVehicleInfo"]) == 0 {
		r.mtx.Unlock()
		return
	}
	ctx, cancel := context.WithCancel(context.Background())
	r.mavrosVehicleInfoPollCancel = cancel
	r.mtx.Unlock()

	go func() {
		poll := func() {
			callCtx, callCancel := context.WithTimeout(ctx, 3*time.Second)
			defer callCancel()
			result, err := r.client.CallService(
				callCtx,
				"/mavros/vehicle_info_get",
				map[string]interface{}{"sysid": 0, "compid": 0, "get_all": false},
				"mavros_msgs/srv/VehicleInfoGet",
			)
			if err != nil {
				if ctx.Err() == nil {
					logrus.Debugf("RosProvider: MAVROS vehicle_info_get unavailable: %v", err)
				}
				return
			}
			r.fanOut("mavrosVehicleInfo", []byte(result))
		}

		poll()
		ticker := time.NewTicker(5 * time.Second)
		defer ticker.Stop()
		for {
			select {
			case <-ctx.Done():
				return
			case <-ticker.C:
				poll()
			}
		}
	}()
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
//
// pollMap() makes one sequential CallService round-trip per area
// (get_mowing_area), and each of those competes with the rest of the ROS2
// stack for foxglove_bridge/DDS bandwidth. Field-reported 2026-09-28: while
// actively mowing, each call was taking ~4s, so a poll with several areas
// ran well past the 5s tick interval. This loop is already single-threaded
// (pollMap always runs to completion before the next `range` iteration reads
// the channel, so pollMap can never actually run twice AT ONCE) — the
// symptom is different: time.Ticker's channel buffers at most one pending
// tick, and while a slow pollMap() is still running, that one tick fires
// and sits waiting. The MOMENT pollMap() returns, the loop reads that
// already-fired tick immediately — giving a second, near-back-to-back poll
// with almost no gap — before the normal 5s cadence resumes. Drain that one
// stale tick (if any) right after pollMap() returns, so a slow poll costs
// one lost tick instead of a rapid-fire double one.
func (r *RosProvider) initMapPolling() {
	go func() {
		// Wait for foxglove_bridge to be ready
		time.Sleep(5 * time.Second)

		ticker := time.NewTicker(5 * time.Second)
		defer ticker.Stop()

		for range ticker.C {
			start := time.Now()
			r.pollMap()
			if elapsed := time.Since(start); elapsed > 5*time.Second {
				logrus.WithField("elapsed", elapsed).Warn(
					"pollMap: took longer than the 5s poll interval (ROS2 service " +
						"calls contended, likely while actively mowing)")
			}
			// Unconditional, non-blocking: drain a tick that already fired while
			// pollMap() was running (there is at most one — see the comment
			// above), so an overrun costs one lost tick instead of an
			// immediate back-to-back poll. A no-op (does nothing, costs
			// nothing) on every normal, on-time cycle.
			select {
			case <-ticker.C:
			default:
			}
		}
	}()
}

// Preserve each working area's ROS ARRAY INDEX (its position in `areas`, what
// map_server's index-based services — get_mowing_area, start_in_area,
// coverage_orientation — expect) when the UI separates mowing and navigation
// areas. This is NOT the stable MapArea.Id (mowglinext#637): the index is
// wire-protocol-required and unavoidably shifts whenever the area list is
// edited/saved (map_server rebuilds it wholesale), but Id survives that edit
// (map_server's on_add_area preserves a caller-supplied one) and is carried
// through verbatim on every returned MapArea — the frontend resolves the
// CURRENT index from Id at the moment it acts (mowingAreaIndexById), rather
// than trusting a position captured earlier.
func splitMapAreas(areas []mowgli.MapArea) (working, navigation []mowgli.MapArea, indices []uint32) {
	for index, area := range areas {
		if area.IsNavigationArea {
			navigation = append(navigation, area)
		} else {
			working = append(working, area)
			indices = append(indices, uint32(index))
		}
	}
	return
}

func (r *RosProvider) pollMap() {
	ctx, cancel := context.WithTimeout(context.Background(), 10*time.Second)
	defer cancel()

	var allAreas []mowgli.MapArea
	complete := false

	// Fetch all areas (index 0..N until success=false)
	for i := uint32(0); i < 100; i++ {
		req := mowgli.GetMowingAreaReq{Index: i}
		var res mowgli.GetMowingAreaRes
		err := r.CallService(ctx, "/map_server_node/get_mowing_area", &req, &res, "mowgli_interfaces/srv/GetMowingArea")
		if err != nil {
			logrus.WithError(err).WithField("index", i).Warn(
				"pollMap: refresh failed; retaining last complete map (stale)")
			return
		}
		if !res.Success {
			complete = true
			break
		}
		allAreas = append(allAreas, res.Area)
	}
	// Only the service's end-of-list response establishes completeness. A
	// transport failure or reaching the enumeration guard is not an empty or
	// shortened map, and must not replace the last complete cached snapshot.
	if !complete {
		logrus.Warn("pollMap: refresh failed: area enumeration limit reached; retaining last complete map (stale)")
		return
	}

	workingAreas, navAreas, workingIndices := splitMapAreas(allAreas)
	if workingAreas == nil {
		workingAreas = []mowgli.MapArea{}
	}
	if navAreas == nil {
		navAreas = []mowgli.MapArea{}
	}

	mapData := mowgli.Map{
		MapWidth:           20.0,
		MapHeight:          20.0,
		MapCenterX:         0.0,
		MapCenterY:         0.0,
		NavigationAreas:    navAreas,
		WorkingArea:        workingAreas,
		WorkingAreaIndices: workingIndices,
	}
	r.addDockPose(&mapData)

	data, err := json.Marshal(mapData)
	if err != nil {
		return
	}

	// pollMap runs on a fixed 5s ticker regardless of whether anything
	// actually changed, and the payload is the WHOLE map (every area,
	// obstacle and corridor point) — easily >1 MB with a real garden's
	// worth of areas/ignore lines. fanOut() itself has no dedup (by design:
	// other logicalKeys legitimately want every tick delivered even when
	// byte-identical, e.g. high-frequency sensor topics), so skip the
	// broadcast here specifically when nothing changed since the last poll,
	// rather than pushing an unchanged multi-MB payload to every connected
	// browser tab every 5s. Field-reported 2026-09-28: this was the other
	// half of a "map updates twice, always huge" slowdown — see the paired
	// frontend fix (useMapStreams.ts) for the actual duplicate-subscribe
	// half of that report.
	r.mtx.Lock()
	unchanged := bytes.Equal(r.lastMessage["map"], data)
	r.mtx.Unlock()
	if unchanged {
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

	if r.subscribers[topic] == nil {
		r.subscribers[topic] = make(map[string]*RosSubscriber)
	}
	if _, exists := r.subscribers[topic][id]; !exists {
		interval := time.Duration(intervalMs) * time.Millisecond
		r.subscribers[topic][id] = NewRosSubscriber(topic, id, interval, cb)
	}

	// Replay the most recent message so the subscriber is immediately usable.
	if last, ok := r.lastMessage[topic]; ok {
		r.subscribers[topic][id].Publish(last)
	}
	r.mtx.Unlock()
	r.reconcileFoxgloveSubscription(topic)
	if topic == "mavrosVehicleInfo" {
		r.startMavrosVehicleInfoPolling()
	}
	return nil
}

// UnSubscribe stops and removes the subscriber identified by (topic, id).
// When the last subscriber for a logical key is removed, the upstream
// foxglove_bridge subscription is dropped too.
func (r *RosProvider) UnSubscribe(topic string, id string) {
	r.mtx.Lock()

	subs, ok := r.subscribers[topic]
	if !ok {
		r.mtx.Unlock()
		return
	}
	sub, exists := subs[id]
	if !exists {
		r.mtx.Unlock()
		return
	}
	sub.Close()
	delete(subs, id)
	if len(subs) == 0 {
		delete(r.subscribers, topic)
		if topic == "mavrosVehicleInfo" && r.mavrosVehicleInfoPollCancel != nil {
			r.mavrosVehicleInfoPollCancel()
			r.mavrosVehicleInfoPollCancel = nil
		}
		if def, ok := topicMap[topic]; ok && def.MsgType != "" {
			// Live telemetry is stale once the last listener leaves. Virtual
			// topics keep their existing internal polling/cache semantics.
			delete(r.lastMessage, topic)
		}
	}
	r.mtx.Unlock()
	r.reconcileFoxgloveSubscription(topic)
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

// GetParameters lists ROS2 parameters via the foxglove bridge.
func (r *RosProvider) GetParameters(ctx context.Context, names []string) ([]types2.RosParameter, error) {
	params, err := r.client.GetParameters(ctx, names)
	if err != nil {
		return nil, err
	}
	return fromFoxgloveParams(params), nil
}

// SetParameters updates ROS2 parameters live via the foxglove bridge.
func (r *RosProvider) SetParameters(ctx context.Context, params []types2.RosParameter) ([]types2.RosParameter, error) {
	echoed, err := r.client.SetParameters(ctx, toFoxgloveParams(params))
	if err != nil {
		return nil, err
	}
	return fromFoxgloveParams(echoed), nil
}

func fromFoxgloveParams(in []foxglove.Parameter) []types2.RosParameter {
	out := make([]types2.RosParameter, len(in))
	for i, p := range in {
		out[i] = types2.RosParameter{Name: p.Name, Value: p.Value, Type: p.Type}
	}
	return out
}

func toFoxgloveParams(in []types2.RosParameter) []foxglove.Parameter {
	out := make([]foxglove.Parameter, len(in))
	for i, p := range in {
		out[i] = foxglove.Parameter{Name: p.Name, Value: p.Value, Type: p.Type}
	}
	return out
}
