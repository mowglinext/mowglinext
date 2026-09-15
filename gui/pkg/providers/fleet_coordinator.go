package providers

import (
	"context"
	"encoding/json"
	"errors"
	"reflect"
	"strconv"
	"sync"
	"time"

	"github.com/mowglinext/mowglinext/pkg/msgs/geometry"
	"github.com/mowglinext/mowglinext/pkg/msgs/mowgli"
	"github.com/mowglinext/mowglinext/pkg/types"
	"github.com/sirupsen/logrus"
)

// FleetCoordinator runs on EVERY robot for its own behavior tree (leaderless):
// it reads the fleet snapshot, decides which areas this robot must leave to
// the others, pushes that to the local BT, publishes the peers' poses into
// the local ROS graph for the costmap, and applies the proximity yield rule.
// docs/MULTI_ROBOT.md § 3b/3c. Inert while fleet.coordination.enabled is off.

const (
	coordinationSettingsKey   = "fleet.coordination"
	coordinationMemoryKey     = "fleet.session.completed"
	coordinatorTick           = 2 * time.Second
	assignmentRefresh         = 30 * time.Second
	fleetPeersTopic           = "/fleet/peers"
	fleetPeersMsgType         = "geometry_msgs/msg/PoseArray"
	setFleetAssignmentService = "/behavior_tree_node/set_fleet_assignment"
	setFleetAssignmentType    = "mowgli_interfaces/srv/SetFleetAssignment"
	highLevelControlService   = "/behavior_tree_node/high_level_control"
	highLevelControlType      = "mowgli_interfaces/srv/HighLevelControl"
	coordinatorCallTimeout    = 5 * time.Second
	hlCommandStart            = 1
	hlCommandStop             = 8
)

// CoordinatorStatus is the live state shown on the Fleet page.
type CoordinatorStatus struct {
	Enabled        bool       `json:"enabled"`
	ExcludedAreas  []uint32   `json:"excluded_areas"`
	PreferredStart int32      `json:"preferred_start"`
	Yielded        bool       `json:"yielded"`
	CompletedAreas []uint32   `json:"completed_areas"`
	LastPushAt     *time.Time `json:"last_push_at,omitempty"`
	LastError      string     `json:"last_error,omitempty"`
}

type FleetCoordinator struct {
	db       types.IDBProvider
	ros      types.IRosProvider
	snapshot func() ([]FleetRobot, error)
	identity func() (RobotIdentity, error)
	now      func() time.Time

	mu         sync.Mutex
	memory     map[uint32]time.Time
	lastPushed *FleetAssignment
	lastPushAt time.Time
	yield      yieldState
	wasEnabled bool
	lastError  string

	stop chan struct{}
	done chan struct{}
}

// NewFleetCoordinator wires the coordinator to the live fleet and starts it.
func NewFleetCoordinator(db types.IDBProvider, ros types.IRosProvider, fleet *FleetProvider) *FleetCoordinator {
	c := newFleetCoordinator(db, ros, fleet.Robots, fleet.Identity, time.Now)
	c.start()
	return c
}

func newFleetCoordinator(db types.IDBProvider, ros types.IRosProvider, snapshot func() ([]FleetRobot, error), identity func() (RobotIdentity, error), now func() time.Time) *FleetCoordinator {
	c := &FleetCoordinator{
		db:       db,
		ros:      ros,
		snapshot: snapshot,
		identity: identity,
		now:      now,
		memory:   map[uint32]time.Time{},
		stop:     make(chan struct{}),
		done:     make(chan struct{}),
	}
	c.memory = c.loadMemory()
	return c
}

func (c *FleetCoordinator) start() {
	go func() {
		defer close(c.done)
		ticker := time.NewTicker(coordinatorTick)
		defer ticker.Stop()
		for {
			select {
			case <-c.stop:
				return
			case <-ticker.C:
				c.tick(c.now())
			}
		}
	}()
}

// Close stops the loop (idempotent).
func (c *FleetCoordinator) Close() {
	select {
	case <-c.stop:
	default:
		close(c.stop)
	}
	<-c.done
}

// Settings returns the persisted coordinator settings (defaults when unset).
func (c *FleetCoordinator) Settings() CoordinatorSettings {
	s := DefaultCoordinatorSettings()
	if raw, err := c.db.Get(coordinationSettingsKey); err == nil && len(raw) > 0 {
		if err := json.Unmarshal(raw, &s); err != nil {
			logrus.Warnf("fleet coordinator: unreadable %s, using defaults: %v", coordinationSettingsKey, err)
			return DefaultCoordinatorSettings()
		}
	}
	return s.Normalize()
}

// SetSettings persists new settings and applies enable/disable immediately.
func (c *FleetCoordinator) SetSettings(s CoordinatorSettings) error {
	s = s.Normalize()
	raw, err := json.Marshal(s)
	if err != nil {
		return err
	}
	if err := c.db.Set(coordinationSettingsKey, raw); err != nil {
		return err
	}
	c.tick(c.now())
	return nil
}

// Status is the live state for the GUI.
func (c *FleetCoordinator) Status() CoordinatorStatus {
	c.mu.Lock()
	defer c.mu.Unlock()
	st := CoordinatorStatus{
		Enabled:        c.wasEnabled,
		ExcludedAreas:  []uint32{},
		PreferredStart: -1,
		Yielded:        c.yield.Yielded,
		CompletedAreas: sortedAreas(c.memory),
		LastError:      c.lastError,
	}
	if c.lastPushed != nil {
		st.ExcludedAreas = append(st.ExcludedAreas, c.lastPushed.Excluded...)
		st.PreferredStart = c.lastPushed.PreferredStart
	}
	if !c.lastPushAt.IsZero() {
		t := c.lastPushAt
		st.LastPushAt = &t
	}
	return st
}

// Reset forgets every completed area of the fleet session. Callers that want
// a true "start fresh" also clear each member's coverage resume (the Fleet
// page does both).
func (c *FleetCoordinator) Reset() error {
	c.mu.Lock()
	c.memory = map[uint32]time.Time{}
	c.lastPushed = nil
	c.mu.Unlock()
	if err := c.saveMemory(map[uint32]time.Time{}); err != nil {
		return err
	}
	c.tick(c.now())
	return nil
}

// tick is one coordinator step; exposed unexported for tests.
func (c *FleetCoordinator) tick(now time.Time) {
	settings := c.Settings()
	if !settings.Enabled {
		c.mu.Lock()
		wasEnabled := c.wasEnabled
		c.wasEnabled = false
		c.yield = yieldState{}
		c.mu.Unlock()
		if wasEnabled {
			// Hand the whole lawn back to the local BT exactly once.
			c.pushAssignment(FleetAssignment{Excluded: []uint32{}, PreferredStart: -1}, "coordination disabled", now)
		}
		return
	}
	rows, err := c.snapshot()
	if err != nil {
		c.setError("fleet snapshot: " + err.Error())
		return
	}
	members := make([]fleetMember, 0, len(rows))
	for _, r := range rows {
		members = append(members, memberFromRobot(r))
	}
	self, peers := splitMembers(members)

	c.mu.Lock()
	c.wasEnabled = true
	memory := updateCompletedMemory(c.memory, members, now, time.Duration(settings.CompletedTTLHours*float64(time.Hour)))
	memoryChanged := !reflect.DeepEqual(memory, c.memory)
	c.memory = memory
	assignment := computeAssignment(self, peers, memory)
	mustPush := c.lastPushed == nil || !assignmentEqual(*c.lastPushed, assignment) ||
		now.Sub(c.lastPushAt) >= assignmentRefresh
	yieldBefore := c.yield
	c.mu.Unlock()

	if memoryChanged {
		if err := c.saveMemory(memory); err != nil {
			c.setError("persist completed areas: " + err.Error())
		}
	}
	if mustPush {
		c.pushAssignment(assignment, "fleet coordinator", now)
	}
	c.publishPeers(peers)

	next, action := decideYield(self, peers, yieldBefore, settings, now)
	c.mu.Lock()
	c.yield = next
	c.mu.Unlock()
	switch action {
	case YieldStop:
		logrus.Info("fleet coordinator: a peer with priority is close — pausing this mower (COMMAND_STOP)")
		c.sendHighLevel(hlCommandStop)
	case YieldStart:
		logrus.Info("fleet coordinator: peers are clear — resuming this mower (COMMAND_START)")
		c.sendHighLevel(hlCommandStart)
	}
}

func (c *FleetCoordinator) pushAssignment(a FleetAssignment, reason string, now time.Time) {
	ctx, cancel := context.WithTimeout(context.Background(), coordinatorCallTimeout)
	defer cancel()
	req := mowgli.SetFleetAssignmentReq{
		ExcludedAreas:       append([]uint32{}, a.Excluded...),
		PreferredStartIndex: a.PreferredStart,
		Reason:              reason,
	}
	var res mowgli.SetFleetAssignmentRes
	if err := c.ros.CallService(ctx, setFleetAssignmentService, &req, &res, setFleetAssignmentType); err != nil {
		c.setError("set_fleet_assignment: " + err.Error())
		return
	}
	if !res.Success {
		c.setError("set_fleet_assignment refused: " + res.Message)
		return
	}
	c.mu.Lock()
	copyA := a
	c.lastPushed = &copyA
	c.lastPushAt = now
	c.lastError = ""
	c.mu.Unlock()
}

// publishPeers republishes every online peer with a fix as a pose in OUR map
// frame; an empty array when alone, so fleet_peer_obstacles.py keeps a fresh,
// empty cloud flowing to the costmap.
func (c *FleetCoordinator) publishPeers(peers []fleetMember) {
	id, err := c.identity()
	if err != nil {
		return
	}
	if id.DatumLat == 0 && id.DatumLon == 0 {
		return // no map frame yet; nothing to project into
	}
	datum := LatLon{Lat: id.DatumLat, Lon: id.DatumLon}
	stamp := c.now()
	msg := poseArrayMsg{
		Header: geometry.Header{
			Stamp:   geometry.Stamp{Sec: uint32(stamp.Unix()), Nanosec: uint32(stamp.Nanosecond())},
			FrameId: "map",
		},
		Poses: []geometry.Pose{},
	}
	for _, p := range peers {
		if p.Pos == nil {
			continue
		}
		x, y := enuFromDatum(datum, *p.Pos)
		msg.Poses = append(msg.Poses, geometry.Pose{
			Position:    geometry.Point{X: x, Y: y, Z: 0},
			Orientation: geometry.Quaternion{W: 1},
		})
	}
	if err := c.ros.Publish(fleetPeersTopic, fleetPeersMsgType, &msg); err != nil {
		c.setError("publish /fleet/peers: " + err.Error())
	}
}

type poseArrayMsg struct {
	Header geometry.Header `json:"header"`
	Poses  []geometry.Pose `json:"poses"`
}

func (c *FleetCoordinator) sendHighLevel(command uint8) {
	ctx, cancel := context.WithTimeout(context.Background(), coordinatorCallTimeout)
	defer cancel()
	req := mowgli.HighLevelControlReq{Command: command}
	if err := c.ros.CallService(ctx, highLevelControlService, &req, &mowgli.HighLevelControlRes{}, highLevelControlType); err != nil {
		c.setError("high_level_control: " + err.Error())
	}
}

func (c *FleetCoordinator) setError(msg string) {
	c.mu.Lock()
	c.lastError = msg
	c.mu.Unlock()
	logrus.Warn("fleet coordinator: " + msg)
}

func (c *FleetCoordinator) loadMemory() map[uint32]time.Time {
	out := map[uint32]time.Time{}
	raw, err := c.db.Get(coordinationMemoryKey)
	if err != nil || len(raw) == 0 {
		return out
	}
	var stored map[string]int64
	if err := json.Unmarshal(raw, &stored); err != nil {
		return out
	}
	for k, unix := range stored {
		area, err := strconv.ParseUint(k, 10, 32)
		if err != nil {
			continue
		}
		out[uint32(area)] = time.Unix(unix, 0)
	}
	return out
}

func (c *FleetCoordinator) saveMemory(memory map[uint32]time.Time) error {
	stored := make(map[string]int64, len(memory))
	for area, seen := range memory {
		stored[strconv.FormatUint(uint64(area), 10)] = seen.Unix()
	}
	raw, err := json.Marshal(stored)
	if err != nil {
		return err
	}
	if c.db == nil {
		return errors.New("no database")
	}
	return c.db.Set(coordinationMemoryKey, raw)
}

func sortedAreas(memory map[uint32]time.Time) []uint32 {
	out := make([]uint32, 0, len(memory))
	for area := range memory {
		out = append(out, area)
	}
	for i := 1; i < len(out); i++ {
		for j := i; j > 0 && out[j-1] > out[j]; j-- {
			out[j-1], out[j] = out[j], out[j-1]
		}
	}
	return out
}
