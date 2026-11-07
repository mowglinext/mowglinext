package providers

import (
	"context"
	"encoding/json"
	"math"
	"testing"
	"time"

	"github.com/mowglinext/mowglinext/pkg/types"
	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
)

func poseTopic(x, y, yaw float64) json.RawMessage {
	b, _ := json.Marshal(map[string]any{
		"pose":           map[string]any{"pose": map[string]any{"position": map[string]float64{"x": x, "y": y, "z": 0}}},
		"motion_heading": yaw,
	})
	return b
}

func peerRow(id string, datum LatLon, topics map[string]json.RawMessage, ages map[string]float64) FleetRobot {
	return FleetRobot{
		Identity:  RobotIdentity{ID: id, Name: id, DatumLat: datum.Lat, DatumLon: datum.Lon},
		Online:    true,
		Topics:    topics,
		TopicAgeS: ages,
	}
}

var ourDatum = LatLon{Lat: 48.0, Lon: 2.0}

func TestPeerMapPoses_FusedPoseSharedDatumIsUsedAsIs(t *testing.T) {
	rows := []FleetRobot{
		{Identity: RobotIdentity{ID: "me"}, Self: true, Online: true},
		peerRow("a", ourDatum, map[string]json.RawMessage{"pose": poseTopic(3.0, -4.0, 1.0)}, map[string]float64{"pose": 0.2}),
	}

	poses := peerMapPoses(rows, ourDatum, peerPoseMaxAge)

	require.Len(t, poses, 1)
	assert.Equal(t, "a", poses[0].ID)
	assert.InDelta(t, 3.0, poses[0].X, 1e-9)
	assert.InDelta(t, -4.0, poses[0].Y, 1e-9)
	assert.True(t, poses[0].HasYaw)
	assert.InDelta(t, 1.0, poses[0].Yaw, 1e-9)
}

func TestPeerMapPoses_FusedPoseIsReprojectedAcrossDatums(t *testing.T) {
	// The peer's datum sits 11.13 m north of ours: its origin lands at y≈+11.13 here.
	peerDatum := LatLon{Lat: 48.0001, Lon: 2.0}
	rows := []FleetRobot{peerRow("a", peerDatum, map[string]json.RawMessage{"pose": poseTopic(0, 0, 0)}, map[string]float64{"pose": 0})}

	poses := peerMapPoses(rows, ourDatum, peerPoseMaxAge)

	require.Len(t, poses, 1)
	assert.InDelta(t, 0.0, poses[0].X, 1e-6)
	assert.InDelta(t, 11.13, poses[0].Y, 0.01)
}

func TestPeerMapPoses_UnknownPeerDatumIsAssumedShared(t *testing.T) {
	rows := []FleetRobot{peerRow("a", LatLon{}, map[string]json.RawMessage{"pose": poseTopic(1, 2, 0)}, map[string]float64{"pose": 0})}
	poses := peerMapPoses(rows, ourDatum, peerPoseMaxAge)
	require.Len(t, poses, 1)
	assert.InDelta(t, 1.0, poses[0].X, 1e-9)
	assert.InDelta(t, 2.0, poses[0].Y, 1e-9)
}

func TestPeerMapPoses_StalePoseFallsBackToTheFixWithoutHeading(t *testing.T) {
	rows := []FleetRobot{peerRow("a", ourDatum, map[string]json.RawMessage{
		"pose": poseTopic(1, 1, 0.5),
		"gps":  gpsTopic(48.0001, 2.0),
	}, map[string]float64{"pose": 9.0, "gps": 0.1})}

	poses := peerMapPoses(rows, ourDatum, peerPoseMaxAge)

	require.Len(t, poses, 1)
	assert.False(t, poses[0].HasYaw, "the antenna fix carries no heading")
	assert.InDelta(t, 11.13, poses[0].Y, 0.01)
}

func TestPeerMapPoses_SkipsSelfOfflineStaleAndUnpositioned(t *testing.T) {
	rows := []FleetRobot{
		{Identity: RobotIdentity{ID: "me"}, Self: true, Online: true, Topics: map[string]json.RawMessage{"pose": poseTopic(0, 0, 0)}, TopicAgeS: map[string]float64{"pose": 0}},
		{Identity: RobotIdentity{ID: "off"}, Online: false, Topics: map[string]json.RawMessage{"pose": poseTopic(0, 0, 0)}, TopicAgeS: map[string]float64{"pose": 0}},
		peerRow("stale", ourDatum, map[string]json.RawMessage{"pose": poseTopic(0, 0, 0), "gps": gpsTopic(48, 2.0001)}, map[string]float64{"pose": 5, "gps": 5}),
		peerRow("nofix", ourDatum, map[string]json.RawMessage{"gps": gpsTopic(0, 0)}, map[string]float64{"gps": 0}),
		peerRow("noage", ourDatum, map[string]json.RawMessage{"pose": poseTopic(0, 0, 0)}, nil),
	}
	assert.Empty(t, peerMapPoses(rows, ourDatum, peerPoseMaxAge))
}

func TestPeerPoseArray_EncodesHeadingAndTheNoHeadingMarker(t *testing.T) {
	msg := peerPoseArray([]peerMapPose{
		{ID: "a", X: 1, Y: 2, Yaw: math.Pi / 2, HasYaw: true},
		{ID: "b", X: 3, Y: 4},
	}, time.Unix(100, 5))

	assert.Equal(t, "map", msg.Header.FrameId)
	assert.Equal(t, uint32(100), msg.Header.Stamp.Sec)
	require.Len(t, msg.Poses, 2)
	assert.InDelta(t, math.Sqrt(0.5), msg.Poses[0].Orientation.Z, 1e-9)
	assert.InDelta(t, math.Sqrt(0.5), msg.Poses[0].Orientation.W, 1e-9)
	assert.Equal(t, 0.0, msg.Poses[1].Orientation.W, "no heading → all-zero quaternion, the fleet node's disc marker")
	assert.Equal(t, 0.0, msg.Poses[1].Orientation.Z)
	assert.Equal(t, 3.0, msg.Poses[1].Position.X)
}

func TestLatLonFromMapInvertsEnuFromDatum(t *testing.T) {
	ll := latLonFromMap(ourDatum, 123.4, -56.7)
	x, y := enuFromDatum(ourDatum, ll)
	assert.InDelta(t, 123.4, x, 1e-6)
	assert.InDelta(t, -56.7, y, 1e-6)
}

func TestPeerFeed_PublishesOnlyWithPeersAndADatum(t *testing.T) {
	db := types.NewMockDBProvider()
	writeRobotYaml(t, db, "mowgli:\n  ros__parameters:\n    robot_name: alpha\n    datum_lat: 48.0\n    datum_lon: 2.0\n")
	ros := types.NewMockRosProvider()
	f := NewFleetProvider(db, ros)
	defer f.Close()
	now := time.Now()

	// Alone: nothing to publish, feed idle.
	f.publishPeerFeed(now)
	assert.False(t, f.PeerFeedStatus().Active)
	assert.Empty(t, ros.Publishes)

	// A registered (unreachable) peer: the feed is live and publishes an
	// EMPTY array — the peer is offline, so it must not be placed.
	_, err := f.RegisterPeer("b", "bravo", "127.0.0.1", 1)
	require.NoError(t, err)
	f.publishPeerFeed(now)
	st := f.PeerFeedStatus()
	assert.True(t, st.Active)
	assert.Equal(t, 0, st.PeersPublished)
	assert.Equal(t, uint64(1), st.Publishes)
	require.Len(t, ros.Publishes, 3, "/fleet/peers, its docked subset, then the stations")
	assert.Equal(t, fleetPeersTopic, ros.Publishes[0].Topic)
	assert.Equal(t, fleetPeersMsgType, ros.Publishes[0].MsgType)
	assert.Empty(t, ros.Publishes[0].Msg.(*poseArrayMsg).Poses)
	// The docked list goes out every tick, empty too, so the fleet node
	// learns within one tick that a neighbour left its charger.
	assert.Equal(t, fleetDockedPeersTopic, ros.Publishes[1].Topic)
	assert.Equal(t, fleetPeersMsgType, ros.Publishes[1].MsgType)
	assert.Empty(t, ros.Publishes[1].Msg.(*poseArrayMsg).Poses)
	assert.Equal(t, 5.0, st.RateHz)
}

func hlsTopic(isCharging bool) json.RawMessage {
	b, _ := json.Marshal(map[string]any{"state": 1, "state_name": "CHARGING", "is_charging": isCharging})
	return b
}

func TestPeerMapPoses_FlagsAPeerOnItsChargerOnlyFromAFreshStatus(t *testing.T) {
	rows := []FleetRobot{
		peerRow("docked", ourDatum,
			map[string]json.RawMessage{"pose": poseTopic(1, 0, 0), "highLevelStatus": hlsTopic(true)},
			map[string]float64{"pose": 0.1, "highLevelStatus": 0.5}),
		peerRow("moving", ourDatum,
			map[string]json.RawMessage{"pose": poseTopic(2, 0, 0), "highLevelStatus": hlsTopic(false)},
			map[string]float64{"pose": 0.1, "highLevelStatus": 0.5}),
		// "Charging" 9 s ago says nothing about now: it may have undocked since.
		peerRow("stale", ourDatum,
			map[string]json.RawMessage{"pose": poseTopic(3, 0, 0), "highLevelStatus": hlsTopic(true)},
			map[string]float64{"pose": 0.1, "highLevelStatus": 9.0}),
		peerRow("nostatus", ourDatum,
			map[string]json.RawMessage{"pose": poseTopic(4, 0, 0)},
			map[string]float64{"pose": 0.1}),
	}

	poses := peerMapPoses(rows, ourDatum, peerPoseMaxAge)

	require.Len(t, poses, 4, "every peer stays on /fleet/peers whatever its charger state")
	onCharger := map[string]bool{}
	for _, p := range poses {
		onCharger[p.ID] = p.OnCharger
	}
	assert.Equal(t, map[string]bool{"docked": true, "moving": false, "stale": false, "nostatus": false}, onCharger)
}

func TestDockedPeerPoses_KeepsOnlyOrientedPeersOnTheirCharger(t *testing.T) {
	in := []peerMapPose{
		{ID: "docked", X: 1, HasYaw: true, OnCharger: true},
		{ID: "moving", X: 2, HasYaw: true},
		{ID: "disc", X: 3, OnCharger: true}, // fix-only fallback: no heading
	}

	out := dockedPeerPoses(in)

	require.Len(t, out, 1)
	assert.Equal(t, "docked", out[0].ID)
	assert.Len(t, in, 3, "the input is not modified")
}

func TestPeerFeed_PublishesTheDockedSubsetWithTheSamePoses(t *testing.T) {
	poses := []peerMapPose{
		{ID: "docked", X: -3.469, Y: 1.5, Yaw: -1.342, HasYaw: true, OnCharger: true},
		{ID: "moving", X: 5, Y: 5, Yaw: 0.3, HasYaw: true},
	}
	now := time.Unix(200, 0)

	all := peerPoseArray(poses, now)
	docked := peerPoseArray(dockedPeerPoses(poses), now)

	require.Len(t, all.Poses, 2)
	require.Len(t, docked.Poses, 1)
	// The fleet node matches the two topics by pose: they must be identical.
	assert.Equal(t, all.Poses[0], docked.Poses[0])
	assert.Equal(t, all.Header, docked.Header)
}

func TestPeerFeed_StandsDownWithoutADatum(t *testing.T) {
	db := types.NewMockDBProvider()
	writeRobotYaml(t, db, "mowgli:\n  ros__parameters:\n    robot_name: alpha\n")
	ros := types.NewMockRosProvider()
	f := NewFleetProvider(db, ros)
	defer f.Close()
	_, err := f.RegisterPeer("b", "bravo", "127.0.0.1", 1)
	require.NoError(t, err)

	f.publishPeerFeed(time.Now())

	st := f.PeerFeedStatus()
	assert.False(t, st.Active)
	assert.Contains(t, st.LastError, "no datum")
	assert.Empty(t, ros.Publishes)
}

func TestReadDockPose_AllZeroMeansNoDock(t *testing.T) {
	db := types.NewMockDBProvider()
	writeRobotYaml(t, db, "mowgli:\n  ros__parameters:\n    dock_pose_x: 0.0\n")
	assert.Nil(t, ReadDockPose(db))
	writeRobotYaml(t, db, "mowgli:\n  ros__parameters:\n    dock_pose_x: 1.5\n    dock_pose_y: -2.0\n    dock_pose_yaw: 0.5\n")
	assert.Equal(t, &RobotDockPose{X: 1.5, Y: -2.0, Yaw: 0.5}, ReadDockPose(db))
}

func TestPeerDockPoses_ReprojectsThroughThePeerDatumAndSkipsUnknown(t *testing.T) {
	self := LatLon{Lat: 48.0, Lon: 2.0}
	peer := LatLon{Lat: 48.001, Lon: 2.0}
	poses := peerDockPoses([]FleetPeer{
		{ID: "none"},
		{ID: "shared", Dock: &RobotDockPose{X: 1, Y: 2, Yaw: 0.5}},
		{ID: "same", Dock: &RobotDockPose{X: 3, Y: 4, Yaw: 1}, DatumLat: self.Lat, DatumLon: self.Lon},
		{ID: "other", Dock: &RobotDockPose{X: 0, Y: 0, Yaw: 0.25}, DatumLat: peer.Lat, DatumLon: peer.Lon},
		{ID: "broken", Dock: &RobotDockPose{X: math.NaN(), Y: 0, Yaw: 0}},
	}, self)
	require.Len(t, poses, 3)
	assert.Equal(t, "shared", poses[0].ID)
	assert.InDelta(t, 1.0, poses[0].X, 1e-9)
	assert.True(t, poses[0].HasYaw)
	assert.InDelta(t, 3.0, poses[1].X, 1e-9)
	wantX, wantY := enuFromDatum(self, peer)
	assert.InDelta(t, wantX, poses[2].X, 1e-6)
	assert.InDelta(t, wantY, poses[2].Y, 1e-6)
	assert.InDelta(t, 0.25, poses[2].Yaw, 1e-9)
}

func TestPeerFeed_StationOutlivesItsOfflinePeerAndIsRetractedWithIt(t *testing.T) {
	db := types.NewMockDBProvider()
	writeRobotYaml(t, db, "mowgli:\n  ros__parameters:\n    robot_name: alpha\n    datum_lat: 48.0\n    datum_lon: 2.0\n")
	ros := types.NewMockRosProvider()
	f := NewFleetProvider(db, ros)
	defer f.Close()
	_, err := f.RegisterPeer("b", "bravo", "127.0.0.1", 1)
	require.NoError(t, err)
	f.mu.Lock()
	entry := f.peers["b"]
	entry.Dock = &RobotDockPose{X: 5, Y: 6, Yaw: 1}
	f.peers["b"] = entry
	f.mu.Unlock()

	// Unreachable peer: not on /fleet/peers, but its station stays on the lawn.
	f.publishPeerFeed(time.Now())
	require.Len(t, ros.Publishes, 3)
	assert.Empty(t, ros.Publishes[0].Msg.(*poseArrayMsg).Poses)
	assert.Equal(t, fleetPeerDocksTopic, ros.Publishes[2].Topic)
	docks := ros.Publishes[2].Msg.(*poseArrayMsg).Poses
	require.Len(t, docks, 1)
	assert.InDelta(t, 5.0, docks[0].Position.X, 1e-9)

	// Peer removed: one empty list, then silence.
	require.NoError(t, f.RemovePeer(context.Background(), "b"))
	before := len(ros.Publishes)
	f.publishPeerFeed(time.Now())
	require.Len(t, ros.Publishes, before+1)
	assert.Equal(t, fleetPeerDocksTopic, ros.Publishes[before].Topic)
	assert.Empty(t, ros.Publishes[before].Msg.(*poseArrayMsg).Poses)
	f.publishPeerFeed(time.Now())
	assert.Len(t, ros.Publishes, before+1)
}
