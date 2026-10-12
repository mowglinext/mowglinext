package providers

import (
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
	require.Len(t, ros.Publishes, 1)
	assert.Equal(t, fleetPeersTopic, ros.Publishes[0].Topic)
	assert.Equal(t, fleetPeersMsgType, ros.Publishes[0].MsgType)
	assert.Empty(t, ros.Publishes[0].Msg.(*poseArrayMsg).Poses)
	assert.Equal(t, 5.0, st.RateHz)
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
