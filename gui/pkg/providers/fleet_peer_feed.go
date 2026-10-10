package providers

import (
	"encoding/json"
	"math"
	"sync"
	"time"

	"github.com/mowglinext/mowglinext/pkg/msgs/geometry"
	"github.com/sirupsen/logrus"
)

// Peer feed (docs/MULTI_ROBOT.md § 3c): the other fleet members' live poses,
// published into THIS robot's ROS graph on /fleet/peers so the Nav2 costmaps
// and the collision monitor see them as obstacles. It runs whenever at least
// one peer is registered — independently of the coordinated-mowing toggle,
// at 5 Hz, from the peers' FUSED pose (base_footprint, with heading) rather
// than the antenna fix — so two mowers on one lawn never lose sight of each
// other because the operator left coordination off or a fix lagged.

const (
	fleetPeersTopic   = "/fleet/peers"
	fleetPeersMsgType = "geometry_msgs/msg/PoseArray"
	peerFeedPeriod    = 200 * time.Millisecond
	// A mirrored pose older than this is not placed: the peer's socket is up
	// but its localizer went quiet, and a stale footprint would block a lawn
	// cell the robot has long left (the fleet node drops it after 3 s too).
	peerPoseMaxAge = 3 * time.Second
)

// PeerFeedStatus is shown on the Fleet page so the operator can tell whether
// the other mowers actually reach this robot's costmap.
type PeerFeedStatus struct {
	Active         bool       `json:"active"`
	PeersPublished int        `json:"peers_published"`
	RateHz         float64    `json:"rate_hz"`
	Publishes      uint64     `json:"publishes"`
	LastPublishAt  *time.Time `json:"last_publish_at,omitempty"`
	LastError      string     `json:"last_error,omitempty"`
}

// peerMapPose is one peer placed in THIS robot's map frame (metres, x east,
// y north). HasYaw is false when only the antenna fix was fresh: the fleet
// node then marks a disc instead of an oriented footprint.
type peerMapPose struct {
	ID     string
	X      float64
	Y      float64
	Yaw    float64
	HasYaw bool
}

type poseArrayMsg struct {
	Header geometry.Header `json:"header"`
	Poses  []geometry.Pose `json:"poses"`
}

// latLonFromMap is the inverse of enuFromDatum: a map-frame point of a robot
// anchored at `datum` back to WGS84 (same equirectangular math as the
// localizer and the browser's transpose()).
func latLonFromMap(datum LatLon, x, y float64) LatLon {
	return LatLon{
		Lat: datum.Lat + y/metersPerDegreeLat,
		Lon: datum.Lon + x/(metersPerDegreeLat*math.Cos(datum.Lat*math.Pi/180)),
	}
}

// peerMapPoses places every ONLINE peer with a FRESH position into our map
// frame. The fused pose is preferred (it is the body, carries the heading and
// is what the peer itself navigates on); it is re-projected through the
// peer's datum when the peer advertises one, else the peer is assumed to share
// ours (a fleet mowing one map must). The antenna fix is the fallback when the
// fused pose is stale or absent.
func peerMapPoses(rows []FleetRobot, selfDatum LatLon, maxAge time.Duration) []peerMapPose {
	out := []peerMapPose{}
	for _, r := range rows {
		if r.Self || !r.Online {
			continue
		}
		if p, ok := fusedMapPose(r, selfDatum, maxAge); ok {
			out = append(out, p)
			continue
		}
		if p, ok := fixMapPose(r, selfDatum, maxAge); ok {
			out = append(out, p)
		}
	}
	return out
}

func topicFresh(r FleetRobot, key string, maxAge time.Duration) bool {
	age, ok := r.TopicAgeS[key]
	return ok && age >= 0 && age <= maxAge.Seconds()
}

func fusedMapPose(r FleetRobot, selfDatum LatLon, maxAge time.Duration) (peerMapPose, bool) {
	raw, ok := r.Topics["pose"]
	if !ok || !topicFresh(r, "pose", maxAge) {
		return peerMapPose{}, false
	}
	var pose struct {
		Pose struct {
			Pose struct {
				Position struct {
					X float64 `json:"x"`
					Y float64 `json:"y"`
				} `json:"position"`
			} `json:"pose"`
		} `json:"pose"`
		MotionHeading float64 `json:"motion_heading"`
	}
	if err := json.Unmarshal(raw, &pose); err != nil {
		return peerMapPose{}, false
	}
	x, y := pose.Pose.Pose.Position.X, pose.Pose.Pose.Position.Y
	if math.IsNaN(x) || math.IsNaN(y) || math.IsInf(x, 0) || math.IsInf(y, 0) {
		return peerMapPose{}, false
	}
	peerDatum := LatLon{Lat: r.Identity.DatumLat, Lon: r.Identity.DatumLon}
	if (peerDatum.Lat != 0 || peerDatum.Lon != 0) && !sameDatum(peerDatum, selfDatum) {
		x, y = enuFromDatum(selfDatum, latLonFromMap(peerDatum, x, y))
	}
	return peerMapPose{ID: r.Identity.ID, X: x, Y: y, Yaw: pose.MotionHeading, HasYaw: true}, true
}

func fixMapPose(r FleetRobot, selfDatum LatLon, maxAge time.Duration) (peerMapPose, bool) {
	raw, ok := r.Topics["gps"]
	if !ok || !topicFresh(r, "gps", maxAge) {
		return peerMapPose{}, false
	}
	// The "gps" key is the adapted AbsolutePose: latitude in pose.position.x,
	// longitude in pose.position.y (transform.go adaptGPS).
	var gps struct {
		Pose struct {
			Pose struct {
				Position struct {
					X float64 `json:"x"`
					Y float64 `json:"y"`
				} `json:"position"`
			} `json:"pose"`
		} `json:"pose"`
	}
	if err := json.Unmarshal(raw, &gps); err != nil {
		return peerMapPose{}, false
	}
	lat, lon := gps.Pose.Pose.Position.X, gps.Pose.Pose.Position.Y
	if lat == 0 && lon == 0 {
		return peerMapPose{}, false
	}
	x, y := enuFromDatum(selfDatum, LatLon{Lat: lat, Lon: lon})
	return peerMapPose{ID: r.Identity.ID, X: x, Y: y}, true
}

// peerPoseArray encodes the poses for /fleet/peers. A peer without a heading
// gets an all-zero (non-unit) quaternion — the wire-level "no orientation"
// the fleet node recognises, since a real quaternion is never zero.
func peerPoseArray(poses []peerMapPose, stamp time.Time) poseArrayMsg {
	msg := poseArrayMsg{
		Header: geometry.Header{
			Stamp:   geometry.Stamp{Sec: uint32(stamp.Unix()), Nanosec: uint32(stamp.Nanosecond())},
			FrameId: "map",
		},
		Poses: make([]geometry.Pose, 0, len(poses)),
	}
	for _, p := range poses {
		pose := geometry.Pose{Position: geometry.Point{X: p.X, Y: p.Y}}
		if p.HasYaw {
			pose.Orientation = geometry.Quaternion{Z: math.Sin(p.Yaw / 2), W: math.Cos(p.Yaw / 2)}
		}
		msg.Poses = append(msg.Poses, pose)
	}
	return msg
}

// peerFeed is the 5 Hz publisher owned by the FleetProvider.
type peerFeed struct {
	mu     sync.Mutex
	status PeerFeedStatus
	stop   chan struct{}
	done   chan struct{}
}

func newPeerFeed() *peerFeed {
	return &peerFeed{
		status: PeerFeedStatus{RateHz: float64(time.Second) / float64(peerFeedPeriod)},
		stop:   make(chan struct{}),
		done:   make(chan struct{}),
	}
}

func (f *FleetProvider) startPeerFeed() {
	go func() {
		defer close(f.feed.done)
		ticker := time.NewTicker(peerFeedPeriod)
		defer ticker.Stop()
		for {
			select {
			case <-f.feed.stop:
				return
			case <-ticker.C:
				f.publishPeerFeed(f.now())
			}
		}
	}()
}

func (f *FleetProvider) stopPeerFeed() {
	select {
	case <-f.feed.stop:
	default:
		close(f.feed.stop)
	}
	<-f.feed.done
}

// PeerFeedStatus reports the live state of the /fleet/peers publisher.
func (f *FleetProvider) PeerFeedStatus() PeerFeedStatus {
	f.feed.mu.Lock()
	defer f.feed.mu.Unlock()
	return f.feed.status
}

// publishPeerFeed is one feed step: nothing is sent while no peer is
// registered (a lone robot has nothing to avoid), or while our own datum is
// unknown (there is no map frame to place a peer in).
func (f *FleetProvider) publishPeerFeed(now time.Time) {
	if len(f.Peers()) == 0 {
		f.setFeed(func(s *PeerFeedStatus) { s.Active = false; s.PeersPublished = 0 })
		return
	}
	self, err := f.Identity()
	if err != nil {
		f.setFeed(func(s *PeerFeedStatus) { s.Active = false; s.LastError = err.Error() })
		return
	}
	if self.DatumLat == 0 && self.DatumLon == 0 {
		f.setFeed(func(s *PeerFeedStatus) {
			s.Active = false
			s.LastError = "no datum on this robot yet — peers cannot be placed in its map frame"
		})
		return
	}
	rows, err := f.Robots()
	if err != nil {
		f.setFeed(func(s *PeerFeedStatus) { s.LastError = err.Error() })
		return
	}
	poses := peerMapPoses(rows, LatLon{Lat: self.DatumLat, Lon: self.DatumLon}, peerPoseMaxAge)
	msg := peerPoseArray(poses, now)
	if err := f.ros.Publish(fleetPeersTopic, fleetPeersMsgType, &msg); err != nil {
		f.setFeed(func(s *PeerFeedStatus) { s.Active = false; s.LastError = "publish /fleet/peers: " + err.Error() })
		return
	}
	f.setFeed(func(s *PeerFeedStatus) {
		s.Active = true
		s.PeersPublished = len(poses)
		s.Publishes++
		t := now
		s.LastPublishAt = &t
		s.LastError = ""
	})
}

func (f *FleetProvider) setFeed(apply func(*PeerFeedStatus)) {
	f.feed.mu.Lock()
	before := f.feed.status.LastError
	apply(&f.feed.status)
	after := f.feed.status.LastError
	f.feed.mu.Unlock()
	if after != "" && after != before {
		logrus.Warn("fleet peer feed: " + after)
	}
}
