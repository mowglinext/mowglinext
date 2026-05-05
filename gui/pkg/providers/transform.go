package providers

import (
	"encoding/json"
	"math"

	"github.com/cedbossneo/mowglinext/pkg/msgs/geometry"
	"github.com/cedbossneo/mowglinext/pkg/msgs/mowgli"
)

// ---------------------------------------------------------------------------
// Raw input structs – used only for unmarshalling snake_case JSON from CDR
// deserialization. These mirror the geometry/sensor/nav types but carry
// explicit json tags.
// ---------------------------------------------------------------------------

type rawStamp struct {
	Sec     uint32 `json:"sec"`
	Nanosec uint32 `json:"nanosec"`
}

type rawHeader struct {
	Stamp   rawStamp `json:"stamp"`
	FrameId string   `json:"frame_id"`
}

type rawPoint struct {
	X float64 `json:"x"`
	Y float64 `json:"y"`
	Z float64 `json:"z"`
}

type rawQuaternion struct {
	X float64 `json:"x"`
	Y float64 `json:"y"`
	Z float64 `json:"z"`
	W float64 `json:"w"`
}

type rawVector3 struct {
	X float64 `json:"x"`
	Y float64 `json:"y"`
	Z float64 `json:"z"`
}

type rawPose struct {
	Position    rawPoint     `json:"position"`
	Orientation rawQuaternion `json:"orientation"`
}

type rawPoseWithCovariance struct {
	Pose       rawPose    `json:"pose"`
	Covariance [36]float64 `json:"covariance"`
}

type rawTwist struct {
	Linear  rawVector3 `json:"linear"`
	Angular rawVector3 `json:"angular"`
}

type rawTwistWithCovariance struct {
	Twist      rawTwist    `json:"twist"`
	Covariance [36]float64 `json:"covariance"`
}

type rawNavSatStatus struct {
	Status  int8  `json:"status"`
	Service uint16 `json:"service"`
}

type rawNavSatFix struct {
	Header             rawHeader       `json:"header"`
	Status             rawNavSatStatus `json:"status"`
	Latitude           float64         `json:"latitude"`
	Longitude          float64         `json:"longitude"`
	Altitude           float64         `json:"altitude"`
	PositionCovariance [9]float64      `json:"position_covariance"`
	PositionCovarianceType uint8       `json:"position_covariance_type"`
}

type rawOdometry struct {
	Header          rawHeader             `json:"header"`
	ChildFrameId    string                `json:"child_frame_id"`
	Pose            rawPoseWithCovariance `json:"pose"`
	Twist           rawTwistWithCovariance `json:"twist"`
}

// ---------------------------------------------------------------------------
// NavSatStatus → AbsolutePose Flags mapping (bitmask:
//   FLAG_GPS_RTK=1, FLAG_GPS_RTK_FIXED=2, FLAG_GPS_RTK_FLOAT=4)
//
//	status == 2 (STATUS_GBAS_FIX)  → Flags = 3  (RTK | FIXED)
//	status == 1 (STATUS_SBAS_FIX)  → Flags = 5  (RTK | FLOAT)
//	status == 0 (STATUS_FIX)       → Flags = 1  (plain RTK/basic fix bit)
//	status == -1 (STATUS_NO_FIX)   → Flags = 0
// ---------------------------------------------------------------------------

func navSatStatusToFlags(status int8) uint16 {
	switch status {
	case 2:
		return 3
	case 1:
		return 5
	case 0:
		return 1 // plain fix — only the FLAG_GPS_RTK base bit, NOT FIXED
	default:
		return 0
	}
}

// ---------------------------------------------------------------------------
// Adapter functions
// ---------------------------------------------------------------------------

// adaptGPS converts a sensor_msgs/NavSatFix payload (snake_case JSON) into
// an mowgli.AbsolutePose JSON payload (snake_case, suitable for the frontend).
func adaptGPS(raw []byte) ([]byte, error) {
	var fix rawNavSatFix
	if err := json.Unmarshal(raw, &fix); err != nil {
		return nil, err
	}

	// Derive position accuracy from the first diagonal element of the 3×3
	// position covariance matrix (row-major).
	accuracy := float32(math.Sqrt(fix.PositionCovariance[0]))

	pose := mowgli.AbsolutePose{
		Flags:            navSatStatusToFlags(fix.Status.Status),
		PositionAccuracy: accuracy,
		Pose: geometry.PoseWithCovariance{
			Pose: geometry.Pose{
				Position: geometry.Point{
					X: fix.Latitude,
					Y: fix.Longitude,
					Z: fix.Altitude,
				},
			},
		},
	}

	return json.Marshal(pose)
}

// adaptPose converts a nav_msgs/Odometry payload (snake_case JSON) into an
// mowgli.AbsolutePose JSON payload (snake_case via json tags).
// The heading (yaw) is derived from the orientation quaternion.
func adaptPose(raw []byte) ([]byte, error) {
	var odom rawOdometry
	if err := json.Unmarshal(raw, &odom); err != nil {
		return nil, err
	}

	q := odom.Pose.Pose.Orientation
	// Standard yaw extraction from a unit quaternion:
	//   yaw = atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
	heading := math.Atan2(
		2*(q.W*q.Z+q.X*q.Y),
		1-2*(q.Y*q.Y+q.Z*q.Z),
	)

	p := odom.Pose.Pose.Position
	pose := mowgli.AbsolutePose{
		Pose: geometry.PoseWithCovariance{
			Pose: geometry.Pose{
				Position: geometry.Point{
					X: p.X,
					Y: p.Y,
					Z: p.Z,
				},
				Orientation: geometry.Quaternion{
					X: q.X,
					Y: q.Y,
					Z: q.Z,
					W: q.W,
				},
			},
			Covariance: odom.Pose.Covariance,
		},
		MotionHeading: heading,
	}

	return json.Marshal(pose)
}

