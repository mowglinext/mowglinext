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
	Position    rawPoint      `json:"position"`
	Orientation rawQuaternion `json:"orientation"`
}

type rawPoseWithCovariance struct {
	Pose       rawPose     `json:"pose"`
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
	Status  int8   `json:"status"`
	Service uint16 `json:"service"`
}

type rawNavSatFix struct {
	Header                 rawHeader       `json:"header"`
	Status                 rawNavSatStatus `json:"status"`
	Latitude               float64         `json:"latitude"`
	Longitude              float64         `json:"longitude"`
	Altitude               float64         `json:"altitude"`
	PositionCovariance     [9]float64      `json:"position_covariance"`
	PositionCovarianceType uint8           `json:"position_covariance_type"`
}

type rawOdometry struct {
	Header       rawHeader              `json:"header"`
	ChildFrameId string                 `json:"child_frame_id"`
	Pose         rawPoseWithCovariance  `json:"pose"`
	Twist        rawTwistWithCovariance `json:"twist"`
}

type rawUniversalGnssStatus struct {
	Stamp                   rawStamp `json:"stamp"`
	FixValid                bool     `json:"fix_valid"`
	FixType                 uint8    `json:"fix_type"`
	RtkMode                 uint8    `json:"rtk_mode"`
	CapabilityFlags         uint32   `json:"capability_flags"`
	ValueFlags              uint32   `json:"value_flags"`
	HorizontalAccuracyM     float32  `json:"horizontal_accuracy_m"`
	VerticalAccuracyM       float32  `json:"vertical_accuracy_m"`
	Hdop                    float32  `json:"hdop"`
	Vdop                    float32  `json:"vdop"`
	SatellitesUsed          uint16   `json:"satellites_used"`
	SatellitesVisible       uint16   `json:"satellites_visible"`
	SatellitesTracked       uint16   `json:"satellites_tracked"`
	MeanCn0DbHz             float32  `json:"mean_cn0_db_hz"`
	MaxCn0DbHz              float32  `json:"max_cn0_db_hz"`
	CorrectionAgeS          float32  `json:"correction_age_s"`
	HeadingDeg              float32  `json:"heading_deg"`
	HeadingAccuracyDeg      float32  `json:"heading_accuracy_deg"`
	DifferentialCorrections bool     `json:"differential_corrections"`
	CorrectionsActive       bool     `json:"corrections_active"`
	DualAntennaHeading      bool     `json:"dual_antenna_heading"`
	InterferenceDetected    bool     `json:"interference_detected"`
	JammingDetected         bool     `json:"jamming_detected"`
}

const (
	mowgliFixTypeNoFix          = 0
	mowgliFixTypeGPSFix         = 1
	mowgliFixTypeRTKFloat       = 2
	mowgliFixTypeRTKFixed       = 3
	mowgliFixTypeDeadReckoning  = 4
	mowgliRtkModeUnknown        = 0
	mowgliRtkModeNone           = 1
	mowgliRtkModeFloat          = 2
	mowgliRtkModeFixed          = 3
	mowgliCapRtkMode            = 1
	mowgliCapHdop               = 2
	mowgliCapVdop               = 4
	mowgliCapHorizontalAccuracy = 8
	mowgliCapVerticalAccuracy   = 16
	mowgliCapHeading            = 32
	mowgliCapHeadingAccuracy    = 64
	mowgliCapSatellitesUsed     = 128
	mowgliCapSatellitesVisible  = 256
	mowgliCapSatellitesTracked  = 512
	mowgliCapDiffCorrections    = 1024
	mowgliCapCorrectionsActive  = 2048
	mowgliCapCorrectionAge      = 4096
	mowgliCapMeanCn0            = 8192
	mowgliCapMaxCn0             = 16384
	mowgliCapDualAntennaStatus  = 32768
	mowgliCapInterferenceStatus = 65536
	mowgliCapJammingStatus      = 131072

	universalFixTypeUnknown        = 0
	universalFixTypeNoFix          = 1
	universalFixTypeFix            = 2
	universalFixTypeRTKFloat       = 3
	universalFixTypeRTKFixed       = 4
	universalFixTypeDeadReckoning  = 5
	universalRtkModeUnknown        = 0
	universalRtkModeNone           = 1
	universalRtkModeFloat          = 2
	universalRtkModeFixed          = 3
	universalCapRtkMode            = 1
	universalCapHorizontalAccuracy = 2
	universalCapVerticalAccuracy   = 4
	universalCapHdop               = 8
	universalCapVdop               = 16
	universalCapSatellitesUsed     = 32
	universalCapSatellitesVisible  = 64
	universalCapSatellitesTracked  = 128
	universalCapMeanCn0            = 256
	universalCapMaxCn0             = 512
	universalCapCorrectionAge      = 1024
	universalCapHeading            = 2048
	universalCapDualAntennaHeading = 4096
	universalCapInterferenceState  = 8192
	universalCapJammingState       = 16384
	universalCapHeadingAccuracy    = 32768
	universalCapDiffCorrections    = 65536
	universalCapCorrectionsActive  = 131072
)

// ---------------------------------------------------------------------------
// NavSatStatus → AbsolutePose Flags mapping (bitmask:
//   FLAG_GPS_RTK=1, FLAG_GPS_RTK_FIXED=2, FLAG_GPS_RTK_FLOAT=4)
//
//	status == 2 (STATUS_GBAS_FIX)  → Flags = 1  (generic GPS fix)
//	status == 1 (STATUS_SBAS_FIX)  → Flags = 1  (generic GPS fix)
//	status == 0 (STATUS_FIX)       → Flags = 1  (generic GPS fix)
//	status == -1 (STATUS_NO_FIX)   → Flags = 0
// ---------------------------------------------------------------------------

func navSatStatusToFlags(status int8) uint16 {
	switch status {
	case 0, 1, 2:
		return 1
	default:
		return 0
	}
}

// ---------------------------------------------------------------------------
// Adapter functions
// ---------------------------------------------------------------------------

// adaptGPS converts a sensor_msgs/NavSatFix payload (snake_case JSON) into
// an mowgli.AbsolutePose JSON payload (snake_case, suitable for the frontend).
// lidarMaxPoints caps how many beams a LaserScan keeps before it is shipped to
// the GUI. The map view downsamples to well under this for display, so sending a
// full ~1000+ beam scan as JSON ~12x/second is wasted bandwidth and parse time
// on both ends. adaptLidar decimates with a uniform stride and scales
// angle_increment by that stride so the kept beams still map to the right angles.
const lidarMaxPoints = 360

// adaptLidar decimates a sensor_msgs/LaserScan payload (snake_case JSON) in
// place. It forwards the payload unchanged when decimation does not apply (parse
// failure, missing/short ranges) so a frame is never dropped by this transform.
func adaptLidar(raw []byte) ([]byte, error) {
	var msg map[string]any
	if err := json.Unmarshal(raw, &msg); err != nil {
		return raw, nil
	}
	ranges, ok := msg["ranges"].([]any)
	if !ok || len(ranges) <= lidarMaxPoints {
		return raw, nil
	}
	stride := (len(ranges) + lidarMaxPoints - 1) / lidarMaxPoints // ceil division
	if stride < 2 {
		return raw, nil
	}

	decimate := func(arr []any) []any {
		out := make([]any, 0, len(arr)/stride+1)
		for i := 0; i < len(arr); i += stride {
			out = append(out, arr[i])
		}
		return out
	}

	msg["ranges"] = decimate(ranges)
	if intensities, ok := msg["intensities"].([]any); ok && len(intensities) == len(ranges) {
		msg["intensities"] = decimate(intensities)
	}
	if inc, ok := msg["angle_increment"].(float64); ok {
		msg["angle_increment"] = inc * float64(stride)
	}
	return json.Marshal(msg)
}

func adaptGPS(raw []byte) ([]byte, error) {
	var fix rawNavSatFix
	if err := json.Unmarshal(raw, &fix); err != nil {
		return nil, err
	}

	// Derive position accuracy from the first diagonal element of the 3×3
	// position covariance matrix (row-major).
	accuracy := float32(0)
	if cov := fix.PositionCovariance[0]; cov > 0 && !math.IsNaN(cov) && !math.IsInf(cov, 0) {
		accuracy = float32(math.Sqrt(cov))
	}

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

func adaptGnssStatus(raw []byte) ([]byte, error) {
	var envelope map[string]json.RawMessage
	if err := json.Unmarshal(raw, &envelope); err != nil {
		return nil, err
	}

	if _, ok := envelope["header"]; ok {
		return raw, nil
	}
	if _, ok := envelope["stamp"]; !ok {
		return raw, nil
	}

	var status rawUniversalGnssStatus
	if err := json.Unmarshal(raw, &status); err != nil {
		return nil, err
	}

	fixType := mapUniversalGnssFixType(status.FixType)
	adapted := mowgli.GnssStatus{
		Header: geometry.Header{
			Stamp: geometry.Stamp{
				Sec:     status.Stamp.Sec,
				Nanosec: status.Stamp.Nanosec,
			},
		},
		Backend:                 "universal",
		FixType:                 fixType,
		FixValid:                status.FixValid,
		DeadReckoning:           fixType == mowgliFixTypeDeadReckoning,
		RtkMode:                 mapUniversalGnssRtkMode(status.RtkMode),
		QualityPercent:          qualityPercentForFixType(fixType),
		CapabilityFlags:         mapUniversalGnssCapabilityFlags(status.CapabilityFlags),
		ValueFlags:              mapUniversalGnssCapabilityFlags(status.ValueFlags),
		Hdop:                    status.Hdop,
		Vdop:                    status.Vdop,
		HorizontalAccuracyM:     status.HorizontalAccuracyM,
		VerticalAccuracyM:       status.VerticalAccuracyM,
		HeadingDeg:              status.HeadingDeg,
		HeadingAccuracyDeg:      status.HeadingAccuracyDeg,
		DifferentialCorrections: status.DifferentialCorrections,
		CorrectionsActive:       status.CorrectionsActive,
		SatellitesUsed:          status.SatellitesUsed,
		SatellitesVisible:       status.SatellitesVisible,
		SatellitesTracked:       status.SatellitesTracked,
		CorrectionAgeS:          status.CorrectionAgeS,
		MeanCn0DbHz:             status.MeanCn0DbHz,
		MaxCn0DbHz:              status.MaxCn0DbHz,
		DualAntennaHeading:      status.DualAntennaHeading,
		InterferenceDetected:    status.InterferenceDetected,
		JammingDetected:         status.JammingDetected,
	}

	return json.Marshal(adapted)
}

func mapUniversalGnssFixType(fixType uint8) uint8 {
	switch fixType {
	case universalFixTypeFix:
		return mowgliFixTypeGPSFix
	case universalFixTypeRTKFloat:
		return mowgliFixTypeRTKFloat
	case universalFixTypeRTKFixed:
		return mowgliFixTypeRTKFixed
	case universalFixTypeDeadReckoning:
		return mowgliFixTypeDeadReckoning
	case universalFixTypeUnknown, universalFixTypeNoFix:
		fallthrough
	default:
		return mowgliFixTypeNoFix
	}
}

func mapUniversalGnssRtkMode(rtkMode uint8) uint8 {
	switch rtkMode {
	case universalRtkModeNone:
		return mowgliRtkModeNone
	case universalRtkModeFloat:
		return mowgliRtkModeFloat
	case universalRtkModeFixed:
		return mowgliRtkModeFixed
	case universalRtkModeUnknown:
		fallthrough
	default:
		return mowgliRtkModeUnknown
	}
}

func qualityPercentForFixType(fixType uint8) float32 {
	switch fixType {
	case mowgliFixTypeRTKFixed:
		return 100.0
	case mowgliFixTypeRTKFloat:
		return 50.0
	case mowgliFixTypeGPSFix:
		return 25.0
	case mowgliFixTypeDeadReckoning:
		return 10.0
	default:
		return 0.0
	}
}

func mapUniversalGnssCapabilityFlags(flags uint32) uint32 {
	var mapped uint32
	if flags&universalCapRtkMode != 0 {
		mapped |= mowgliCapRtkMode
	}
	if flags&universalCapHorizontalAccuracy != 0 {
		mapped |= mowgliCapHorizontalAccuracy
	}
	if flags&universalCapVerticalAccuracy != 0 {
		mapped |= mowgliCapVerticalAccuracy
	}
	if flags&universalCapHdop != 0 {
		mapped |= mowgliCapHdop
	}
	if flags&universalCapVdop != 0 {
		mapped |= mowgliCapVdop
	}
	if flags&universalCapSatellitesUsed != 0 {
		mapped |= mowgliCapSatellitesUsed
	}
	if flags&universalCapSatellitesVisible != 0 {
		mapped |= mowgliCapSatellitesVisible
	}
	if flags&universalCapSatellitesTracked != 0 {
		mapped |= mowgliCapSatellitesTracked
	}
	if flags&universalCapMeanCn0 != 0 {
		mapped |= mowgliCapMeanCn0
	}
	if flags&universalCapMaxCn0 != 0 {
		mapped |= mowgliCapMaxCn0
	}
	if flags&universalCapCorrectionAge != 0 {
		mapped |= mowgliCapCorrectionAge
	}
	if flags&universalCapHeading != 0 {
		mapped |= mowgliCapHeading
	}
	if flags&universalCapHeadingAccuracy != 0 {
		mapped |= mowgliCapHeadingAccuracy
	}
	if flags&universalCapDiffCorrections != 0 {
		mapped |= mowgliCapDiffCorrections
	}
	if flags&universalCapCorrectionsActive != 0 {
		mapped |= mowgliCapCorrectionsActive
	}
	if flags&universalCapDualAntennaHeading != 0 {
		mapped |= mowgliCapDualAntennaStatus
	}
	if flags&universalCapInterferenceState != 0 {
		mapped |= mowgliCapInterferenceStatus
	}
	if flags&universalCapJammingState != 0 {
		mapped |= mowgliCapJammingStatus
	}
	return mapped
}
