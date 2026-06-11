package providers

import (
	"encoding/json"
	"testing"

	"github.com/cedbossneo/mowglinext/pkg/msgs/mowgli"
	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
)

func TestAdaptGPSMapsNavSatFixToAbsolutePose(t *testing.T) {
	raw := []byte(`{
		"header":{"stamp":{"sec":7,"nanosec":8},"frame_id":"gps_link"},
		"status":{"status":0,"service":0},
		"latitude":43.9542,
		"longitude":2.2022,
		"altitude":170.06,
		"position_covariance":[7.5625,0,0,0,7.5625,0,0,0,9.0],
		"position_covariance_type":1
	}`)

	adapted, err := adaptGPS(raw)
	require.NoError(t, err)

	var pose mowgli.AbsolutePose
	require.NoError(t, json.Unmarshal(adapted, &pose))
	assert.Equal(t, uint16(1), pose.Flags)
	assert.Equal(t, float32(2.75), pose.PositionAccuracy)
	assert.Equal(t, 43.9542, pose.Pose.Pose.Position.X)
	assert.Equal(t, 2.2022, pose.Pose.Pose.Position.Y)
	assert.Equal(t, 170.06, pose.Pose.Pose.Position.Z)
}

func TestAdaptGnssStatusPassesThroughLegacyPayload(t *testing.T) {
	raw := []byte(`{
		"header":{"stamp":{"sec":1,"nanosec":2},"frame_id":"gps_link"},
		"backend":"ublox",
		"fix_type":3,
		"fix_valid":true,
		"differential_corrections":false,
		"corrections_active":false,
		"quality_percent":100,
		"capability_flags":10,
		"value_flags":8
	}`)

	adapted, err := adaptGnssStatus(raw)
	require.NoError(t, err)

	var status mowgli.GnssStatus
	require.NoError(t, json.Unmarshal(adapted, &status))
	assert.Equal(t, "ublox", status.Backend)
	assert.True(t, status.FixValid)
	assert.False(t, status.DifferentialCorrections)
	assert.False(t, status.CorrectionsActive)
	assert.Equal(t, uint32(10|mowgliCapDiffCorrections|mowgliCapCorrectionsActive), status.CapabilityFlags)
	assert.Equal(t, uint32(8|mowgliCapDiffCorrections|mowgliCapCorrectionsActive), status.ValueFlags)
}

func TestAdaptGnssStatusMapsUniversalPayloadToMowgliShape(t *testing.T) {
	raw := []byte(`{
		"stamp":{"sec":12,"nanosec":34},
		"fix_valid":true,
		"fix_type":4,
		"rtk_mode":3,
		"capability_flags":32767,
		"value_flags":32767,
		"horizontal_accuracy_m":0.02,
		"vertical_accuracy_m":0.04,
		"hdop":0.8,
		"vdop":1.1,
		"satellites_used":18,
		"satellites_visible":24,
		"satellites_tracked":19,
		"mean_cn0_db_hz":41.5,
		"max_cn0_db_hz":50.0,
		"correction_age_s":0.4,
		"heading_deg":182.3,
		"dual_antenna_heading":true,
		"interference_detected":false,
		"jamming_detected":true
	}`)

	adapted, err := adaptGnssStatus(raw)
	require.NoError(t, err)

	var status mowgli.GnssStatus
	require.NoError(t, json.Unmarshal(adapted, &status))
	assert.Equal(t, uint32(12), status.Header.Stamp.Sec)
	assert.Equal(t, uint32(34), status.Header.Stamp.Nanosec)
	assert.Equal(t, "universal", status.Backend)
	assert.Equal(t, uint8(mowgliFixTypeRTKFixed), status.FixType)
	assert.True(t, status.FixValid)
	assert.False(t, status.DeadReckoning)
	assert.Equal(t, uint8(mowgliRtkModeFixed), status.RtkMode)
	assert.Equal(t, float32(100.0), status.QualityPercent)
	assert.Equal(t, float32(0.02), status.HorizontalAccuracyM)
	assert.Equal(t, float32(0.04), status.VerticalAccuracyM)
	assert.Equal(t, float32(0.8), status.Hdop)
	assert.Equal(t, float32(1.1), status.Vdop)
	assert.Equal(t, uint16(18), status.SatellitesUsed)
	assert.Equal(t, uint16(24), status.SatellitesVisible)
	assert.Equal(t, uint16(19), status.SatellitesTracked)
	assert.Equal(t, float32(41.5), status.MeanCn0DbHz)
	assert.Equal(t, float32(50.0), status.MaxCn0DbHz)
	assert.Equal(t, float32(0.4), status.CorrectionAgeS)
	assert.Equal(t, float32(182.3), status.HeadingDeg)
	assert.True(t, status.DualAntennaHeading)
	assert.False(t, status.InterferenceDetected)
	assert.True(t, status.JammingDetected)
	assert.Equal(t, uint32(
		mowgliCapRtkMode|
			mowgliCapHorizontalAccuracy|
			mowgliCapVerticalAccuracy|
			mowgliCapHdop|
			mowgliCapVdop|
			mowgliCapSatellitesUsed|
			mowgliCapSatellitesVisible|
			mowgliCapSatellitesTracked|
			mowgliCapMeanCn0|
			mowgliCapMaxCn0|
			mowgliCapCorrectionAge|
			mowgliCapHeading|
			mowgliCapDualAntennaStatus|
			mowgliCapInterferenceStatus|
			mowgliCapJammingStatus,
	), status.CapabilityFlags)
	assert.Equal(t, status.CapabilityFlags, status.ValueFlags)
	assert.False(t, status.DifferentialCorrections)
	assert.False(t, status.CorrectionsActive)
}

func TestAdaptGnssStatusKeepsUnsupportedUniversalFieldsUnset(t *testing.T) {
	raw := []byte(`{
		"stamp":{"sec":2,"nanosec":3},
		"fix_valid":false,
		"fix_type":2,
		"rtk_mode":0,
		"capability_flags":1,
		"value_flags":0
	}`)

	adapted, err := adaptGnssStatus(raw)
	require.NoError(t, err)

	var status mowgli.GnssStatus
	require.NoError(t, json.Unmarshal(adapted, &status))
	assert.Equal(t, uint8(mowgliFixTypeGPSFix), status.FixType)
	assert.Equal(t, float32(25.0), status.QualityPercent)
	assert.Equal(t, uint32(mowgliCapRtkMode), status.CapabilityFlags)
	assert.Zero(t, status.ValueFlags)
	assert.Zero(t, status.HeadingAccuracyDeg)
	assert.Zero(t, status.ReceiverVendor)
	assert.Zero(t, status.ReceiverModel)
	assert.Zero(t, status.ReceiverFirmware)
}
