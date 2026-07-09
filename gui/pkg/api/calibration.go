package api

import (
	"context"
	"net/http"
	"time"

	"github.com/mowglinext/mowglinext/pkg/msgs/mowgli"
	"github.com/mowglinext/mowglinext/pkg/types"
	"github.com/gin-gonic/gin"
)

// ---------------------------------------------------------------------------
// Request / Response types
// ---------------------------------------------------------------------------

// CalibrateImuYawRequest is the JSON body for POST /calibration/imu-yaw.
type CalibrateImuYawRequest struct {
	DurationSec float64 `json:"duration_sec"`
}

// CalibrateImuYawResponse mirrors the ROS service response 1:1.
type CalibrateImuYawResponse struct {
	Success                 bool    `json:"success"`
	Message                 string  `json:"message"`
	ImuYawRad               float64 `json:"imu_yaw_rad"`
	ImuYawDeg               float64 `json:"imu_yaw_deg"`
	SamplesUsed             int32   `json:"samples_used"`
	StdDevDeg               float64 `json:"std_dev_deg"`
	ImuPitchRad             float64 `json:"imu_pitch_rad"`
	ImuPitchDeg             float64 `json:"imu_pitch_deg"`
	ImuRollRad              float64 `json:"imu_roll_rad"`
	ImuRollDeg              float64 `json:"imu_roll_deg"`
	StationarySamplesUsed   int32   `json:"stationary_samples_used"`
	GravityMagMps2          float64 `json:"gravity_mag_mps2"`
	// Dock fields populated when the service was invoked while
	// charging (see calibrate_imu_yaw_node dock pre-phase). Null-ish
	// when DockValid is false.
	DockValid               bool    `json:"dock_valid"`
	DockPoseX               float64 `json:"dock_pose_x"`
	DockPoseY               float64 `json:"dock_pose_y"`
	DockPoseYawRad          float64 `json:"dock_pose_yaw_rad"`
	DockPoseYawDeg          float64 `json:"dock_pose_yaw_deg"`
	DockYawSigmaDeg         float64 `json:"dock_yaw_sigma_deg"`
	DockUndockDisplacementM float64 `json:"dock_undock_displacement_m"`
}

// ---------------------------------------------------------------------------
// Route registration
// ---------------------------------------------------------------------------

// CalibrationRoutes registers sensor-calibration endpoints.
func CalibrationRoutes(r *gin.RouterGroup, rosProvider types.IRosProvider, dbProvider types.IDBProvider) {
	group := r.Group("/calibration")
	group.POST("/imu-yaw", postCalibrateImuYaw(rosProvider))
	group.POST("/magnetometer", postCalibrateMagnetometer(rosProvider))
	registerCalibrationStatusRoute(group, dbProvider)
}

// ---------------------------------------------------------------------------
// POST /calibration/imu-yaw
// ---------------------------------------------------------------------------

// postCalibrateImuYaw forwards the request to the ROS service
// `/calibrate_imu_yaw_node/calibrate`. The service blocks for the whole
// calibration routine, which now includes:
//   - up to 25 s for the dock-undock 2 m reverse + RTK settle (when
//     the robot is on the dock)
//   - ~20 s for the 3 forward-backward cycles (imu_yaw / pitch / roll)
//   - optional 30 s mag rotation (disabled by default)
// We therefore budget 150 s total to leave headroom over the ROS-side
// maximum (~75 s), independent of the client-provided duration_sec
// (which is kept in the .srv but no longer drives the motion — it's
// the node's motion profile that dictates timing now).
func postCalibrateImuYaw(rosProvider types.IRosProvider) gin.HandlerFunc {
	return func(c *gin.Context) {
		var body CalibrateImuYawRequest
		if err := c.BindJSON(&body); err != nil {
			c.JSON(http.StatusBadRequest, ErrorResponse{Error: "Invalid request body: " + err.Error()})
			return
		}

		timeout := 150 * time.Second
		ctx, cancel := context.WithTimeout(c.Request.Context(), timeout)
		defer cancel()

		req := mowgli.CalibrateImuYawReq{DurationSec: body.DurationSec}
		var res mowgli.CalibrateImuYawRes
		if err := rosProvider.CallService(
			ctx,
			"/calibrate_imu_yaw_node/calibrate",
			&req,
			&res,
			"mowgli_interfaces/srv/CalibrateImuYaw",
		); err != nil {
			c.JSON(http.StatusInternalServerError, ErrorResponse{
				Error: "Failed to call calibration service: " + err.Error(),
			})
			return
		}

		c.JSON(http.StatusOK, CalibrateImuYawResponse{
			Success:                 res.Success,
			Message:                 res.Message,
			ImuYawRad:               res.ImuYawRad,
			ImuYawDeg:               res.ImuYawDeg,
			SamplesUsed:             res.SamplesUsed,
			StdDevDeg:               res.StdDevDeg,
			ImuPitchRad:             res.ImuPitchRad,
			ImuPitchDeg:             res.ImuPitchDeg,
			ImuRollRad:              res.ImuRollRad,
			ImuRollDeg:              res.ImuRollDeg,
			StationarySamplesUsed:   res.StationarySamplesUsed,
			GravityMagMps2:          res.GravityMagMps2,
			DockValid:               res.DockValid,
			DockPoseX:               res.DockPoseX,
			DockPoseY:               res.DockPoseY,
			DockPoseYawRad:          res.DockPoseYawRad,
			DockPoseYawDeg:          res.DockPoseYawDeg,
			DockYawSigmaDeg:         res.DockYawSigmaDeg,
			DockUndockDisplacementM: res.DockUndockDisplacementM,
		})
	}
}

// ---------------------------------------------------------------------------
// POST /calibration/magnetometer
// ---------------------------------------------------------------------------

// CalibrateMagnetometerResponse is a slim mirror of the CalibrateImuYaw
// service response, scoped to the fields a magnetometer-only calibration
// can populate. samples_used here is the magnetometer figure-8 sample
// count (collected in calibrate_imu_yaw_node's mag phase) rather than the
// IMU-yaw motion sample count.
type CalibrateMagnetometerResponse struct {
	Success bool   `json:"success"`
	Message string `json:"message"`
}

// postCalibrateMagnetometer triggers the figure-8 magnetometer
// calibration on `/calibrate_imu_yaw_node/calibrate` with `mag_only=true`.
// The node skips the IMU-yaw forward/back drives and runs only the
// figure-8 rotation phase, fitting hard/soft-iron parameters and writing
// /ros2_ws/maps/mag_calibration.yaml. Operators must keep ~1.5 m clear in
// front and behind the robot — collision_monitor stays armed.
//
// Same 150 s budget as imu-yaw: the figure-8 alone is ~30 s, but the
// service also runs the dock pre-phase if the robot starts on the dock.
func postCalibrateMagnetometer(rosProvider types.IRosProvider) gin.HandlerFunc {
	return func(c *gin.Context) {
		timeout := 150 * time.Second
		ctx, cancel := context.WithTimeout(c.Request.Context(), timeout)
		defer cancel()

		req := mowgli.CalibrateImuYawReq{MagOnly: true}
		var res mowgli.CalibrateImuYawRes
		if err := rosProvider.CallService(
			ctx,
			"/calibrate_imu_yaw_node/calibrate",
			&req,
			&res,
			"mowgli_interfaces/srv/CalibrateImuYaw",
		); err != nil {
			c.JSON(http.StatusInternalServerError, ErrorResponse{
				Error: "Failed to call magnetometer calibration service: " + err.Error(),
			})
			return
		}

		c.JSON(http.StatusOK, CalibrateMagnetometerResponse{
			Success: res.Success,
			Message: res.Message,
		})
	}
}
