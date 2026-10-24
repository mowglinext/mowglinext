package api

import (
	"bufio"
	"math"
	"net/http"
	"os"
	"strconv"
	"strings"
	"time"

	"github.com/gin-gonic/gin"
	"github.com/sirupsen/logrus"
	"gopkg.in/yaml.v3"

	"github.com/mowglinext/mowglinext/pkg/types"
)

// Paths to the runtime calibration artefacts. The mowgli_robot.yaml
// path is provided at runtime by the DB provider (key
// system.mower.yamlConfigFile) because it differs between containers
// (gui sees /mowgli_config/mowgli_robot.yaml, ros2 sees
// /ros2_ws/config/mowgli_robot.yaml). The earlier hardcoded constant
// caused this endpoint to silently report Present=false on the gui
// container, which then surfaced as the "dock calibration missing"
// banner in the GUI even when the dock pose was correctly persisted.
const (
	imuCalibrationPath = "/ros2_ws/maps/imu_calibration.txt"
	magCalibrationPath = "/ros2_ws/maps/mag_calibration.yaml"
)

// ---------------------------------------------------------------------------
// Response types
// ---------------------------------------------------------------------------

// DockCalibrationStatus reports the dock pose currently persisted in
// mowgli_robot.yaml. Present is false when the file is missing or all
// three pose values are zero (an uninitialised configuration).
type DockCalibrationStatus struct {
	Present         bool     `json:"present"`
	PoseValuesValid bool     `json:"pose_values_valid"`
	DockPoseX       *float64 `json:"dock_pose_x"`
	DockPoseY       *float64 `json:"dock_pose_y"`
	DockPoseYawRad  *float64 `json:"dock_pose_yaw_rad"`
	DockPoseYawDeg  float64  `json:"dock_pose_yaw_deg,omitempty"`
	Error           string   `json:"error,omitempty"`
}

// ImuCalibrationStatus mirrors the plaintext imu_calibration.txt format
// written by hardware_bridge_node (v1 header).
type ImuCalibrationStatus struct {
	Present         bool    `json:"present"`
	CalibratedAt    string  `json:"calibrated_at,omitempty"`
	SamplesUsed     int     `json:"samples_used,omitempty"`
	AccelBiasX      float64 `json:"accel_bias_x,omitempty"`
	AccelBiasY      float64 `json:"accel_bias_y,omitempty"`
	GyroBiasX       float64 `json:"gyro_bias_x,omitempty"`
	GyroBiasY       float64 `json:"gyro_bias_y,omitempty"`
	GyroBiasZ       float64 `json:"gyro_bias_z,omitempty"`
	ImpliedPitchDeg float64 `json:"implied_pitch_deg,omitempty"`
	ImpliedRollDeg  float64 `json:"implied_roll_deg,omitempty"`
	Error           string  `json:"error,omitempty"`
}

// MagCalibrationStatus mirrors the relevant subset of mag_calibration.yaml.
type MagCalibrationStatus struct {
	Present         bool    `json:"present"`
	CalibratedAt    string  `json:"calibrated_at,omitempty"`
	MagnitudeMeanUT float64 `json:"magnitude_mean_uT,omitempty"`
	MagnitudeStdUT  float64 `json:"magnitude_std_uT,omitempty"`
	SampleCount     int     `json:"sample_count,omitempty"`
	Error           string  `json:"error,omitempty"`
}

// CalibrationStatusResponse is the payload for GET /calibration/status.
type CalibrationStatusResponse struct {
	Dock DockCalibrationStatus `json:"dock"`
	Imu  ImuCalibrationStatus  `json:"imu"`
	Mag  MagCalibrationStatus  `json:"mag"`
}

// ---------------------------------------------------------------------------
// Route registration
// ---------------------------------------------------------------------------

// RegisterCalibrationStatusRoute wires GET /calibration/status into the
// existing calibration group. The DB provider is needed so we can
// resolve the runtime path of mowgli_robot.yaml instead of relying on
// a hardcoded constant (the gui and ros2 containers mount the file at
// different paths).
func registerCalibrationStatusRoute(group *gin.RouterGroup, dbProvider types.IDBProvider) {
	group.GET("/status", getCalibrationStatus(dbProvider))
}

// ---------------------------------------------------------------------------
// GET /calibration/status
// ---------------------------------------------------------------------------

func getCalibrationStatus(dbProvider types.IDBProvider) gin.HandlerFunc {
	return func(c *gin.Context) {
		resp := CalibrationStatusResponse{
			Dock: readDockCalibrationStatus(dbProvider),
			Imu:  readImuCalibrationStatus(),
			Mag:  readMagCalibrationStatus(),
		}
		c.JSON(http.StatusOK, resp)
	}
}

// readDockCalibrationStatus reads dock_pose_x/y/yaw from
// mowgli_robot.yaml — the single source of truth, written by
// calibrate_imu_yaw_node and /map_server_node/set_docking_point.
// Reports {Present: false} when the file is missing or the dock pose
// is still all-zero (uninitialised configuration).
func readDockCalibrationStatus(dbProvider types.IDBProvider) DockCalibrationStatus {
	mowgliRobotYamlPath, err := dbProvider.Get("system.mower.yamlConfigFile")
	if err != nil {
		logrus.Warnf("calibration_status: cannot resolve yaml path from db: %v", err)
		return DockCalibrationStatus{Present: false, Error: "cannot resolve yaml path"}
	}
	data, err := os.ReadFile(string(mowgliRobotYamlPath))
	if err != nil {
		if os.IsNotExist(err) {
			return DockCalibrationStatus{Present: false}
		}
		logrus.Warnf("calibration_status: cannot read %s: %v", mowgliRobotYamlPath, err)
		return DockCalibrationStatus{Present: true, Error: err.Error()}
	}
	var yamlData map[string]interface{}
	if err := yaml.Unmarshal(data, &yamlData); err != nil {
		return DockCalibrationStatus{Present: true, Error: "parse: " + err.Error()}
	}
	x := extractYAMLFloat(yamlData, "dock_pose_x")
	y := extractYAMLFloat(yamlData, "dock_pose_y")
	yawRad := extractYAMLFloat(yamlData, "dock_pose_yaw")
	storedX := dockPoseValue(yamlData, "dock_pose_x")
	storedY := dockPoseValue(yamlData, "dock_pose_y")
	storedYaw := dockPoseValue(yamlData, "dock_pose_yaw")
	valid := storedX != nil && storedY != nil && storedYaw != nil
	yawDeg := yawRad * 180.0 / math.Pi
	if math.IsNaN(yawDeg) || math.IsInf(yawDeg, 0) {
		yawDeg = 0
	} // omit unavailable derived degrees
	return DockCalibrationStatus{
		Present:         x != 0 || y != 0 || yawRad != 0,
		PoseValuesValid: valid,
		DockPoseX:       storedX, DockPoseY: storedY, DockPoseYawRad: storedYaw,
		DockPoseYawDeg: yawDeg,
	}
}

// Numeric presence is independent of the historical all-zero calibration flag.
// Duplicate occurrences are ambiguous, even if traversal happens to find a value.
func dockPoseValue(data map[string]interface{}, key string) *float64 {
	found := 0
	var result *float64
	var visit func(map[string]interface{})
	visit = func(node map[string]interface{}) {
		for name, raw := range node {
			if name == key {
				found++
				var number float64
				valid := true
				switch value := raw.(type) {
				case int:
					number = float64(value)
				case int64:
					number = float64(value)
				case uint64:
					number = float64(value)
				case float64:
					number = value
				default:
					valid = false
				}
				if valid && !math.IsNaN(number) && !math.IsInf(number, 0) {
					result = &number
				}
			}
			if child, ok := raw.(map[string]interface{}); ok {
				visit(child)
			}
		}
	}
	visit(data)
	if found != 1 {
		return nil
	}
	return result
}

// readImuCalibrationStatus parses the v1 plaintext format written by
// hardware_bridge_node. Format:
//
//	# mowgli_imu_calibration_v1
//	<ts_unix> <n_samples>
//	<off_ax> <off_ay> <off_gx> <off_gy> <off_gz>
//	<cov_ax> <cov_ay> <cov_gx> <cov_gy> <cov_gz>
//	<implied_pitch_deg> <implied_roll_deg>
func readImuCalibrationStatus() ImuCalibrationStatus {
	f, err := os.Open(imuCalibrationPath)
	if err != nil {
		if os.IsNotExist(err) {
			return ImuCalibrationStatus{Present: false}
		}
		return ImuCalibrationStatus{Present: true, Error: err.Error()}
	}
	defer f.Close()

	scanner := bufio.NewScanner(f)
	lines := []string{}
	for scanner.Scan() {
		lines = append(lines, strings.TrimSpace(scanner.Text()))
	}
	if err := scanner.Err(); err != nil {
		return ImuCalibrationStatus{Present: true, Error: err.Error()}
	}
	if len(lines) < 4 {
		return ImuCalibrationStatus{Present: true, Error: "unexpected line count"}
	}
	if lines[0] != "# mowgli_imu_calibration_v1" {
		return ImuCalibrationStatus{Present: true, Error: "header mismatch: " + lines[0]}
	}
	status := ImuCalibrationStatus{Present: true}

	// Line 1: <ts_unix> <n_samples>
	if parts := strings.Fields(lines[1]); len(parts) >= 2 {
		if ts, err := strconv.ParseInt(parts[0], 10, 64); err == nil {
			status.CalibratedAt = time.Unix(ts, 0).UTC().Format(time.RFC3339)
		}
		if n, err := strconv.Atoi(parts[1]); err == nil {
			status.SamplesUsed = n
		}
	}
	// Line 2: accel biases (X, Y) + gyro biases (X, Y, Z)
	if parts := strings.Fields(lines[2]); len(parts) >= 5 {
		status.AccelBiasX = parseFloat(parts[0])
		status.AccelBiasY = parseFloat(parts[1])
		status.GyroBiasX = parseFloat(parts[2])
		status.GyroBiasY = parseFloat(parts[3])
		status.GyroBiasZ = parseFloat(parts[4])
	}
	// Line 4 (optional): implied pitch/roll
	if len(lines) >= 5 {
		if parts := strings.Fields(lines[4]); len(parts) >= 2 {
			status.ImpliedPitchDeg = parseFloat(parts[0])
			status.ImpliedRollDeg = parseFloat(parts[1])
		}
	}
	return status
}

// readMagCalibrationStatus loads /ros2_ws/maps/mag_calibration.yaml.
func readMagCalibrationStatus() MagCalibrationStatus {
	data, err := os.ReadFile(magCalibrationPath)
	if err != nil {
		if os.IsNotExist(err) {
			return MagCalibrationStatus{Present: false}
		}
		return MagCalibrationStatus{Present: true, Error: err.Error()}
	}
	var parsed struct {
		MagCalibration struct {
			CalibratedAt    string  `yaml:"calibrated_at"`
			MagnitudeMeanUT float64 `yaml:"magnitude_mean_uT"`
			MagnitudeStdUT  float64 `yaml:"magnitude_std_uT"`
			SampleCount     int     `yaml:"sample_count"`
		} `yaml:"mag_calibration"`
	}
	if err := yaml.Unmarshal(data, &parsed); err != nil {
		return MagCalibrationStatus{Present: true, Error: "parse: " + err.Error()}
	}
	return MagCalibrationStatus{
		Present:         true,
		CalibratedAt:    parsed.MagCalibration.CalibratedAt,
		MagnitudeMeanUT: parsed.MagCalibration.MagnitudeMeanUT,
		MagnitudeStdUT:  parsed.MagCalibration.MagnitudeStdUT,
		SampleCount:     parsed.MagCalibration.SampleCount,
	}
}

func parseFloat(s string) float64 {
	v, _ := strconv.ParseFloat(s, 64)
	return v
}
