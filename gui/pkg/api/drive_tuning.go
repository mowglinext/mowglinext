package api

import (
	"bufio"
	"context"
	"errors"
	"fmt"
	"math"
	"net/http"
	"os"
	"path/filepath"
	"strconv"
	"strings"
	"sync"
	"time"

	"github.com/cedbossneo/mowglinext/pkg/types"
	dockertypes "github.com/docker/docker/api/types"
	"github.com/gin-gonic/gin"
	"gopkg.in/yaml.v3"
)

const (
	driveTuningRos2ContainerName = "mowgli-ros2"
	driveTuningContainerDir      = "/ros2_ws/config/drive_tuning"
	driveTuningRobotConfigPath   = "/ros2_ws/config/mowgli_robot.yaml"
	driveTuningBackupFile        = driveTuningContainerDir + "/drive_pid_last_backup.yaml"
	driveTuningYamlHeader        = "# Mowgli Robot Configuration — managed by mowglinext-gui\n# This file is the single source of truth for robot parameters.\n# Changes made here are picked up on container restart.\n\n"
	maxDriveTuningLogBytes       = 128 * 1024
	// Feed-forward/odometry runs need a stable closed-loop wheel baseline so
	// the very first pass does not inherit whatever live gains happen to be
	// loaded in hardware_bridge.
	driveTuningFFDefaultWheelKp            = 0.2
	driveTuningFFDefaultWheelKi            = 0.100
	driveTuningFFDefaultWheelKd            = 0.010
	driveTuningFFDefaultWheelIntegralLimit = 15.0
)

type driveTuningMode string

const (
	driveTuningModeFeedForward driveTuningMode = "ff"
	driveTuningModePID         driveTuningMode = "pid"
)

type driveTuningValidationStatus string

const (
	driveTuningStatusNotValidated driveTuningValidationStatus = "not_validated"
	driveTuningStatusValidated    driveTuningValidationStatus = "validated"
	driveTuningStatusWarning      driveTuningValidationStatus = "warning"
)

type driveTuningRunState string

const (
	driveTuningRunIdle      driveTuningRunState = "idle"
	driveTuningRunRunning   driveTuningRunState = "running"
	driveTuningRunSucceeded driveTuningRunState = "succeeded"
	driveTuningRunWarning   driveTuningRunState = "warning"
	driveTuningRunFailed    driveTuningRunState = "failed"
)

type driveTuningStartRequest struct {
	Apply          bool    `json:"apply"`
	AllowUndock    bool    `json:"allow_undock"`
	UndockDistance float64 `json:"undock_distance_m"`
}

type driveFFCalibrationStartRequest struct {
	driveTuningStartRequest
	DistanceMeters float64 `json:"distance_m"`
	TestSpeedMps   float64 `json:"test_speed_mps"`
	OdomTimeoutS   float64 `json:"odom_timeout_s"`
	Passes         int     `json:"passes"`
	AutoTurn       *bool   `json:"auto_turn"`
	TurnDirection  string  `json:"turn_direction"`
}

type drivePIDTuningStartRequest struct {
	driveTuningStartRequest
	MaxSpeedMps      float64 `json:"max_speed_mps"`
	SegmentDurationS float64 `json:"segment_duration_s"`
	Passes           int     `json:"passes"`
}

type driveTuningRollbackRequest struct {
	Confirm bool `json:"confirm"`
}

type driveTuningStartResponse struct {
	JobID      string `json:"job_id"`
	Mode       string `json:"mode"`
	State      string `json:"state"`
	StartedAt  string `json:"started_at"`
	Apply      bool   `json:"apply"`
	ReportPath string `json:"report_path"`
}

type driveTuningValidationSummary struct {
	Status      driveTuningValidationStatus `json:"status"`
	Message     string                      `json:"message,omitempty"`
	GeneratedAt string                      `json:"generated_at,omitempty"`
	ReportPath  string                      `json:"report_path,omitempty"`
}

type driveTuningLatestReportMeta struct {
	Mode        string `json:"mode"`
	GeneratedAt string `json:"generated_at,omitempty"`
	ReportPath  string `json:"report_path"`
}

type driveTuningJobStatus struct {
	ID         string `json:"id"`
	Mode       string `json:"mode"`
	State      string `json:"state"`
	StartedAt  string `json:"started_at"`
	FinishedAt string `json:"finished_at,omitempty"`
	Apply      bool   `json:"apply"`
	ReportPath string `json:"report_path"`
	ExecID     string `json:"exec_id,omitempty"`
	ExitCode   *int64 `json:"exit_code,omitempty"`
	Error      string `json:"error,omitempty"`
	Logs       string `json:"logs,omitempty"`
}

type driveTuningStatusResponse struct {
	Job          *driveTuningJobStatus        `json:"job,omitempty"`
	FeedForward  driveTuningValidationSummary `json:"feed_forward"`
	PID          driveTuningValidationSummary `json:"pid"`
	LatestReport *driveTuningLatestReportMeta `json:"latest_report,omitempty"`
}

type driveTuningLatestReportResponse struct {
	LatestReport *driveTuningLatestReportMeta `json:"latest_report,omitempty"`
	FeedForward  driveTuningValidationSummary `json:"feed_forward"`
	PID          driveTuningValidationSummary `json:"pid"`
	Parsed       *driveTuningReport           `json:"parsed,omitempty"`
	RawYAML      string                       `json:"raw_yaml,omitempty"`
}

type driveTuningRollbackResponse struct {
	Success      bool               `json:"success"`
	Message      string             `json:"message,omitempty"`
	Restored     map[string]float64 `json:"restored,omitempty"`
	BackupPath   string             `json:"backup_path,omitempty"`
	ReportPath   string             `json:"report_path,omitempty"`
	ExecutionLog string             `json:"execution_log,omitempty"`
}

type driveTuningReport struct {
	GeneratedAt     string                   `json:"generated_at" yaml:"generated_at"`
	Mode            string                   `json:"mode" yaml:"mode"`
	Profile         string                   `json:"profile" yaml:"profile"`
	BackupFile      string                   `json:"backup_file" yaml:"backup_file"`
	CmdTopic        string                   `json:"cmd_topic" yaml:"cmd_topic"`
	CmdVelTopic     string                   `json:"cmd_vel_topic,omitempty" yaml:"cmd_vel_topic"`
	AppliedLive     bool                     `json:"applied_live" yaml:"applied_live"`
	RequestedApply  bool                     `json:"requested_apply" yaml:"requested_apply"`
	DistanceM       float64                  `json:"distance_m" yaml:"distance_m"`
	MaxSpeedMps     float64                  `json:"max_speed_mps" yaml:"max_speed_mps"`
	TestSpeedMps    *float64                 `json:"test_speed_mps,omitempty" yaml:"test_speed_mps"`
	SegmentDuration float64                  `json:"segment_duration_s" yaml:"segment_duration_s"`
	OdomTimeoutS    float64                  `json:"odom_timeout_s,omitempty" yaml:"odom_timeout_s"`
	Passes          int                      `json:"passes" yaml:"passes"`
	AutoTurn        bool                     `json:"auto_turn" yaml:"auto_turn"`
	TurnDirection   string                   `json:"turn_direction" yaml:"turn_direction"`
	RobotMassKg     *float64                 `json:"robot_mass_kg,omitempty" yaml:"robot_mass_kg"`
	InternalTier    string                   `json:"internal_tuning_tier,omitempty" yaml:"internal_tuning_tier"`
	HardwareConfig  string                   `json:"hardware_config_path,omitempty" yaml:"hardware_config_path"`
	Drivetrain      *driveTuningDrivetrain   `json:"drivetrain_diagnostics,omitempty" yaml:"drivetrain_diagnostics"`
	CurrentParams   map[string]float64       `json:"current_params" yaml:"current_params"`
	StartingParams  map[string]float64       `json:"starting_params" yaml:"starting_params"`
	ProposedParams  map[string]float64       `json:"proposed_params" yaml:"proposed_params"`
	FailureMessage  string                   `json:"failure_message,omitempty" yaml:"failure_message"`
	StatusSnapshot  *driveTuningStatusReport `json:"status_snapshot,omitempty" yaml:"status_snapshot"`
	Reasons         []string                 `json:"reasons" yaml:"reasons"`
	Trials          []driveTuningTrialReport `json:"trials" yaml:"trials"`
}

type driveTuningStatusReport struct {
	ActiveEmergency        *bool    `json:"active_emergency,omitempty" yaml:"active_emergency"`
	LatchedEmergency       *bool    `json:"latched_emergency,omitempty" yaml:"latched_emergency"`
	IsCharging             *bool    `json:"is_charging,omitempty" yaml:"is_charging"`
	MowerStatus            *int     `json:"mower_status,omitempty" yaml:"mower_status"`
	EscPower               *bool    `json:"esc_power,omitempty" yaml:"esc_power"`
	WheelTickFactor        *float64 `json:"wheel_tick_factor,omitempty" yaml:"wheel_tick_factor"`
	LastWheelTickTimestamp string   `json:"last_wheel_tick_timestamp,omitempty" yaml:"last_wheel_tick_timestamp"`
}

type driveTuningDrivetrain struct {
	WheelRadiusM                             *float64 `json:"wheel_radius_m,omitempty" yaml:"wheel_radius_m"`
	WheelCircumferenceM                      *float64 `json:"wheel_circumference_m,omitempty" yaml:"wheel_circumference_m"`
	EstimatedWheelRevolutionsPerMeter        *float64 `json:"estimated_wheel_revolutions_per_meter,omitempty" yaml:"estimated_wheel_revolutions_per_meter"`
	EstimatedEncoderCountsPerWheelRevolution *float64 `json:"estimated_encoder_counts_per_wheel_revolution,omitempty" yaml:"estimated_encoder_counts_per_wheel_revolution"`
	ConfiguredTicksPerRevolution             *float64 `json:"configured_ticks_per_revolution,omitempty" yaml:"configured_ticks_per_revolution"`
	Notes                                    []string `json:"notes,omitempty" yaml:"notes"`
}

type driveTuningTrialReport struct {
	Name                        string   `json:"name" yaml:"name"`
	Phase                       string   `json:"phase" yaml:"phase"`
	TargetSpeed                 float64  `json:"target_speed" yaml:"target_speed"`
	MeasuredSpeedMean           float64  `json:"measured_speed_mean" yaml:"measured_speed_mean"`
	Overshoot                   float64  `json:"overshoot" yaml:"overshoot"`
	SettlingTime                *float64 `json:"settling_time,omitempty" yaml:"settling_time"`
	StallDetected               bool     `json:"stall_detected" yaml:"stall_detected"`
	OscillationDetected         bool     `json:"oscillation_detected" yaml:"oscillation_detected"`
	LiveOscillationDetected     bool     `json:"live_oscillation_detected,omitempty" yaml:"live_oscillation_detected"`
	TrialQuality                string   `json:"trial_quality,omitempty" yaml:"trial_quality"`
	IntegralSaturationSuspected bool     `json:"integral_saturation_suspected" yaml:"integral_saturation_suspected"`
	GroundSpeedMean             *float64 `json:"ground_speed_mean,omitempty" yaml:"ground_speed_mean"`
	OdomDistanceM               *float64 `json:"odom_distance_m,omitempty" yaml:"odom_distance_m"`
	RTKDistanceM                *float64 `json:"rtk_distance_m,omitempty" yaml:"rtk_distance_m"`
	RTKAccepted                 bool     `json:"rtk_accepted" yaml:"rtk_accepted"`
	LeftRightTickImbalance      *float64 `json:"left_right_tick_imbalance,omitempty" yaml:"left_right_tick_imbalance"`
	Warnings                    []string `json:"warnings,omitempty" yaml:"warnings"`
	Notes                       []string `json:"notes,omitempty" yaml:"notes"`
}

type driveTuningBackupPayload struct {
	Parameters map[string]float64 `yaml:"parameters"`
}

type driveTuningJob struct {
	mu         sync.Mutex
	id         string
	mode       driveTuningMode
	state      driveTuningRunState
	startedAt  string
	finishedAt string
	apply      bool
	reportPath string
	execID     string
	exitCode   *int64
	errorText  string
	logs       string
}

func (j *driveTuningJob) appendLog(text string) {
	j.mu.Lock()
	defer j.mu.Unlock()
	j.logs += text
	if len(j.logs) > maxDriveTuningLogBytes {
		j.logs = j.logs[len(j.logs)-maxDriveTuningLogBytes:]
	}
}

func (j *driveTuningJob) setExecID(execID string) {
	j.mu.Lock()
	defer j.mu.Unlock()
	j.execID = execID
}

func (j *driveTuningJob) finish(state driveTuningRunState, exitCode *int64, errorText string) {
	j.mu.Lock()
	defer j.mu.Unlock()
	j.state = state
	j.finishedAt = time.Now().UTC().Format(time.RFC3339)
	j.exitCode = exitCode
	j.errorText = errorText
}

func (j *driveTuningJob) snapshot() *driveTuningJobStatus {
	j.mu.Lock()
	defer j.mu.Unlock()
	return &driveTuningJobStatus{
		ID:         j.id,
		Mode:       string(j.mode),
		State:      string(j.state),
		StartedAt:  j.startedAt,
		FinishedAt: j.finishedAt,
		Apply:      j.apply,
		ReportPath: j.reportPath,
		ExecID:     j.execID,
		ExitCode:   j.exitCode,
		Error:      j.errorText,
		Logs:       j.logs,
	}
}

type driveTuningManager struct {
	dbProvider     types.IDBProvider
	dockerProvider types.IDockerProvider
	mu             sync.Mutex
	current        *driveTuningJob
}

func DriveTuningRoutes(r *gin.RouterGroup, dbProvider types.IDBProvider, dockerProvider types.IDockerProvider) {
	manager := &driveTuningManager{
		dbProvider:     dbProvider,
		dockerProvider: dockerProvider,
	}
	group := r.Group("/tools/drive")
	group.POST("/ff-calibration/start", manager.postFeedForwardStart())
	group.POST("/pid-tuning/start", manager.postPIDStart())
	group.POST("/tuning/rollback", manager.postRollback())
	group.GET("/tuning/status", manager.getStatus())
	group.GET("/tuning/report/latest", manager.getLatestReport())
}

func (m *driveTuningManager) postFeedForwardStart() gin.HandlerFunc {
	return func(c *gin.Context) {
		var req driveFFCalibrationStartRequest
		if err := c.ShouldBindJSON(&req); err != nil {
			c.JSON(http.StatusBadRequest, ErrorResponse{Error: err.Error()})
			return
		}
		normalized, err := normalizeFFRequest(req)
		if err != nil {
			c.JSON(http.StatusBadRequest, ErrorResponse{Error: err.Error()})
			return
		}

		commandArgs, reportPath := buildFeedForwardCommand(normalized)
		job, err := m.startJob(driveTuningModeFeedForward, normalized.Apply, reportPath, commandArgs)
		if err != nil {
			status := http.StatusInternalServerError
			if errors.Is(err, errDriveTuningAlreadyRunning) {
				status = http.StatusConflict
			}
			c.JSON(status, ErrorResponse{Error: err.Error()})
			return
		}
		c.JSON(http.StatusAccepted, driveTuningStartResponse{
			JobID:      job.id,
			Mode:       string(job.mode),
			State:      string(driveTuningRunRunning),
			StartedAt:  job.startedAt,
			Apply:      job.apply,
			ReportPath: job.reportPath,
		})
	}
}

func (m *driveTuningManager) postPIDStart() gin.HandlerFunc {
	return func(c *gin.Context) {
		var req drivePIDTuningStartRequest
		if err := c.ShouldBindJSON(&req); err != nil {
			c.JSON(http.StatusBadRequest, ErrorResponse{Error: err.Error()})
			return
		}
		normalized, err := normalizePIDRequest(req)
		if err != nil {
			c.JSON(http.StatusBadRequest, ErrorResponse{Error: err.Error()})
			return
		}

		commandArgs, reportPath := buildPIDCommand(normalized)
		job, err := m.startJob(driveTuningModePID, normalized.Apply, reportPath, commandArgs)
		if err != nil {
			status := http.StatusInternalServerError
			if errors.Is(err, errDriveTuningAlreadyRunning) {
				status = http.StatusConflict
			}
			c.JSON(status, ErrorResponse{Error: err.Error()})
			return
		}
		c.JSON(http.StatusAccepted, driveTuningStartResponse{
			JobID:      job.id,
			Mode:       string(job.mode),
			State:      string(driveTuningRunRunning),
			StartedAt:  job.startedAt,
			Apply:      job.apply,
			ReportPath: job.reportPath,
		})
	}
}

func (m *driveTuningManager) postRollback() gin.HandlerFunc {
	return func(c *gin.Context) {
		var req driveTuningRollbackRequest
		if err := c.ShouldBindJSON(&req); err != nil {
			c.JSON(http.StatusBadRequest, ErrorResponse{Error: err.Error()})
			return
		}
		if !req.Confirm {
			c.JSON(http.StatusBadRequest, ErrorResponse{Error: "rollback requires confirm=true"})
			return
		}
		if m.isRunning() {
			c.JSON(http.StatusConflict, ErrorResponse{Error: "a drive tuning job is already running"})
			return
		}

		ctx, cancel := context.WithTimeout(c.Request.Context(), 2*time.Minute)
		defer cancel()

		containerDetails, err := m.getRos2ContainerDetails(ctx)
		if err != nil {
			c.JSON(http.StatusInternalServerError, ErrorResponse{Error: err.Error()})
			return
		}

		command := buildRollbackExecCommand()
		execution, err := m.dockerProvider.ContainerExec(ctx, containerDetails.ID, types.ContainerExecSpec{
			Cmd: command,
			Env: []string{
				"PYTHONUNBUFFERED=1",
				"RCUTILS_COLORIZED_OUTPUT=0",
			},
		})
		if err != nil {
			c.JSON(http.StatusInternalServerError, ErrorResponse{Error: err.Error()})
			return
		}
		if execution.ExitCode != 0 {
			c.JSON(http.StatusInternalServerError, ErrorResponse{Error: strings.TrimSpace(execution.Stdout + execution.Stderr)})
			return
		}

		backup, err := m.readBackup(ctx, containerDetails.ID)
		if err != nil {
			c.JSON(http.StatusInternalServerError, ErrorResponse{Error: err.Error()})
			return
		}
		if len(backup.Parameters) == 0 {
			c.JSON(http.StatusInternalServerError, ErrorResponse{Error: "backup file did not contain drive parameters"})
			return
		}
		if err := persistRobotYamlUpdates(m.dbProvider, floatMapToAnyMap(backup.Parameters)); err != nil {
			c.JSON(http.StatusInternalServerError, ErrorResponse{Error: err.Error()})
			return
		}

		c.JSON(http.StatusOK, driveTuningRollbackResponse{
			Success:      true,
			Message:      "Drive tuning rollback applied live and persisted to mowgli_robot.yaml",
			Restored:     backup.Parameters,
			BackupPath:   driveTuningBackupFile,
			ExecutionLog: execution.Stdout + execution.Stderr,
		})
	}
}

func (m *driveTuningManager) getStatus() gin.HandlerFunc {
	return func(c *gin.Context) {
		ctx, cancel := context.WithTimeout(c.Request.Context(), 5*time.Second)
		defer cancel()
		response, err := m.buildStatusResponse(ctx)
		if err != nil {
			c.JSON(http.StatusInternalServerError, ErrorResponse{Error: err.Error()})
			return
		}
		c.JSON(http.StatusOK, response)
	}
}

func (m *driveTuningManager) getLatestReport() gin.HandlerFunc {
	return func(c *gin.Context) {
		ctx, cancel := context.WithTimeout(c.Request.Context(), 10*time.Second)
		defer cancel()

		response, err := m.buildStatusResponse(ctx)
		if err != nil {
			c.JSON(http.StatusInternalServerError, ErrorResponse{Error: err.Error()})
			return
		}
		if response.LatestReport == nil {
			c.JSON(http.StatusOK, driveTuningLatestReportResponse{
				FeedForward: response.FeedForward,
				PID:         response.PID,
			})
			return
		}

		containerDetails, err := m.getRos2ContainerDetails(ctx)
		if err != nil {
			c.JSON(http.StatusInternalServerError, ErrorResponse{Error: err.Error()})
			return
		}
		raw, report, err := m.readReport(ctx, containerDetails.ID, response.LatestReport.ReportPath)
		if err != nil {
			c.JSON(http.StatusInternalServerError, ErrorResponse{Error: err.Error()})
			return
		}
		c.JSON(http.StatusOK, driveTuningLatestReportResponse{
			LatestReport: response.LatestReport,
			FeedForward:  response.FeedForward,
			PID:          response.PID,
			Parsed:       report,
			RawYAML:      raw,
		})
	}
}

var errDriveTuningAlreadyRunning = errors.New("a drive tuning job is already running")

func (m *driveTuningManager) startJob(mode driveTuningMode, apply bool, reportPath string, commandArgs []string) (*driveTuningJob, error) {
	m.mu.Lock()
	defer m.mu.Unlock()
	if m.current != nil {
		current := m.current.snapshot()
		if current != nil && current.State == string(driveTuningRunRunning) {
			return nil, errDriveTuningAlreadyRunning
		}
	}

	jobID := fmt.Sprintf("%s-%d", mode, time.Now().UTC().Unix())
	job := &driveTuningJob{
		id:         jobID,
		mode:       mode,
		state:      driveTuningRunRunning,
		startedAt:  time.Now().UTC().Format(time.RFC3339),
		apply:      apply,
		reportPath: reportPath,
	}
	m.current = job
	go m.runJob(job, commandArgs)
	return job, nil
}

func (m *driveTuningManager) isRunning() bool {
	m.mu.Lock()
	defer m.mu.Unlock()
	if m.current == nil {
		return false
	}
	snapshot := m.current.snapshot()
	return snapshot != nil && snapshot.State == string(driveTuningRunRunning)
}

func (m *driveTuningManager) runJob(job *driveTuningJob, commandArgs []string) {
	ctx := context.Background()

	containerDetails, err := m.getRos2ContainerDetails(ctx)
	if err != nil {
		job.finish(driveTuningRunFailed, nil, err.Error())
		return
	}

	handle, err := m.dockerProvider.ContainerExecStart(ctx, containerDetails.ID, types.ContainerExecSpec{
		Cmd: commandArgs,
		Env: []string{
			"PYTHONUNBUFFERED=1",
			"RCUTILS_COLORIZED_OUTPUT=0",
		},
	})
	if err != nil {
		job.finish(driveTuningRunFailed, nil, err.Error())
		return
	}
	defer handle.Reader.Close()

	job.setExecID(handle.ExecID)
	scanner := bufio.NewScanner(handle.Reader)
	scanner.Buffer(make([]byte, 0, 64*1024), 1024*1024)
	for scanner.Scan() {
		job.appendLog(scanner.Text() + "\n")
	}
	if err := scanner.Err(); err != nil {
		job.appendLog("log stream error: " + err.Error() + "\n")
	}

	execInspect, err := waitForContainerExec(ctx, m.dockerProvider, handle.ExecID)
	if err != nil {
		job.finish(driveTuningRunFailed, nil, err.Error())
		return
	}
	exitCode := execInspect.ExitCode

	if exitCode != 0 {
		job.finish(driveTuningRunFailed, &exitCode, "drive tuning command failed")
		return
	}

	rawReport, report, err := m.readReport(ctx, containerDetails.ID, job.reportPath)
	if err != nil {
		job.finish(driveTuningRunWarning, &exitCode, fmt.Sprintf("drive tuning finished, but the report could not be read: %v", err))
		return
	}
	job.appendLog("\nSaved report: " + job.reportPath + "\n")

	if job.apply {
		if err := persistRobotYamlUpdates(m.dbProvider, floatMapToAnyMap(report.ProposedParams)); err != nil {
			job.appendLog("\nYAML persistence error: " + err.Error() + "\n")
			job.finish(driveTuningRunWarning, &exitCode, "drive tuning applied live, but mowgli_robot.yaml persistence failed")
			return
		}
		job.appendLog("\nPersisted proposed drive parameters to mowgli_robot.yaml\n")
	}

	job.appendLog("\nReport bytes: " + strconv.Itoa(len(rawReport)) + "\n")
	job.finish(driveTuningRunSucceeded, &exitCode, "")
}

func (m *driveTuningManager) buildStatusResponse(ctx context.Context) (driveTuningStatusResponse, error) {
	var jobStatus *driveTuningJobStatus
	m.mu.Lock()
	if m.current != nil {
		jobStatus = m.current.snapshot()
	}
	m.mu.Unlock()

	containerDetails, err := m.getRos2ContainerDetails(ctx)
	if err != nil {
		return driveTuningStatusResponse{Job: jobStatus}, err
	}

	reportPaths, err := listDriveReportPaths(ctx, m.dockerProvider, containerDetails.ID)
	if err != nil {
		return driveTuningStatusResponse{Job: jobStatus}, err
	}

	ffSummary := driveTuningValidationSummary{Status: driveTuningStatusNotValidated}
	pidSummary := driveTuningValidationSummary{Status: driveTuningStatusNotValidated}
	var latestMeta *driveTuningLatestReportMeta

	for _, reportPath := range reportPaths {
		raw, report, err := m.readReport(ctx, containerDetails.ID, reportPath)
		if err != nil {
			continue
		}
		_ = raw
		meta := &driveTuningLatestReportMeta{
			Mode:        report.Mode,
			GeneratedAt: report.GeneratedAt,
			ReportPath:  reportPath,
		}
		if latestMeta == nil {
			latestMeta = meta
		}
		switch driveTuningMode(report.Mode) {
		case driveTuningModeFeedForward:
			if ffSummary.Status == driveTuningStatusNotValidated {
				ffSummary = evaluateFeedForwardReport(report, reportPath)
			}
		case driveTuningModePID:
			if pidSummary.Status == driveTuningStatusNotValidated {
				pidSummary = evaluatePIDReport(report, reportPath)
			}
		}
		if ffSummary.Status != driveTuningStatusNotValidated && pidSummary.Status != driveTuningStatusNotValidated && latestMeta != nil {
			break
		}
	}

	return driveTuningStatusResponse{
		Job:          jobStatus,
		FeedForward:  ffSummary,
		PID:          pidSummary,
		LatestReport: latestMeta,
	}, nil
}

func (m *driveTuningManager) getRos2ContainerDetails(ctx context.Context) (types.ContainerDetails, error) {
	containers, err := m.dockerProvider.ContainerList(ctx)
	if err != nil {
		return types.ContainerDetails{}, err
	}
	for _, container := range containers {
		if containerNameMatches(container, driveTuningRos2ContainerName) {
			return m.dockerProvider.ContainerInspect(ctx, container.ID)
		}
	}
	return types.ContainerDetails{}, fmt.Errorf("%s container not found", driveTuningRos2ContainerName)
}

func (m *driveTuningManager) readReport(ctx context.Context, containerID string, reportPath string) (string, *driveTuningReport, error) {
	result, err := m.dockerProvider.ContainerExec(ctx, containerID, types.ContainerExecSpec{
		Cmd: []string{"bash", "-lc", "cat " + shellQuote(reportPath)},
	})
	if err != nil {
		return "", nil, err
	}
	if result.ExitCode != 0 {
		return "", nil, fmt.Errorf("failed to read report %s: %s%s", reportPath, result.Stdout, result.Stderr)
	}
	raw := result.Stdout
	var report driveTuningReport
	if err := yaml.Unmarshal([]byte(raw), &report); err != nil {
		return raw, nil, err
	}
	if report.CmdVelTopic == "" {
		report.CmdVelTopic = report.CmdTopic
	}
	sanitizeDriveTuningReport(&report)
	return raw, &report, nil
}

func (m *driveTuningManager) readBackup(ctx context.Context, containerID string) (*driveTuningBackupPayload, error) {
	result, err := m.dockerProvider.ContainerExec(ctx, containerID, types.ContainerExecSpec{
		Cmd: []string{"bash", "-lc", "cat " + shellQuote(driveTuningBackupFile)},
	})
	if err != nil {
		return nil, err
	}
	if result.ExitCode != 0 {
		return nil, fmt.Errorf("failed to read backup %s: %s%s", driveTuningBackupFile, result.Stdout, result.Stderr)
	}
	var backup driveTuningBackupPayload
	if err := yaml.Unmarshal([]byte(result.Stdout), &backup); err != nil {
		return nil, err
	}
	return &backup, nil
}

func normalizeFFRequest(req driveFFCalibrationStartRequest) (driveFFCalibrationStartRequest, error) {
	if req.DistanceMeters == 0 {
		req.DistanceMeters = 3.0
	}
	if req.TestSpeedMps == 0 {
		req.TestSpeedMps = 0.3
	}
	if req.OdomTimeoutS == 0 {
		req.OdomTimeoutS = 4.0
	}
	if req.Passes == 0 {
		req.Passes = 3
	}
	if req.AutoTurn == nil {
		defaultAutoTurn := true
		req.AutoTurn = &defaultAutoTurn
	}
	if strings.TrimSpace(req.TurnDirection) == "" {
		req.TurnDirection = "right"
	}
	if req.UndockDistance == 0 && req.AllowUndock {
		req.UndockDistance = 2.0
	}
	if req.DistanceMeters < 2.0 || req.DistanceMeters > 10.0 {
		return req, errors.New("distance_m must be between 2 and 10")
	}
	if req.TestSpeedMps <= 0 || req.TestSpeedMps > 0.5 {
		return req, errors.New("test_speed_mps must be > 0 and <= 0.5")
	}
	if req.OdomTimeoutS <= 0 {
		return req, errors.New("odom_timeout_s must be > 0")
	}
	if req.Passes < 1 {
		return req, errors.New("passes must be >= 1")
	}
	if req.TurnDirection != "left" && req.TurnDirection != "right" {
		return req, errors.New("turn_direction must be 'left' or 'right'")
	}
	return req, nil
}

func normalizePIDRequest(req drivePIDTuningStartRequest) (drivePIDTuningStartRequest, error) {
	if req.MaxSpeedMps == 0 {
		req.MaxSpeedMps = 0.3
	}
	if req.SegmentDurationS == 0 {
		req.SegmentDurationS = 5.0
	}
	if req.Passes == 0 {
		req.Passes = 3
	}
	if req.UndockDistance == 0 && req.AllowUndock {
		req.UndockDistance = 2.0
	}
	if req.MaxSpeedMps <= 0 || req.MaxSpeedMps > 0.5 {
		return req, errors.New("max_speed_mps must be > 0 and <= 0.5")
	}
	if req.SegmentDurationS < 2.0 {
		return req, errors.New("segment_duration_s must be >= 2.0")
	}
	if req.Passes < 1 {
		return req, errors.New("passes must be >= 1")
	}
	return req, nil
}

func buildFeedForwardCommand(req driveFFCalibrationStartRequest) ([]string, string) {
	reportPath := driveReportPath(driveTuningModeFeedForward)
	args := []string{
		"--mode", "ff",
		"--profile", "custom",
		"--hardware-config", driveTuningRobotConfigPath,
		"--cmd-topic", "/cmd_vel_tuning",
		"--max-speed", formatFloat(req.TestSpeedMps),
		"--test-speed", formatFloat(req.TestSpeedMps),
		"--distance", formatFloat(req.DistanceMeters),
		"--passes", strconv.Itoa(req.Passes),
		"--odom-timeout", formatFloat(req.OdomTimeoutS),
		"--turn-direction", req.TurnDirection,
		"--output", reportPath,
		"--backup-file", driveTuningBackupFile,
		"--custom-kp", formatFloat(driveTuningFFDefaultWheelKp),
		"--custom-ki", formatFloat(driveTuningFFDefaultWheelKi),
		"--custom-kd", formatFloat(driveTuningFFDefaultWheelKd),
		"--custom-integral-limit", formatFloat(driveTuningFFDefaultWheelIntegralLimit),
	}
	if req.AutoTurn != nil && *req.AutoTurn {
		args = append(args, "--auto-turn")
	}
	if req.AllowUndock {
		args = append(args, "--undock-distance", formatFloat(req.UndockDistance), "--undock-speed", "0.16")
	}
	if req.Apply {
		args = append(args, "--apply")
	}
	return buildDriveTuningExecCommand(args), reportPath
}

func buildPIDCommand(req drivePIDTuningStartRequest) ([]string, string) {
	reportPath := driveReportPath(driveTuningModePID)
	args := []string{
		"--mode", "pid",
		"--profile", "custom",
		"--hardware-config", driveTuningRobotConfigPath,
		"--cmd-topic", "/cmd_vel_tuning",
		"--max-speed", formatFloat(req.MaxSpeedMps),
		"--duration", formatFloat(req.SegmentDurationS),
		"--passes", strconv.Itoa(req.Passes),
		"--output", reportPath,
		"--backup-file", driveTuningBackupFile,
		"--ramp-time", "1.4",
	}
	if req.AllowUndock {
		args = append(args, "--undock-distance", formatFloat(req.UndockDistance), "--undock-speed", "0.16")
	}
	if req.Apply {
		args = append(args, "--apply")
	}
	return buildDriveTuningExecCommand(args), reportPath
}

func buildRollbackExecCommand() []string {
	args := []string{
		"--rollback",
		"--backup-file", driveTuningBackupFile,
	}
	return buildDriveTuningExecCommand(args)
}

func buildDriveTuningExecCommand(args []string) []string {
	quoted := make([]string, 0, len(args))
	for _, arg := range args {
		quoted = append(quoted, shellQuote(arg))
	}
	script := strings.Join([]string{
		"set -e",
		"mkdir -p " + shellQuote(driveTuningContainerDir),
		"source /opt/ros/*/setup.bash",
		"if [ -f /ros2_ws/install/setup.bash ]; then source /ros2_ws/install/setup.bash; fi",
		"ros2 run mowgli_tools tune_drive_pid -- " + strings.Join(quoted, " "),
	}, " && ")
	return []string{"bash", "-lc", script}
}

func driveReportPath(mode driveTuningMode) string {
	stamp := time.Now().UTC().Format("20060102T150405Z")
	return filepath.ToSlash(filepath.Join(driveTuningContainerDir, fmt.Sprintf("%s_%s.yaml", stamp, mode)))
}

func listDriveReportPaths(ctx context.Context, dockerProvider types.IDockerProvider, containerID string) ([]string, error) {
	command := "if [ -d " + shellQuote(driveTuningContainerDir) + " ]; then " +
		"find " + shellQuote(driveTuningContainerDir) + " -maxdepth 1 -type f -name '*.yaml' ! -name 'drive_pid_last_backup.yaml' -printf '%T@ %p\\n' | sort -nr | cut -d' ' -f2-; " +
		"fi"
	result, err := dockerProvider.ContainerExec(ctx, containerID, types.ContainerExecSpec{
		Cmd: []string{"bash", "-lc", command},
	})
	if err != nil {
		return nil, err
	}
	if result.ExitCode != 0 {
		return nil, fmt.Errorf("failed to list drive tuning reports: %s%s", result.Stdout, result.Stderr)
	}
	lines := strings.Split(strings.TrimSpace(result.Stdout), "\n")
	paths := make([]string, 0, len(lines))
	for _, line := range lines {
		trimmed := strings.TrimSpace(line)
		if trimmed != "" {
			paths = append(paths, trimmed)
		}
	}
	return paths, nil
}

func persistRobotYamlUpdates(dbProvider types.IDBProvider, payload map[string]any) error {
	configFilePath, err := dbProvider.Get("system.mower.yamlConfigFile")
	if err != nil {
		return err
	}

	existingYAML := map[string]any{}
	file, err := os.ReadFile(string(configFilePath))
	if err == nil {
		_ = yaml.Unmarshal(file, &existingYAML)
	}

	existing := flattenROS2YAML(existingYAML)
	schema, err := getSchema(dbProvider)
	nodeMappings := map[string]string{}
	if err == nil {
		defaults := map[string]any{}
		extractDefaults(schema, defaults)
		for key, value := range defaults {
			if _, exists := existing[key]; !exists {
				existing[key] = value
			}
		}
		nodeMappings = extractNodeMappings(schema)
	}

	for key, value := range payload {
		if value == nil {
			delete(existing, key)
		} else {
			existing[key] = value
		}
	}

	nested := nestToROS2YAML(existing, nodeMappings, existingYAML)
	out, err := yaml.Marshal(nested)
	if err != nil {
		return fmt.Errorf("failed to marshal YAML: %w", err)
	}

	if err := os.MkdirAll(filepath.Dir(string(configFilePath)), 0o755); err != nil {
		return err
	}
	return writePreservingPerms(string(configFilePath), []byte(driveTuningYamlHeader+string(out)))
}

func floatMapToAnyMap(values map[string]float64) map[string]any {
	out := make(map[string]any, len(values))
	for key, value := range values {
		out[key] = value
	}
	return out
}

func waitForContainerExec(ctx context.Context, dockerProvider types.IDockerProvider, execID string) (types.ContainerExecInspectResult, error) {
	ticker := time.NewTicker(100 * time.Millisecond)
	defer ticker.Stop()
	for {
		inspect, err := dockerProvider.ContainerExecInspect(ctx, execID)
		if err != nil {
			return types.ContainerExecInspectResult{}, err
		}
		if !inspect.Running {
			return inspect, nil
		}
		select {
		case <-ctx.Done():
			return types.ContainerExecInspectResult{}, ctx.Err()
		case <-ticker.C:
		}
	}
}

func evaluateFeedForwardReport(report *driveTuningReport, reportPath string) driveTuningValidationSummary {
	trials := filterTrialsByPhase(report.Trials, "feedforward")
	if len(trials) == 0 {
		return driveTuningValidationSummary{
			Status:      driveTuningStatusNotValidated,
			Message:     "No feed-forward trials recorded yet.",
			GeneratedAt: report.GeneratedAt,
			ReportPath:  reportPath,
		}
	}
	odomErrors := []float64{}
	speedErrors := []float64{}
	hasCalibrationWarnings := false
	hasLiveOscillation := false
	hasPostAnalysisOscillation := false
	for _, trial := range trials {
		if trial.StallDetected {
			return driveTuningValidationSummary{
				Status:      driveTuningStatusWarning,
				Message:     "A feed-forward pass stalled or produced no wheel motion. Re-run after checking mechanics, traction power, and odometry.",
				GeneratedAt: report.GeneratedAt,
				ReportPath:  reportPath,
			}
		}
		if trial.LiveOscillationDetected {
			hasLiveOscillation = true
		}
		if trial.OscillationDetected && !trial.LiveOscillationDetected {
			hasPostAnalysisOscillation = true
		}
		if trial.LiveOscillationDetected || trial.TrialQuality == "warning" || trial.TrialQuality == "poor" || len(trial.Warnings) > 0 {
			hasCalibrationWarnings = true
		}
		if trial.RTKAccepted && trial.OdomDistanceM != nil && trial.RTKDistanceM != nil && *trial.RTKDistanceM > 1e-6 {
			odomErrors = append(odomErrors, absFloat(*trial.OdomDistanceM-*trial.RTKDistanceM)/absFloat(*trial.RTKDistanceM))
		}
		measured := trial.MeasuredSpeedMean
		if trial.RTKAccepted && trial.GroundSpeedMean != nil && *trial.GroundSpeedMean > 0 {
			measured = *trial.GroundSpeedMean
		}
		if trial.TargetSpeed > 1e-6 {
			speedErrors = append(speedErrors, absFloat(trial.TargetSpeed-measured)/trial.TargetSpeed)
		}
	}
	if hasLiveOscillation {
		return driveTuningValidationSummary{
			Status:      driveTuningStatusWarning,
			Message:     "Live oscillation was observed during feed-forward calibration. Review mechanics and feed-forward before accepting the result.",
			GeneratedAt: report.GeneratedAt,
			ReportPath:  reportPath,
		}
	}
	if len(odomErrors) == 0 {
		return driveTuningValidationSummary{
			Status:      driveTuningStatusWarning,
			Message:     "Feed-forward report exists, but no RTK/GPS-backed odometry validation was accepted.",
			GeneratedAt: report.GeneratedAt,
			ReportPath:  reportPath,
		}
	}
	maxOdomError := maxFloatSlice(odomErrors)
	maxSpeedError := maxFloatSlice(speedErrors)
	status := driveTuningStatusWarning
	message := fmt.Sprintf("Odom error %.1f%%, speed error %.1f%%.", maxOdomError*100.0, maxSpeedError*100.0)
	if maxOdomError <= 0.02 && maxSpeedError <= 0.05 {
		status = driveTuningStatusValidated
		if maxOdomError <= 0.01 {
			message = fmt.Sprintf("Validated with excellent odometry (%.1f%%) and acceptable feed-forward speed error (%.1f%%).", maxOdomError*100.0, maxSpeedError*100.0)
		} else {
			message = fmt.Sprintf("Validated with acceptable odometry (%.1f%%) and feed-forward speed error (%.1f%%).", maxOdomError*100.0, maxSpeedError*100.0)
		}
	}
	if hasPostAnalysisOscillation {
		status = driveTuningStatusWarning
		message = fmt.Sprintf("Feed-forward calibration completed with a post-analysis oscillation warning (odom %.1f%%, speed %.1f%%). Review the report before accepting the result.", maxOdomError*100.0, maxSpeedError*100.0)
	}
	if hasCalibrationWarnings {
		status = driveTuningStatusWarning
		if !hasPostAnalysisOscillation {
			message = fmt.Sprintf("Calibration completed with warnings (odom %.1f%%, speed %.1f%%). Review trial notes before accepting the result.", maxOdomError*100.0, maxSpeedError*100.0)
		}
	}
	return driveTuningValidationSummary{
		Status:      status,
		Message:     message,
		GeneratedAt: report.GeneratedAt,
		ReportPath:  reportPath,
	}
}

func driveTuningInternalTier(report *driveTuningReport) string {
	if report.InternalTier != "" {
		return report.InternalTier
	}
	if report.RobotMassKg == nil {
		return "medium"
	}
	mass := *report.RobotMassKg
	if math.IsNaN(mass) || math.IsInf(mass, 0) || mass <= 0.0 {
		return "medium"
	}
	switch {
	case mass <= 12.0:
		return "lightweight"
	case mass <= 30.0:
		return "medium"
	case mass <= 60.0:
		return "heavy"
	default:
		return "extra-heavy"
	}
}

func pidOvershootThresholds(report *driveTuningReport) (float64, float64) {
	switch driveTuningInternalTier(report) {
	case "light", "lightweight":
		return 0.10, 0.16
	case "heavy":
		return 0.06, 0.10
	case "extra-heavy":
		return 0.05, 0.08
	default:
		return 0.08, 0.14
	}
}

func hasStopBehaviorWarning(trial driveTuningTrialReport) bool {
	if absFloat(trial.TargetSpeed) >= 0.05 {
		return false
	}
	if absFloat(trial.MeasuredSpeedMean) > 0.03 || absFloat(trial.Overshoot) > 0.05 {
		return true
	}
	for _, warning := range trial.Warnings {
		if strings.Contains(warning, "Stop behavior warning:") {
			return true
		}
	}
	return false
}

func hasSevereStopBehavior(trial driveTuningTrialReport) bool {
	if absFloat(trial.TargetSpeed) >= 0.05 {
		return false
	}
	return trial.TrialQuality == "poor" || absFloat(trial.MeasuredSpeedMean) > 0.06 || absFloat(trial.Overshoot) > 0.10
}

func evaluatePIDReport(report *driveTuningReport, reportPath string) driveTuningValidationSummary {
	trials := filterTrialsByPhase(report.Trials, "pid")
	usable := make([]driveTuningTrialReport, 0, len(trials))
	stopTrialCount := 0
	stopBehaviorWarning := false
	severeStopBehavior := false
	hasCalibrationWarnings := false
	for _, trial := range trials {
		if absFloat(trial.TargetSpeed) < 0.05 {
			stopTrialCount++
			stopBehaviorWarning = stopBehaviorWarning || hasStopBehaviorWarning(trial)
			severeStopBehavior = severeStopBehavior || hasSevereStopBehavior(trial)
			continue
		}
		usable = append(usable, trial)
		if trial.TrialQuality == "poor" || len(trial.Warnings) > 0 {
			hasCalibrationWarnings = true
		}
	}
	if severeStopBehavior {
		return driveTuningValidationSummary{
			Status:      driveTuningStatusWarning,
			Message:     "Stop behavior warning: residual motion detected after zero-speed command. Stop trials were excluded from PID gain selection, but braking/control should be reviewed before accepting this tune.",
			GeneratedAt: report.GeneratedAt,
			ReportPath:  reportPath,
		}
	}
	if stopBehaviorWarning && len(usable) == 0 {
		return driveTuningValidationSummary{
			Status:      driveTuningStatusWarning,
			Message:     "Stop behavior warning: residual motion detected after zero-speed command. Stop trials were excluded from PID gain selection.",
			GeneratedAt: report.GeneratedAt,
			ReportPath:  reportPath,
		}
	}
	if len(usable) == 0 {
		return driveTuningValidationSummary{
			Status:      driveTuningStatusNotValidated,
			Message:     "No PID step-response report recorded yet.",
			GeneratedAt: report.GeneratedAt,
			ReportPath:  reportPath,
		}
	}

	acceptableOvershootPct, severeOvershootPct := pidOvershootThresholds(report)
	maxOvershootPct := 0.0
	liveOscillationCount := 0
	postAnalysisOnlyCount := 0
	severeIntegralSaturationCount := 0
	for _, trial := range usable {
		target := absFloat(trial.TargetSpeed)
		if target <= 1e-6 {
			continue
		}
		if trial.StallDetected {
			return driveTuningValidationSummary{
				Status:      driveTuningStatusWarning,
				Message:     "PID report shows a stall or no-motion step. Re-run after checking mechanics and feed-forward.",
				GeneratedAt: report.GeneratedAt,
				ReportPath:  reportPath,
			}
		}
		if trial.SettlingTime == nil || *trial.SettlingTime > 3.0 {
			hasCalibrationWarnings = true
		}
		overshootPct := trial.Overshoot / target
		if overshootPct > maxOvershootPct {
			maxOvershootPct = overshootPct
		}
		if trial.LiveOscillationDetected {
			liveOscillationCount++
		}
		if trial.OscillationDetected && !trial.LiveOscillationDetected {
			postAnalysisOnlyCount++
		}
		if trial.IntegralSaturationSuspected && trial.MeasuredSpeedMean < target*0.90 {
			severeIntegralSaturationCount++
		}
	}
	if liveOscillationCount > 0 {
		return driveTuningValidationSummary{
			Status:      driveTuningStatusWarning,
			Message:     "Live oscillation was observed during PID tuning. Re-run after reducing aggression or reviewing feed-forward.",
			GeneratedAt: report.GeneratedAt,
			ReportPath:  reportPath,
		}
	}
	if maxOvershootPct > severeOvershootPct {
		return driveTuningValidationSummary{
			Status:      driveTuningStatusWarning,
			Message:     fmt.Sprintf("PID overshoot reached %.1f%% on at least one step.", maxOvershootPct*100.0),
			GeneratedAt: report.GeneratedAt,
			ReportPath:  reportPath,
		}
	}
	if severeIntegralSaturationCount > 0 {
		return driveTuningValidationSummary{
			Status:      driveTuningStatusWarning,
			Message:     "PID report suggests sustained integral saturation under load. Review KI and integral limit before accepting the result.",
			GeneratedAt: report.GeneratedAt,
			ReportPath:  reportPath,
		}
	}
	if stopBehaviorWarning {
		return driveTuningValidationSummary{
			Status:      driveTuningStatusWarning,
			Message:     "Stop behavior warning: residual motion detected after zero-speed command. Stop trials were excluded from PID gain selection.",
			GeneratedAt: report.GeneratedAt,
			ReportPath:  reportPath,
		}
	}
	if postAnalysisOnlyCount > 0 && maxOvershootPct <= acceptableOvershootPct {
		return driveTuningValidationSummary{
			Status:      driveTuningStatusWarning,
			Message:     fmt.Sprintf("Post-analysis oscillation detected but no live oscillation was observed; max overshoot %.1f%% remains usable for conservative tuning.", maxOvershootPct*100.0),
			GeneratedAt: report.GeneratedAt,
			ReportPath:  reportPath,
		}
	}
	if hasCalibrationWarnings {
		message := fmt.Sprintf("PID response completed with warnings; max overshoot %.1f%%. Review trial notes before accepting the result.", maxOvershootPct*100.0)
		if stopTrialCount > 0 {
			message += " Zero-speed stop trials were excluded from gain selection."
		}
		return driveTuningValidationSummary{
			Status:      driveTuningStatusWarning,
			Message:     message,
			GeneratedAt: report.GeneratedAt,
			ReportPath:  reportPath,
		}
	}
	message := fmt.Sprintf("Validated PID step response with max overshoot %.1f%%.", maxOvershootPct*100.0)
	if stopTrialCount > 0 {
		message += " Zero-speed stop trials were excluded from gain selection."
	}
	return driveTuningValidationSummary{
		Status:      driveTuningStatusValidated,
		Message:     message,
		GeneratedAt: report.GeneratedAt,
		ReportPath:  reportPath,
	}
}

func filterTrialsByPhase(trials []driveTuningTrialReport, phase string) []driveTuningTrialReport {
	filtered := make([]driveTuningTrialReport, 0, len(trials))
	for _, trial := range trials {
		if trial.Phase == phase {
			filtered = append(filtered, trial)
		}
	}
	return filtered
}

func containerNameMatches(container dockertypes.Container, name string) bool {
	for _, rawName := range container.Names {
		if strings.TrimPrefix(rawName, "/") == name {
			return true
		}
	}
	return false
}

func shellQuote(value string) string {
	return "'" + strings.ReplaceAll(value, "'", "'\"'\"'") + "'"
}

func formatFloat(value float64) string {
	return strconv.FormatFloat(value, 'f', -1, 64)
}

func isFiniteFloat64(value float64) bool {
	return !math.IsNaN(value) && !math.IsInf(value, 0)
}

func sanitizeFloat64Value(value float64) float64 {
	if !isFiniteFloat64(value) {
		return 0.0
	}
	return value
}

func sanitizeFloat64Ptr(value *float64) *float64 {
	if value == nil || !isFiniteFloat64(*value) {
		return nil
	}
	return value
}

func sanitizeFloat64Map(values map[string]float64) map[string]float64 {
	if len(values) == 0 {
		return values
	}
	sanitized := make(map[string]float64, len(values))
	for key, value := range values {
		if isFiniteFloat64(value) {
			sanitized[key] = value
		}
	}
	if len(sanitized) == 0 {
		return nil
	}
	return sanitized
}

func sanitizeDriveTuningDrivetrain(drivetrain *driveTuningDrivetrain) *driveTuningDrivetrain {
	if drivetrain == nil {
		return nil
	}
	drivetrain.WheelRadiusM = sanitizeFloat64Ptr(drivetrain.WheelRadiusM)
	drivetrain.WheelCircumferenceM = sanitizeFloat64Ptr(drivetrain.WheelCircumferenceM)
	drivetrain.EstimatedWheelRevolutionsPerMeter = sanitizeFloat64Ptr(drivetrain.EstimatedWheelRevolutionsPerMeter)
	drivetrain.EstimatedEncoderCountsPerWheelRevolution = sanitizeFloat64Ptr(drivetrain.EstimatedEncoderCountsPerWheelRevolution)
	drivetrain.ConfiguredTicksPerRevolution = sanitizeFloat64Ptr(drivetrain.ConfiguredTicksPerRevolution)
	return drivetrain
}

func sanitizeDriveTuningStatusReport(status *driveTuningStatusReport) *driveTuningStatusReport {
	if status == nil {
		return nil
	}
	status.WheelTickFactor = sanitizeFloat64Ptr(status.WheelTickFactor)
	return status
}

func sanitizeDriveTuningReport(report *driveTuningReport) {
	report.DistanceM = sanitizeFloat64Value(report.DistanceM)
	report.MaxSpeedMps = sanitizeFloat64Value(report.MaxSpeedMps)
	report.SegmentDuration = sanitizeFloat64Value(report.SegmentDuration)
	report.OdomTimeoutS = sanitizeFloat64Value(report.OdomTimeoutS)
	report.TestSpeedMps = sanitizeFloat64Ptr(report.TestSpeedMps)
	report.RobotMassKg = sanitizeFloat64Ptr(report.RobotMassKg)
	report.StatusSnapshot = sanitizeDriveTuningStatusReport(report.StatusSnapshot)
	report.Drivetrain = sanitizeDriveTuningDrivetrain(report.Drivetrain)
	report.CurrentParams = sanitizeFloat64Map(report.CurrentParams)
	report.StartingParams = sanitizeFloat64Map(report.StartingParams)
	report.ProposedParams = sanitizeFloat64Map(report.ProposedParams)
	for index := range report.Trials {
		report.Trials[index].TargetSpeed = sanitizeFloat64Value(report.Trials[index].TargetSpeed)
		report.Trials[index].MeasuredSpeedMean = sanitizeFloat64Value(report.Trials[index].MeasuredSpeedMean)
		report.Trials[index].Overshoot = sanitizeFloat64Value(report.Trials[index].Overshoot)
		report.Trials[index].SettlingTime = sanitizeFloat64Ptr(report.Trials[index].SettlingTime)
		report.Trials[index].GroundSpeedMean = sanitizeFloat64Ptr(report.Trials[index].GroundSpeedMean)
		report.Trials[index].OdomDistanceM = sanitizeFloat64Ptr(report.Trials[index].OdomDistanceM)
		report.Trials[index].RTKDistanceM = sanitizeFloat64Ptr(report.Trials[index].RTKDistanceM)
		report.Trials[index].LeftRightTickImbalance = sanitizeFloat64Ptr(report.Trials[index].LeftRightTickImbalance)
	}
}

func absFloat(value float64) float64 {
	if value < 0 {
		return -value
	}
	return value
}

func maxFloatSlice(values []float64) float64 {
	if len(values) == 0 {
		return 0
	}
	maxValue := values[0]
	for _, value := range values[1:] {
		if value > maxValue {
			maxValue = value
		}
	}
	return maxValue
}
