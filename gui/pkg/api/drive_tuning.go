package api

import (
	"bufio"
	"context"
	"errors"
	"fmt"
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
	driveTuningBackupFile        = driveTuningContainerDir + "/drive_pid_last_backup.yaml"
	driveTuningYamlHeader        = "# Mowgli Robot Configuration — managed by mowglinext-gui\n# This file is the single source of truth for robot parameters.\n# Changes made here are picked up on container restart.\n\n"
	maxDriveTuningLogBytes       = 128 * 1024
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
	AppliedLive     bool                     `json:"applied_live" yaml:"applied_live"`
	RequestedApply  bool                     `json:"requested_apply" yaml:"requested_apply"`
	DistanceM       float64                  `json:"distance_m" yaml:"distance_m"`
	MaxSpeedMps     float64                  `json:"max_speed_mps" yaml:"max_speed_mps"`
	TestSpeedMps    *float64                 `json:"test_speed_mps,omitempty" yaml:"test_speed_mps"`
	SegmentDuration float64                  `json:"segment_duration_s" yaml:"segment_duration_s"`
	Passes          int                      `json:"passes" yaml:"passes"`
	AutoTurn        bool                     `json:"auto_turn" yaml:"auto_turn"`
	TurnDirection   string                   `json:"turn_direction" yaml:"turn_direction"`
	CurrentParams   map[string]float64       `json:"current_params" yaml:"current_params"`
	StartingParams  map[string]float64       `json:"starting_params" yaml:"starting_params"`
	ProposedParams  map[string]float64       `json:"proposed_params" yaml:"proposed_params"`
	Reasons         []string                 `json:"reasons" yaml:"reasons"`
	Trials          []driveTuningTrialReport `json:"trials" yaml:"trials"`
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
	IntegralSaturationSuspected bool     `json:"integral_saturation_suspected" yaml:"integral_saturation_suspected"`
	GroundSpeedMean             *float64 `json:"ground_speed_mean,omitempty" yaml:"ground_speed_mean"`
	OdomDistanceM               *float64 `json:"odom_distance_m,omitempty" yaml:"odom_distance_m"`
	RTKDistanceM                *float64 `json:"rtk_distance_m,omitempty" yaml:"rtk_distance_m"`
	RTKAccepted                 bool     `json:"rtk_accepted" yaml:"rtk_accepted"`
	LeftRightTickImbalance      *float64 `json:"left_right_tick_imbalance,omitempty" yaml:"left_right_tick_imbalance"`
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
		"--cmd-topic", "/cmd_vel_teleop",
		"--max-speed", formatFloat(req.TestSpeedMps),
		"--test-speed", formatFloat(req.TestSpeedMps),
		"--distance", formatFloat(req.DistanceMeters),
		"--passes", strconv.Itoa(req.Passes),
		"--turn-direction", req.TurnDirection,
		"--output", reportPath,
		"--backup-file", driveTuningBackupFile,
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
		"--cmd-topic", "/cmd_vel_teleop",
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
	for _, trial := range trials {
		if trial.StallDetected || trial.OscillationDetected {
			return driveTuningValidationSummary{
				Status:      driveTuningStatusWarning,
				Message:     "A feed-forward pass stalled or oscillated. Re-run in a safer open area.",
				GeneratedAt: report.GeneratedAt,
				ReportPath:  reportPath,
			}
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
	return driveTuningValidationSummary{
		Status:      status,
		Message:     message,
		GeneratedAt: report.GeneratedAt,
		ReportPath:  reportPath,
	}
}

func evaluatePIDReport(report *driveTuningReport, reportPath string) driveTuningValidationSummary {
	trials := filterTrialsByPhase(report.Trials, "pid")
	usable := make([]driveTuningTrialReport, 0, len(trials))
	for _, trial := range trials {
		if absFloat(trial.TargetSpeed) >= 0.05 {
			usable = append(usable, trial)
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

	maxOvershootPct := 0.0
	for _, trial := range usable {
		target := absFloat(trial.TargetSpeed)
		if target <= 1e-6 {
			continue
		}
		if trial.StallDetected || trial.OscillationDetected || trial.IntegralSaturationSuspected {
			return driveTuningValidationSummary{
				Status:      driveTuningStatusWarning,
				Message:     "PID report shows stall, oscillation, or integral saturation. Re-run after checking mechanics and feed-forward.",
				GeneratedAt: report.GeneratedAt,
				ReportPath:  reportPath,
			}
		}
		if trial.SettlingTime == nil || *trial.SettlingTime > 3.0 {
			return driveTuningValidationSummary{
				Status:      driveTuningStatusWarning,
				Message:     "PID response did not settle quickly enough on at least one step.",
				GeneratedAt: report.GeneratedAt,
				ReportPath:  reportPath,
			}
		}
		overshootPct := trial.Overshoot / target
		if overshootPct > maxOvershootPct {
			maxOvershootPct = overshootPct
		}
		if overshootPct > 0.20 {
			return driveTuningValidationSummary{
				Status:      driveTuningStatusWarning,
				Message:     fmt.Sprintf("PID overshoot reached %.1f%% on at least one step.", overshootPct*100.0),
				GeneratedAt: report.GeneratedAt,
				ReportPath:  reportPath,
			}
		}
	}
	return driveTuningValidationSummary{
		Status:      driveTuningStatusValidated,
		Message:     fmt.Sprintf("Validated PID step response with max overshoot %.1f%%.", maxOvershootPct*100.0),
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
