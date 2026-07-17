package api

import (
	"context"
	"encoding/json"
	"fmt"
	"net/http"
	"os"
	"path/filepath"
	"strconv"
	"strings"
	"time"

	"github.com/gin-gonic/gin"
	pkgtypes "github.com/mowglinext/mowglinext/pkg/types"
)

const (
	gnssContainerName       = "mowgli-gps"
	gnssConfigPlanBinary    = "/opt/gnss_sidecar/bin/gnss_config_plan"
	gnssConfigApplyBinary   = "/opt/gnss_sidecar/bin/gnss_config_apply"
	gnssApplyModeRuntime    = "runtime-only"
	gnssApplyModeFactory    = "factory-reset"
	gnssBaudAuto            = "auto"
	gnssApplyTimeoutMs      = "5000"
	gnssRouteCommandTimeout = 2 * time.Minute
	gnssSettingsHeader      = "# Mowgli Robot Configuration — managed by mowglinext-gui\n# This file is the single source of truth for robot parameters.\n# Changes made here are picked up on container restart.\n\n"
)

var allowedGNSSBauds = map[string]bool{
	"9600":   true,
	"38400":  true,
	"57600":  true,
	"115200": true,
	"230400": true,
	"460800": true,
	"921600": true,
}

type gnssSavedConfig struct {
	ConfigPath     string
	RuntimeEnvPath string
	ExistingYAML   map[string]any
	NodeMappings   map[string]string
	Flat           map[string]any
	ReceiverFamily string
	SerialDevice   string
	RuntimeBaud    string
	ConfigBaud     string
	ExecutionBaud  string
	Profile        string
	SignalProfile  string
	SignalGroup    string
	ReceiverModel  string
	ProfileRateHz  string
}

type GNSSCommandExecution struct {
	Tool     string   `json:"tool"`
	Command  []string `json:"command"`
	ExitCode int64    `json:"exit_code"`
	Stdout   string   `json:"stdout,omitempty"`
	Stderr   string   `json:"stderr,omitempty"`
	Success  bool     `json:"success"`
}

type GNSSActionResponse struct {
	Success                      bool                   `json:"success"`
	PartialFailure               bool                   `json:"partial_failure,omitempty"`
	Action                       string                 `json:"action"`
	Message                      string                 `json:"message,omitempty"`
	Warnings                     []string               `json:"warnings,omitempty"`
	ReceiverFamily               string                 `json:"receiver_family,omitempty"`
	Profile                      string                 `json:"profile,omitempty"`
	SignalProfile                string                 `json:"signal_profile,omitempty"`
	ProfileRateHz                string                 `json:"profile_rate_hz,omitempty"`
	SerialDevice                 string                 `json:"serial_device,omitempty"`
	ExecutionBaud                string                 `json:"execution_baud,omitempty"`
	DetectedBaud                 string                 `json:"detected_baud,omitempty"`
	RuntimeBaud                  string                 `json:"runtime_baud,omitempty"`
	ConfigBaud                   string                 `json:"config_baud,omitempty"`
	RuntimeBaudDiffersFromConfig bool                   `json:"runtime_baud_differs_from_config"`
	RuntimeBaudUpdated           bool                   `json:"runtime_baud_updated,omitempty"`
	GPSContainer                 string                 `json:"gps_container,omitempty"`
	GPSImage                     string                 `json:"gps_image,omitempty"`
	GPSContainerWasRunning       bool                   `json:"gps_container_was_running"`
	StopAttempted                bool                   `json:"stop_attempted,omitempty"`
	RestartAttempted             bool                   `json:"restart_attempted,omitempty"`
	RestartSucceeded             bool                   `json:"restart_succeeded,omitempty"`
	RestartError                 string                 `json:"restart_error,omitempty"`
	Executions                   []GNSSCommandExecution `json:"executions,omitempty"`
}

type gnssApplyRequest struct {
	Confirm bool `json:"confirm"`
}

type gnssFactoryResetRequest struct {
	ConfirmFactoryReset bool `json:"confirm_factory_reset"`
}

func GNSSRoutes(r *gin.RouterGroup, dbProvider pkgtypes.IDBProvider, dockerProvider pkgtypes.IDockerProvider) {
	group := r.Group("/settings/gnss")
	group.GET("/runtime-config", getGNSSRuntimeConfig(dbProvider))
	group.POST("/plan", postGNSSPlan(dbProvider, dockerProvider))
	group.POST("/apply", postGNSSApply(dbProvider, dockerProvider))
	group.POST("/factory-reset-apply", postGNSSFactoryResetApply(dbProvider, dockerProvider))
	group.POST("/restart", postGNSSRestart(dbProvider, dockerProvider))
}

func postGNSSPlan(dbProvider pkgtypes.IDBProvider, dockerProvider pkgtypes.IDockerProvider) gin.HandlerFunc {
	return func(c *gin.Context) {
		cfg, err := loadSavedGNSSConfig(dbProvider)
		if err != nil {
			c.JSON(http.StatusBadRequest, ErrorResponse{Error: err.Error()})
			return
		}
		if cfg.ReceiverFamily == "auto" {
			c.JSON(http.StatusBadRequest, ErrorResponse{Error: "GNSS_RECEIVER_FAMILY=auto cannot be planned safely with gnss_config_plan; set an explicit receiver family first"})
			return
		}

		containerDetails, err := getGPSContainerDetails(c.Request.Context(), dockerProvider)
		if err != nil {
			c.JSON(http.StatusInternalServerError, ErrorResponse{Error: err.Error()})
			return
		}

		execution, err := runGNSSTool(c.Request.Context(), dockerProvider, containerDetails, false, buildGNSSPlanCommand(cfg))
		if err != nil {
			c.JSON(http.StatusInternalServerError, ErrorResponse{Error: err.Error()})
			return
		}

		response := newGNSSActionResponse("plan", cfg, containerDetails)
		response.Executions = []GNSSCommandExecution{execution}
		response.Success = execution.Success
		if !execution.Success {
			response.Message = "GNSS profile plan failed"
		}
		addGNSSConfigWarnings(&response, cfg, false)
		applyGNSSCommandReport(&response, execution.Stdout)

		c.JSON(http.StatusOK, response)
	}
}

func postGNSSApply(dbProvider pkgtypes.IDBProvider, dockerProvider pkgtypes.IDockerProvider) gin.HandlerFunc {
	return func(c *gin.Context) {
		var req gnssApplyRequest
		if err := c.ShouldBindJSON(&req); err != nil {
			c.JSON(http.StatusBadRequest, ErrorResponse{Error: err.Error()})
			return
		}
		if !req.Confirm {
			c.JSON(http.StatusBadRequest, ErrorResponse{Error: "apply requires confirm=true"})
			return
		}

		cfg, err := loadSavedGNSSConfig(dbProvider)
		if err != nil {
			c.JSON(http.StatusBadRequest, ErrorResponse{Error: err.Error()})
			return
		}
		if cfg.Profile == "factory_reset" {
			c.JSON(http.StatusBadRequest, ErrorResponse{Error: "GNSS_PROFILE=factory_reset is destructive, expert-only, and must use the dedicated factory reset endpoint"})
			return
		}

		response, httpStatus, err := runApplyFlow(c.Request.Context(), dbProvider, dockerProvider, cfg)
		if err != nil {
			c.JSON(httpStatus, ErrorResponse{Error: err.Error()})
			return
		}
		c.JSON(http.StatusOK, response)
	}
}

func postGNSSFactoryResetApply(dbProvider pkgtypes.IDBProvider, dockerProvider pkgtypes.IDockerProvider) gin.HandlerFunc {
	return func(c *gin.Context) {
		var req gnssFactoryResetRequest
		if err := c.ShouldBindJSON(&req); err != nil {
			c.JSON(http.StatusBadRequest, ErrorResponse{Error: err.Error()})
			return
		}
		if !req.ConfirmFactoryReset {
			c.JSON(http.StatusBadRequest, ErrorResponse{Error: "factory reset requires confirm_factory_reset=true"})
			return
		}

		cfg, err := loadSavedGNSSConfig(dbProvider)
		if err != nil {
			c.JSON(http.StatusBadRequest, ErrorResponse{Error: err.Error()})
			return
		}
		if cfg.Profile == "factory_reset" {
			c.JSON(http.StatusBadRequest, ErrorResponse{Error: "select a non-reset GNSS profile before using factory reset + apply; factory_reset is destructive and not intended for normal GUI recovery"})
			return
		}

		response, httpStatus, err := runFactoryResetApplyFlow(c.Request.Context(), dbProvider, dockerProvider, cfg)
		if err != nil {
			c.JSON(httpStatus, ErrorResponse{Error: err.Error()})
			return
		}
		c.JSON(http.StatusOK, response)
	}
}

func postGNSSRestart(dbProvider pkgtypes.IDBProvider, dockerProvider pkgtypes.IDockerProvider) gin.HandlerFunc {
	return func(c *gin.Context) {
		cfg, err := loadSavedGNSSConfig(dbProvider)
		if err != nil {
			c.JSON(http.StatusBadRequest, ErrorResponse{Error: err.Error()})
			return
		}

		containerDetails, err := getGPSContainerDetails(c.Request.Context(), dockerProvider)
		if err != nil {
			c.JSON(http.StatusInternalServerError, ErrorResponse{Error: err.Error()})
			return
		}

		response := newGNSSActionResponse("restart", cfg, containerDetails)
		addGNSSConfigWarnings(&response, cfg, false)

		actionErr := dockerProvider.ContainerStart(c.Request.Context(), containerDetails.ID)
		if containerDetails.Running {
			actionErr = dockerProvider.ContainerRestart(c.Request.Context(), containerDetails.ID)
		}
		response.RestartAttempted = true
		response.RestartSucceeded = actionErr == nil
		response.Success = actionErr == nil
		if actionErr != nil {
			response.RestartError = actionErr.Error()
			response.Message = "Failed to restart mowgli-gps"
		}

		c.JSON(http.StatusOK, response)
	}
}

func runApplyFlow(parentCtx context.Context, dbProvider pkgtypes.IDBProvider, dockerProvider pkgtypes.IDockerProvider, cfg gnssSavedConfig) (GNSSActionResponse, int, error) {
	if err := validateGNSSSerialDeviceExists(cfg.SerialDevice); err != nil {
		return GNSSActionResponse{}, http.StatusBadRequest, err
	}

	containerDetails, err := getGPSContainerDetails(parentCtx, dockerProvider)
	if err != nil {
		return GNSSActionResponse{}, http.StatusInternalServerError, err
	}

	response := newGNSSActionResponse("apply", cfg, containerDetails)
	addGNSSConfigWarnings(&response, cfg, true)

	if containerDetails.Running {
		response.StopAttempted = true
		if err := dockerProvider.ContainerStop(parentCtx, containerDetails.ID); err != nil {
			return GNSSActionResponse{}, http.StatusInternalServerError, fmt.Errorf("failed to stop %s: %w", gnssContainerName, err)
		}
	}

	execution, err := runGNSSTool(parentCtx, dockerProvider, containerDetails, true, buildGNSSApplyCommand(cfg, cfg.Profile))
	if err != nil {
		return GNSSActionResponse{}, http.StatusInternalServerError, err
	}
	response.Executions = []GNSSCommandExecution{execution}
	response.Success = execution.Success
	report := applyGNSSCommandReport(&response, execution.Stdout)
	if !execution.Success {
		if strings.TrimSpace(response.Message) == "" || response.Message == "GNSS profile apply failed" {
			response.Message = "GNSS profile apply failed"
		}
		return response, http.StatusOK, nil
	}

	resolvedRuntimeBaud := cfg.ConfigBaud
	if report.TransportBaud != "" {
		resolvedRuntimeBaud = report.TransportBaud
	}

	if cfg.RuntimeBaud != resolvedRuntimeBaud {
		if err := persistGNSSRuntimeBaud(dbProvider, resolvedRuntimeBaud); err != nil {
			response.Success = false
			response.PartialFailure = true
			response.Message = "GNSS apply succeeded but runtime baud persistence update failed"
			response.RuntimeBaudUpdated = false
			return response, http.StatusOK, nil
		}
		response.RuntimeBaud = resolvedRuntimeBaud
		response.RuntimeBaudUpdated = true
	}
	response.RuntimeBaud = resolvedRuntimeBaud
	response.RuntimeBaudDiffersFromConfig = resolvedRuntimeBaud != cfg.ConfigBaud

	if containerDetails.Running {
		response.RestartAttempted = true
		if err := dockerProvider.ContainerStart(parentCtx, containerDetails.ID); err != nil {
			response.Success = false
			response.PartialFailure = true
			response.RestartSucceeded = false
			response.RestartError = err.Error()
			response.Message = "GNSS apply succeeded but mowgli-gps failed to restart"
			return response, http.StatusOK, nil
		}
		response.RestartSucceeded = true
	}

	response.Success = true
	response.Message = "GNSS profile apply succeeded"
	return response, http.StatusOK, nil
}

func runFactoryResetApplyFlow(parentCtx context.Context, dbProvider pkgtypes.IDBProvider, dockerProvider pkgtypes.IDockerProvider, cfg gnssSavedConfig) (GNSSActionResponse, int, error) {
	if err := validateGNSSFactoryResetRecoveryPolicy(cfg); err != nil {
		return GNSSActionResponse{}, http.StatusBadRequest, err
	}
	if err := validateGNSSSerialDeviceExists(cfg.SerialDevice); err != nil {
		return GNSSActionResponse{}, http.StatusBadRequest, err
	}

	containerDetails, err := getGPSContainerDetails(parentCtx, dockerProvider)
	if err != nil {
		return GNSSActionResponse{}, http.StatusInternalServerError, err
	}

	response := newGNSSActionResponse("factory_reset_apply", cfg, containerDetails)
	addGNSSConfigWarnings(&response, cfg, true)
	addGNSSFactoryResetRecoveryWarning(&response, cfg)

	if containerDetails.Running {
		response.StopAttempted = true
		if err := dockerProvider.ContainerStop(parentCtx, containerDetails.ID); err != nil {
			return GNSSActionResponse{}, http.StatusInternalServerError, fmt.Errorf("failed to stop %s: %w", gnssContainerName, err)
		}
	}

	resetExecution, err := runGNSSTool(parentCtx, dockerProvider, containerDetails, true, buildGNSSApplyCommand(cfg, "factory_reset"))
	if err != nil {
		return GNSSActionResponse{}, http.StatusInternalServerError, err
	}
	response.Executions = []GNSSCommandExecution{resetExecution}
	response.Success = resetExecution.Success
	applyGNSSCommandReport(&response, resetExecution.Stdout)
	if !resetExecution.Success {
		if strings.TrimSpace(response.Message) == "" || response.Message == "GNSS profile apply failed" {
			response.Message = "Factory reset + apply failed"
		}
		return response, http.StatusOK, nil
	}

	applyExecution, err := runGNSSTool(parentCtx, dockerProvider, containerDetails, true, buildGNSSApplyCommand(cfg, cfg.Profile))
	if err != nil {
		return GNSSActionResponse{}, http.StatusInternalServerError, err
	}
	response.Executions = append(response.Executions, applyExecution)
	response.Success = applyExecution.Success
	report := applyGNSSCommandReport(&response, applyExecution.Stdout)
	if !applyExecution.Success {
		if strings.TrimSpace(response.Message) == "" || response.Message == "GNSS profile apply failed" {
			response.Message = "Factory reset + apply failed"
		}
		return response, http.StatusOK, nil
	}

	resolvedRuntimeBaud := cfg.ConfigBaud
	if report.TransportBaud != "" {
		resolvedRuntimeBaud = report.TransportBaud
	}

	if cfg.RuntimeBaud != resolvedRuntimeBaud {
		if err := persistGNSSRuntimeBaud(dbProvider, resolvedRuntimeBaud); err != nil {
			response.Success = false
			response.PartialFailure = true
			response.Message = "Factory reset + apply succeeded but runtime baud persistence update failed"
			response.RuntimeBaudUpdated = false
			return response, http.StatusOK, nil
		}
		response.RuntimeBaud = resolvedRuntimeBaud
		response.RuntimeBaudUpdated = true
	}
	response.RuntimeBaud = resolvedRuntimeBaud
	response.RuntimeBaudDiffersFromConfig = resolvedRuntimeBaud != cfg.ConfigBaud

	if containerDetails.Running {
		response.RestartAttempted = true
		if err := dockerProvider.ContainerStart(parentCtx, containerDetails.ID); err != nil {
			response.Success = false
			response.PartialFailure = true
			response.RestartSucceeded = false
			response.RestartError = err.Error()
			response.Message = "Factory reset + apply succeeded but mowgli-gps failed to restart"
			return response, http.StatusOK, nil
		}
		response.RestartSucceeded = true
	}

	response.Success = true
	response.Message = "Factory reset + apply succeeded"
	return response, http.StatusOK, nil
}

func runGNSSTool(parentCtx context.Context, dockerProvider pkgtypes.IDockerProvider, containerDetails pkgtypes.ContainerDetails, needsSerial bool, command []string) (GNSSCommandExecution, error) {
	ctx, cancel := context.WithTimeout(parentCtx, gnssRouteCommandTimeout)
	defer cancel()

	spec := pkgtypes.ContainerRunSpec{
		Image:      containerDetails.Image,
		Cmd:        append([]string(nil), command...),
		AutoRemove: true,
	}
	if needsSerial {
		spec.Binds = []string{deviceBind(containerDetails.Binds)}
		spec.Privileged = containerDetails.Privileged
	}

	result, err := dockerProvider.ContainerRun(ctx, spec)
	if err != nil {
		return GNSSCommandExecution{}, err
	}

	return GNSSCommandExecution{
		Tool:     filepath.Base(command[0]),
		Command:  command,
		ExitCode: result.ExitCode,
		Stdout:   result.Stdout,
		Stderr:   result.Stderr,
		Success:  result.ExitCode == 0,
	}, nil
}

func newGNSSActionResponse(action string, cfg gnssSavedConfig, containerDetails pkgtypes.ContainerDetails) GNSSActionResponse {
	return GNSSActionResponse{
		Action:                       action,
		ReceiverFamily:               cfg.ReceiverFamily,
		Profile:                      cfg.Profile,
		SignalProfile:                cfg.SignalProfile,
		ProfileRateHz:                cfg.ProfileRateHz,
		SerialDevice:                 cfg.SerialDevice,
		ExecutionBaud:                normalizeGNSSExecutionBaudDisplay(cfg.ExecutionBaud),
		RuntimeBaud:                  cfg.RuntimeBaud,
		ConfigBaud:                   cfg.ConfigBaud,
		RuntimeBaudDiffersFromConfig: cfg.RuntimeBaud != cfg.ConfigBaud,
		GPSContainer:                 containerDetails.Name,
		GPSImage:                     containerDetails.Image,
		GPSContainerWasRunning:       containerDetails.Running,
	}
}

func addGNSSConfigWarnings(response *GNSSActionResponse, cfg gnssSavedConfig, liveApply bool) {
	if cfg.RuntimeBaud != cfg.ConfigBaud {
		warning := "Configured receiver baud differs from runtime baud."
		if liveApply {
			warning += " After a successful apply, the backend will persist whichever baud Universal GNSS reports as still live. That is usually the configured baud, but Unicore runtime-only apply may keep the previously detected baud active until a save/reboot workflow makes the new baud live."
		} else {
			warning += " A live apply will re-check which baud is actually live before restarting mowgli-gps."
		}
		response.Warnings = append(response.Warnings, warning)
	}

	if normalizeGNSSExecutionBaudDisplay(cfg.ExecutionBaud) == gnssBaudAuto {
		response.Warnings = append(response.Warnings, "GNSS execution baud is set to auto. Universal GNSS will probe the receiver before live apply instead of assuming the target configured baud is already active.")
	}
}

func addGNSSFactoryResetRecoveryWarning(response *GNSSActionResponse, cfg gnssSavedConfig) {
	switch cfg.ReceiverFamily {
	case "unicore":
		response.Warnings = append(response.Warnings,
			fmt.Sprintf(
				"Factory-reset recovery mode is active for Unicore. This path is destructive, expert-only, and should not be used for normal GUI recovery. Universal GNSS will reset %s at the current baud, reopen the same device at 115200, reprobe with VERSIONA, restore COM1 to %s, verify VERSIONA again at %s, and only then this backend will replay the selected %s profile in runtime-only mode.",
				cfg.SerialDevice,
				cfg.ConfigBaud,
				cfg.ConfigBaud,
				cfg.Profile,
			),
		)
	}
}

func validateGNSSFactoryResetRecoveryPolicy(cfg gnssSavedConfig) error {
	switch cfg.ReceiverFamily {
	case "unicore":
		return nil
	case "ublox":
		return fmt.Errorf("factory reset + apply is unsupported for GNSS receiver family ublox because no safe u-blox reset recovery policy is configured in MowgliNext")
	case "nmea", "auto":
		return fmt.Errorf("factory reset + apply is unsupported for GNSS receiver family %s because no safe reset recovery policy exists", cfg.ReceiverFamily)
	default:
		return fmt.Errorf("factory reset + apply is unsupported for GNSS receiver family %s because no safe reset recovery policy exists", cfg.ReceiverFamily)
	}
}

func buildGNSSPlanCommand(cfg gnssSavedConfig) []string {
	command := []string{
		gnssConfigPlanBinary,
		"--json",
		"--config-baud", cfg.ConfigBaud,
		"--rate-hz", cfg.ProfileRateHz,
	}
	if cfg.SignalProfile != "" {
		command = append(command, "--signal-profile", cfg.SignalProfile)
	}
	if cfg.ReceiverModel != "" {
		command = append(command, "--model", cfg.ReceiverModel)
	}
	if shouldPassGNSSSignalGroup(cfg) {
		command = append(command, "--signal-group", cfg.SignalGroup)
	}
	command = append(command, cfg.ReceiverFamily, cfg.Profile)
	return command
}

func buildGNSSApplyCommand(cfg gnssSavedConfig, profile string) []string {
	applyMode := gnssApplyModeRuntime
	if canonicalProfile, ok := canonicalGNSSProfile(profile); ok && canonicalProfile == "factory_reset" {
		applyMode = gnssApplyModeFactory
	}
	executionBaud := normalizeGNSSExecutionBaudDisplay(cfg.ExecutionBaud)

	command := []string{
		gnssConfigApplyBinary,
		"--json",
		"--family", cfg.ReceiverFamily,
		"--device", cfg.SerialDevice,
		"--baud", executionBaud,
		"--profile", profile,
		"--apply-mode", applyMode,
		"--config-baud", cfg.ConfigBaud,
		"--rate-hz", cfg.ProfileRateHz,
		"--timeout-ms", gnssApplyTimeoutMs,
		"--confirm",
	}

	if executionBaud == gnssBaudAuto {
		if probeBauds := buildGNSSProbeBaudCandidates(cfg); len(probeBauds) > 0 {
			command = append(command, "--probe-bauds", strings.Join(probeBauds, ","))
		}
	}

	if cfg.SignalProfile != "" {
		command = append(command, "--signal-profile", cfg.SignalProfile)
	}

	if cfg.ReceiverModel != "" {
		command = append(command, "--model", cfg.ReceiverModel)
	}

	if shouldPassGNSSSignalGroup(cfg) {
		command = append(command, "--signal-group", cfg.SignalGroup)
	}

	return command
}

func shouldPassGNSSSignalGroup(cfg gnssSavedConfig) bool {
	return cfg.ReceiverFamily == "unicore" && strings.TrimSpace(cfg.SignalGroup) != ""
}

func buildGNSSProbeBaudCandidates(cfg gnssSavedConfig) []string {
	if cfg.ReceiverFamily != "unicore" {
		return nil
	}

	candidates := make([]string, 0, 6)
	seen := make(map[string]struct{}, 6)
	appendCandidate := func(value string) {
		baud, ok := validateGNSSBaud(strings.TrimSpace(value))
		if !ok {
			return
		}
		if _, exists := seen[baud]; exists {
			return
		}
		seen[baud] = struct{}{}
		candidates = append(candidates, baud)
	}

	appendCandidate(cfg.RuntimeBaud)
	appendCandidate(cfg.ConfigBaud)
	for _, baud := range []string{"115200", "230400", "460800", "921600"} {
		appendCandidate(baud)
	}

	return candidates
}

func getGPSContainerDetails(ctx context.Context, dockerProvider pkgtypes.IDockerProvider) (pkgtypes.ContainerDetails, error) {
	containers, err := dockerProvider.ContainerList(ctx)
	if err != nil {
		return pkgtypes.ContainerDetails{}, err
	}
	for _, container := range containers {
		for _, name := range container.Names {
			if strings.TrimPrefix(name, "/") == gnssContainerName {
				return dockerProvider.ContainerInspect(ctx, container.ID)
			}
		}
	}
	return pkgtypes.ContainerDetails{}, fmt.Errorf("container %s not found", gnssContainerName)
}

func deviceBind(existingBinds []string) string {
	for _, bind := range existingBinds {
		if strings.HasPrefix(bind, "/dev:/dev") {
			return bind
		}
	}
	return "/dev:/dev"
}

func loadSavedGNSSConfig(dbProvider pkgtypes.IDBProvider) (gnssSavedConfig, error) {
	doc, err := loadGNSSSettingsDocument(dbProvider)
	if err != nil {
		return gnssSavedConfig{}, err
	}

	receiverFamily, ok := canonicalGNSSReceiverFamily(doc.Flat["gnss_receiver_family"])
	if !ok {
		return gnssSavedConfig{}, fmt.Errorf("unsupported GNSS receiver family: %v", doc.Flat["gnss_receiver_family"])
	}
	profile, ok := canonicalGNSSProfile(doc.Flat["gnss_profile"])
	if !ok {
		return gnssSavedConfig{}, fmt.Errorf("unsupported GNSS profile: %v", doc.Flat["gnss_profile"])
	}
	rateHz, ok := canonicalGNSSRate(firstValue(doc.Flat, "gnss_profile_rate_hz", "gnss_rate_hz"))
	if !ok {
		return gnssSavedConfig{}, fmt.Errorf("unsupported GNSS profile rate: %v", firstValue(doc.Flat, "gnss_profile_rate_hz", "gnss_rate_hz"))
	}

	runtimeBaud, ok := validateGNSSBaud(stringValue(doc.Flat["gnss_serial_baud"], "921600"))
	if !ok {
		return gnssSavedConfig{}, fmt.Errorf("unsupported GNSS runtime baud: %v", doc.Flat["gnss_serial_baud"])
	}
	configBaud, ok := validateGNSSBaud(stringValue(firstValue(doc.Flat, "gnss_config_baud", "gnss_serial_baud"), runtimeBaud))
	if !ok {
		return gnssSavedConfig{}, fmt.Errorf("unsupported GNSS configured baud: %v", firstValue(doc.Flat, "gnss_config_baud", "gnss_serial_baud"))
	}
	executionBaud, ok := validateGNSSExecutionBaud(doc.Flat["gnss_execution_baud"])
	if !ok {
		return gnssSavedConfig{}, fmt.Errorf("unsupported GNSS execution baud: %v", doc.Flat["gnss_execution_baud"])
	}

	serialDevice := stringValue(doc.Flat["gnss_serial_device"], "/dev/ttyAMA4")
	if err := validateGNSSSerialDevice(serialDevice); err != nil {
		return gnssSavedConfig{}, err
	}
	signalGroup := ""
	if receiverFamily == "unicore" {
		var err error
		signalGroup, err = validateGNSSSignalGroup(doc.Flat["gnss_signal_group"])
		if err != nil {
			return gnssSavedConfig{}, err
		}
	}

	return gnssSavedConfig{
		ConfigPath:     doc.ConfigPath,
		RuntimeEnvPath: doc.RuntimeEnvPath,
		ExistingYAML:   doc.ExistingYAML,
		NodeMappings:   doc.NodeMappings,
		Flat:           doc.Flat,
		ReceiverFamily: receiverFamily,
		SerialDevice:   serialDevice,
		RuntimeBaud:    runtimeBaud,
		ConfigBaud:     configBaud,
		ExecutionBaud:  executionBaud,
		Profile:        profile,
		SignalProfile:  normalizeGnssSignalProfile(doc.Flat["gnss_signal_profile"], "balanced"),
		SignalGroup:    signalGroup,
		ReceiverModel:  normalizeGnssReceiverModel(doc.Flat["gnss_receiver_model"]),
		ProfileRateHz:  rateHz,
	}, nil
}

func validateGNSSSignalGroup(value any) (string, error) {
	normalized := normalizeGnssSignalGroup(value)
	if normalized == "" {
		return "", nil
	}

	fields := strings.Fields(normalized)
	if len(fields) != 2 {
		return "", fmt.Errorf("invalid GNSS signal group %q: expected two groups like \"3 6\"; collapsed values like \"36\" are not allowed", stringValue(value, ""))
	}

	groups := make([]string, 0, 2)
	for _, field := range fields {
		group, err := strconv.Atoi(field)
		if err != nil || group < 0 || group > 9 {
			return "", fmt.Errorf("invalid GNSS signal group %q: each group must be an integer in the range 0..9", stringValue(value, ""))
		}
		groups = append(groups, strconv.Itoa(group))
	}

	return strings.Join(groups, " "), nil
}

type gnssToolJSONOutput struct {
	Warnings     []string `json:"warnings"`
	ErrorMessage string   `json:"error_message"`
	Discovery    struct {
		Baud any `json:"baud"`
	} `json:"discovery"`
	Transport struct {
		Baud any `json:"baud"`
	} `json:"transport"`
	ExecutionSummary struct {
		FinalStatus string `json:"final_status"`
	} `json:"execution_summary"`
}

type gnssToolReport struct {
	Warnings      []string
	ErrorMessage  string
	FinalStatus   string
	DetectedBaud  string
	TransportBaud string
}

func extractGNSSReport(stdout string) gnssToolReport {
	trimmed := strings.TrimSpace(stdout)
	if trimmed == "" {
		return gnssToolReport{}
	}

	var output gnssToolJSONOutput
	if err := json.Unmarshal([]byte(trimmed), &output); err != nil {
		return gnssToolReport{}
	}

	warnings := make([]string, 0, len(output.Warnings))
	for _, warning := range output.Warnings {
		normalized := strings.TrimSpace(warning)
		if normalized == "" {
			continue
		}
		warnings = append(warnings, normalized)
	}
	return gnssToolReport{
		Warnings:      warnings,
		ErrorMessage:  strings.TrimSpace(output.ErrorMessage),
		FinalStatus:   strings.TrimSpace(output.ExecutionSummary.FinalStatus),
		DetectedBaud:  normalizeGNSSReportBaud(output.Discovery.Baud),
		TransportBaud: normalizeGNSSReportBaud(output.Transport.Baud),
	}
}

func applyGNSSCommandReport(response *GNSSActionResponse, stdout string) gnssToolReport {
	report := extractGNSSReport(stdout)
	response.Warnings = mergeGNSSWarnings(response.Warnings, report.Warnings)
	if report.DetectedBaud != "" {
		response.DetectedBaud = report.DetectedBaud
	} else if response.ExecutionBaud == gnssBaudAuto && report.TransportBaud != "" {
		response.DetectedBaud = report.TransportBaud
	}
	if report.TransportBaud != "" {
		response.RuntimeBaud = report.TransportBaud
		response.RuntimeBaudDiffersFromConfig = report.TransportBaud != response.ConfigBaud
	}
	if !response.Success && report.ErrorMessage != "" {
		switch report.FinalStatus {
		case "":
			response.Message = report.ErrorMessage
		default:
			response.Message = report.FinalStatus + ": " + report.ErrorMessage
		}
	}
	return report
}

func mergeGNSSWarnings(existing []string, additions []string) []string {
	if len(additions) == 0 {
		return existing
	}

	seen := make(map[string]struct{}, len(existing)+len(additions))
	merged := make([]string, 0, len(existing)+len(additions))
	for _, warning := range existing {
		if strings.TrimSpace(warning) == "" {
			continue
		}
		if _, ok := seen[warning]; ok {
			continue
		}
		seen[warning] = struct{}{}
		merged = append(merged, warning)
	}
	for _, warning := range additions {
		if strings.TrimSpace(warning) == "" {
			continue
		}
		if _, ok := seen[warning]; ok {
			continue
		}
		seen[warning] = struct{}{}
		merged = append(merged, warning)
	}
	return merged
}

func persistGNSSRuntimeBaud(dbProvider pkgtypes.IDBProvider, runtimeBaud string) error {
	doc, err := loadGNSSSettingsDocument(dbProvider)
	if err != nil {
		return err
	}

	if baudInt, convErr := strconv.Atoi(runtimeBaud); convErr == nil {
		doc.Flat["gnss_serial_baud"] = baudInt
	} else {
		doc.Flat["gnss_serial_baud"] = runtimeBaud
	}
	defaults := loadSchemaDefaults(dbProvider)
	applyUniversalGnssCompatibility(doc.Flat, defaults)

	// doc.Flat came out of loadGNSSSettingsDocument with every schema default
	// pre-merged in (so callers reading it see a complete parameter set) —
	// writing it straight to disk would blow the installed config's
	// sparseness (Architecture Invariant 15), persisting the FULL parameter
	// set on every baud change instead of just the GNSS keys that actually
	// changed. Prune back to only the values that differ from their default,
	// the same way PostSettingsYAML does.
	prunedKeys := sparsifyFlat(doc.Flat, defaults)

	nested := nestToROS2YAML(doc.Flat, doc.NodeMappings, doc.ExistingYAML)
	pruneNestedKeys(nested, prunedKeys)
	out, err := marshalROS2YAMLWithGeoPrecision(nested)
	if err != nil {
		return fmt.Errorf("failed to marshal YAML: %w", err)
	}
	if err := os.MkdirAll(filepath.Dir(doc.ConfigPath), 0755); err != nil {
		return err
	}
	if err := writePreservingPerms(doc.ConfigPath, []byte(gnssSettingsHeader+string(out))); err != nil {
		return err
	}
	if strings.TrimSpace(doc.RuntimeEnvPath) != "" {
		// Persisting a runtime baud must only refresh the serial-baud fallback in
		// the runtime env. Receiver-specific config lives in YAML and Universal
		// GNSS tooling; .env is fallback-only.
		baudEnvUpdates := map[string]string{
			"GNSS_SERIAL_BAUD": stringValue(doc.Flat["gnss_serial_baud"], runtimeBaud),
		}
		if err := writeRuntimeEnvFile(doc.RuntimeEnvPath, baudEnvUpdates); err != nil {
			return err
		}
	}
	return nil
}

func validateGNSSSerialDevice(device string) error {
	if strings.TrimSpace(device) == "" {
		return fmt.Errorf("GNSS serial device is required")
	}
	if strings.ContainsAny(device, " \t\r\n") {
		return fmt.Errorf("GNSS serial device must not contain whitespace: %s", device)
	}
	cleaned := filepath.Clean(device)
	if cleaned != device || !strings.HasPrefix(cleaned, "/dev/") || strings.Contains(cleaned, "..") {
		return fmt.Errorf("GNSS serial device must be a whitelisted /dev/* path: %s", device)
	}
	return nil
}

func validateGNSSBaud(value string) (string, bool) {
	text := strings.TrimSpace(value)
	_, ok := allowedGNSSBauds[text]
	return text, ok
}

func validateGNSSExecutionBaud(value any) (string, bool) {
	text := strings.TrimSpace(stringValue(value, ""))
	switch strings.ToLower(text) {
	case "", gnssBaudAuto:
		return gnssBaudAuto, true
	default:
		return validateGNSSBaud(text)
	}
}

func normalizeGNSSExecutionBaudDisplay(value string) string {
	text := strings.TrimSpace(value)
	if strings.EqualFold(text, gnssBaudAuto) || text == "" {
		return gnssBaudAuto
	}
	return text
}

func normalizeGNSSReportBaud(value any) string {
	switch typed := value.(type) {
	case float64:
		if typed <= 0 {
			return ""
		}
		return strconv.FormatInt(int64(typed), 10)
	case string:
		return strings.TrimSpace(typed)
	case json.Number:
		return typed.String()
	default:
		return ""
	}
}

func canonicalGNSSReceiverFamily(value any) (string, bool) {
	switch strings.ToLower(stringValue(value, "auto")) {
	case "", "auto":
		return "auto", true
	case "u-blox", "ublox":
		return "ublox", true
	case "unicore":
		return "unicore", true
	case "nmea":
		return "nmea", true
	default:
		return "", false
	}
}

func canonicalGNSSProfile(value any) (string, bool) {
	switch strings.ToLower(strings.ReplaceAll(stringValue(value, "runtime_only"), "-", "_")) {
	case "", "runtime_only", "balanced", "power_saving":
		return "runtime_only", true
	case "high_precision", "survey", "rover_high_precision":
		return "rover_high_precision", true
	case "debug", "rover_high_precision_debug":
		return "rover_high_precision_debug", true
	case "factory_reset":
		return "factory_reset", true
	default:
		return "", false
	}
}

func canonicalGNSSRate(value any) (string, bool) {
	switch stringValue(value, "5") {
	case "1", "5", "7", "10":
		return stringValue(value, "5"), true
	default:
		return "", false
	}
}
