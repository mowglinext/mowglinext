package api

import (
	"context"
	"fmt"
	"net/http"
	"os"
	"path/filepath"
	"strconv"
	"strings"
	"time"

	pkgtypes "github.com/cedbossneo/mowglinext/pkg/types"
	"github.com/gin-gonic/gin"
	"gopkg.in/yaml.v3"
)

const (
	gnssContainerName       = "mowgli-gps"
	gnssConfigPlanBinary    = "/opt/gnss_sidecar/bin/gnss_config_plan"
	gnssConfigApplyBinary   = "/opt/gnss_sidecar/bin/gnss_config_apply"
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
	Profile        string
	SignalProfile  string
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
			c.JSON(http.StatusBadRequest, ErrorResponse{Error: "GNSS_PROFILE=factory_reset must use the dedicated factory reset endpoint"})
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
			c.JSON(http.StatusBadRequest, ErrorResponse{Error: "select a non-reset GNSS profile before using factory reset + apply"})
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
	if !execution.Success {
		response.Success = false
		response.Message = "GNSS profile apply failed"
		return response, http.StatusOK, nil
	}

	if cfg.RuntimeBaud != cfg.ConfigBaud {
		if err := persistGNSSRuntimeBaud(dbProvider, cfg.ConfigBaud); err != nil {
			response.Success = false
			response.PartialFailure = true
			response.Message = "GNSS apply succeeded but runtime baud persistence update failed"
			response.RuntimeBaudUpdated = false
			return response, http.StatusOK, nil
		}
		response.RuntimeBaud = cfg.ConfigBaud
		response.RuntimeBaudUpdated = true
		response.RuntimeBaudDiffersFromConfig = false
	}

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
	containerDetails, err := getGPSContainerDetails(parentCtx, dockerProvider)
	if err != nil {
		return GNSSActionResponse{}, http.StatusInternalServerError, err
	}

	response := newGNSSActionResponse("factory_reset_apply", cfg, containerDetails)
	addGNSSConfigWarnings(&response, cfg, true)

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
	if !resetExecution.Success {
		response.Success = false
		response.Message = "Factory reset + apply failed"
		return response, http.StatusOK, nil
	}

	applyExecution, err := runGNSSTool(parentCtx, dockerProvider, containerDetails, true, buildGNSSApplyCommand(cfg, cfg.Profile))
	if err != nil {
		return GNSSActionResponse{}, http.StatusInternalServerError, err
	}
	response.Executions = append(response.Executions, applyExecution)
	if !applyExecution.Success {
		response.Success = false
		response.Message = "Factory reset + apply failed"
		return response, http.StatusOK, nil
	}

	if cfg.RuntimeBaud != cfg.ConfigBaud {
		if err := persistGNSSRuntimeBaud(dbProvider, cfg.ConfigBaud); err != nil {
			response.Success = false
			response.PartialFailure = true
			response.Message = "Factory reset + apply succeeded but runtime baud persistence update failed"
			response.RuntimeBaudUpdated = false
			return response, http.StatusOK, nil
		}
		response.RuntimeBaud = cfg.ConfigBaud
		response.RuntimeBaudUpdated = true
		response.RuntimeBaudDiffersFromConfig = false
	}

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
			warning += " The backend will update the persisted runtime baud to match the configured receiver baud after a successful apply."
		} else {
			warning += " A live apply will need to synchronize runtime config before restarting mowgli-gps."
		}
		response.Warnings = append(response.Warnings, warning)
	}
	if cfg.SignalProfile != "" {
		response.Warnings = append(response.Warnings, "GNSS_SIGNAL_PROFILE is persisted in the UI, but backend translation to Universal GNSS tool arguments is not implemented yet.")
	}
}

func buildGNSSPlanCommand(cfg gnssSavedConfig) []string {
	return []string{
		gnssConfigPlanBinary,
		"--json",
		"--persistent",
		"--config-baud", cfg.ConfigBaud,
		"--rate-hz", cfg.ProfileRateHz,
		cfg.ReceiverFamily,
		cfg.Profile,
	}
}

func buildGNSSApplyCommand(cfg gnssSavedConfig, profile string) []string {
	return []string{
		gnssConfigApplyBinary,
		"--json",
		"--family", cfg.ReceiverFamily,
		"--device", cfg.SerialDevice,
		"--baud", cfg.RuntimeBaud,
		"--profile", profile,
		"--apply-mode", "persistent",
		"--config-baud", cfg.ConfigBaud,
		"--rate-hz", cfg.ProfileRateHz,
		"--timeout-ms", gnssApplyTimeoutMs,
		"--confirm",
	}
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

	serialDevice := stringValue(doc.Flat["gnss_serial_device"], "/dev/ttyAMA4")
	if err := validateGNSSSerialDevice(serialDevice); err != nil {
		return gnssSavedConfig{}, err
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
		Profile:        profile,
		SignalProfile:  normalizeGnssSignalProfile(doc.Flat["gnss_signal_profile"]),
		ProfileRateHz:  rateHz,
	}, nil
}

func loadGNSSSettingsDocument(dbProvider pkgtypes.IDBProvider) (gnssSavedConfig, error) {
	configFilePath, err := dbProvider.Get("system.mower.yamlConfigFile")
	if err != nil {
		return gnssSavedConfig{}, fmt.Errorf("failed to read GNSS YAML config path: %w", err)
	}

	file, err := os.ReadFile(string(configFilePath))
	if err != nil {
		return gnssSavedConfig{}, fmt.Errorf("failed to read saved GNSS config: %w", err)
	}

	existingYAML := map[string]any{}
	if err := yaml.Unmarshal(file, &existingYAML); err != nil {
		return gnssSavedConfig{}, fmt.Errorf("invalid GNSS YAML config: %w", err)
	}

	flat := flattenROS2YAML(existingYAML)
	nodeMappings := map[string]string{}

	schema, err := getSchema(dbProvider)
	if err == nil {
		defaults := map[string]any{}
		extractDefaults(schema, defaults)
		for key, value := range defaults {
			if _, exists := flat[key]; !exists {
				flat[key] = value
			}
		}
		nodeMappings = extractNodeMappings(schema)
	}

	runtimeEnvPath, err := dbProvider.Get("system.mower.runtimeEnvFile")
	runtimeEnv := ""
	if err == nil {
		runtimeEnv = string(runtimeEnvPath)
	}

	return gnssSavedConfig{
		ConfigPath:     string(configFilePath),
		RuntimeEnvPath: runtimeEnv,
		ExistingYAML:   existingYAML,
		NodeMappings:   nodeMappings,
		Flat:           flat,
	}, nil
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
	gnssEnvUpdates := applyUniversalGnssCompatibility(doc.Flat)

	nested := nestToROS2YAML(doc.Flat, doc.NodeMappings, doc.ExistingYAML)
	out, err := yaml.Marshal(nested)
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
		if err := writeRuntimeEnvFile(doc.RuntimeEnvPath, gnssEnvUpdates); err != nil {
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
