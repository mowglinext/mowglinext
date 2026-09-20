package api

import (
	"bytes"
	"context"
	"encoding/json"
	"errors"
	"io"
	"net/http"
	"net/http/httptest"
	"os"
	"path/filepath"
	"strings"
	"testing"
	"time"

	dockertypes "github.com/docker/docker/api/types"
	"github.com/gin-gonic/gin"
	pkgtypes "github.com/mowglinext/mowglinext/pkg/types"
	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
)

type mockDockerProvider struct {
	containers    []dockertypes.Container
	inspectResult pkgtypes.ContainerDetails
	inspectErr    error
	runResults    []pkgtypes.ContainerRunResult
	runErr        error
	runSpecs      []pkgtypes.ContainerRunSpec
	stopCalls     []string
	startCalls    []string
	restartCalls  []string
	stopErr       error
	startErr      error
	restartErr    error
	events        []string
	stopHook      func()
}

func (m *mockDockerProvider) ContainerList(context.Context) ([]dockertypes.Container, error) {
	return m.containers, nil
}

func (m *mockDockerProvider) ContainerLogs(context.Context, string) (io.ReadCloser, error) {
	return io.NopCloser(strings.NewReader("")), nil
}

func (m *mockDockerProvider) ContainerStart(_ context.Context, containerID string) error {
	m.startCalls = append(m.startCalls, containerID)
	m.events = append(m.events, "start")
	return m.startErr
}

func (m *mockDockerProvider) ContainerStop(_ context.Context, containerID string) error {
	m.stopCalls = append(m.stopCalls, containerID)
	m.events = append(m.events, "stop")
	if m.stopHook != nil {
		m.stopHook()
	}
	return m.stopErr
}

func (m *mockDockerProvider) ContainerRestart(_ context.Context, containerID string) error {
	m.restartCalls = append(m.restartCalls, containerID)
	m.events = append(m.events, "restart")
	return m.restartErr
}

func (m *mockDockerProvider) ContainerInspect(context.Context, string) (pkgtypes.ContainerDetails, error) {
	if m.inspectErr != nil {
		return pkgtypes.ContainerDetails{}, m.inspectErr
	}
	return m.inspectResult, nil
}

func (m *mockDockerProvider) ContainerRun(_ context.Context, spec pkgtypes.ContainerRunSpec) (pkgtypes.ContainerRunResult, error) {
	m.runSpecs = append(m.runSpecs, spec)
	m.events = append(m.events, "run")
	if m.runErr != nil {
		return pkgtypes.ContainerRunResult{}, m.runErr
	}
	if len(m.runResults) == 0 {
		return pkgtypes.ContainerRunResult{}, nil
	}
	result := m.runResults[0]
	m.runResults = m.runResults[1:]
	return result, nil
}

func (m *mockDockerProvider) ContainerExec(context.Context, string, pkgtypes.ContainerExecSpec) (pkgtypes.ContainerExecResult, error) {
	return pkgtypes.ContainerExecResult{}, nil
}

func (m *mockDockerProvider) ContainerExecStart(context.Context, string, pkgtypes.ContainerExecSpec) (pkgtypes.ContainerExecHandle, error) {
	return pkgtypes.ContainerExecHandle{Reader: io.NopCloser(strings.NewReader(""))}, nil
}

func (m *mockDockerProvider) ContainerExecInspect(context.Context, string) (pkgtypes.ContainerExecInspectResult, error) {
	return pkgtypes.ContainerExecInspectResult{}, nil
}

func setupGNSSRouter(dbProvider pkgtypes.IDBProvider, dockerProvider pkgtypes.IDockerProvider) *gin.Engine {
	gin.SetMode(gin.TestMode)
	r := gin.New()
	group := r.Group("/api")
	GNSSRoutes(group, dbProvider, dockerProvider)
	return r
}

func writeGNSSConfigFile(t *testing.T, config string) string {
	t.Helper()
	file := createTempConfigFile(t, config)
	return file
}

type stubFileInfo struct{}

func (stubFileInfo) Name() string       { return "stub" }
func (stubFileInfo) Size() int64        { return 0 }
func (stubFileInfo) Mode() os.FileMode  { return 0644 }
func (stubFileInfo) ModTime() time.Time { return time.Time{} }
func (stubFileInfo) IsDir() bool        { return false }
func (stubFileInfo) Sys() any           { return nil }

func stubGNSSDeviceInspection(t *testing.T) {
	t.Helper()

	previousStat := gnssPathStat
	gnssPathStat = func(path string) (os.FileInfo, error) {
		if strings.HasPrefix(path, "/dev/") {
			return stubFileInfo{}, nil
		}
		return previousStat(path)
	}
	t.Cleanup(func() {
		gnssPathStat = previousStat
	})
}

func defaultGNSSYAML(serialDevice string, receiverFamily string, profile string) string {
	return "mowgli:\n" +
		"  ros__parameters:\n" +
		"    gnss_receiver_family: " + receiverFamily + "\n" +
		"    gnss_serial_device: " + serialDevice + "\n" +
		"    gnss_serial_baud: 921600\n" +
		"    gnss_config_baud: 460800\n" +
		"    gnss_profile: " + profile + "\n" +
		"    gnss_signal_profile: balanced\n" +
		"    gnss_profile_rate_hz: 5\n"
}

func defaultGNSSYAMLWithModel(serialDevice string, receiverFamily string, profile string, receiverModel string) string {
	base := defaultGNSSYAML(serialDevice, receiverFamily, profile)
	if strings.TrimSpace(receiverModel) == "" {
		return base
	}
	return base + "    gnss_receiver_model: " + receiverModel + "\n"
}

func defaultGNSSYAMLWithSignalGroup(serialDevice string, receiverFamily string, profile string, signalGroup string) string {
	base := defaultGNSSYAML(serialDevice, receiverFamily, profile)
	if strings.TrimSpace(signalGroup) == "" {
		return base
	}
	return base + "    gnss_signal_group: \"" + signalGroup + "\"\n"
}

func newGNSSTestDB(t *testing.T, yamlContent string) (*pkgtypes.MockDBProvider, string) {
	t.Helper()
	stubGNSSDeviceInspection(t)
	stubGNSSSerialDeviceAccess(t)
	stubGNSSRuntimeRegen(t)
	yamlFile := writeGNSSConfigFile(t, yamlContent)
	envFile := createTempConfigFile(t, "ROS_DOMAIN_ID=0\n")
	db := pkgtypes.NewMockDBProvider()
	require.NoError(t, db.Set("system.mower.yamlConfigFile", []byte(yamlFile)))
	require.NoError(t, db.Set("system.mower.runtimeEnvFile", []byte(envFile)))
	return db, envFile
}

func stubGNSSSerialDeviceAccess(t *testing.T) {
	t.Helper()
	previous := gnssSerialDeviceAccess
	gnssSerialDeviceAccess = func(string) (string, string, error) { return "20", "/dev/ttyUSB0", nil }
	t.Cleanup(func() { gnssSerialDeviceAccess = previous })
}

func stubGNSSRuntimeRegen(t *testing.T) {
	t.Helper()
	previousRegen := runGNSSRuntimeRegen
	previousReconcile := runGNSSRuntimeReconcile
	runGNSSRuntimeRegen = func(context.Context) error { return nil }
	runGNSSRuntimeReconcile = func(context.Context) error { return nil }
	t.Cleanup(func() {
		runGNSSRuntimeRegen = previousRegen
		runGNSSRuntimeReconcile = previousReconcile
	})
}

func defaultMockDocker() *mockDockerProvider {
	return &mockDockerProvider{
		containers: []dockertypes.Container{
			{
				ID:    "gps123",
				Names: []string{"/mowgli-gps"},
			},
		},
		inspectResult: pkgtypes.ContainerDetails{
			ID:         "gps123",
			Name:       "mowgli-gps",
			Image:      "mowgli-gps:test",
			Running:    true,
			Privileged: true,
			Binds:      []string{"/dev:/dev", "/tmp:/tmp:ro"},
		},
	}
}

func TestGNSSRestartReconcilesGPSServiceAndSynchronizesDeviceContract(t *testing.T) {
	const device = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"
	db, envFile := newGNSSTestDB(t, defaultGNSSYAML(device, "unicore", "rover_high_precision"))
	docker := defaultMockDocker()
	reconcileCalls := 0
	runGNSSRuntimeReconcile = func(context.Context) error {
		reconcileCalls++
		return nil
	}
	router := setupGNSSRouter(db, docker)

	w := httptest.NewRecorder()
	req, _ := http.NewRequest("POST", "/api/settings/gnss/restart", nil)
	router.ServeHTTP(w, req)

	require.Equal(t, http.StatusOK, w.Code)
	assert.Equal(t, 1, reconcileCalls)
	assert.Empty(t, docker.startCalls)
	assert.Empty(t, docker.restartCalls)

	envContent, err := os.ReadFile(envFile)
	require.NoError(t, err)
	assert.Contains(t, string(envContent), "GNSS_SERIAL_DEVICE="+device)
	assert.NotContains(t, string(envContent), "GNSS_DEVICE="+device)
	assert.Contains(t, string(envContent), "GNSS_DEVICE_GID=20")
}

func TestGNSSCommandsUseUniversalGNSSRC4CLIContract(t *testing.T) {
	cfg := gnssSavedConfig{
		ReceiverFamily: "unicore",
		SerialDevice:   "/dev/gnss-receiver",
		ExecutionBaud:  gnssBaudAuto,
		ConfigBaud:     "460800",
		Profile:        "rover_high_precision",
		ProfileRateHz:  "5",
	}

	planCommand := buildGNSSPlanCommand(cfg)
	applyCommand := buildGNSSApplyCommand(cfg, cfg.Profile)
	resetCommand := buildGNSSApplyCommand(cfg, "factory_reset")

	assert.Equal(t, "gnss_config_plan", planCommand[0])
	assert.Equal(t, "gnss_config_apply", applyCommand[0])
	assert.Equal(t, "gnss_config_apply", resetCommand[0])
	for _, command := range [][]string{planCommand, applyCommand, resetCommand} {
		assert.NotContains(t, command[0], "/", "Universal GNSS tools must use the PATH contract exposed by the image entrypoint")
	}
}

func TestGNSSGUISourcesDoNotReferenceLegacySidecarLayout(t *testing.T) {
	guiRoot := filepath.Clean(filepath.Join("..", ".."))
	forbiddenPath := "/opt/" + "gnss_sidecar/"
	sourceExtensions := map[string]bool{
		".go": true, ".json": true, ".md": true, ".sh": true,
		".ts": true, ".tsx": true, ".yaml": true, ".yml": true,
	}

	err := filepath.WalkDir(guiRoot, func(path string, entry os.DirEntry, walkErr error) error {
		if walkErr != nil {
			return walkErr
		}
		if entry.IsDir() {
			switch entry.Name() {
			case ".git", "dist", "node_modules":
				return filepath.SkipDir
			}
			return nil
		}
		if !sourceExtensions[filepath.Ext(path)] {
			return nil
		}

		content, err := os.ReadFile(path)
		if err != nil {
			return err
		}
		assert.NotContains(t, string(content), forbiddenPath, "legacy Universal GNSS sidecar path in %s", path)
		return nil
	})
	require.NoError(t, err)
}

func TestGNSSPlan_DoesNotRequireConfirmAndAvoidsSerialAccess(t *testing.T) {
	db, _ := newGNSSTestDB(t, defaultGNSSYAML("/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0", "unicore", "rover_high_precision"))
	docker := defaultMockDocker()
	docker.runResults = []pkgtypes.ContainerRunResult{{
		ExitCode: 0,
		Stdout:   `{"status":"ok","warnings":["Unknown Unicore model; safe fallback kept documented signal-group auto-selection disabled."]}`,
	}}

	router := setupGNSSRouter(db, docker)

	w := httptest.NewRecorder()
	req, _ := http.NewRequest("POST", "/api/settings/gnss/plan", nil)
	router.ServeHTTP(w, req)

	require.Equal(t, http.StatusOK, w.Code)
	require.Len(t, docker.stopCalls, 0)
	require.Len(t, docker.runSpecs, 1)
	assert.Empty(t, docker.runSpecs[0].Binds)
	assert.False(t, docker.runSpecs[0].Privileged)
	assert.Equal(t, gnssConfigPlanCommand, docker.runSpecs[0].Cmd[0])
	assert.Contains(t, docker.runSpecs[0].Cmd, "--config-baud")
	assert.Contains(t, docker.runSpecs[0].Cmd, "460800")
	assert.NotContains(t, docker.runSpecs[0].Cmd, "--persistent")
	assert.Contains(t, docker.runSpecs[0].Cmd, "--signal-profile")
	assert.Contains(t, docker.runSpecs[0].Cmd, "balanced")
	assert.NotContains(t, docker.runSpecs[0].Cmd, "--model")

	var response GNSSActionResponse
	require.NoError(t, json.Unmarshal(w.Body.Bytes(), &response))
	assert.True(t, response.Success)
	assert.Equal(t, "plan", response.Action)
	assert.Contains(t, strings.Join(response.Warnings, "\n"), "safe fallback kept documented signal-group auto-selection disabled")
}

func TestGNSSApply_RequiresConfirm(t *testing.T) {
	db, _ := newGNSSTestDB(t, defaultGNSSYAML("/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0", "unicore", "rover_high_precision"))
	docker := defaultMockDocker()
	router := setupGNSSRouter(db, docker)

	w := httptest.NewRecorder()
	req, _ := http.NewRequest("POST", "/api/settings/gnss/apply", bytes.NewReader([]byte(`{}`)))
	req.Header.Set("Content-Type", "application/json")
	router.ServeHTTP(w, req)

	require.Equal(t, http.StatusBadRequest, w.Code)
	assert.Contains(t, w.Body.String(), "confirm=true")
	assert.Empty(t, docker.stopCalls)
	assert.Empty(t, docker.runSpecs)
}

func TestGNSSFactoryResetApply_RequiresConfirmFactoryReset(t *testing.T) {
	db, _ := newGNSSTestDB(t, defaultGNSSYAML("/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0", "unicore", "rover_high_precision"))
	docker := defaultMockDocker()
	router := setupGNSSRouter(db, docker)

	w := httptest.NewRecorder()
	req, _ := http.NewRequest("POST", "/api/settings/gnss/factory-reset-apply", bytes.NewReader([]byte(`{}`)))
	req.Header.Set("Content-Type", "application/json")
	router.ServeHTTP(w, req)

	require.Equal(t, http.StatusBadRequest, w.Code)
	assert.Contains(t, w.Body.String(), "confirm_factory_reset=true")
	assert.Empty(t, docker.stopCalls)
	assert.Empty(t, docker.runSpecs)
}

func TestGNSSApply_PassesConfigBaudAndRestartsAfterSuccess(t *testing.T) {
	db, envFile := newGNSSTestDB(t, defaultGNSSYAML("/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0", "unicore", "rover_high_precision"))
	docker := defaultMockDocker()
	docker.runResults = []pkgtypes.ContainerRunResult{
		{ExitCode: 0, Stdout: `{"status":"ok","warnings":[],"discovery":{"baud":921600},"transport":{"current_baud":921600},"execution_summary":{"final_status":"dry_run"}}`},
		{ExitCode: 0, Stdout: `{"status":"ok","warnings":["Universal GNSS skipped documented model-specific signal groups because no receiver model was selected."],"discovery":{"baud":921600},"transport":{"current_baud":921600,"target_baud":460800,"active_verified_baud":460800},"execution_summary":{"final_status":"ok"}}`},
	}
	router := setupGNSSRouter(db, docker)

	w := httptest.NewRecorder()
	req, _ := http.NewRequest("POST", "/api/settings/gnss/apply", bytes.NewReader([]byte(`{"confirm":true}`)))
	req.Header.Set("Content-Type", "application/json")
	router.ServeHTTP(w, req)

	require.Equal(t, http.StatusOK, w.Code)
	require.Len(t, docker.stopCalls, 1)
	assert.Empty(t, docker.startCalls)
	require.Len(t, docker.runSpecs, 2)
	assert.Equal(t, []string{"stop", "run", "run"}, docker.events)
	assert.Equal(t, []string{"/dev:/dev"}, docker.runSpecs[0].Binds)
	assert.Equal(t, []string{"20"}, docker.runSpecs[0].GroupAdd)
	assert.Equal(t, []pkgtypes.ContainerDevice{{
		PathOnHost:        "/dev/ttyUSB0",
		PathInContainer:   "/dev/ttyUSB0",
		CgroupPermissions: "rwm",
	}}, docker.runSpecs[0].Devices)
	assert.True(t, docker.runSpecs[0].Privileged)
	assert.Equal(t, gnssConfigApplyCommand, docker.runSpecs[0].Cmd[0])
	assert.NotContains(t, docker.runSpecs[0].Cmd, "--config-baud")
	assert.Contains(t, docker.runSpecs[1].Cmd, "--config-baud")
	assert.Contains(t, docker.runSpecs[1].Cmd, "460800")
	assert.Contains(t, docker.runSpecs[0].Cmd, "--device")
	assert.Contains(t, docker.runSpecs[0].Cmd, "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0")

	assert.Contains(t, docker.runSpecs[1].Cmd, "--apply-mode")
	assert.Contains(t, docker.runSpecs[1].Cmd, gnssApplyModeRuntime)
	assert.Contains(t, docker.runSpecs[0].Cmd, "--baud")
	assert.Contains(t, docker.runSpecs[0].Cmd, gnssBaudAuto)
	assert.Contains(t, docker.runSpecs[0].Cmd, "--probe-bauds")
	assert.Contains(t, docker.runSpecs[0].Cmd, "921600,460800,115200,230400")
	assert.Contains(t, docker.runSpecs[1].Cmd, "--signal-profile")
	assert.Contains(t, docker.runSpecs[1].Cmd, "balanced")

	assert.NotContains(t, docker.runSpecs[0].Cmd, "persistent")
	assert.NotContains(t, docker.runSpecs[1].Cmd, gnssApplyModeFactory)
	assert.NotContains(t, docker.runSpecs[1].Cmd, "--model")
	assert.NotContains(t, docker.runSpecs[1].Cmd, "--signal-group")
	assert.NotContains(t, strings.Join(docker.runSpecs[1].Cmd, " "), "SIGNALGROUP")

	envContent, err := os.ReadFile(envFile)
	require.NoError(t, err)
	assert.Contains(t, string(envContent), "GNSS_SERIAL_BAUD=460800")

	var response GNSSActionResponse
	require.NoError(t, json.Unmarshal(w.Body.Bytes(), &response))
	assert.True(t, response.Success)
	assert.True(t, response.RuntimeBaudUpdated)
	assert.True(t, response.RestartAttempted)
	assert.True(t, response.RestartSucceeded)
	assert.Equal(t, "460800", response.RuntimeBaud)
	assert.False(t, response.RuntimeBaudDiffersFromConfig)
	assert.Contains(t, strings.Join(response.Warnings, "\n"), "Configured receiver baud differs from runtime baud.")
	assert.NotContains(t, strings.Join(response.Warnings, "\n"), "GNSS_SIGNAL_PROFILE is persisted in the UI")
	assert.Contains(t, strings.Join(response.Warnings, "\n"), "no receiver model was selected")
	assert.Contains(t, strings.Join(response.Warnings, "\n"), "execution baud is set to auto")
}

func TestGNSSPlan_AddsConfiguredReceiverModels(t *testing.T) {
	for _, receiverModel := range []string{"UM960", "UM982"} {
		t.Run(receiverModel, func(t *testing.T) {
			db, _ := newGNSSTestDB(t, defaultGNSSYAMLWithModel("/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0", "unicore", "rover_high_precision", receiverModel))
			docker := defaultMockDocker()
			docker.runResults = []pkgtypes.ContainerRunResult{{ExitCode: 0, Stdout: `{"status":"ok","warnings":[]}`}}
			router := setupGNSSRouter(db, docker)

			w := httptest.NewRecorder()
			req, _ := http.NewRequest("POST", "/api/settings/gnss/plan", nil)
			router.ServeHTTP(w, req)

			require.Equal(t, http.StatusOK, w.Code)
			require.Len(t, docker.runSpecs, 1)
			assert.NotContains(t, docker.runSpecs[0].Cmd, "--persistent")
			assert.Contains(t, docker.runSpecs[0].Cmd, "--model")
			assert.Contains(t, docker.runSpecs[0].Cmd, receiverModel)
		})
	}
}

func TestBuildGNSSPlanCommand_UsesRuntimeOnlyPlanByDefault(t *testing.T) {
	command := buildGNSSPlanCommand(gnssSavedConfig{
		ReceiverFamily: "unicore",
		ConfigBaud:     "921600",
		Profile:        "rover_high_precision",
		ProfileRateHz:  "10",
		ReceiverModel:  "UM982",
	})

	assert.Equal(t, gnssConfigPlanCommand, command[0])
	assert.NotContains(t, command, "--persistent")
	assert.Contains(t, command, "--model")
	assert.Contains(t, command, "UM982")
	assert.Equal(t, []string{"unicore", "rover_high_precision"}, command[len(command)-2:])
}

func TestGNSSCommands_IgnoreStaleUnicoreModelForUblox(t *testing.T) {
	cfg := gnssSavedConfig{
		ReceiverFamily:   "ublox",
		ReceiverModel:    "UM982",
		SignalGroup:      "3 6",
		RoverDynamicMode: "uav",
		ConfigBaud:       "921600",
		SerialDevice:     "/dev/ttyUSB0",
		Profile:          "rover_high_precision",
		ProfileRateHz:    "5",
	}

	assert.NotContains(t, buildGNSSPlanCommand(cfg), "--model")
	assert.NotContains(t, buildGNSSDetectCommand(cfg), "--model")
	apply := buildGNSSApplyCommand(cfg, cfg.Profile)
	assert.NotContains(t, apply, "--model")
	assert.NotContains(t, apply, "--signal-group")
	assert.NotContains(t, apply, "--rover-dynamic-mode")
}

func TestGNSSCommands_DoNotPassUnicoreOptionsForAutoFamily(t *testing.T) {
	cfg := gnssSavedConfig{
		ReceiverFamily:   "auto",
		ReceiverModel:    "UM982",
		SignalGroup:      "3 6",
		RoverDynamicMode: "uav",
		ConfigBaud:       "921600",
		SerialDevice:     "/dev/ttyUSB0",
		Profile:          "rover_high_precision",
		ProfileRateHz:    "5",
	}

	for _, command := range [][]string{
		buildGNSSPlanCommand(cfg),
		buildGNSSDetectCommand(cfg),
		buildGNSSApplyCommand(cfg, cfg.Profile),
	} {
		assert.NotContains(t, command, "--model")
		assert.NotContains(t, command, "--signal-group")
		assert.NotContains(t, command, "--rover-dynamic-mode")
	}
}

func TestGNSSPlan_AddsConfiguredSignalGroupWhenPresent(t *testing.T) {
	command := buildGNSSPlanCommand(gnssSavedConfig{
		ReceiverFamily: "unicore",
		ConfigBaud:     "921600",
		Profile:        "rover_high_precision",
		ProfileRateHz:  "10",
		ReceiverModel:  "UM982",
		SignalGroup:    "3 6",
	})

	assert.Contains(t, command, "--signal-group")
	assert.Contains(t, command, "3 6")
}

func TestGNSSApply_AddsConfiguredReceiverModelsAndSignalGroupTranslation(t *testing.T) {
	for _, receiverModel := range []string{"UM980", "UM981", "UM982"} {
		t.Run(receiverModel, func(t *testing.T) {
			yaml := defaultGNSSYAMLWithModel("/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0", "unicore", "rover_high_precision", receiverModel) +
				"    gnss_signal_group: \"2 0\"\n"
			db, _ := newGNSSTestDB(t, yaml)
			docker := defaultMockDocker()
			docker.runResults = []pkgtypes.ContainerRunResult{
				{ExitCode: 0, Stdout: `{"status":"ok","warnings":[],"discovery":{"baud":921600},"transport":{"current_baud":921600}}`},
				{ExitCode: 0, Stdout: `{"status":"ok","warnings":[],"transport":{"active_verified_baud":460800}}`},
			}
			router := setupGNSSRouter(db, docker)

			w := httptest.NewRecorder()
			req, _ := http.NewRequest("POST", "/api/settings/gnss/apply", bytes.NewReader([]byte(`{"confirm":true}`)))
			req.Header.Set("Content-Type", "application/json")
			router.ServeHTTP(w, req)

			require.Equal(t, http.StatusOK, w.Code)
			require.Len(t, docker.runSpecs, 2)
			assert.Contains(t, docker.runSpecs[1].Cmd, "--model")
			assert.Contains(t, docker.runSpecs[1].Cmd, receiverModel)
			assert.Contains(t, docker.runSpecs[1].Cmd, "--apply-mode")
			assert.Contains(t, docker.runSpecs[1].Cmd, gnssApplyModeRuntime)
			assert.NotContains(t, docker.runSpecs[1].Cmd, "persistent")
			assert.NotContains(t, docker.runSpecs[1].Cmd, gnssApplyModeFactory)
			assert.Contains(t, docker.runSpecs[1].Cmd, "--signal-group")
			assert.Contains(t, docker.runSpecs[1].Cmd, "2 0")
		})
	}
}

func TestGNSSPlan_PropagatesSignalGroupToTool(t *testing.T) {
	db, _ := newGNSSTestDB(t, defaultGNSSYAMLWithSignalGroup("/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0", "unicore", "rover_high_precision", "3 6"))
	docker := defaultMockDocker()
	docker.runResults = []pkgtypes.ContainerRunResult{{
		ExitCode: 0,
		Stdout:   `{"status":"ok","commands":[{"command":"CONFIG SIGNALGROUP 3 6"}],"warnings":[]}`,
	}}
	router := setupGNSSRouter(db, docker)

	w := httptest.NewRecorder()
	req, _ := http.NewRequest("POST", "/api/settings/gnss/plan", nil)
	router.ServeHTTP(w, req)

	require.Equal(t, http.StatusOK, w.Code)
	require.Len(t, docker.runSpecs, 1)
	assert.Contains(t, docker.runSpecs[0].Cmd, "--signal-group")
	assert.Contains(t, docker.runSpecs[0].Cmd, "3 6")

	var response GNSSActionResponse
	require.NoError(t, json.Unmarshal(w.Body.Bytes(), &response))
	require.Len(t, response.Executions, 1)
	assert.Contains(t, response.Executions[0].Stdout, "CONFIG SIGNALGROUP 3 6")
}

func TestGNSSFactoryResetApply_UsesDedicatedResetModeThenRuntimeProfileApply(t *testing.T) {
	db, envFile := newGNSSTestDB(t, defaultGNSSYAMLWithModel("/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0", "unicore", "rover_high_precision", "UM982"))
	docker := defaultMockDocker()
	docker.runResults = []pkgtypes.ContainerRunResult{
		{ExitCode: 0, Stdout: `{"status":"ok","warnings":[]}`},
		{ExitCode: 0, Stdout: `{"status":"ok","warnings":[]}`},
	}
	router := setupGNSSRouter(db, docker)

	w := httptest.NewRecorder()
	req, _ := http.NewRequest("POST", "/api/settings/gnss/factory-reset-apply", bytes.NewReader([]byte(`{"confirm_factory_reset":true}`)))
	req.Header.Set("Content-Type", "application/json")
	router.ServeHTTP(w, req)

	require.Equal(t, http.StatusOK, w.Code)
	require.Len(t, docker.stopCalls, 1)
	assert.Empty(t, docker.startCalls)
	require.Len(t, docker.runSpecs, 2)
	assert.Equal(t, []string{"stop", "run", "run"}, docker.events)

	resetCommand := docker.runSpecs[0].Cmd
	assert.Equal(t, gnssConfigApplyCommand, resetCommand[0])
	assert.Contains(t, resetCommand, "--profile")
	assert.Contains(t, resetCommand, "factory_reset")
	assert.Contains(t, resetCommand, "--apply-mode")
	assert.Contains(t, resetCommand, gnssApplyModeFactory)
	assert.NotContains(t, resetCommand, "persistent")
	assert.Contains(t, resetCommand, "--model")
	assert.Contains(t, resetCommand, "UM982")

	applyCommand := docker.runSpecs[1].Cmd
	assert.Equal(t, gnssConfigApplyCommand, applyCommand[0])
	assert.Contains(t, applyCommand, "--profile")
	assert.Contains(t, applyCommand, "rover_high_precision")
	assert.Contains(t, applyCommand, "--apply-mode")
	assert.Contains(t, applyCommand, gnssApplyModeRuntime)
	assert.NotContains(t, applyCommand, "persistent")
	assert.NotContains(t, applyCommand, gnssApplyModeFactory)
	assert.Contains(t, applyCommand, "--model")
	assert.Contains(t, applyCommand, "UM982")

	envContent, err := os.ReadFile(envFile)
	require.NoError(t, err)
	assert.Contains(t, string(envContent), "GNSS_SERIAL_BAUD=460800")

	var response GNSSActionResponse
	require.NoError(t, json.Unmarshal(w.Body.Bytes(), &response))
	assert.True(t, response.Success)
	assert.Equal(t, "factory_reset_apply", response.Action)
	assert.True(t, response.RuntimeBaudUpdated)
	assert.True(t, response.RestartAttempted)
	assert.True(t, response.RestartSucceeded)
	require.Len(t, response.Executions, 2)
	assert.Equal(t, resetCommand, response.Executions[0].Command)
	assert.Equal(t, applyCommand, response.Executions[1].Command)
	assert.Contains(t, strings.Join(response.Warnings, "\n"), "Factory-reset recovery mode is active")
	assert.Contains(t, strings.Join(response.Warnings, "\n"), "destructive")
	assert.Contains(t, strings.Join(response.Warnings, "\n"), "Unicore")
	assert.Contains(t, strings.Join(response.Warnings, "\n"), "115200")
	assert.Contains(t, strings.Join(response.Warnings, "\n"), "VERSIONA")
}

func TestBuildGNSSApplyCommand_FactoryResetProfileIsTheOnlyPathUsingFactoryResetMode(t *testing.T) {
	cfg := gnssSavedConfig{
		ReceiverFamily: "unicore",
		SerialDevice:   "/dev/ttyUSB7",
		RuntimeBaud:    "921600",
		ConfigBaud:     "460800",
		ExecutionBaud:  gnssBaudAuto,
		ProfileRateHz:  "10",
		ReceiverModel:  "UM982",
		SignalGroup:    "3 6",
	}

	normalCommand := buildGNSSApplyCommand(cfg, "rover_high_precision")
	assert.Contains(t, normalCommand, gnssApplyModeRuntime)
	assert.Contains(t, normalCommand, "--baud")
	assert.Contains(t, normalCommand, gnssBaudAuto)
	assert.Contains(t, normalCommand, "--probe-bauds")
	assert.Contains(t, normalCommand, "921600,460800,115200,230400")
	assert.NotContains(t, normalCommand, "persistent")
	assert.NotContains(t, normalCommand, gnssApplyModeFactory)
	assert.Contains(t, normalCommand, "--model")
	assert.Contains(t, normalCommand, "UM982")
	assert.Contains(t, normalCommand, "--signal-group")
	assert.Contains(t, normalCommand, "3 6")

	resetCommand := buildGNSSApplyCommand(cfg, "factory_reset")
	assert.Contains(t, resetCommand, gnssApplyModeFactory)
	assert.NotContains(t, resetCommand, "persistent")
	assert.Contains(t, resetCommand, "--model")
	assert.Contains(t, resetCommand, "UM982")
	assert.Contains(t, resetCommand, "--signal-group")
	assert.Contains(t, resetCommand, "3 6")
}

func TestGNSSFactoryResetApply_RejectsUbloxWithoutRunningDestructiveWorkflow(t *testing.T) {
	db, _ := newGNSSTestDB(t, defaultGNSSYAML("/dev/serial/by-id/usb-u-blox-if00-port0", "ublox", "rover_high_precision"))
	docker := defaultMockDocker()
	router := setupGNSSRouter(db, docker)

	w := httptest.NewRecorder()
	req, _ := http.NewRequest("POST", "/api/settings/gnss/factory-reset-apply", bytes.NewReader([]byte(`{"confirm_factory_reset":true}`)))
	req.Header.Set("Content-Type", "application/json")
	router.ServeHTTP(w, req)

	require.Equal(t, http.StatusBadRequest, w.Code)
	assert.Contains(t, w.Body.String(), "unsupported for GNSS receiver family ublox")
	assert.Contains(t, w.Body.String(), "safe u-blox reset recovery policy")
	assert.Empty(t, docker.stopCalls)
	assert.Empty(t, docker.startCalls)
	assert.Empty(t, docker.runSpecs)
}

func TestGNSSApply_RejectsInvalidSerialDevice(t *testing.T) {
	db, _ := newGNSSTestDB(t, defaultGNSSYAML("/tmp/not-a-device", "unicore", "rover_high_precision"))
	docker := defaultMockDocker()
	router := setupGNSSRouter(db, docker)

	w := httptest.NewRecorder()
	req, _ := http.NewRequest("POST", "/api/settings/gnss/apply", bytes.NewReader([]byte(`{"confirm":true}`)))
	req.Header.Set("Content-Type", "application/json")
	router.ServeHTTP(w, req)

	require.Equal(t, http.StatusBadRequest, w.Code)
	assert.Contains(t, w.Body.String(), "/dev/*")
	assert.Empty(t, docker.runSpecs)
}

func TestGNSSPlan_UsesYAMLFamilyDeviceAndBaudOverEnvFallback(t *testing.T) {
	db, envFile := newGNSSTestDB(t, defaultGNSSYAML("/dev/ttyUSB9", "unicore", "rover_high_precision"))
	require.NoError(t, os.WriteFile(envFile, []byte(strings.Join([]string{
		"GNSS_RECEIVER_FAMILY=ublox",
		"GNSS_SERIAL_DEVICE=/dev/ttyUSB1",
		"GNSS_SERIAL_BAUD=115200",
	}, "\n")+"\n"), 0644))

	docker := defaultMockDocker()
	docker.runResults = []pkgtypes.ContainerRunResult{{ExitCode: 0, Stdout: `{"status":"ok","warnings":[]}`}}
	router := setupGNSSRouter(db, docker)

	w := httptest.NewRecorder()
	req, _ := http.NewRequest("POST", "/api/settings/gnss/apply", bytes.NewReader([]byte(`{"confirm":true}`)))
	req.Header.Set("Content-Type", "application/json")
	router.ServeHTTP(w, req)

	require.Equal(t, http.StatusOK, w.Code)
	require.Len(t, docker.runSpecs, 1)
	assert.Contains(t, docker.runSpecs[0].Cmd, "--family")
	assert.Contains(t, docker.runSpecs[0].Cmd, "unicore")
	assert.Contains(t, docker.runSpecs[0].Cmd, "--device")
	assert.Contains(t, docker.runSpecs[0].Cmd, "/dev/ttyUSB9")
	assert.Contains(t, docker.runSpecs[0].Cmd, "--baud")
	assert.Contains(t, docker.runSpecs[0].Cmd, gnssBaudAuto)
	assert.Contains(t, docker.runSpecs[0].Cmd, "--probe-bauds")
	assert.Contains(t, docker.runSpecs[0].Cmd, "921600,460800,115200,230400")
}

func TestGNSSApply_UsesEnvFallbackWhenYAMLValuesAreMissing(t *testing.T) {
	db, envFile := newGNSSTestDB(t, "mowgli:\n  ros__parameters:\n    gnss_profile: rover_high_precision\n")
	require.NoError(t, os.WriteFile(envFile, []byte(strings.Join([]string{
		"GNSS_RECEIVER_FAMILY=unicore",
		"GNSS_SERIAL_DEVICE=/dev/ttyUSB7",
		"GNSS_SERIAL_BAUD=460800",
	}, "\n")+"\n"), 0644))

	docker := defaultMockDocker()
	docker.runResults = []pkgtypes.ContainerRunResult{{ExitCode: 0, Stdout: `{"status":"ok","warnings":[]}`}}
	router := setupGNSSRouter(db, docker)

	w := httptest.NewRecorder()
	req, _ := http.NewRequest("POST", "/api/settings/gnss/apply", bytes.NewReader([]byte(`{"confirm":true}`)))
	req.Header.Set("Content-Type", "application/json")
	router.ServeHTTP(w, req)

	require.Equal(t, http.StatusOK, w.Code)
	require.Len(t, docker.runSpecs, 1)
	assert.Contains(t, docker.runSpecs[0].Cmd, "unicore")
	assert.Contains(t, docker.runSpecs[0].Cmd, "/dev/ttyUSB7")
	assert.Contains(t, docker.runSpecs[0].Cmd, gnssBaudAuto)
	assert.Contains(t, docker.runSpecs[0].Cmd, "--probe-bauds")
	assert.Contains(t, docker.runSpecs[0].Cmd, "460800,115200,230400,921600")
}

func TestBuildGNSSApplyCommand_UsesExplicitExecutionBaudWhenConfigured(t *testing.T) {
	cfg := gnssSavedConfig{
		ReceiverFamily: "unicore",
		SerialDevice:   "/dev/ttyUSB7",
		RuntimeBaud:    "921600",
		ConfigBaud:     "921600",
		ExecutionBaud:  "115200",
		ProfileRateHz:  "10",
	}

	command := buildGNSSApplyCommand(cfg, "rover_high_precision")

	assert.Contains(t, command, "--baud")
	assert.Contains(t, command, "115200")
	assert.NotContains(t, command, "--probe-bauds")
}

func TestBuildGNSSDetectThenApplyCommandsKeepCurrentAndTargetBaudsSeparate(t *testing.T) {
	cfg := gnssSavedConfig{
		ReceiverFamily: "unicore",
		ReceiverModel:  "UM982",
		SerialDevice:   "/dev/ttyUSB7",
		RuntimeBaud:    "115200",
		ConfigBaud:     "460800",
		ExecutionBaud:  gnssBaudAuto,
		ProfileRateHz:  "5",
	}

	detect := buildGNSSDetectCommand(cfg)
	manualDetect := buildGNSSDetectCommand(cfg, "115200")
	apply := buildGNSSApplyCommand(cfg, "rover_high_precision", "115200")

	assert.Contains(t, detect, gnssBaudAuto)
	assert.Contains(t, detect, "--probe-bauds")
	assert.Contains(t, manualDetect, "115200")
	assert.NotContains(t, manualDetect, gnssBaudAuto)
	assert.NotContains(t, manualDetect, "--probe-bauds")
	assert.Contains(t, apply, "115200")
	assert.Contains(t, apply, "460800")
	assert.NotContains(t, apply, gnssBaudAuto)
	assert.NotContains(t, apply, "--probe-bauds")
}

func TestBuildGNSSCommands_SkipSignalGroupWhenEmpty(t *testing.T) {
	cfg := gnssSavedConfig{
		ReceiverFamily: "unicore",
		SerialDevice:   "/dev/ttyUSB7",
		RuntimeBaud:    "921600",
		ConfigBaud:     "921600",
		ExecutionBaud:  gnssBaudAuto,
		Profile:        "rover_high_precision",
		ProfileRateHz:  "10",
	}

	assert.NotContains(t, buildGNSSPlanCommand(cfg), "--signal-group")
	assert.NotContains(t, buildGNSSApplyCommand(cfg, "rover_high_precision"), "--signal-group")
}

func TestGNSSPlan_RejectsCollapsedSignalGroupBeforeCommandGeneration(t *testing.T) {
	db, _ := newGNSSTestDB(t, defaultGNSSYAMLWithSignalGroup("/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0", "unicore", "rover_high_precision", "36"))
	docker := defaultMockDocker()
	router := setupGNSSRouter(db, docker)

	w := httptest.NewRecorder()
	req, _ := http.NewRequest("POST", "/api/settings/gnss/plan", nil)
	router.ServeHTTP(w, req)

	require.Equal(t, http.StatusBadRequest, w.Code)
	assert.Contains(t, w.Body.String(), "invalid GNSS signal group")
	assert.Contains(t, w.Body.String(), "36")
	assert.Empty(t, docker.runSpecs)
}

func TestBuildGNSSApplyCommand_AutoProbeHintsAreUnicoreSpecific(t *testing.T) {
	cfg := gnssSavedConfig{
		ReceiverFamily: "ublox",
		SerialDevice:   "/dev/ttyACM0",
		RuntimeBaud:    "460800",
		ConfigBaud:     "921600",
		ExecutionBaud:  gnssBaudAuto,
		ProfileRateHz:  "5",
	}

	command := buildGNSSApplyCommand(cfg, "rover_high_precision")

	assert.Contains(t, command, "--baud")
	assert.Contains(t, command, gnssBaudAuto)
	assert.NotContains(t, command, "--probe-bauds")
}

func TestGNSSApply_AutoExecutionBaudSurfacesDetectedBaudFromToolOutput(t *testing.T) {
	db, _ := newGNSSTestDB(t, defaultGNSSYAMLWithModel("/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0", "unicore", "rover_high_precision", "UM982"))
	docker := defaultMockDocker()
	docker.runResults = []pkgtypes.ContainerRunResult{
		{ExitCode: 0, Stdout: `{"status":"ok","warnings":[],"discovery":{"baud":115200},"transport":{"current_baud":115200,"target_baud":null,"active_verified_baud":null},"execution_summary":{"final_status":"dry_run"}}`},
		{ExitCode: 0, Stdout: `{"status":"ok","warnings":[],"discovery":{"baud":115200},"transport":{"current_baud":115200,"target_baud":460800,"active_verified_baud":460800},"execution_summary":{"final_status":"ok"}}`},
	}
	router := setupGNSSRouter(db, docker)

	w := httptest.NewRecorder()
	req, _ := http.NewRequest("POST", "/api/settings/gnss/apply", bytes.NewReader([]byte(`{"confirm":true}`)))
	req.Header.Set("Content-Type", "application/json")
	router.ServeHTTP(w, req)

	require.Equal(t, http.StatusOK, w.Code)

	var response GNSSActionResponse
	require.NoError(t, json.Unmarshal(w.Body.Bytes(), &response))
	assert.True(t, response.Success)
	assert.Equal(t, gnssBaudAuto, response.ExecutionBaud)
	assert.Equal(t, "115200", response.DetectedBaud)
	assert.Equal(t, "460800", response.RuntimeBaud)
	assert.Equal(t, "460800", response.ConfigBaud)
	assert.Equal(t, "460800", response.TargetBaud)
	assert.Equal(t, "460800", response.ActiveVerifiedBaud)
	assert.True(t, response.RuntimeBaudUpdated)
	assert.False(t, response.RuntimeBaudDiffersFromConfig)
}

func TestGNSSApply_ValidatesManualCurrentBaudOnlyAfterAutodetectionFails(t *testing.T) {
	yaml := defaultGNSSYAMLWithModel("/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0", "unicore", "rover_high_precision", "UM982") +
		"    gnss_execution_baud: \"115200\"\n"
	db, _ := newGNSSTestDB(t, yaml)
	docker := defaultMockDocker()
	docker.runResults = []pkgtypes.ContainerRunResult{
		{ExitCode: 1, Stdout: `{"status":"transport_unavailable","discovery":{"baud":null}}`},
		{ExitCode: 0, Stdout: `{"status":"ok","discovery":{"baud":115200},"transport":{"current_baud":115200}}`},
		{ExitCode: 0, Stdout: `{"status":"ok","transport":{"current_baud":115200,"target_baud":460800,"active_verified_baud":460800}}`},
	}
	router := setupGNSSRouter(db, docker)

	w := httptest.NewRecorder()
	req, _ := http.NewRequest("POST", "/api/settings/gnss/apply", bytes.NewReader([]byte(`{"confirm":true}`)))
	req.Header.Set("Content-Type", "application/json")
	router.ServeHTTP(w, req)

	require.Equal(t, http.StatusOK, w.Code)
	require.Len(t, docker.runSpecs, 3)
	assert.Contains(t, docker.runSpecs[0].Cmd, gnssBaudAuto)
	assert.Contains(t, docker.runSpecs[1].Cmd, "115200")
	assert.NotContains(t, docker.runSpecs[1].Cmd, gnssBaudAuto)
	assert.Contains(t, docker.runSpecs[2].Cmd, "115200")
	assert.Contains(t, docker.runSpecs[2].Cmd, "460800")
}

func TestGNSSApply_RefusesUnverifiedTargetBaud(t *testing.T) {
	db, envFile := newGNSSTestDB(t, defaultGNSSYAMLWithModel("/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0", "unicore", "rover_high_precision", "UM980"))
	docker := defaultMockDocker()
	docker.runResults = []pkgtypes.ContainerRunResult{
		{ExitCode: 0, Stdout: `{"status":"ok","warnings":[],"discovery":{"baud":115200},"transport":{"current_baud":115200},"execution_summary":{"final_status":"dry_run"}}`},
		{ExitCode: 1, Stdout: `{"status":"transport_unavailable","warnings":[],"discovery":{"baud":115200},"transport":{"current_baud":115200,"target_baud":460800,"active_verified_baud":null},"execution_summary":{"final_status":"transport_unavailable"},"error_message":"target did not answer VERSIONA"}`},
	}
	router := setupGNSSRouter(db, docker)

	w := httptest.NewRecorder()
	req, _ := http.NewRequest("POST", "/api/settings/gnss/apply", bytes.NewReader([]byte(`{"confirm":true}`)))
	req.Header.Set("Content-Type", "application/json")
	router.ServeHTTP(w, req)

	require.Equal(t, http.StatusOK, w.Code)

	envContent, err := os.ReadFile(envFile)
	require.NoError(t, err)
	assert.NotContains(t, string(envContent), "GNSS_SERIAL_BAUD=460800")

	var response GNSSActionResponse
	require.NoError(t, json.Unmarshal(w.Body.Bytes(), &response))
	assert.False(t, response.Success)
	assert.Equal(t, gnssBaudAuto, response.ExecutionBaud)
	assert.Equal(t, "115200", response.DetectedBaud)
	assert.Equal(t, "460800", response.ConfigBaud)
	assert.False(t, response.RuntimeBaudUpdated)
	assert.Empty(t, response.ActiveVerifiedBaud)
}

func TestGNSSRuntimeConfigEndpoint_ReportsSourcesAndDetectedDevices(t *testing.T) {
	db, envFile := newGNSSTestDB(t, "mowgli:\n  ros__parameters:\n    gnss_serial_baud: 921600\n")
	require.NoError(t, os.WriteFile(envFile, []byte(strings.Join([]string{
		"GNSS_RECEIVER_FAMILY=unicore",
		"GNSS_SERIAL_DEVICE=/dev/serial/by-id/usb-gnss",
		"GNSS_SERIAL_BAUD=460800",
	}, "\n")+"\n"), 0644))

	previousGlob := gnssPathGlob
	previousStat := gnssPathStat
	previousEval := gnssPathEvalLink
	gnssPathGlob = func(pattern string) ([]string, error) {
		if strings.Contains(pattern, "serial/by-id") {
			return []string{"/dev/serial/by-id/usb-gnss"}, nil
		}
		return nil, nil
	}
	gnssPathStat = func(path string) (os.FileInfo, error) {
		switch path {
		case "/dev/serial/by-id/usb-gnss", "/dev/ttyUSB1":
			return stubFileInfo{}, nil
		default:
			return nil, os.ErrNotExist
		}
	}
	gnssPathEvalLink = func(path string) (string, error) {
		if path == "/dev/serial/by-id/usb-gnss" {
			return "/dev/ttyUSB1", nil
		}
		return path, nil
	}
	t.Cleanup(func() {
		gnssPathGlob = previousGlob
		gnssPathStat = previousStat
		gnssPathEvalLink = previousEval
	})

	docker := defaultMockDocker()
	router := setupGNSSRouter(db, docker)

	w := httptest.NewRecorder()
	req, _ := http.NewRequest("GET", "/api/settings/gnss/runtime-config", nil)
	router.ServeHTTP(w, req)

	require.Equal(t, http.StatusOK, w.Code)

	var response gnssRuntimeConfigResponse
	require.NoError(t, json.Unmarshal(w.Body.Bytes(), &response))
	assert.Equal(t, "env", response.ReceiverFamily.Source)
	assert.Equal(t, "unicore", response.ReceiverFamily.ActiveValue)
	assert.Equal(t, "default", response.Transport.Source)
	assert.Equal(t, "serial", response.Transport.ActiveValue)
	assert.Equal(t, "env", response.SerialDevice.Source)
	assert.Equal(t, "/dev/serial/by-id/usb-gnss", response.SerialDevice.ActiveValue)
	assert.Equal(t, "config", response.SerialBaud.Source)
	assert.Equal(t, "921600", response.SerialBaud.ActiveValue)
	assert.Equal(t, "default", response.NTRIPEnabled.Source)
	assert.Equal(t, "true", response.NTRIPEnabled.ActiveValue)
	assert.True(t, response.ActiveSerialDeviceExists)
	assert.Equal(t, "/dev/ttyUSB1", response.ActiveSerialResolvedPath)
	require.Len(t, response.SerialDevices, 1)
	assert.Equal(t, "/dev/serial/by-id/usb-gnss", response.SerialDevices[0].Path)
	assert.Equal(t, "/dev/ttyUSB1", response.SerialDevices[0].ResolvedPath)
}

func TestGNSSApply_ReturnsClearErrorWhenSelectedDeviceIsMissing(t *testing.T) {
	db, envFile := newGNSSTestDB(t, defaultGNSSYAML("/dev/ttyUSB42", "unicore", "rover_high_precision"))
	require.NoError(t, os.WriteFile(envFile, []byte("GNSS_SERIAL_DEVICE=/dev/ttyUSB99\n"), 0644))

	previousStat := gnssPathStat
	gnssPathStat = func(path string) (os.FileInfo, error) {
		return nil, errors.New("missing device")
	}
	t.Cleanup(func() {
		gnssPathStat = previousStat
	})

	docker := defaultMockDocker()
	router := setupGNSSRouter(db, docker)

	w := httptest.NewRecorder()
	req, _ := http.NewRequest("POST", "/api/settings/gnss/apply", bytes.NewReader([]byte(`{"confirm":true}`)))
	req.Header.Set("Content-Type", "application/json")
	router.ServeHTTP(w, req)

	require.Equal(t, http.StatusBadRequest, w.Code)
	assert.Contains(t, w.Body.String(), "GNSS serial device does not exist")
	assert.Empty(t, docker.runSpecs)
}

func TestGNSSApply_RejectsInvalidProfile(t *testing.T) {
	db, _ := newGNSSTestDB(t, defaultGNSSYAML("/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0", "unicore", "danger_zone"))
	docker := defaultMockDocker()
	router := setupGNSSRouter(db, docker)

	w := httptest.NewRecorder()
	req, _ := http.NewRequest("POST", "/api/settings/gnss/apply", bytes.NewReader([]byte(`{"confirm":true}`)))
	req.Header.Set("Content-Type", "application/json")
	router.ServeHTTP(w, req)

	require.Equal(t, http.StatusBadRequest, w.Code)
	assert.Contains(t, w.Body.String(), "unsupported GNSS profile")
	assert.Empty(t, docker.runSpecs)
}

func TestGNSSApply_RejectsInvalidReceiverFamily(t *testing.T) {
	db, _ := newGNSSTestDB(t, defaultGNSSYAML("/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0", "mystery", "rover_high_precision"))
	docker := defaultMockDocker()
	router := setupGNSSRouter(db, docker)

	w := httptest.NewRecorder()
	req, _ := http.NewRequest("POST", "/api/settings/gnss/apply", bytes.NewReader([]byte(`{"confirm":true}`)))
	req.Header.Set("Content-Type", "application/json")
	router.ServeHTTP(w, req)

	require.Equal(t, http.StatusBadRequest, w.Code)
	assert.Contains(t, w.Body.String(), "unsupported GNSS receiver family")
	assert.Empty(t, docker.runSpecs)
}

func TestGNSSApply_FailureReturnsStdoutStderrWithoutRestart(t *testing.T) {
	db, _ := newGNSSTestDB(t, defaultGNSSYAML("/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0", "unicore", "rover_high_precision"))
	docker := defaultMockDocker()
	docker.runResults = []pkgtypes.ContainerRunResult{{
		ExitCode: 2,
		Stdout:   "previewed command output",
		Stderr:   "device rejected command",
	}}
	router := setupGNSSRouter(db, docker)

	w := httptest.NewRecorder()
	req, _ := http.NewRequest("POST", "/api/settings/gnss/apply", bytes.NewReader([]byte(`{"confirm":true}`)))
	req.Header.Set("Content-Type", "application/json")
	router.ServeHTTP(w, req)

	require.Equal(t, http.StatusOK, w.Code)
	require.Len(t, docker.stopCalls, 1)
	assert.Empty(t, docker.startCalls)
	assert.Equal(t, []string{"stop", "run"}, docker.events)

	var response GNSSActionResponse
	require.NoError(t, json.Unmarshal(w.Body.Bytes(), &response))
	assert.False(t, response.Success)
	assert.False(t, response.PartialFailure)
	require.Len(t, response.Executions, 1)
	assert.Equal(t, "previewed command output", response.Executions[0].Stdout)
	assert.Equal(t, "device rejected command", response.Executions[0].Stderr)
	assert.True(t, response.RestartAttempted)
	assert.True(t, response.RestartSucceeded)
}

func TestGNSSApply_MissingActiveVerifiedBaudStillReconcilesStoppedGPS(t *testing.T) {
	db, _ := newGNSSTestDB(t, defaultGNSSYAML("/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0", "unicore", "rover_high_precision"))
	docker := defaultMockDocker()
	docker.runResults = []pkgtypes.ContainerRunResult{
		{ExitCode: 0, Stdout: `{"discovery":{"baud":921600}}`},
		{ExitCode: 0, Stdout: `{"transport":{"current_baud":921600}}`},
	}
	reconcileCalls := 0
	runGNSSRuntimeReconcile = func(context.Context) error {
		reconcileCalls++
		return nil
	}
	router := setupGNSSRouter(db, docker)

	w := httptest.NewRecorder()
	req, _ := http.NewRequest("POST", "/api/settings/gnss/apply", bytes.NewReader([]byte(`{"confirm":true}`)))
	req.Header.Set("Content-Type", "application/json")
	router.ServeHTTP(w, req)

	require.Equal(t, http.StatusOK, w.Code)
	assert.Equal(t, 1, reconcileCalls)
	var response GNSSActionResponse
	require.NoError(t, json.Unmarshal(w.Body.Bytes(), &response))
	assert.True(t, response.PartialFailure)
	assert.True(t, response.RestartAttempted)
	assert.True(t, response.RestartSucceeded)
	assert.Equal(t, "GNSS apply completed without a verified active baud", response.Message)
}

func TestGNSSApply_CancelledRequestStillReconcilesStoppedGPS(t *testing.T) {
	db, _ := newGNSSTestDB(t, defaultGNSSYAML("/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0", "unicore", "rover_high_precision"))
	docker := defaultMockDocker()
	ctx, cancel := context.WithCancel(context.Background())
	docker.stopHook = cancel
	docker.runErr = assert.AnError
	reconcileCalls := 0
	runGNSSRuntimeReconcile = func(reconcileCtx context.Context) error {
		reconcileCalls++
		assert.NoError(t, reconcileCtx.Err())
		_, hasDeadline := reconcileCtx.Deadline()
		assert.True(t, hasDeadline)
		return nil
	}

	cfg, err := loadSavedGNSSConfig(db)
	require.NoError(t, err)
	_, _, err = runApplyFlow(ctx, db, docker, cfg)

	assert.ErrorIs(t, err, assert.AnError)
	assert.Equal(t, 1, reconcileCalls)
}

func TestGNSSApply_RestartFailureIsReportedAsPartialFailure(t *testing.T) {
	db, _ := newGNSSTestDB(t, defaultGNSSYAML("/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0", "unicore", "rover_high_precision"))
	docker := defaultMockDocker()
	docker.runResults = []pkgtypes.ContainerRunResult{
		{
			ExitCode: 0,
			Stdout:   `{"status":"ok","discovery":{"baud":921600},"transport":{"current_baud":921600}}`,
		},
		{
			ExitCode: 0,
			Stdout:   `{"status":"ok","transport":{"active_verified_baud":460800}}`,
		},
	}
	runGNSSRuntimeReconcile = func(context.Context) error { return assert.AnError }
	router := setupGNSSRouter(db, docker)

	w := httptest.NewRecorder()
	req, _ := http.NewRequest("POST", "/api/settings/gnss/apply", bytes.NewReader([]byte(`{"confirm":true}`)))
	req.Header.Set("Content-Type", "application/json")
	router.ServeHTTP(w, req)

	require.Equal(t, http.StatusOK, w.Code)
	assert.Equal(t, []string{"stop", "run", "run"}, docker.events)

	var response GNSSActionResponse
	require.NoError(t, json.Unmarshal(w.Body.Bytes(), &response))
	assert.False(t, response.Success)
	assert.True(t, response.PartialFailure)
	assert.True(t, response.RestartAttempted)
	assert.False(t, response.RestartSucceeded)
	assert.Contains(t, response.RestartError, "general error for testing")
	assert.Equal(t, "GNSS apply succeeded but mowgli-gps failed to restart", response.Message)
}

// TestPersistGNSSRuntimeBaud_KeepsInstalledConfigSparse guards against the
// sparseness leak found during #20: loadGNSSSettingsDocument's doc.Flat comes
// back with every schema default pre-merged in (so in-process readers see a
// complete parameter set), but persistGNSSRuntimeBaud used to write that
// FULL map straight to disk on every baud change — silently materializing
// every unrelated parameter (chassis geometry, datum, dock pose, ...) into
// the installed sparse config (Architecture Invariant 15) as a side effect
// of a routine baud probe. Uses the real schema (via chdirToGuiRoot), not a
// stub, so this proves the fix against the actual default set.
func TestPersistGNSSRuntimeBaud_KeepsInstalledConfigSparse(t *testing.T) {
	chdirToGuiRoot(t)
	resetSchemaCache()
	t.Cleanup(resetSchemaCache)

	// datum_lat is an explicit, non-GNSS value the operator set away from its
	// schema default (0.0) — it must survive persistGNSSRuntimeBaud untouched.
	yamlContent := "mowgli:\n  ros__parameters:\n    datum_lat: 48.5\n"
	db, envFile := newGNSSTestDB(t, yamlContent)

	// 115200 differs from the gnss_serial_baud schema default (921600), so it
	// is expected to persist explicitly.
	require.NoError(t, persistGNSSRuntimeBaud(db, "115200"))

	yamlPath, err := db.Get("system.mower.yamlConfigFile")
	require.NoError(t, err)
	content, err := os.ReadFile(string(yamlPath))
	require.NoError(t, err)

	assert.Contains(t, string(content), "datum_lat: 48.5")
	assert.Contains(t, string(content), "gnss_serial_baud: 115200")

	// None of these were touched by a GNSS baud persist and each equals its
	// schema default — the regression case is a hardware_settings key
	// (tool_width, wheel_radius) or an unrelated gps_settings key getting
	// materialized purely because doc.Flat carried the full merged default
	// set into the write path.
	for _, key := range []string{
		"tool_width", "wheel_radius", "chassis_length", "mowing_speed",
		"dock_pose_x", "dock_pose_y",
	} {
		assert.NotContainsf(t, string(content), key, "expected %s to stay out of the installed config", key)
	}

	envContent, err := os.ReadFile(envFile)
	require.NoError(t, err)
	assert.Contains(t, string(envContent), "GNSS_SERIAL_BAUD=115200")
}
