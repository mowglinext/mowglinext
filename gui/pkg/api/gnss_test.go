package api

import (
	"bytes"
	"context"
	"encoding/json"
	"io"
	"net/http"
	"net/http/httptest"
	"os"
	"strings"
	"testing"

	pkgtypes "github.com/cedbossneo/mowglinext/pkg/types"
	dockertypes "github.com/docker/docker/api/types"
	"github.com/gin-gonic/gin"
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

func newGNSSTestDB(t *testing.T, yamlContent string) (*pkgtypes.MockDBProvider, string) {
	t.Helper()
	yamlFile := writeGNSSConfigFile(t, yamlContent)
	envFile := createTempConfigFile(t, "ROS_DOMAIN_ID=0\n")
	db := pkgtypes.NewMockDBProvider()
	require.NoError(t, db.Set("system.mower.yamlConfigFile", []byte(yamlFile)))
	require.NoError(t, db.Set("system.mower.runtimeEnvFile", []byte(envFile)))
	return db, envFile
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

func TestGNSSPlan_DoesNotRequireConfirmAndAvoidsSerialAccess(t *testing.T) {
	db, _ := newGNSSTestDB(t, defaultGNSSYAML("/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0", "unicore", "rover_high_precision"))
	docker := defaultMockDocker()
	docker.runResults = []pkgtypes.ContainerRunResult{{ExitCode: 0, Stdout: "ok"}}

	router := setupGNSSRouter(db, docker)

	w := httptest.NewRecorder()
	req, _ := http.NewRequest("POST", "/api/settings/gnss/plan", nil)
	router.ServeHTTP(w, req)

	require.Equal(t, http.StatusOK, w.Code)
	require.Len(t, docker.stopCalls, 0)
	require.Len(t, docker.runSpecs, 1)
	assert.Empty(t, docker.runSpecs[0].Binds)
	assert.False(t, docker.runSpecs[0].Privileged)
	assert.Equal(t, gnssConfigPlanBinary, docker.runSpecs[0].Cmd[0])
	assert.Contains(t, docker.runSpecs[0].Cmd, "--config-baud")
	assert.Contains(t, docker.runSpecs[0].Cmd, "460800")

	var response GNSSActionResponse
	require.NoError(t, json.Unmarshal(w.Body.Bytes(), &response))
	assert.True(t, response.Success)
	assert.Equal(t, "plan", response.Action)
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
	docker.runResults = []pkgtypes.ContainerRunResult{{ExitCode: 0, Stdout: "apply ok"}}
	router := setupGNSSRouter(db, docker)

	w := httptest.NewRecorder()
	req, _ := http.NewRequest("POST", "/api/settings/gnss/apply", bytes.NewReader([]byte(`{"confirm":true}`)))
	req.Header.Set("Content-Type", "application/json")
	router.ServeHTTP(w, req)

	require.Equal(t, http.StatusOK, w.Code)
	require.Len(t, docker.stopCalls, 1)
	require.Len(t, docker.startCalls, 1)
	require.Len(t, docker.runSpecs, 1)
	assert.Equal(t, []string{"stop", "run", "start"}, docker.events)
	assert.Equal(t, []string{"/dev:/dev"}, docker.runSpecs[0].Binds)
	assert.True(t, docker.runSpecs[0].Privileged)
	assert.Equal(t, gnssConfigApplyBinary, docker.runSpecs[0].Cmd[0])
	assert.Contains(t, docker.runSpecs[0].Cmd, "--config-baud")
	assert.Contains(t, docker.runSpecs[0].Cmd, "460800")
	assert.Contains(t, docker.runSpecs[0].Cmd, "--device")
	assert.Contains(t, docker.runSpecs[0].Cmd, "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0")

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
	assert.Contains(t, strings.Join(response.Warnings, "\n"), "GNSS_SIGNAL_PROFILE is persisted in the UI")
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
	assert.False(t, response.RestartAttempted)
}

func TestGNSSApply_RestartFailureIsReportedAsPartialFailure(t *testing.T) {
	db, _ := newGNSSTestDB(t, defaultGNSSYAML("/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0", "unicore", "rover_high_precision"))
	docker := defaultMockDocker()
	docker.runResults = []pkgtypes.ContainerRunResult{{ExitCode: 0, Stdout: "apply ok"}}
	docker.startErr = assert.AnError
	router := setupGNSSRouter(db, docker)

	w := httptest.NewRecorder()
	req, _ := http.NewRequest("POST", "/api/settings/gnss/apply", bytes.NewReader([]byte(`{"confirm":true}`)))
	req.Header.Set("Content-Type", "application/json")
	router.ServeHTTP(w, req)

	require.Equal(t, http.StatusOK, w.Code)
	assert.Equal(t, []string{"stop", "run", "start"}, docker.events)

	var response GNSSActionResponse
	require.NoError(t, json.Unmarshal(w.Body.Bytes(), &response))
	assert.False(t, response.Success)
	assert.True(t, response.PartialFailure)
	assert.True(t, response.RestartAttempted)
	assert.False(t, response.RestartSucceeded)
	assert.Contains(t, response.RestartError, "general error for testing")
	assert.Equal(t, "GNSS apply succeeded but mowgli-gps failed to restart", response.Message)
}
