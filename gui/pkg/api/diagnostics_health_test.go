package api

import (
	"encoding/json"
	"errors"
	docker "github.com/docker/docker/api/types"
	"net/http/httptest"
	"os"
	"path/filepath"
	"testing"

	"github.com/gin-gonic/gin"
	"github.com/mowglinext/mowglinext/pkg/types"
	"github.com/stretchr/testify/require"
)

func TestDiagnosticsSnapshotReportsDockerAvailability(t *testing.T) {
	for _, unavailable := range []bool{false, true} {
		t.Run(map[bool]string{false: "empty but available", true: "unavailable"}[unavailable], func(t *testing.T) {
			docker := &versionDocker{}
			if unavailable {
				docker.listErr = errors.New("fixture docker unavailable")
			}
			gin.SetMode(gin.TestMode)
			r := gin.New()
			r.GET("/snapshot", getDiagnosticsSnapshot(docker, nil, types.NewMockDBProvider()))
			response := httptest.NewRecorder()
			r.ServeHTTP(response, httptest.NewRequest("GET", "/snapshot", nil))
			var result map[string]json.RawMessage
			require.NoError(t, json.Unmarshal(response.Body.Bytes(), &result))
			require.JSONEq(t, map[bool]string{false: "true", true: "false"}[unavailable], string(result["containers_available"]))
		})
	}
}

func TestCPUTemperatureAvailability(t *testing.T) {
	for _, test := range []struct {
		name, raw string
		value     float64
		valid     bool
	}{
		{"zero", "0\n", 0, true}, {"normal", "40000", 40, true}, {"negative", "-1000", -1, true},
		{"malformed", "not a number", 0, false}, {"nan", "NaN", 0, false}, {"infinite", "+Inf", 0, false}, {"empty", "", 0, false},
	} {
		t.Run(test.name, func(t *testing.T) {
			path := filepath.Join(t.TempDir(), "temperature")
			require.NoError(t, os.WriteFile(path, []byte(test.raw), 0600))
			actual, valid := readCPUTemperature(path)
			require.Equal(t, test.valid, valid)
			require.Equal(t, test.value, actual)
		})
	}
	_, valid := readCPUTemperature(filepath.Join(t.TempDir(), "missing"))
	require.False(t, valid)
}

func TestDiagnosticsSnapshotPreservesCoreRoleMetadata(t *testing.T) {
	provider := &versionDocker{containers: []docker.Container{
		{Names: []string{"/custom-robot-name"}, State: "exited", Labels: map[string]string{"garden.mowgli.update.image": "mowgli-ros2"}},
		{Names: []string{"/unrelated"}, State: "running"},
	}}
	gin.SetMode(gin.TestMode)
	r := gin.New()
	r.GET("/snapshot", getDiagnosticsSnapshot(provider, nil, types.NewMockDBProvider()))
	response := httptest.NewRecorder()
	r.ServeHTTP(response, httptest.NewRequest("GET", "/snapshot", nil))
	var result DiagnosticsSnapshot
	require.NoError(t, json.Unmarshal(response.Body.Bytes(), &result))
	require.True(t, result.ContainersAvailable)
	require.Equal(t, "mowgli-ros2", result.Containers[0].Component)
	require.Empty(t, result.Containers[1].Component)
	require.Equal(t, "exited", result.Containers[0].State)
	require.Equal(t, "no-store", response.Header().Get("Cache-Control"))
}
