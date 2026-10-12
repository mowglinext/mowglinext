package api

import (
	"bytes"
	"net/http"
	"net/http/httptest"
	"os"
	"path/filepath"
	"testing"

	"github.com/mowglinext/mowglinext/pkg/types"
	"github.com/stretchr/testify/require"
)

func TestPostSettingsYAML_PreservesMalformedDocument(t *testing.T) {
	for _, document := range []string{
		"mowgli:\n  ros__parameters:\n    broken: [unterminated\n",
		"mowgli:\n  ros__parameters:\n    datum_lat: 48.1\n    datum_lat: 49.1\n",
		"- this is a sequence rather than a configuration mapping\n",
	} {
		t.Run(document, func(t *testing.T) {
			seedSchemaCache(t, "datum_lat")
			path := createTempYAMLFile(t, document)
			envPath := filepath.Join(t.TempDir(), "runtime.env")
			env := []byte("GNSS_DEVICE=/dev/original\n")
			require.NoError(t, os.WriteFile(envPath, env, 0600))
			db := types.NewMockDBProvider()
			db.Set("system.mower.yamlConfigFile", []byte(path))
			db.Set("system.mower.runtimeEnvFile", []byte(envPath))
			router := setupSettingsRouter(db)
			req := httptest.NewRequest(http.MethodPost, "/api/settings/yaml", bytes.NewBufferString(`{"datum_lat":48.5,"gnss_device":"/dev/replacement"}`))
			req.Header.Set("Content-Type", "application/json")
			response := httptest.NewRecorder()
			router.ServeHTTP(response, req)
			require.Equal(t, http.StatusInternalServerError, response.Code)
			require.Contains(t, response.Body.String(), "repair the configuration before saving")
			out, err := os.ReadFile(path)
			require.NoError(t, err)
			require.Equal(t, []byte(document), out)
			out, err = os.ReadFile(envPath)
			require.NoError(t, err)
			require.Equal(t, env, out)
		})
	}
}

func TestPostSettingsYAML_RejectsUnreadableExistingPath(t *testing.T) {
	seedSchemaCache(t, "datum_lat")
	// A directory produces a deterministic read error even for privileged users.
	path := t.TempDir()
	marker := filepath.Join(path, "original")
	require.NoError(t, os.WriteFile(marker, []byte("keep"), 0600))
	db := types.NewMockDBProvider()
	db.Set("system.mower.yamlConfigFile", []byte(path))
	req := httptest.NewRequest(http.MethodPost, "/api/settings/yaml", bytes.NewBufferString(`{"datum_lat":48.5}`))
	req.Header.Set("Content-Type", "application/json")
	response := httptest.NewRecorder()
	setupSettingsRouter(db).ServeHTTP(response, req)
	require.Equal(t, http.StatusInternalServerError, response.Code)
	require.Contains(t, response.Body.String(), "failed to read existing YAML")
	out, err := os.ReadFile(marker)
	require.NoError(t, err)
	require.Equal(t, "keep", string(out))
}
