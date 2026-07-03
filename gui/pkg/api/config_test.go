package api

import (
	"bytes"
	"encoding/json"
	"net/http"
	"net/http/httptest"
	"testing"

	"github.com/mowglinext/mowglinext/pkg/types"
	"github.com/gin-gonic/gin"
	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
)

func setupConfigRouter(db types.IDBProvider) *gin.Engine {
	gin.SetMode(gin.TestMode)
	r := gin.New()
	group := r.Group("/api")
	ConfigRoute(group, db)
	return r
}

func TestConfigEnvRoute_Success(t *testing.T) {
	db := types.NewMockDBProvider()
	db.Set("system.map.enabled", []byte("true"))
	db.Set("system.map.tileUri", []byte("/tiles/test/{x}/{y}/{z}"))

	router := setupConfigRouter(db)

	w := httptest.NewRecorder()
	req, _ := http.NewRequest("GET", "/api/config/envs", nil)
	router.ServeHTTP(w, req)

	assert.Equal(t, http.StatusOK, w.Code)

	var resp GetConfigResponse
	err := json.Unmarshal(w.Body.Bytes(), &resp)
	require.NoError(t, err)
	assert.Equal(t, "/tiles/test/{x}/{y}/{z}", resp.TileUri)
}

func TestConfigEnvRoute_TileProxyDisabled(t *testing.T) {
	db := types.NewMockDBProvider()
	// system.map.enabled defaults to false; the tile proxy is not mounted, so
	// the env endpoint must return an empty tileUri rather than a /tiles/...
	// URL the GUI would 404 on.
	db.Set("system.map.tileUri", []byte("/tiles/test/{x}/{y}/{z}"))

	router := setupConfigRouter(db)

	w := httptest.NewRecorder()
	req, _ := http.NewRequest("GET", "/api/config/envs", nil)
	router.ServeHTTP(w, req)

	assert.Equal(t, http.StatusOK, w.Code)

	var resp GetConfigResponse
	err := json.Unmarshal(w.Body.Bytes(), &resp)
	require.NoError(t, err)
	assert.Equal(t, "", resp.TileUri)
}

func TestConfigSetKeysRoute_Success(t *testing.T) {
	db := types.NewMockDBProvider()

	router := setupConfigRouter(db)

	payload := map[string]string{
		"system.map.tileUri": "/tiles/new/{x}/{y}/{z}",
		"custom.key":         "custom-value",
	}
	body, _ := json.Marshal(payload)

	w := httptest.NewRecorder()
	req, _ := http.NewRequest("POST", "/api/config/keys/set", bytes.NewReader(body))
	req.Header.Set("Content-Type", "application/json")
	router.ServeHTTP(w, req)

	assert.Equal(t, http.StatusOK, w.Code)

	// Verify values were stored
	val, err := db.Get("system.map.tileUri")
	require.NoError(t, err)
	assert.Equal(t, "/tiles/new/{x}/{y}/{z}", string(val))

	val, err = db.Get("custom.key")
	require.NoError(t, err)
	assert.Equal(t, "custom-value", string(val))
}

func TestConfigGetKeysRoute_Success(t *testing.T) {
	db := types.NewMockDBProvider()
	db.Set("key1", []byte("value1"))
	db.Set("key2", []byte("value2"))

	router := setupConfigRouter(db)

	payload := map[string]string{
		"key1": "",
		"key2": "",
		"key3": "", // doesn't exist
	}
	body, _ := json.Marshal(payload)

	w := httptest.NewRecorder()
	req, _ := http.NewRequest("POST", "/api/config/keys/get", bytes.NewReader(body))
	req.Header.Set("Content-Type", "application/json")
	router.ServeHTTP(w, req)

	assert.Equal(t, http.StatusOK, w.Code)

	var resp map[string]interface{}
	err := json.Unmarshal(w.Body.Bytes(), &resp)
	require.NoError(t, err)

	assert.Equal(t, "value1", resp["key1"])
	assert.Equal(t, "value2", resp["key2"])
	// key3 should remain empty string since it wasn't found
	assert.Equal(t, "", resp["key3"])
}

func TestConfigGetKeysRoute_InvalidJSON(t *testing.T) {
	db := types.NewMockDBProvider()

	router := setupConfigRouter(db)

	w := httptest.NewRecorder()
	req, _ := http.NewRequest("POST", "/api/config/keys/get", bytes.NewReader([]byte("not json")))
	req.Header.Set("Content-Type", "application/json")
	router.ServeHTTP(w, req)

	// Gin's BindJSON returns 400 for malformed JSON
	assert.Equal(t, http.StatusBadRequest, w.Code)
}

func TestConfigSetKeysRoute_InvalidJSON(t *testing.T) {
	db := types.NewMockDBProvider()

	router := setupConfigRouter(db)

	w := httptest.NewRecorder()
	req, _ := http.NewRequest("POST", "/api/config/keys/set", bytes.NewReader([]byte("not json")))
	req.Header.Set("Content-Type", "application/json")
	router.ServeHTTP(w, req)

	// Gin's BindJSON returns 400 for malformed JSON
	assert.Equal(t, http.StatusBadRequest, w.Code)
}
