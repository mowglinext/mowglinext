package api

import (
	"bytes"
	"encoding/json"
	"errors"
	"net/http"
	"net/http/httptest"
	"testing"

	"github.com/gin-gonic/gin"
	"github.com/mowglinext/mowglinext/pkg/types"
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

// Only the injectable operations differ from the application's ordinary mock.
// Failure positions count attempted writes, independent of Go map iteration.
type configFailureDB struct {
	*types.MockDBProvider
	writeAttempts []string
	failWriteAt   int
	readErrors    map[string]error
}

func (db *configFailureDB) Set(key string, value []byte) error {
	db.writeAttempts = append(db.writeAttempts, key)
	if db.failWriteAt > 0 && len(db.writeAttempts) == db.failWriteAt {
		return errors.New("private-storage-detail: submitted-secret")
	}
	return db.MockDBProvider.Set(key, value)
}

func (db *configFailureDB) Get(key string) ([]byte, error) {
	if err := db.readErrors[key]; err != nil {
		return nil, err
	}
	return db.MockDBProvider.Get(key)
}

func TestConfigSetKeysRoute_PersistenceIntegrity(t *testing.T) {
	tests := []struct {
		name      string
		payload   string
		failAt    int
		status    int
		attempts  []string
		stored    map[string]string
		failedKey string
	}{
		{name: "single key", payload: `{"a":"new-a"}`, status: http.StatusOK, attempts: []string{"a"}, stored: map[string]string{"a": "new-a"}},
		{name: "multiple keys", payload: `{"c":"new-c","b":"new-b","a":"new-a"}`, status: http.StatusOK, attempts: []string{"a", "b", "c"}, stored: map[string]string{"a": "new-a", "b": "new-b", "c": "new-c"}},
		{name: "empty string", payload: `{"a":""}`, status: http.StatusOK, attempts: []string{"a"}, stored: map[string]string{"a": ""}},
		{name: "empty object", payload: `{}`, status: http.StatusOK, stored: map[string]string{"a": "old-a"}},
		{name: "whitespace after document", payload: "{\"a\":\"new-a\"} \n\t", status: http.StatusOK, attempts: []string{"a"}, stored: map[string]string{"a": "new-a"}},
		{name: "malformed JSON", payload: `{"a":`, status: http.StatusBadRequest, stored: map[string]string{"a": "old-a"}},
		{name: "empty body", payload: ``, status: http.StatusBadRequest, stored: map[string]string{"a": "old-a"}},
		{name: "top-level null", payload: `null`, status: http.StatusBadRequest, stored: map[string]string{"a": "old-a"}},
		{name: "top-level array", payload: `[]`, status: http.StatusBadRequest, stored: map[string]string{"a": "old-a"}},
		{name: "top-level string", payload: `"submitted-secret"`, status: http.StatusBadRequest, stored: map[string]string{"a": "old-a"}},
		{name: "top-level number", payload: `42`, status: http.StatusBadRequest, stored: map[string]string{"a": "old-a"}},
		{name: "second document", payload: `{"a":"new-a"}{"b":"new-b"}`, status: http.StatusBadRequest, stored: map[string]string{"a": "old-a"}},
		{name: "trailing garbage", payload: `{"a":"new-a"}submitted-secret`, status: http.StatusBadRequest, stored: map[string]string{"a": "old-a"}},
		{name: "number value", payload: `{"a":"new-a","b":42}`, status: http.StatusBadRequest, stored: map[string]string{"a": "old-a"}, failedKey: "b"},
		{name: "boolean value", payload: `{"a":"new-a","b":false}`, status: http.StatusBadRequest, stored: map[string]string{"a": "old-a"}, failedKey: "b"},
		{name: "null value", payload: `{"a":"new-a","b":null}`, status: http.StatusBadRequest, stored: map[string]string{"a": "old-a"}, failedKey: "b"},
		{name: "array value", payload: `{"a":"new-a","b":[]}`, status: http.StatusBadRequest, stored: map[string]string{"a": "old-a"}, failedKey: "b"},
		{name: "object value", payload: `{"a":"new-a","b":{}}`, status: http.StatusBadRequest, stored: map[string]string{"a": "old-a"}, failedKey: "b"},
		{name: "first write fails", payload: `{"c":"new-c","b":"new-b","a":"submitted-secret"}`, failAt: 1, status: http.StatusInternalServerError, attempts: []string{"a"}, stored: map[string]string{"a": "old-a"}, failedKey: "a"},
		{name: "failure after successful write", payload: `{"c":"new-c","b":"submitted-secret","a":"new-a"}`, failAt: 2, status: http.StatusInternalServerError, attempts: []string{"a", "b"}, stored: map[string]string{"a": "new-a"}, failedKey: "b"},
	}
	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			db := &configFailureDB{MockDBProvider: types.NewMockDBProvider(), failWriteAt: tt.failAt}
			require.NoError(t, db.MockDBProvider.Set("a", []byte("old-a")))
			router := setupConfigRouter(db)
			response := httptest.NewRecorder()
			request := httptest.NewRequest(http.MethodPost, "/api/config/keys/set", bytes.NewBufferString(tt.payload))
			request.Header.Set("Content-Type", "application/json")
			router.ServeHTTP(response, request)
			require.Equal(t, tt.status, response.Code, response.Body.String())
			assert.Equal(t, tt.attempts, db.writeAttempts)
			assert.Contains(t, response.Header().Get("Content-Type"), "application/json")
			for key, expected := range tt.stored {
				value, err := db.MockDBProvider.Get(key)
				require.NoError(t, err)
				assert.Equal(t, expected, string(value))
			}
			if tt.status == http.StatusOK {
				var expected map[string]string
				require.NoError(t, json.Unmarshal([]byte(tt.payload), &expected))
				var actual map[string]string
				require.NoError(t, json.Unmarshal(response.Body.Bytes(), &actual))
				assert.Equal(t, expected, actual)
			} else {
				var envelope map[string]json.RawMessage
				require.NoError(t, json.Unmarshal(response.Body.Bytes(), &envelope))
				require.Len(t, envelope, 1)
				var message string
				require.NoError(t, json.Unmarshal(envelope["error"], &message))
				require.NotEmpty(t, message)
				if tt.failedKey != "" {
					assert.Contains(t, message, `"`+tt.failedKey+`"`)
				}
				assert.NotContains(t, response.Body.String(), "submitted-secret")
				assert.NotContains(t, response.Body.String(), "private-storage-detail")
				assert.NotContains(t, response.Body.String(), "new-a")
				assert.NotContains(t, response.Body.String(), "old-a")
			}
			if tt.failAt > 0 {
				// The failed and later keys are not acknowledged or retried, and
				// an earlier successful write is left in place rather than rolled back.
				for _, key := range []string{"b", "c"} {
					_, err := db.MockDBProvider.Get(key)
					assert.Error(t, err)
				}
				assert.Contains(t, response.Body.String(), "earlier keys may already have been stored")
			}
		})
	}
}

func TestConfigGetKeysRoute_LegacyReadContract(t *testing.T) {
	tests := []struct {
		name        string
		value       *string
		readError   error
		placeholder string
		expected    string
	}{
		{name: "stored value", value: stringPointer("stored"), placeholder: "caller-default", expected: "stored"},
		{name: "stored empty string", value: stringPointer(""), placeholder: "caller-default", expected: ""},
		{name: "missing with empty placeholder", placeholder: "", expected: ""},
		{name: "missing with legacy placeholder", placeholder: "caller-default", expected: "caller-default"},
		// IDBProvider.Get does not classify missing vs operational errors. Keep
		// this documented compatibility limitation until the provider contract
		// has a typed distinction; never guess by matching error-message text.
		{name: "unclassified operational read error", readError: errors.New("private-read-failure"), placeholder: "caller-default", expected: "caller-default"},
	}
	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			db := &configFailureDB{MockDBProvider: types.NewMockDBProvider(), readErrors: map[string]error{"key": tt.readError}}
			if tt.value != nil {
				require.NoError(t, db.MockDBProvider.Set("key", []byte(*tt.value)))
			}
			body, err := json.Marshal(map[string]string{"key": tt.placeholder})
			require.NoError(t, err)
			response := httptest.NewRecorder()
			setupConfigRouter(db).ServeHTTP(response, httptest.NewRequest(http.MethodPost, "/api/config/keys/get", bytes.NewReader(body)))
			require.Equal(t, http.StatusOK, response.Code)
			var actual map[string]string
			require.NoError(t, json.Unmarshal(response.Body.Bytes(), &actual))
			assert.Equal(t, map[string]string{"key": tt.expected}, actual)
			assert.Empty(t, db.writeAttempts)
			assert.NotContains(t, response.Body.String(), "private-read-failure")
		})
	}
}

func stringPointer(value string) *string { return &value }
