package api

import (
	"encoding/json"
	"errors"
	"fmt"
	"net/http"
	"net/http/httptest"
	"testing"

	"github.com/gin-gonic/gin"
	"github.com/mowglinext/mowglinext/pkg/providers"
	"github.com/mowglinext/mowglinext/pkg/types"
	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
)

type statusContractDB struct {
	types.IDBProvider
	gets int
	sets int
}

func (d *statusContractDB) Get(key string) ([]byte, error) {
	d.gets++
	return d.IDBProvider.Get(key)
}
func (d *statusContractDB) Set(key string, value []byte) error {
	d.sets++
	return d.IDBProvider.Set(key, value)
}
func statusContractRouter(db types.IDBProvider) *gin.Engine {
	gin.SetMode(gin.TestMode)
	r := gin.New()
	group := r.Group("/api")
	GetSettingsStatus(group, db)
	PostSettingsStatus(group, db)
	return r
}
func statusContractRequest(r *gin.Engine, method string) *httptest.ResponseRecorder {
	w := httptest.NewRecorder()
	r.ServeHTTP(w, httptest.NewRequest(method, "/api/settings/status", nil))
	return w
}
func TestSettingsStatus_GetContract(t *testing.T) {
	for _, tc := range []struct {
		name  string
		value *string
		want  bool
	}{
		{"missing", nil, false}, {"true", statusValue("true"), true}, {"false", statusValue("false"), false},
		{"empty", statusValue(""), false}, {"nonstandard", statusValue("TRUE"), false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			storage := types.NewMockDBProvider()
			if tc.value != nil {
				require.NoError(t, storage.Set("onboarding.completed", []byte(*tc.value)))
			}
			db := &statusContractDB{IDBProvider: storage}
			w := statusContractRequest(statusContractRouter(db), http.MethodGet)
			require.Equal(t, 200, w.Code)
			assert.JSONEq(t, `{"onboarding_completed":`+map[bool]string{true: "true", false: "false"}[tc.want]+`}`, w.Body.String())
			assert.Equal(t, 1, db.gets)
			assert.Zero(t, db.sets)
		})
	}
}
func TestSettingsStatus_ReadFailure(t *testing.T) {
	for _, failure := range []error{errors.New("storage I/O failure"), errors.New("permission denied"), errors.New("corrupt record")} {
		t.Run(failure.Error(), func(t *testing.T) {
			storage := types.NewMockDBProvider()
			require.NoError(t, storage.Set("onboarding.completed", []byte("true")))
			storage.GetErr = failure
			db := &statusContractDB{IDBProvider: storage}
			w := statusContractRequest(statusContractRouter(db), http.MethodGet)
			require.Equal(t, 500, w.Code)
			var body map[string]any
			require.NoError(t, json.Unmarshal(w.Body.Bytes(), &body))
			assert.Equal(t, failure.Error(), body["error"])
			assert.NotContains(t, body, "onboarding_completed")
			assert.Zero(t, db.sets)
		})
	}
}
func TestSettingsStatus_PostContract(t *testing.T) {
	for _, failure := range []error{nil, errors.New("write failed")} {
		t.Run(map[bool]string{true: "failure", false: "success"}[failure != nil], func(t *testing.T) {
			storage := types.NewMockDBProvider()
			storage.SetErr = failure
			db := &statusContractDB{IDBProvider: storage}
			r := statusContractRouter(db)
			w := statusContractRequest(r, http.MethodPost)
			assert.Equal(t, 1, db.sets)
			assert.Zero(t, db.gets)
			if failure != nil {
				assert.Equal(t, 500, w.Code)
				assert.JSONEq(t, `{"error":"write failed"}`, w.Body.String())
				return
			}
			assert.Equal(t, 200, w.Code)
			assert.JSONEq(t, `{}`, w.Body.String())
			get := statusContractRequest(r, http.MethodGet)
			assert.Equal(t, 200, get.Code)
			assert.JSONEq(t, `{"onboarding_completed":true}`, get.Body.String())
			assert.Equal(t, 1, db.sets)
		})
	}
}
func TestSettingsStatus_FreshDatabase(t *testing.T) {
	t.Setenv("DB_PATH", t.TempDir())
	db := &statusContractDB{IDBProvider: providers.NewDBProvider()}
	r := statusContractRouter(db)
	first := statusContractRequest(r, http.MethodGet)
	assert.Equal(t, 200, first.Code)
	assert.JSONEq(t, `{"onboarding_completed":false}`, first.Body.String())
	assert.Zero(t, db.sets)
	post := statusContractRequest(r, http.MethodPost)
	require.Equal(t, 200, post.Code)
	after := statusContractRequest(r, http.MethodGet)
	assert.Equal(t, 200, after.Code)
	assert.JSONEq(t, `{"onboarding_completed":true}`, after.Body.String())
	assert.Equal(t, 1, db.sets)
}
func statusValue(value string) *string { return &value }

func TestSettingsStatus_WrappedMissingKey(t *testing.T) {
	storage := types.NewMockDBProvider()
	storage.GetErr = fmt.Errorf("lookup: %w", types.ErrKeyNotFound)
	db := &statusContractDB{IDBProvider: storage}
	w := statusContractRequest(statusContractRouter(db), http.MethodGet)
	assert.Equal(t, 200, w.Code)
	assert.JSONEq(t, `{"onboarding_completed":false}`, w.Body.String())
	assert.Zero(t, db.sets)
}
