package api

import (
	"bytes"
	"encoding/json"
	"net/http"
	"net/http/httptest"
	"strings"
	"testing"
	"time"

	"github.com/gin-gonic/gin"
	"github.com/mowglinext/mowglinext/pkg/providers"
	"github.com/mowglinext/mowglinext/pkg/types"
	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
)

type checkedScheduleStore struct {
	*types.MockDBProvider
	unlocked []string
}

func (s *checkedScheduleStore) check(operation, key string) {
	if strings.HasPrefix(key, scheduleKeyPrefix) && providers.ScheduleWriteMu.TryLock() {
		providers.ScheduleWriteMu.Unlock()
		s.unlocked = append(s.unlocked, operation)
	}
}
func (s *checkedScheduleStore) Get(key string) ([]byte, error) {
	s.check("get", key)
	return s.MockDBProvider.Get(key)
}
func (s *checkedScheduleStore) Set(key string, value []byte) error {
	s.check("set", key)
	return s.MockDBProvider.Set(key, value)
}
func (s *checkedScheduleStore) Delete(key string) error {
	s.check("delete", key)
	return s.MockDBProvider.Delete(key)
}

func TestScheduleStoreHTTPAndMQTTShareMetadataTransaction(t *testing.T) {
	for _, action := range []string{"http-create", "http-update", "http-delete", "mqtt-update", "mqtt-delete"} {
		t.Run(action, func(t *testing.T) {
			stamp := time.Date(2026, 10, 21, 9, 0, 0, 0, time.UTC)
			store := &checkedScheduleStore{MockDBProvider: types.NewMockDBProvider()}
			seed := Schedule{ID: "existing", Time: "09:00", DaysOfWeek: []int{1}, Enabled: true, CreatedAt: stamp, LastRun: &stamp, LastSkipReason: "soil wet", LastSkippedAt: &stamp}
			data, err := json.Marshal(seed)
			require.NoError(t, err)
			require.NoError(t, store.MockDBProvider.Set(scheduleKeyPrefix+seed.ID, data))
			body := []byte(`{"id":"existing","time":"10:30","daysOfWeek":[2],"enabled":false}`)
			if strings.HasPrefix(action, "http") {
				gin.SetMode(gin.TestMode)
				r := gin.New()
				ScheduleRoutes(r.Group("/api"), store)
				method, path := http.MethodPut, "/api/schedules/existing"
				if action == "http-create" {
					method, path = http.MethodPost, "/api/schedules"
				}
				if action == "http-delete" {
					method = http.MethodDelete
				}
				w := httptest.NewRecorder()
				req := httptest.NewRequest(method, path, bytes.NewReader(body))
				req.Header.Set("Content-Type", "application/json")
				r.ServeHTTP(w, req)
				require.Equal(t, http.StatusOK, w.Code, w.Body.String())
			} else {
				b := &ScheduleMqttBridge{dbProvider: store}
				if action == "mqtt-delete" {
					b.handleDelete([]byte("existing"), false)
				} else {
					b.handleSet(body, false)
				}
			}
			assert.Empty(t, store.unlocked, "every mutation and update re-read must participate in the scheduler's lock")
			data, err = store.MockDBProvider.Get(scheduleKeyPrefix + seed.ID)
			if strings.HasSuffix(action, "delete") {
				assert.ErrorIs(t, err, types.ErrKeyNotFound)
				return
			}
			require.NoError(t, err)
			var current Schedule
			require.NoError(t, json.Unmarshal(data, &current))
			if strings.HasSuffix(action, "update") {
				assert.False(t, current.Enabled)
				assert.Equal(t, "10:30", current.Time)
				assert.Equal(t, []int{2}, current.DaysOfWeek)
				assert.Equal(t, seed.LastRun, current.LastRun)
				assert.Equal(t, seed.LastSkippedAt, current.LastSkippedAt)
				assert.Equal(t, seed.LastSkipReason, current.LastSkipReason)
			}
		})
	}
}
