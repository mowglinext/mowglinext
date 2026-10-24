package api

import (
	"bytes"
	"context"
	"encoding/json"
	"errors"
	"net/http/httptest"
	"strings"
	"testing"

	"github.com/gin-gonic/gin"
	"github.com/mowglinext/mowglinext/pkg/msgs/mowgli"
	"github.com/mowglinext/mowglinext/pkg/types"
	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
)

func corridorRouter(t *testing.T, ros *types.MockRosProvider, body string) *httptest.ResponseRecorder {
	t.Helper()
	gin.SetMode(gin.TestMode)
	r := gin.New()
	ServiceRoute(r.Group("/api/mowglinext"), ros)
	w := httptest.NewRecorder()
	q := httptest.NewRequest("POST", "/api/mowglinext/call/set_lidar_ignore_corridors", bytes.NewBufferString(body))
	q.Header.Set("Content-Type", "application/json")
	r.ServeHTTP(w, q)
	return w
}
func TestCorridorReplace_RequiresExplicitArray(t *testing.T) {
	for _, body := range []string{`{}`, `null`, `{"corridors":null}`, `{"corridors":{}}`, `{"corridors":"x"}`, `{"corridors":[]} {}`, `{"corridors":[{"id":4294967296}]}`, `{"corridors":[{"id":-1}]}`} {
		t.Run(body, func(t *testing.T) {
			ros := types.NewMockRosProvider()
			w := corridorRouter(t, ros, body)
			assert.Equal(t, 400, w.Code, w.Body.String())
			assert.Empty(t, ros.ServiceCalls)
		})
	}
	t.Run("bounded-body", func(t *testing.T) {
		ros := types.NewMockRosProvider()
		w := corridorRouter(t, ros, `{"corridors":[],"extra":"`+strings.Repeat("a", (1<<20)+1)+`"}`)
		assert.Equal(t, 413, w.Code)
		assert.Empty(t, ros.ServiceCalls)
	})
}
func TestCorridorReplace_FunctionalResults(t *testing.T) {
	for _, tc := range []struct {
		outcome                                  string
		success, committed, persisted, published bool
		want                                     int
	}{
		{"COMMITTED", true, true, true, true, 200},
		{"VALIDATION_REJECTED", false, false, false, false, 422},
		{"PERSISTENCE_NOT_CONFIGURED", false, false, false, false, 503},
		{"PERSISTENCE_FAILED_BEFORE_COMMIT", false, false, false, false, 500},
		{"COMMITTED_DURABILITY_UNCONFIRMED", false, true, false, true, 503},
		{"COMMITTED_DURABILITY_UNCONFIRMED", false, true, false, false, 503},
		{"COMMITTED_PUBLICATION_PENDING", false, true, true, false, 503},
		{"", false, false, false, false, 502},
		{"COMMITTED", true, false, false, false, 502},
	} {
		t.Run(tc.outcome, func(t *testing.T) {
			ros := types.NewMockRosProvider()
			ros.ServiceResponder = func(_ string, req, res any) {
				if r, ok := res.(*mowgli.ReplaceLidarIgnoreCorridorsRes); ok {
					r.Success = tc.success
					r.Outcome = tc.outcome
					r.Committed = tc.committed
					r.Persisted = tc.persisted
					r.Published = tc.published
					r.Message = "producer detail"
					r.Corridors = []mowgli.LidarIgnoreCorridor{}
				}
			}
			w := corridorRouter(t, ros, `{"corridors":[]}`)
			assert.Equal(t, tc.want, w.Code, w.Body.String())
			require.Len(t, ros.ServiceCalls, 1, "no Clear/Add fallback")
			assert.Equal(t, "/map_server_node/replace_lidar_ignore_corridors", ros.ServiceCalls[0].Service)
			req, ok := ros.ServiceCalls[0].Req.(*mowgli.ReplaceLidarIgnoreCorridorsReq)
			require.True(t, ok)
			assert.NotNil(t, req.Corridors)
			assert.NotEmpty(t, req.ClientOperationId)
			var actual map[string]any
			require.NoError(t, json.Unmarshal(w.Body.Bytes(), &actual))
			if tc.want != 502 {
				assert.Equal(t, tc.outcome, actual["outcome"])
				assert.Equal(t, tc.committed, actual["committed"])
				assert.Equal(t, tc.persisted, actual["persisted"])
				assert.Equal(t, tc.published, actual["published"])
			} else {
				assert.Equal(t, "UNKNOWN_AFTER_SEND", actual["outcome"])
				assert.Nil(t, actual["committed"])
			}
		})
	}
}
func TestCorridorReplace_UnknownTransportNoRetry(t *testing.T) {
	for _, tc := range []struct {
		name string
		err  error
		want int
	}{
		{"lost-response", errors.New("connection closed after application"), 502},
		{"deadline-after-commit", context.DeadlineExceeded, 504},
		{"older-ROS-image", errors.New("foxglove: service not advertised"), 502},
	} {
		t.Run(tc.name, func(t *testing.T) {
			ros := types.NewMockRosProvider()
			ros.ServiceErr = tc.err
			w := corridorRouter(t, ros, `{"corridors":[]}`)
			assert.Equal(t, tc.want, w.Code)
			require.Len(t, ros.ServiceCalls, 1)
			assert.Equal(t, "/map_server_node/replace_lidar_ignore_corridors", ros.ServiceCalls[0].Service)
			var actual map[string]any
			require.NoError(t, json.Unmarshal(w.Body.Bytes(), &actual))
			assert.Equal(t, "UNKNOWN_AFTER_SEND", actual["outcome"])
			assert.Nil(t, actual["committed"])
			assert.NotEmpty(t, actual["error"])
		})
	}
}

func TestCorridorReplace_CanceledBehindOtherMapWriterDoesNotSend(t *testing.T) {
	require.NoError(t, lockMapReplacement(context.Background(), nil))
	defer unlockMapReplacement()
	ctx, cancel := context.WithCancel(context.Background())
	cancel()
	ros := types.NewMockRosProvider()
	gin.SetMode(gin.TestMode)
	router := gin.New()
	ServiceRoute(router.Group("/api/mowglinext"), ros)
	req := httptest.NewRequest("POST", "/api/mowglinext/call/set_lidar_ignore_corridors", bytes.NewBufferString(`{"corridors":[]}`)).WithContext(ctx)
	req.Header.Set("Content-Type", "application/json")
	w := httptest.NewRecorder()
	router.ServeHTTP(w, req)
	assert.Equal(t, 503, w.Code)
	assert.Empty(t, ros.ServiceCalls)
	assert.Contains(t, w.Body.String(), "NOT_SENT")
}
