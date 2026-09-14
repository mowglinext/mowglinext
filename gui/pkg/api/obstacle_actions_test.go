package api

import (
	"net/http"
	"net/http/httptest"
	"strings"
	"testing"

	"github.com/mowglinext/mowglinext/pkg/msgs/mowgli"
	"github.com/mowglinext/mowglinext/pkg/types"
	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
)

func TestObstacleDismissRoutes(t *testing.T) {
	for command, service := range map[string]string{
		"ignore_obstacle":  "/obstacle_tracker/clear_obstacle",
		"discard_obstacle": "/map_server_node/discard_obstacle",
	} {
		for _, outcome := range []string{"success", "not found", "unavailable", "invalid body"} {
			t.Run(command+"/"+outcome, func(t *testing.T) {
				mock := types.NewMockRosProvider()
				mock.ServiceResponder = func(_ string, req any, res any) {
					assert.Equal(t, uint32(42), req.(*mowgli.ClearObstacleReq).ObstacleId)
					response := res.(*mowgli.ClearObstacleRes)
					response.Success = outcome == "success"
					response.Message = outcome
				}
				if outcome == "unavailable" {
					mock.ServiceErr = assert.AnError
				}
				body := `{"obstacle_id":42}`
				if outcome == "invalid body" {
					body = `{"obstacle_id":`
				}
				req := httptest.NewRequest(http.MethodPost, "/api/mowglinext/call/"+command, strings.NewReader(body))
				req.Header.Set("Content-Type", "application/json")
				w := httptest.NewRecorder()
				setupMowgliNextRouter(mock).ServeHTTP(w, req)
				if outcome == "invalid body" {
					assert.Equal(t, http.StatusBadRequest, w.Code)
					assert.Empty(t, mock.ServiceCalls)
					return
				}
				require.Len(t, mock.ServiceCalls, 1)
				assert.Equal(t, service, mock.ServiceCalls[0].Service)
				if outcome == "success" {
					assert.Equal(t, http.StatusOK, w.Code)
					assert.JSONEq(t, `{"message":"success"}`, w.Body.String())
				} else {
					assert.Equal(t, http.StatusInternalServerError, w.Code)
					assert.Contains(t, w.Body.String(), `"error"`)
				}
			})
		}
	}
}
