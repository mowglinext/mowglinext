package api

import (
	"bytes"
	"encoding/json"
	"errors"
	"net/http"
	"net/http/httptest"
	"testing"

	"github.com/cedbossneo/mowglinext/pkg/types"
	"github.com/gin-gonic/gin"
	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
)

func newParamsRouter(ros types.IRosProvider) *gin.Engine {
	gin.SetMode(gin.TestMode)
	r := gin.New()
	ParamsRoutes(r.Group("/api"), ros)
	return r
}

func TestGetParams_ReturnsList(t *testing.T) {
	ros := types.NewMockRosProvider()
	ros.Parameters = []types.RosParameter{
		{Name: "fusion_graph_node.node_period_s", Value: 0.1, Type: "float64"},
		{Name: "map_server_node.tool_width", Value: 0.18, Type: "float64"},
	}
	r := newParamsRouter(ros)

	w := httptest.NewRecorder()
	r.ServeHTTP(w, httptest.NewRequest(http.MethodGet, "/api/params", nil))
	require.Equal(t, http.StatusOK, w.Code)

	var resp ParamsListResponse
	require.NoError(t, json.Unmarshal(w.Body.Bytes(), &resp))
	assert.Len(t, resp.Parameters, 2)
	assert.Equal(t, "fusion_graph_node.node_period_s", resp.Parameters[0].Name)
}

func TestGetParams_BridgeError(t *testing.T) {
	ros := types.NewMockRosProvider()
	ros.ParamErr = errors.New("not connected")
	r := newParamsRouter(ros)

	w := httptest.NewRecorder()
	r.ServeHTTP(w, httptest.NewRequest(http.MethodGet, "/api/params", nil))
	assert.Equal(t, http.StatusServiceUnavailable, w.Code)
}

func TestSetParams_UpdatesAndEchoes(t *testing.T) {
	ros := types.NewMockRosProvider()
	r := newParamsRouter(ros)

	body, _ := json.Marshal(SetParamsRequest{Parameters: []types.RosParameter{
		{Name: "map_server_node.tool_width", Value: 0.22},
	}})
	w := httptest.NewRecorder()
	req := httptest.NewRequest(http.MethodPost, "/api/params", bytes.NewReader(body))
	req.Header.Set("Content-Type", "application/json")
	r.ServeHTTP(w, req)

	require.Equal(t, http.StatusOK, w.Code)
	require.Len(t, ros.SetParams, 1)
	assert.Equal(t, "map_server_node.tool_width", ros.SetParams[0][0].Name)
	assert.Equal(t, 0.22, ros.SetParams[0][0].Value)
}

func TestSetParams_RejectsEmpty(t *testing.T) {
	ros := types.NewMockRosProvider()
	r := newParamsRouter(ros)

	body, _ := json.Marshal(SetParamsRequest{Parameters: []types.RosParameter{}})
	w := httptest.NewRecorder()
	req := httptest.NewRequest(http.MethodPost, "/api/params", bytes.NewReader(body))
	req.Header.Set("Content-Type", "application/json")
	r.ServeHTTP(w, req)

	assert.Equal(t, http.StatusBadRequest, w.Code)
}
