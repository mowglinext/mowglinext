package api

import (
	"bytes"
	"encoding/json"
	"net/http"
	"net/http/httptest"
	"reflect"
	"testing"

	"github.com/mowglinext/mowglinext/pkg/types"
	"github.com/gin-gonic/gin"
	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
)

func newFirmwareDebugRouter(rosProvider types.IRosProvider) *gin.Engine {
	gin.SetMode(gin.TestMode)
	r := gin.New()
	g := r.Group("/diagnostics")
	g.POST("/firmware_debug", setFirmwareDebug(rosProvider))
	return r
}

func TestSetFirmwareDebugRoute_CallsHardwareBridgeService(t *testing.T) {
	mock := types.NewMockRosProvider()
	mock.ServiceResponder = func(_ string, _ any, res any) {
		reply := reflect.ValueOf(res).Elem()
		reply.FieldByName("Success").SetBool(true)
		reply.FieldByName("Message").SetString("ok")
	}
	router := newFirmwareDebugRouter(mock)

	body, err := json.Marshal(FirmwareDebugRequest{Enabled: true})
	require.NoError(t, err)

	rec := httptest.NewRecorder()
	req := httptest.NewRequest(http.MethodPost, "/diagnostics/firmware_debug", bytes.NewReader(body))
	req.Header.Set("Content-Type", "application/json")
	router.ServeHTTP(rec, req)

	assert.Equal(t, http.StatusOK, rec.Code)
	require.Len(t, mock.ServiceCalls, 1)
	assert.Equal(t, "/hardware_bridge/set_firmware_debug", mock.ServiceCalls[0].Service)

	reqJSON, err := json.Marshal(mock.ServiceCalls[0].Req)
	require.NoError(t, err)
	var payload map[string]any
	require.NoError(t, json.Unmarshal(reqJSON, &payload))
	assert.Equal(t, true, payload["data"])
}
