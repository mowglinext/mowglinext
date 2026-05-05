package api

import (
	"bytes"
	"encoding/base64"
	"encoding/json"
	"net/http"
	"net/http/httptest"
	"strings"
	"testing"
	"time"

	"github.com/cedbossneo/mowglinext/pkg/types"
	"github.com/gin-gonic/gin"
	"github.com/gorilla/websocket"
	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
)

func setupMowgliNextRouter(provider types.IRosProvider) *gin.Engine {
	gin.SetMode(gin.TestMode)
	r := gin.New()
	group := r.Group("/api")
	MowgliNextRoutes(group, provider)
	return r
}

func TestServiceRoute_HighLevelControl(t *testing.T) {
	mock := types.NewMockRosProvider()
	router := setupMowgliNextRouter(mock)

	payload := map[string]any{"Command": 1}
	body, _ := json.Marshal(payload)

	w := httptest.NewRecorder()
	req, _ := http.NewRequest("POST", "/api/mowglinext/call/high_level_control", bytes.NewReader(body))
	req.Header.Set("Content-Type", "application/json")
	router.ServeHTTP(w, req)

	assert.Equal(t, http.StatusOK, w.Code)
	require.Len(t, mock.ServiceCalls, 1)
	assert.Equal(t, "/behavior_tree_node/high_level_control", mock.ServiceCalls[0].Service)
}

func TestServiceRoute_Emergency(t *testing.T) {
	mock := types.NewMockRosProvider()
	router := setupMowgliNextRouter(mock)

	payload := map[string]any{"Emergency": 1}
	body, _ := json.Marshal(payload)

	w := httptest.NewRecorder()
	req, _ := http.NewRequest("POST", "/api/mowglinext/call/emergency", bytes.NewReader(body))
	req.Header.Set("Content-Type", "application/json")
	router.ServeHTTP(w, req)

	assert.Equal(t, http.StatusOK, w.Code)
	require.Len(t, mock.ServiceCalls, 1)
	assert.Equal(t, "/hardware_bridge/emergency_stop", mock.ServiceCalls[0].Service)
}

func TestServiceRoute_MowEnabled(t *testing.T) {
	mock := types.NewMockRosProvider()
	router := setupMowgliNextRouter(mock)

	payload := map[string]any{"MowEnabled": 1, "MowDirection": 0}
	body, _ := json.Marshal(payload)

	w := httptest.NewRecorder()
	req, _ := http.NewRequest("POST", "/api/mowglinext/call/mow_enabled", bytes.NewReader(body))
	req.Header.Set("Content-Type", "application/json")
	router.ServeHTTP(w, req)

	assert.Equal(t, http.StatusOK, w.Code)
	require.Len(t, mock.ServiceCalls, 1)
	assert.Equal(t, "/hardware_bridge/mower_control", mock.ServiceCalls[0].Service)
}

func TestServiceRoute_UnknownCommand(t *testing.T) {
	mock := types.NewMockRosProvider()
	router := setupMowgliNextRouter(mock)

	payload := map[string]any{}
	body, _ := json.Marshal(payload)

	w := httptest.NewRecorder()
	req, _ := http.NewRequest("POST", "/api/mowglinext/call/unknown_command", bytes.NewReader(body))
	req.Header.Set("Content-Type", "application/json")
	router.ServeHTTP(w, req)

	assert.Equal(t, http.StatusInternalServerError, w.Code)

	var resp ErrorResponse
	err := json.Unmarshal(w.Body.Bytes(), &resp)
	require.NoError(t, err)
	assert.Equal(t, "unknown command", resp.Error)
}

func TestServiceRoute_ServiceError(t *testing.T) {
	mock := types.NewMockRosProvider()
	mock.ServiceErr = assert.AnError
	router := setupMowgliNextRouter(mock)

	payload := map[string]any{"Command": 1}
	body, _ := json.Marshal(payload)

	w := httptest.NewRecorder()
	req, _ := http.NewRequest("POST", "/api/mowglinext/call/high_level_control", bytes.NewReader(body))
	req.Header.Set("Content-Type", "application/json")
	router.ServeHTTP(w, req)

	assert.Equal(t, http.StatusInternalServerError, w.Code)
}

func TestServiceRoute_StartInArea(t *testing.T) {
	mock := types.NewMockRosProvider()
	router := setupMowgliNextRouter(mock)

	payload := map[string]any{"Area": 2}
	body, _ := json.Marshal(payload)

	w := httptest.NewRecorder()
	req, _ := http.NewRequest("POST", "/api/mowglinext/call/start_in_area", bytes.NewReader(body))
	req.Header.Set("Content-Type", "application/json")
	router.ServeHTTP(w, req)

	assert.Equal(t, http.StatusOK, w.Code)
	require.Len(t, mock.ServiceCalls, 1)
	assert.Equal(t, "/behavior_tree_node/start_in_area", mock.ServiceCalls[0].Service)
}

func TestClearMapRoute(t *testing.T) {
	mock := types.NewMockRosProvider()
	router := setupMowgliNextRouter(mock)

	w := httptest.NewRecorder()
	req, _ := http.NewRequest("DELETE", "/api/mowglinext/map", nil)
	router.ServeHTTP(w, req)

	assert.Equal(t, http.StatusOK, w.Code)
	require.Len(t, mock.ServiceCalls, 1)
	assert.Equal(t, "/map_server_node/clear_map", mock.ServiceCalls[0].Service)
}

func TestClearMapRoute_Error(t *testing.T) {
	mock := types.NewMockRosProvider()
	mock.ServiceErr = assert.AnError
	router := setupMowgliNextRouter(mock)

	w := httptest.NewRecorder()
	req, _ := http.NewRequest("DELETE", "/api/mowglinext/map", nil)
	router.ServeHTTP(w, req)

	assert.Equal(t, http.StatusInternalServerError, w.Code)
}

func TestSetDockingPointRoute(t *testing.T) {
	mock := types.NewMockRosProvider()
	router := setupMowgliNextRouter(mock)

	payload := map[string]any{
		"dockX":   1.5,
		"dockY":   2.5,
		"heading": 0.78,
	}
	body, _ := json.Marshal(payload)

	w := httptest.NewRecorder()
	req, _ := http.NewRequest("POST", "/api/mowglinext/map/docking", bytes.NewReader(body))
	req.Header.Set("Content-Type", "application/json")
	router.ServeHTTP(w, req)

	assert.Equal(t, http.StatusOK, w.Code)
	require.Len(t, mock.ServiceCalls, 1)
	assert.Equal(t, "/map_server_node/set_docking_point", mock.ServiceCalls[0].Service)
}

// dialMultiplex opens the test server's /multiplex WebSocket. Returns the
// connection plus a teardown closure that closes both the connection and
// the test server.
func dialMultiplex(t *testing.T, server *httptest.Server) *websocket.Conn {
	t.Helper()
	wsURL := "ws" + strings.TrimPrefix(server.URL, "http") + "/api/mowglinext/multiplex"
	conn, _, err := websocket.DefaultDialer.Dial(wsURL, nil)
	require.NoError(t, err)
	return conn
}

func readMultiplexFrame(t *testing.T, conn *websocket.Conn) (string, []byte) {
	t.Helper()
	_ = conn.SetReadDeadline(time.Now().Add(2 * time.Second))
	_, raw, err := conn.ReadMessage()
	require.NoError(t, err)
	var frame struct {
		Topic string `json:"topic"`
		Data  string `json:"data"`
	}
	require.NoError(t, json.Unmarshal(raw, &frame))
	decoded, err := base64.StdEncoding.DecodeString(frame.Data)
	require.NoError(t, err)
	return frame.Topic, decoded
}

func TestMultiplexRoute_FansOutOnSubscribe(t *testing.T) {
	mock := types.NewMockRosProvider()
	server := httptest.NewServer(setupMowgliNextRouter(mock))
	defer server.Close()
	conn := dialMultiplex(t, server)
	defer conn.Close()

	require.NoError(t, conn.WriteJSON(map[string]string{"op": "subscribe", "topic": "highLevelStatus"}))
	// Give the server's read loop a moment to register the subscription.
	time.Sleep(50 * time.Millisecond)

	mock.Dispatch("highLevelStatus", []byte(`{"hello":"world"}`))

	topic, payload := readMultiplexFrame(t, conn)
	assert.Equal(t, "highLevelStatus", topic)
	assert.JSONEq(t, `{"hello":"world"}`, string(payload))
}

func TestMultiplexRoute_UnsubscribeStopsDelivery(t *testing.T) {
	mock := types.NewMockRosProvider()
	server := httptest.NewServer(setupMowgliNextRouter(mock))
	defer server.Close()
	conn := dialMultiplex(t, server)
	defer conn.Close()

	require.NoError(t, conn.WriteJSON(map[string]string{"op": "subscribe", "topic": "status"}))
	time.Sleep(50 * time.Millisecond)
	require.NoError(t, conn.WriteJSON(map[string]string{"op": "unsubscribe", "topic": "status"}))
	time.Sleep(50 * time.Millisecond)

	mock.Dispatch("status", []byte(`{"x":1}`))

	_ = conn.SetReadDeadline(time.Now().Add(200 * time.Millisecond))
	_, _, err := conn.ReadMessage()
	assert.Error(t, err, "no frame should arrive after unsubscribe")
}

func TestMultiplexRoute_IgnoresUnknownTopic(t *testing.T) {
	mock := types.NewMockRosProvider()
	server := httptest.NewServer(setupMowgliNextRouter(mock))
	defer server.Close()
	conn := dialMultiplex(t, server)
	defer conn.Close()

	require.NoError(t, conn.WriteJSON(map[string]string{"op": "subscribe", "topic": "made_up"}))
	time.Sleep(50 * time.Millisecond)

	mock.Dispatch("made_up", []byte(`x`))
	_ = conn.SetReadDeadline(time.Now().Add(200 * time.Millisecond))
	_, _, err := conn.ReadMessage()
	assert.Error(t, err)
}

func TestMultiplexRoute_DropsSubscriptionsOnDisconnect(t *testing.T) {
	mock := types.NewMockRosProvider()
	server := httptest.NewServer(setupMowgliNextRouter(mock))
	defer server.Close()
	conn := dialMultiplex(t, server)

	require.NoError(t, conn.WriteJSON(map[string]string{"op": "subscribe", "topic": "imu"}))
	time.Sleep(50 * time.Millisecond)
	_ = conn.Close()
	time.Sleep(100 * time.Millisecond)

	// Dispatch after the client is gone should be a no-op (no panic).
	mock.Dispatch("imu", []byte(`{"a":1}`))
}
