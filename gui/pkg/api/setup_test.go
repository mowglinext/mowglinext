package api

import (
	"bytes"
	"io"
	"net/http"
	"net/http/httptest"
	"testing"

	"github.com/gin-gonic/gin"
	"github.com/mowglinext/mowglinext/pkg/types"
	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
)

// scriptedFirmwareProvider replays a fixed log so the SSE framing can be
// asserted without openocd.
type scriptedFirmwareProvider struct {
	log string
	err error
}

func (s scriptedFirmwareProvider) FlashFirmware(w io.Writer, _ types.FirmwareConfig) error {
	_, _ = io.WriteString(w, s.log)
	return s.err
}

func (s scriptedFirmwareProvider) AvailableFirmware() (types.FirmwareAvailability, error) {
	return types.FirmwareAvailability{}, nil
}

// streamRecorder adds the CloseNotifier gin's c.Stream asserts on, which the
// plain httptest recorder lacks.
type streamRecorder struct {
	*httptest.ResponseRecorder
	closed chan bool
}

func (s streamRecorder) CloseNotify() <-chan bool { return s.closed }

func flashBoardResponse(t *testing.T, provider types.IFirmwareProvider) string {
	t.Helper()
	gin.SetMode(gin.TestMode)
	r := gin.New()
	SetupRoutes(r.Group("/api"), provider)
	req := httptest.NewRequest(http.MethodPost, "/api/setup/flashBoard", bytes.NewBufferString(`{"boardType":"BOARD_YARDFORCE500"}`))
	req.Header.Set("Content-Type", "application/json")
	rec := streamRecorder{ResponseRecorder: httptest.NewRecorder(), closed: make(chan bool)}
	r.ServeHTTP(rec, req)
	require.Equal(t, http.StatusOK, rec.Code)
	return rec.Body.String()
}

// A marker line becomes its own `stage` event carrying the JSON payload, while
// ordinary log lines stay `message` events — that split is what the GUI's
// progress bar relies on.
func TestFlashBoardStreamLiftsStageMarkersIntoStageEvents(t *testing.T) {
	body := flashBoardResponse(t, scriptedFirmwareProvider{
		log: "------> Fetching firmware manifest...\n" +
			types.FlashStageMarker + `{"stages":["manifest","flash"],"current":1}` + "\n" +
			"------> Flashing firmware...\n",
	})

	assert.Contains(t, body, "event:message\ndata:------> Fetching firmware manifest...\n")
	assert.Contains(t, body, "event:stage\ndata:{\"stages\":[\"manifest\",\"flash\"],\"current\":1}\n")
	assert.NotContains(t, body, "data:"+types.FlashStageMarker)
	assert.Contains(t, body, "event:end\n")
}

func TestFlashBoardStreamEndsWithErrorEventOnFailure(t *testing.T) {
	body := flashBoardResponse(t, scriptedFirmwareProvider{
		log: "------> Flashing firmware...\n",
		err: io.ErrUnexpectedEOF,
	})

	assert.Contains(t, body, "event:error\ndata:"+io.ErrUnexpectedEOF.Error()+"\n")
	assert.NotContains(t, body, "event:end\n")
}

func TestFlashStreamEventClassification(t *testing.T) {
	event, payload := flashStreamEvent(types.FlashStageMarker + `{"stages":["flash"],"current":0}`)
	assert.Equal(t, "stage", event)
	assert.Equal(t, `{"stages":["flash"],"current":0}`, payload)

	event, payload = flashStreamEvent("------> plain log line")
	assert.Equal(t, "message", event)
	assert.Equal(t, "------> plain log line", payload)
}
