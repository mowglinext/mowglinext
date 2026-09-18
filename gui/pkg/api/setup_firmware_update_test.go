package api

import (
	"net/http"
	"net/http/httptest"
	"testing"

	"github.com/gin-gonic/gin"
	"github.com/mowglinext/mowglinext/pkg/providers"
	"github.com/mowglinext/mowglinext/pkg/types"
)

func TestSetupRegistersUSBUpdateRoutesForProductionProvider(t *testing.T) {
	gin.SetMode(gin.TestMode)
	provider := providers.NewFirmwareProvider(types.NewMockDBProvider(), nil)
	if _, ok := any(provider).(types.IFirmwareUSBUpdater); !ok {
		t.Fatal("production firmware provider must expose USB updater interface")
	}
	r := gin.New()
	SetupRoutes(r.Group("/api"), provider)
	for _, path := range []string{"/api/setup/firmware-update", "/api/setup/firmware-update/:id", "/api/setup/firmware-update/:id/cancel"} {
		found := false
		for _, route := range r.Routes() {
			if route.Path == path {
				found = true
				break
			}
		}
		if !found {
			t.Errorf("route %s not registered", path)
		}
	}
	// With no ROS runtime the route must still exist and fail closed rather than
	// accidentally invoking the legacy ST-Link endpoint.
	w := httptest.NewRecorder()
	req := httptest.NewRequest(http.MethodPost, "/api/setup/firmware-update", nil)
	r.ServeHTTP(w, req)
	if w.Code != http.StatusBadRequest {
		t.Fatalf("got %d, want invalid request response", w.Code)
	}
}
