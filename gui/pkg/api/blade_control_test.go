package api

import (
	"context"
	"errors"
	"net/http/httptest"
	"strings"
	"testing"
	"time"

	"github.com/mowglinext/mowglinext/pkg/msgs/mowgli"
	"github.com/mowglinext/mowglinext/pkg/types"
	"github.com/stretchr/testify/require"
)

type bladeProvider struct {
	*types.MockRosProvider
	call func(context.Context, string, any, any) error
}

func (p *bladeProvider) CallService(ctx context.Context, name string, req, res any, _ ...string) error {
	return p.call(ctx, name, req, res)
}

func TestBladeControlOutcomes(t *testing.T) {
	for _, tc := range []struct {
		name, body                 string
		latch, forwarded, hardware bool
		unavailable                bool
		status                     int
		contains                   string
		calls                      int
	}{
		{"forward", `{"mow_enabled":1,"mow_direction":0}`, true, true, true, false, 200, "requested", 1},
		{"reverse", `{"mow_enabled":1,"mow_direction":1}`, true, true, true, false, 200, "requested", 1},
		{"rejected", `{"mow_enabled":1,"mow_direction":1}`, false, false, true, false, 503, "specific rejection", 1},
		{"old ROS ON", `{"mow_enabled":1,"mow_direction":1}`, false, false, true, true, 503, "update ROS and GUI", 1},
		{"queue failure", `{"mow_enabled":1,"mow_direction":1}`, true, false, true, false, 200, "warning", 1},
		{"off", `{"mow_enabled":0,"mow_direction":0}`, true, true, true, false, 200, "latched and requested", 2},
		{"off unused direction", `{"mow_enabled":0,"mow_direction":255}`, true, true, true, false, 200, "latched and requested", 2},
		{"old ROS OFF", `{"mow_enabled":0}`, false, false, true, true, 200, "may turn the blade back on", 2},
		{"hardware failure", `{"mow_enabled":0}`, true, false, false, false, 200, "OFF is latched", 2},
		{"both failures", `{"mow_enabled":0}`, false, false, false, true, 503, "Session controller", 2},
		{"bad ON direction", `{"mow_enabled":1,"mow_direction":255}`, false, false, false, false, 400, "ON direction", 0},
		{"bad enabled", `{"mow_enabled":2}`, false, false, false, false, 400, "enable", 0},
		{"bad JSON", `{`, false, false, false, false, 400, "error", 0},
	} {
		t.Run(tc.name, func(t *testing.T) {
			calls := []string{}
			p := &bladeProvider{MockRosProvider: types.NewMockRosProvider()}
			p.call = func(ctx context.Context, name string, req, res any) error {
				calls = append(calls, name)
				_, bounded := ctx.Deadline()
				require.True(t, bounded)
				if name == "/behavior_tree_node/blade_control" {
					require.Len(t, calls, 1, "the inhibition attempt must precede direct OFF")
					if tc.unavailable {
						return errors.New("service not advertised")
					}
					result := res.(*mowgli.BladeControlRes)
					result.Success = tc.latch
					result.Forwarded = tc.forwarded
					result.Message = "specific rejection"
					if tc.latch {
						result.Message = "direction requested"
					}
				} else {
					require.Equal(t, "/hardware_bridge/mower_control", name)
					command := req.(*mowgli.MowerControlReq)
					require.Equal(t, uint8(0), command.MowEnabled, "never use a direct ON fallback")
					require.Equal(t, uint8(0), command.MowDirection)
					res.(*mowgli.MowerControlRes).Success = tc.hardware
				}
				return nil
			}
			router := setupMowgliNextRouter(p)
			w := httptest.NewRecorder()
			router.ServeHTTP(w, httptest.NewRequest("POST", "/api/mowglinext/call/blade_control", strings.NewReader(tc.body)))
			require.Equal(t, tc.status, w.Code, w.Body.String())
			require.Contains(t, w.Body.String(), tc.contains)
			require.Len(t, calls, tc.calls)
		})
	}
}

func TestBladeOffFallbackHasIndependentBudgetAndSurvivesBrowserClose(t *testing.T) {
	for _, cancelBrowser := range []bool{false, true} {
		p := &bladeProvider{MockRosProvider: types.NewMockRosProvider()}
		started := time.Now()
		calls := 0
		p.call = func(ctx context.Context, name string, req, res any) error {
			calls++
			if calls == 1 {
				<-ctx.Done()
				return ctx.Err()
			}
			require.NoError(t, ctx.Err(), "fallback must retain its own live budget")
			require.Less(t, time.Since(started), time.Second)
			res.(*mowgli.MowerControlRes).Success = true
			return nil
		}
		ctx, cancel := context.WithCancel(context.Background())
		if cancelBrowser {
			cancel()
		}
		req := httptest.NewRequest("POST", "/api/mowglinext/call/blade_control", strings.NewReader(`{"mow_enabled":0}`)).WithContext(ctx)
		w := httptest.NewRecorder()
		setupMowgliNextRouter(p).ServeHTTP(w, req)
		cancel()
		require.Equal(t, 200, w.Code)
		require.Equal(t, 2, calls)
		require.Contains(t, w.Body.String(), "may turn the blade back on")
	}
}
