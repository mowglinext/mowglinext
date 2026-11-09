package api

import (
	"encoding/json"
	"errors"
	"net/http"
	"testing"

	"github.com/mowglinext/mowglinext/pkg/types"
	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
)

func TestHardwareCommandAcknowledgements(t *testing.T) {
	for _, command := range []string{"emergency", "mow_enabled"} {
		for _, tc := range []struct {
			name string
			raw  string
			err  error
			code int
		}{
			{"accepted", `{"success":true}`, nil, http.StatusOK},
			{"refused", `{"success":false}`, nil, http.StatusInternalServerError},
			{"transport", `{"success":true}`, errors.New("ROS unavailable"), http.StatusInternalServerError},
			{"missing-result", `{}`, nil, http.StatusInternalServerError},
			{"null-result", `null`, nil, http.StatusInternalServerError},
			{"invalid-response", `{"success":"invalid"}`, nil, http.StatusInternalServerError},
			{"malformed-response", `{`, nil, http.StatusInternalServerError},
		} {
			t.Run(command+"/"+tc.name, func(t *testing.T) {
				a := newFleetRobot(t, "alpha", func(ros *types.MockRosProvider) {
					// RosProvider.CallService decodes the response via json.Unmarshal.
					var response struct {
						Success bool `json:"success"`
					}
					ros.ServiceErr = json.Unmarshal([]byte(tc.raw), &response)
					if tc.err != nil {
						ros.ServiceErr = tc.err
					}
					ros.ServiceResponder = func(_ string, _ any, res any) { _ = json.Unmarshal([]byte(tc.raw), res) }
				})
				paths := []string{"/api/mowglinext/call/" + command}
				if command == "emergency" {
					paths = append(paths, "/api/fleet/robots/"+a.id+"/call/"+command)
				}
				for _, path := range paths {
					for _, value := range []int{0, 1} {
						payload := map[string]int{command: value, "mow_direction": 0}
						res, body := a.post(t, path, payload)
						assert.Equal(t, tc.code, res.StatusCode, path+": "+string(body))
						if tc.code != http.StatusOK {
							var result ErrorResponse
							require.NoError(t, json.Unmarshal(body, &result))
							assert.NotEmpty(t, result.Error)
							if tc.name == "refused" {
								assert.Contains(t, result.Error, command+" rejected")
							}
							if tc.name == "transport" {
								assert.Contains(t, result.Error, "ROS unavailable")
							}
						}
					}
				}
			})
		}
	}
}
