package api

import (
	"context"
	"encoding/json"
	"net/http"
	"net/http/httptest"
	"strings"
	"testing"

	"github.com/gorilla/websocket"
	"github.com/mowglinext/mowglinext/pkg/foxglove"
	"github.com/mowglinext/mowglinext/pkg/types"
	"github.com/stretchr/testify/require"
)

// Use the real client for parameter writes; unrelated provider methods are inert.
type parameterClientProvider struct {
	types.IRosProvider
	client *foxglove.Client
}

func (p parameterClientProvider) SetParameters(ctx context.Context, params []types.RosParameter) ([]types.RosParameter, error) {
	request := make([]foxglove.Parameter, len(params))
	for i, param := range params {
		request[i] = foxglove.Parameter{Name: param.Name, Value: param.Value, Type: param.Type}
	}
	values, err := p.client.SetParameters(ctx, request)
	if err != nil {
		return nil, err
	}
	result := make([]types.RosParameter, len(values))
	for i, param := range values {
		result[i] = types.RosParameter{Name: param.Name, Value: param.Value, Type: param.Type}
	}
	return result, nil
}

func TestSetParamsRequiresAcknowledgement(t *testing.T) {
	for _, acknowledged := range []bool{false, true} {
		name := "no reply"
		if acknowledged {
			name = "acknowledged"
		}
		t.Run(name, func(t *testing.T) {
			ctx, cancel := context.WithCancel(context.Background())
			defer cancel()
			server := httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
				upgrader := websocket.Upgrader{Subprotocols: []string{"foxglove.sdk.v1"}}
				conn, err := upgrader.Upgrade(w, r, nil)
				if err != nil {
					t.Error(err)
					return
				}
				defer conn.Close()
				var request struct {
					ID         string               `json:"id"`
					Parameters []foxglove.Parameter `json:"parameters"`
				}
				if err := conn.ReadJSON(&request); err != nil {
					t.Error(err)
					return
				}
				if acknowledged {
					_ = conn.WriteJSON(map[string]any{"op": "parameterValues", "id": request.ID, "parameters": request.Parameters})
				} else {
					cancel() // peer received the write, but has not acknowledged it
				}
				_, _, _ = conn.ReadMessage()
			}))
			defer server.Close()
			client := foxglove.NewClient("ws" + strings.TrimPrefix(server.URL, "http"))
			require.NoError(t, client.Connect(context.Background()))
			defer client.Close()
			router := newParamsRouter(parameterClientProvider{client: client})
			req := httptest.NewRequest(http.MethodPost, "/api/params", strings.NewReader(`{"parameters":[{"name":"mowgli.speed","value":0.2,"type":"float64"}]}`)).WithContext(ctx)
			req.Header.Set("Content-Type", "application/json")
			response := httptest.NewRecorder()
			router.ServeHTTP(response, req)
			if acknowledged {
				require.Equal(t, http.StatusOK, response.Code)
				var body ParamsListResponse
				require.NoError(t, json.Unmarshal(response.Body.Bytes(), &body))
				require.Len(t, body.Parameters, 1)
				require.Equal(t, 0.2, body.Parameters[0].Value)
			} else {
				require.Equal(t, http.StatusServiceUnavailable, response.Code)
				require.Contains(t, response.Body.String(), "update unconfirmed")
				require.Contains(t, response.Body.String(), "read back parameters before retrying")
				require.NotContains(t, response.Body.String(), `"parameters"`)
			}
		})
	}
}
