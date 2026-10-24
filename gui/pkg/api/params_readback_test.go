package api

import (
	"bytes"
	"context"
	"encoding/json"
	"errors"
	"net/http"
	"net/http/httptest"
	"testing"

	"github.com/mowglinext/mowglinext/pkg/types"
	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
)

// readbackProvider models a node that holds `current` values and answers a
// write the way the foxglove bridge does: with the values it holds AFTER the
// write. apply decides what a requested value becomes (refusal returns the old
// value, a clamp returns a third one).
type readbackProvider struct {
	types.IRosProvider
	current map[string]any
	apply   func(name string, requested, previous any) any
	getErr  error
	setErr  error
	omit    bool
}

func (p *readbackProvider) GetParameters(_ context.Context, names []string) ([]types.RosParameter, error) {
	if p.getErr != nil {
		return nil, p.getErr
	}
	out := []types.RosParameter{}
	for _, name := range names {
		if value, ok := p.current[name]; ok {
			out = append(out, types.RosParameter{Name: name, Value: value, Type: "float64"})
		}
	}
	return out, nil
}

func (p *readbackProvider) SetParameters(_ context.Context, params []types.RosParameter) ([]types.RosParameter, error) {
	if p.setErr != nil {
		return nil, p.setErr
	}
	out := []types.RosParameter{}
	for _, param := range params {
		p.current[param.Name] = p.apply(param.Name, param.Value, p.current[param.Name])
		if !p.omit {
			out = append(out, types.RosParameter{Name: param.Name, Value: p.current[param.Name], Type: "float64"})
		}
	}
	return out, nil
}

func postHardwareParam(t *testing.T, ros types.IRosProvider, name string, value float64) *httptest.ResponseRecorder {
	t.Helper()
	body, err := json.Marshal(SetParamsRequest{Parameters: []types.RosParameter{{Name: name, Value: value}}})
	require.NoError(t, err)
	w := httptest.NewRecorder()
	req := httptest.NewRequest(http.MethodPost, "/api/params", bytes.NewReader(body))
	req.Header.Set("Content-Type", "application/json")
	newParamsRouter(ros).ServeHTTP(w, req)
	return w
}

const testKp = "hardware_bridge.wheel_pid_kp"

func TestSetParams_HardwareExactReadbackSucceedsWithoutAdjustment(t *testing.T) {
	ros := &readbackProvider{current: map[string]any{testKp: 10.0}, apply: func(_ string, requested, _ any) any { return requested }}

	w := postHardwareParam(t, ros, testKp, 12.0)

	require.Equal(t, http.StatusOK, w.Code, w.Body.String())
	var resp ParamsListResponse
	require.NoError(t, json.Unmarshal(w.Body.Bytes(), &resp))
	assert.False(t, resp.Adjusted)
	assert.Empty(t, resp.Adjustments)
	assert.NotContains(t, w.Body.String(), "adjust")
}

func TestSetParams_HardwareClampedReadbackSucceedsWithAppliedValue(t *testing.T) {
	ros := &readbackProvider{current: map[string]any{testKp: 10.0}, apply: func(_ string, _, _ any) any { return 15.0 }}

	w := postHardwareParam(t, ros, testKp, 20.0)

	require.Equal(t, http.StatusOK, w.Code, w.Body.String())
	var resp ParamsListResponse
	require.NoError(t, json.Unmarshal(w.Body.Bytes(), &resp))
	assert.True(t, resp.Adjusted)
	require.Len(t, resp.Adjustments, 1)
	assert.Equal(t, testKp, resp.Adjustments[0].Name)
	assert.Equal(t, 20.0, resp.Adjustments[0].RequestedValue)
	assert.Equal(t, 15.0, resp.Adjustments[0].AppliedValue)
	require.Len(t, resp.Parameters, 1)
	assert.Equal(t, 15.0, resp.Parameters[0].Value, "parameters carry the value actually applied")
}

func TestSetParams_HardwareRefusedWriteFails(t *testing.T) {
	// hardware_bridge's range descriptor / deadband gate refuses the write:
	// the bridge answers with the unchanged value.
	ros := &readbackProvider{current: map[string]any{testKp: 10.0}, apply: func(_ string, _, previous any) any { return previous }}

	w := postHardwareParam(t, ros, testKp, 0.2)

	assert.Equal(t, http.StatusUnprocessableEntity, w.Code)
	assert.Contains(t, w.Body.String(), "refused")
	assert.Contains(t, w.Body.String(), testKp)
}

func TestSetParams_HardwareMismatchWithoutPreviousValueIsUnconfirmed(t *testing.T) {
	ros := &readbackProvider{current: map[string]any{testKp: 10.0}, getErr: errors.New("timeout"),
		apply: func(_ string, _, _ any) any { return 15.0 }}

	w := postHardwareParam(t, ros, testKp, 20.0)

	assert.Equal(t, http.StatusServiceUnavailable, w.Code)
	assert.Contains(t, w.Body.String(), "not confirmed")
}

func TestSetParams_HardwareMissingReadbackFails(t *testing.T) {
	ros := &readbackProvider{current: map[string]any{testKp: 10.0}, omit: true, apply: func(_ string, requested, _ any) any { return requested }}

	w := postHardwareParam(t, ros, testKp, 12.0)

	assert.Equal(t, http.StatusServiceUnavailable, w.Code)
	assert.Contains(t, w.Body.String(), "not confirmed")
}

func TestSetParams_HardwareSetErrorFails(t *testing.T) {
	ros := &readbackProvider{current: map[string]any{testKp: 10.0}, setErr: errors.New("foxglove: SetParameters: update unconfirmed"),
		apply: func(_ string, requested, _ any) any { return requested }}

	w := postHardwareParam(t, ros, testKp, 12.0)

	assert.Equal(t, http.StatusServiceUnavailable, w.Code)
	assert.Contains(t, w.Body.String(), "update unconfirmed")
}
