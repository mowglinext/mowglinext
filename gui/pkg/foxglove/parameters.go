package foxglove

import (
	"context"
	"encoding/json"
	"fmt"
	"strconv"
	"sync/atomic"
	"time"
)

const parametersTimeout = 10 * time.Second

// Parameter is a single ROS2 parameter exchanged over the foxglove
// `parameters` capability. Value is the raw JSON value (number / bool / string
// / array) as sent by the bridge.
type Parameter struct {
	Name  string      `json:"name"`
	Value interface{} `json:"value"`
	Type  string      `json:"type,omitempty"`
}

var paramRequestSeq uint64

func nextParamRequestID() string {
	return "param-" + strconv.FormatUint(atomic.AddUint64(&paramRequestSeq, 1), 10)
}

// GetParameters fetches parameters from the bridge. An empty/nil names slice
// requests every parameter the bridge knows about.
func (c *Client) GetParameters(ctx context.Context, names []string) ([]Parameter, error) {
	if !c.Connected() {
		return nil, fmt.Errorf("foxglove: GetParameters: not connected")
	}
	if names == nil {
		names = []string{}
	}
	id, ch := c.registerParamRequest()
	defer c.releaseParamRequest(id)

	if err := c.writeJSON(map[string]interface{}{
		"op":             "getParameters",
		"parameterNames": names,
		"id":             id,
	}); err != nil {
		return nil, fmt.Errorf("foxglove: GetParameters send: %w", err)
	}

	tctx, cancel := context.WithTimeout(ctx, parametersTimeout)
	defer cancel()
	select {
	case params := <-ch:
		return params, nil
	case <-tctx.Done():
		return nil, fmt.Errorf("foxglove: GetParameters: %w", tctx.Err())
	}
}

// SetParameters updates parameters on their owning nodes and returns the values
// the bridge echoes back. Some bridges do not echo; a timeout is then treated as
// best-effort success and the requested values are returned.
func (c *Client) SetParameters(ctx context.Context, params []Parameter) ([]Parameter, error) {
	if !c.Connected() {
		return nil, fmt.Errorf("foxglove: SetParameters: not connected")
	}
	id, ch := c.registerParamRequest()
	defer c.releaseParamRequest(id)

	if err := c.writeJSON(map[string]interface{}{
		"op":         "setParameters",
		"parameters": params,
		"id":         id,
	}); err != nil {
		return nil, fmt.Errorf("foxglove: SetParameters send: %w", err)
	}

	tctx, cancel := context.WithTimeout(ctx, parametersTimeout)
	defer cancel()
	select {
	case echoed := <-ch:
		return echoed, nil
	case <-tctx.Done():
		return params, nil
	}
}

func (c *Client) registerParamRequest() (string, chan []Parameter) {
	id := nextParamRequestID()
	ch := make(chan []Parameter, 1)
	c.paramMu.Lock()
	c.pendingParam[id] = ch
	c.paramMu.Unlock()
	return id, ch
}

func (c *Client) releaseParamRequest(id string) {
	c.paramMu.Lock()
	delete(c.pendingParam, id)
	c.paramMu.Unlock()
}

// handleParameterValues routes a `parameterValues` reply to the waiting request.
// Unsolicited updates (no id, from parametersSubscribe) are ignored.
func (c *Client) handleParameterValues(data []byte) {
	var resp struct {
		ID         string      `json:"id"`
		Parameters []Parameter `json:"parameters"`
	}
	if err := json.Unmarshal(data, &resp); err != nil {
		return
	}
	if resp.ID == "" {
		return
	}
	c.paramMu.Lock()
	ch, ok := c.pendingParam[resp.ID]
	c.paramMu.Unlock()
	if ok {
		select {
		case ch <- resp.Parameters:
		default:
		}
	}
}
