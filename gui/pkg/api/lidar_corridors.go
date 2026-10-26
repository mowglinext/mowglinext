package api

import (
	"bytes"
	"context"
	"encoding/json"
	"errors"
	"io"
	"net/http"

	"github.com/docker/distribution/uuid"
	"github.com/gin-gonic/gin"
	"github.com/mowglinext/mowglinext/pkg/msgs/geometry"
	"github.com/mowglinext/mowglinext/pkg/msgs/mowgli"
	"github.com/mowglinext/mowglinext/pkg/types"
)

// 16x the producer's 64 KiB staging payload permits verbose Point32 JSON and
// string escaping while bounding decoding before allocation. The producer remains
// authoritative for geometry/identity limits and its decoded staging budget.
const corridorReplaceJSONLimit = 1 << 20

func replaceLidarCorridors(c *gin.Context, ctx context.Context, provider types.IRosProvider) {
	var envelope struct {
		Corridors json.RawMessage `json:"corridors"`
	}
	decoder := json.NewDecoder(http.MaxBytesReader(c.Writer, c.Request.Body, corridorReplaceJSONLimit))
	err := decoder.Decode(&envelope)
	if err == nil {
		var trailing any
		if next := decoder.Decode(&trailing); next != io.EOF {
			err = next
			if err == nil {
				err = errors.New("expected one JSON object")
			}
		}
	}
	raw := bytes.TrimSpace(envelope.Corridors)
	if err != nil || len(raw) == 0 || raw[0] != '[' {
		status := http.StatusBadRequest
		var limit *http.MaxBytesError
		if errors.As(err, &limit) {
			status = http.StatusRequestEntityTooLarge
		}
		c.JSON(status, ErrorResponse{Error: "corridors must be an explicitly present JSON array"})
		return
	}
	var corridors []mowgli.LidarIgnoreCorridor
	if err := json.Unmarshal(raw, &corridors); err != nil {
		c.JSON(http.StatusBadRequest, ErrorResponse{Error: err.Error()})
		return
	}
	for i := range corridors {
		if corridors[i].Polyline.Points == nil {
			corridors[i].Polyline.Points = []geometry.Point32{}
		}
	}
	// Reuse the GUI's map/import/backup lock as well as the producer's writer lock.
	// A canceled waiter never sends an RPC. This is not a cross-process transaction:
	// once sent, a lost response still has an explicitly unknown outcome.
	if err := lockMapReplacement(ctx, nil); err != nil {
		c.JSON(http.StatusServiceUnavailable, gin.H{"error": err.Error(), "outcome": "NOT_SENT", "committed": false, "persisted": false, "published": false})
		return
	}
	defer unlockMapReplacement()
	req := mowgli.ReplaceLidarIgnoreCorridorsReq{Corridors: corridors, ClientOperationId: uuid.Generate().String()}
	var res mowgli.ReplaceLidarIgnoreCorridorsRes
	err = provider.CallService(ctx, "/map_server_node/replace_lidar_ignore_corridors", &req, &res, "mowgli_interfaces/srv/ReplaceLidarIgnoreCorridors")
	unknown := func(message string, status int) {
		c.JSON(status, gin.H{"error": message, "outcome": "UNKNOWN_AFTER_SEND", "committed": nil, "persisted": nil, "published": nil, "client_operation_id": req.ClientOperationId})
	}
	if err != nil {
		status := http.StatusBadGateway
		if errors.Is(err, context.DeadlineExceeded) {
			status = http.StatusGatewayTimeout
		}
		unknown("corridor replacement unconfirmed; reload before any new save (ROS Replace service required): "+err.Error(), status)
		return
	}
	status := 0
	switch res.Outcome {
	case "COMMITTED":
		if res.Success && res.Committed && res.Persisted && res.Published {
			status = http.StatusOK
		}
	case "VALIDATION_REJECTED":
		if !res.Success && !res.Committed && !res.Persisted && !res.Published {
			status = http.StatusUnprocessableEntity
		}
	case "PERSISTENCE_NOT_CONFIGURED":
		if !res.Success && !res.Committed && !res.Persisted && !res.Published {
			status = http.StatusServiceUnavailable
		}
	case "PERSISTENCE_FAILED_BEFORE_COMMIT":
		if !res.Success && !res.Committed && !res.Persisted && !res.Published {
			status = http.StatusInternalServerError
		}
	case "COMMITTED_DURABILITY_UNCONFIRMED":
		if !res.Success && res.Committed && !res.Persisted {
			status = http.StatusServiceUnavailable
		}
	case "COMMITTED_PUBLICATION_PENDING":
		if !res.Success && res.Committed && res.Persisted && !res.Published {
			status = http.StatusServiceUnavailable
		}
	}
	if status == 0 {
		unknown("invalid corridor replacement response; reload state", http.StatusBadGateway)
		return
	}
	if res.Corridors == nil {
		res.Corridors = []mowgli.LidarIgnoreCorridor{}
	}
	response := struct {
		mowgli.ReplaceLidarIgnoreCorridorsRes
		Error string `json:"error,omitempty"`
	}{ReplaceLidarIgnoreCorridorsRes: res}
	if status != http.StatusOK {
		response.Error = res.Message
		if response.Error == "" {
			response.Error = res.Outcome
		}
	}
	c.JSON(status, response)
}
