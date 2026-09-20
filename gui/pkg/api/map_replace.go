package api

import (
	"context"
	"errors"
	"fmt"
	"time"

	"github.com/mowglinext/mowglinext/pkg/msgs/geometry"
	"github.com/mowglinext/mowglinext/pkg/msgs/mowgli"
	"github.com/mowglinext/mowglinext/pkg/types"
)

// Map replace ("Save map" in the editor, and the OpenMower importer).
//
// map_server_node has no "replace" service: a replace is clear_map →
// add_area × N → save_areas, i.e. the old map is gone before the new one is
// complete. Two things keep that from hurting the operator:
//
//   - the CURRENT map is read back first (get_mowing_area × N). If map_server
//     cannot even answer that, nothing has been touched yet and the save fails
//     cleanly; if a later step fails, the snapshot is put back, so a failed
//     save does not leave the robot with half a garden (issue #655);
//   - every call has its own deadline and says WHICH step failed. The old flow
//     shared one budget, so a map_server busy rebuilding its keepout mask
//     surfaced as "add_area: context deadline exceeded" a minute later with no
//     hint of what had been applied.

const (
	mapServiceAddArea       = "/map_server_node/add_area"
	mapServiceClearMap      = "/map_server_node/clear_map"
	mapServiceSaveAreas     = "/map_server_node/save_areas"
	mapServiceGetMowingArea = "/map_server_node/get_mowing_area"

	// mapServiceCallTimeout bounds ONE map_server call. add_area no longer does
	// any grid work inside the service callback, so it answers in milliseconds;
	// the only legitimate wait left is map_server's single executor thread
	// finishing a keepout-mask rebuild (well under a second on a Pi now, up to
	// a minute on a robot still running an older ROS2 image — hence the size).
	mapServiceCallTimeout = 60 * time.Second

	// maxSnapshotAreas stops the read-back loop if map_server never reports
	// the end of its area list.
	maxSnapshotAreas = 512
)

// triggerRes decodes a std_srvs/srv/Trigger response. The generated
// mowgli.ClearMapRes has no message field, which is where map_server explains
// a refused save ("Save failed: …").
type triggerRes struct {
	Success bool   `json:"success"`
	Message string `json:"message"`
}

// mapWriteBudget returns the context timeout for a whole replace: read-back +
// clear_map + add_area×N + save_areas. It used to have to absorb seconds of
// rasterisation PER AREA on a slow SBC (issue #341: "can't save a big map,
// ~30 s timeout"); the per-call deadline above is what bounds a stuck
// map_server now, this is only the ceiling for the sequence. A 60 s base plus
// per-area headroom, capped at 6 min. Shared by ReplaceMapRoute and the
// OpenMower importer so the two never drift.
func mapWriteBudget(nAreas int) time.Duration {
	budget := 60*time.Second + time.Duration(nAreas)*5*time.Second
	if budget > 6*time.Minute {
		budget = 6 * time.Minute
	}
	return budget
}

// callMapService runs one map_server call under its own deadline.
func callMapService(ctx context.Context, provider types.IRosProvider, service string, req, res any, srvType string) error {
	callCtx, cancel := context.WithTimeout(ctx, mapServiceCallTimeout)
	defer cancel()
	err := provider.CallService(callCtx, service, req, res, srvType)
	if err != nil && errors.Is(err, context.DeadlineExceeded) {
		return fmt.Errorf("map_server did not answer within %s (is the ROS2 container busy or restarting?): %w", mapServiceCallTimeout, err)
	}
	return err
}

// normalizedArea returns the area as add_area must receive it: repeated fields
// are empty slices, never nil (foxglove_bridge rejects null — "msg is not a
// list type"), and dig PROPOSALS are dropped — map_server only ever creates
// those itself (root CLAUDE.md Invariant 16).
func normalizedArea(area mowgli.MapArea) mowgli.MapArea {
	if area.Obstacles == nil {
		area.Obstacles = []geometry.Polygon{}
	}
	if area.ObstacleInfo == nil {
		area.ObstacleInfo = []mowgli.MapObstacleInfo{}
	}
	area.ProposedObstacles = []geometry.Polygon{}
	area.ProposedObstacleInfo = []mowgli.MapObstacleInfo{}
	return area
}

// snapshotMap reads the live area list back from map_server.
func snapshotMap(ctx context.Context, provider types.IRosProvider) ([]mowgli.ReplaceMapArea, error) {
	snapshot := []mowgli.ReplaceMapArea{}
	for i := uint32(0); i < maxSnapshotAreas; i++ {
		req := mowgli.GetMowingAreaReq{Index: i}
		var res mowgli.GetMowingAreaRes
		if err := callMapService(ctx, provider, mapServiceGetMowingArea, &req, &res, "mowgli_interfaces/srv/GetMowingArea"); err != nil {
			return nil, err
		}
		if !res.Success {
			return snapshot, nil // past the last area
		}
		snapshot = append(snapshot, mowgli.ReplaceMapArea{
			Area:             normalizedArea(res.Area),
			IsNavigationArea: res.Area.IsNavigationArea,
		})
	}
	return nil, fmt.Errorf("map_server reported more than %d areas", maxSnapshotAreas)
}

// writeAreas is clear_map → add_area × N. It returns the step that failed.
func writeAreas(ctx context.Context, provider types.IRosProvider, areas []mowgli.ReplaceMapArea) error {
	var cleared triggerRes
	if err := callMapService(ctx, provider, mapServiceClearMap, &mowgli.ClearMapReq{}, &cleared, "std_srvs/srv/Trigger"); err != nil {
		return fmt.Errorf("clearing the old map: %w", err)
	}
	for i, element := range areas {
		req := mowgli.AddMowingAreaReq{
			Area:             normalizedArea(element.Area),
			IsNavigationArea: element.IsNavigationArea,
		}
		var res mowgli.AddMowingAreaRes
		step := fmt.Sprintf("adding area %d/%d %q", i+1, len(areas), element.Area.Name)
		if err := callMapService(ctx, provider, mapServiceAddArea, &req, &res, "mowgli_interfaces/srv/AddMowingArea"); err != nil {
			return fmt.Errorf("%s: %w", step, err)
		}
		if !res.Success {
			return fmt.Errorf("%s: map_server rejected it (a polygon needs at least 3 points)", step)
		}
	}
	return nil
}

// persistAreas is the explicit save_areas commit.
func persistAreas(ctx context.Context, provider types.IRosProvider) error {
	var saved triggerRes
	if err := callMapService(ctx, provider, mapServiceSaveAreas, &mowgli.ClearMapReq{}, &saved, "std_srvs/srv/Trigger"); err != nil {
		return err
	}
	if !saved.Success {
		return fmt.Errorf("map_server refused: %s", saved.Message)
	}
	return nil
}

// replaceMapInternal is the ROS-side flow shared by the public PUT handler and
// the OpenMower importer; HTTP decoding and status codes are the caller's job.
//
// Failure contract (the returned error says which case applies):
//   - reading the current map failed  → nothing was changed;
//   - clear/add failed                → the previous map was put back, or, if
//     that failed too, map_server holds a partial map and the error says so;
//   - only save_areas failed          → the NEW map is live but not on disk.
func replaceMapInternal(ctx context.Context, provider types.IRosProvider, req *mowgli.ReplaceMapReq) error {
	if req == nil {
		return errors.New("replaceMapInternal: nil request")
	}
	previous, err := snapshotMap(ctx, provider)
	if err != nil {
		return fmt.Errorf("replace map: could not read the current map, so nothing was changed: %w", err)
	}

	if err := writeAreas(ctx, provider, req.Areas); err != nil {
		return fmt.Errorf("replace map failed while %w — %s", err, restorePreviousMap(ctx, provider, previous))
	}
	if err := persistAreas(ctx, provider); err != nil {
		return fmt.Errorf("replace map: the new map is live but was NOT written to disk and will be lost at the next restart — check free space on the robot, then save again: %w", err)
	}
	return nil
}

// restorePreviousMap puts the snapshot back after a failed replace and
// describes the outcome for the operator. It runs on a fresh deadline: the
// usual reason for getting here is that the original one expired.
func restorePreviousMap(ctx context.Context, provider types.IRosProvider, previous []mowgli.ReplaceMapArea) string {
	restoreCtx, cancel := context.WithTimeout(context.WithoutCancel(ctx), mapWriteBudget(len(previous)))
	defer cancel()

	err := writeAreas(restoreCtx, provider, previous)
	if err == nil {
		err = persistAreas(restoreCtx, provider)
	}
	if err != nil {
		return fmt.Sprintf("restoring the previous map ALSO failed (%v): the robot now holds a PARTIAL map. Do not start mowing; keep this editor open and save again once ROS2 responds", err)
	}
	return fmt.Sprintf("the previous map (%d areas) was restored; your edits are still in the editor, save again", len(previous))
}
