package providers

import (
	"context"
	"errors"
	"fmt"
	"net/http"
	"strings"
	"time"

	"github.com/mowglinext/mowglinext/pkg/msgs/geometry"
	"github.com/mowglinext/mowglinext/pkg/msgs/mowgli"
)

// Shared map (docs/MULTI_ROBOT.md § 3a): coordinated mowing needs every
// member to hold the SAME areas.dat — same area order (areas have no id,
// only their index) and the same datum (map_server re-projects a foreign
// datum on load and shifts the robot's own dock pose by the difference).

const (
	peerMapPath     = "/api/mowglinext/map"
	maxMapAreas     = 100
	mapPushBaseWait = 60 * time.Second
	mapPushPerArea  = 5 * time.Second
	mapPushMaxWait  = 6 * time.Minute
)

// MapPushPeerResult is the outcome for one peer.
type MapPushPeerResult struct {
	ID      string `json:"id"`
	Name    string `json:"name"`
	Address string `json:"address"`
	OK      bool   `json:"ok"`
	Error   string `json:"error,omitempty"`
}

// MapPushResult summarises a push to the whole fleet.
type MapPushResult struct {
	Areas int                 `json:"areas"`
	Peers []MapPushPeerResult `json:"peers"`
}

// PushMap replaces every peer's map with this robot's areas, in this robot's
// order. Peers whose datum differs are refused (the operator must align the
// datums first). Pending obstacle proposals (dig keepouts awaiting sign-off)
// are stripped: they are this robot's session proposals, not map content.
func (f *FleetProvider) PushMap(ctx context.Context) (MapPushResult, error) {
	self, err := f.Identity()
	if err != nil {
		return MapPushResult{}, err
	}
	if self.DatumLat == 0 && self.DatumLon == 0 {
		return MapPushResult{}, errors.New("this robot has no datum yet; set it in Settings → GPS before sharing the map")
	}
	areas, err := f.localAreas(ctx)
	if err != nil {
		return MapPushResult{}, err
	}
	if len(areas) == 0 {
		return MapPushResult{}, errors.New("this robot has no recorded area to share")
	}
	req := mowgli.ReplaceMapReq{Areas: make([]mowgli.ReplaceMapArea, 0, len(areas))}
	for _, a := range areas {
		clean := stripPendingObstacles(a)
		req.Areas = append(req.Areas, mowgli.ReplaceMapArea{Area: clean, IsNavigationArea: clean.IsNavigationArea})
	}

	result := MapPushResult{Areas: len(areas), Peers: []MapPushPeerResult{}}
	datum := LatLon{Lat: self.DatumLat, Lon: self.DatumLon}
	for _, peer := range f.Peers() {
		row := MapPushPeerResult{ID: peer.ID, Name: peer.Name, Address: peer.Address}
		remote, err := fetchPeerIdentity(ctx, f.http, peer.Address)
		switch {
		case err != nil:
			row.Error = "unreachable: " + err.Error()
		case !sameDatum(datum, LatLon{Lat: remote.DatumLat, Lon: remote.DatumLon}):
			row.Error = fmt.Sprintf("datum differs (%.8f, %.8f vs ours %.8f, %.8f); align the datums first",
				remote.DatumLat, remote.DatumLon, datum.Lat, datum.Lon)
		default:
			pushCtx, cancel := context.WithTimeout(ctx, mapPushBudget(len(areas)))
			status, body, err := requestPeerJSON(pushCtx, &http.Client{Timeout: mapPushBudget(len(areas))}, http.MethodPut, peer.Address, peerMapPath, req)
			cancel()
			switch {
			case err != nil:
				row.Error = "push failed: " + err.Error()
			case status != http.StatusOK:
				row.Error = fmt.Sprintf("peer answered HTTP %d %s", status, strings.TrimSpace(string(body)))
			default:
				row.OK = true
			}
		}
		result.Peers = append(result.Peers, row)
	}
	return result, nil
}

// localAreas probes map_server for every area in areas.dat order.
func (f *FleetProvider) localAreas(ctx context.Context) ([]mowgli.MapArea, error) {
	var areas []mowgli.MapArea
	for i := uint32(0); i < maxMapAreas; i++ {
		req := mowgli.GetMowingAreaReq{Index: i}
		var res mowgli.GetMowingAreaRes
		callCtx, cancel := context.WithTimeout(ctx, 10*time.Second)
		err := f.ros.CallService(callCtx, "/map_server_node/get_mowing_area", &req, &res, "mowgli_interfaces/srv/GetMowingArea")
		cancel()
		if err != nil {
			return nil, fmt.Errorf("get_mowing_area(%d): %w", i, err)
		}
		if !res.Success {
			break
		}
		areas = append(areas, res.Area)
	}
	return areas, nil
}

// stripPendingObstacles returns a copy of the area without its pending
// (unaccepted) obstacle proposals. ObstacleInfo is index-aligned with
// Obstacles; when it is absent every obstacle is kept.
func stripPendingObstacles(a mowgli.MapArea) mowgli.MapArea {
	out := a
	out.Obstacles = make([]geometry.Polygon, 0, len(a.Obstacles))
	out.ObstacleInfo = make([]mowgli.MapObstacleInfo, 0, len(a.ObstacleInfo))
	for i, poly := range a.Obstacles {
		if i < len(a.ObstacleInfo) {
			if a.ObstacleInfo[i].Pending {
				continue
			}
			out.ObstacleInfo = append(out.ObstacleInfo, a.ObstacleInfo[i])
		}
		out.Obstacles = append(out.Obstacles, poly)
	}
	return out
}

func mapPushBudget(nAreas int) time.Duration {
	budget := mapPushBaseWait + time.Duration(nAreas)*mapPushPerArea
	if budget > mapPushMaxWait {
		budget = mapPushMaxWait
	}
	return budget
}
