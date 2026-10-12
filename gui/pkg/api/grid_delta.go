package api

import (
	"bytes"
	"encoding/json"
	"sync"
)

// Delta updates for the occupancy-grid topics.
//
// A mow-progress grid is the bounding box of the whole garden at 5 cm, i.e. over a million
// cells, and while the robot mows only a few dozen of them change per update. Sending the
// whole grid each time (even compressed) wastes the mower's wifi and makes the browser
// decode and rebuild it again. For a connection that asks for it (`"delta": true` on the
// subscribe op) the multiplex route therefore sends:
//
//   - a FULL frame {topic, data, seq} on subscribe (page load, or the tab coming back to the
//     front), when the grid's geometry changes, every gridDeltaFullEvery frames as a safety
//     net, and when a patch would be a large part of the grid anyway;
//   - otherwise a PATCH frame {topic, patch:{base, seq, header, gaps, vals}} holding only the
//     changed cells: gaps[i] is the distance from the previous changed index (from 0 for the
//     first), vals[i] the new value.
//
// The browser applies a patch to the grid it holds and hands the unchanged consumers a full
// grid object. A patch whose base is not the seq the browser holds (a frame was lost or
// reordered) is dropped and a {"op":"resync"} asks for a full frame, so a wrong picture can
// never persist.

// gridDeltaTopics are the topics that can be sent as patches.
var gridDeltaTopics = map[string]bool{"mowProgress": true, "lidarMap": true}

const (
	// gridDeltaFullEvery bounds how long a missed patch could go unnoticed.
	gridDeltaFullEvery = 30
	// A patch touching more than 1/gridDeltaMaxFraction of the cells is sent as a full frame.
	gridDeltaMaxFraction = 4
)

// gridWire is the part of a nav_msgs/OccupancyGrid the diff needs. Header and the origin are
// kept raw: they are copied through, never interpreted. map_load_time is deliberately not
// compared (it may change on every publish and says nothing about the cells).
type gridWire struct {
	Header json.RawMessage `json:"header"`
	Info   struct {
		Resolution float64         `json:"resolution"`
		Width      uint32          `json:"width"`
		Height     uint32          `json:"height"`
		Origin     json.RawMessage `json:"origin"`
	} `json:"info"`
	Data []int8 `json:"data"`
}

// gridFrame is what to send for one grid message.
type gridFrame struct {
	Full   bool
	Seq    uint64
	Base   uint64          // patches: the seq the browser must hold
	Raw    []byte          // full frames: the original message
	Header json.RawMessage // patches
	Gaps   []int
	Vals   []int
}

// gridDelta is the per-subscription state of one delta-capable topic.
type gridDelta struct {
	mu        sync.Mutex
	seq       uint64
	cells     []int8
	width     uint32
	height    uint32
	res       float64
	origin    []byte
	sinceFull int
	last      []byte
}

func (g *gridDelta) remember(w *gridWire, raw []byte) {
	g.cells = w.Data
	g.width, g.height, g.res = w.Info.Width, w.Info.Height, w.Info.Resolution
	g.origin = append(g.origin[:0], w.Info.Origin...)
	g.last = raw
}

// next decides how to send msg. ok is false when msg is not a decodable grid; the caller then
// sends it as an ordinary frame.
func (g *gridDelta) next(msg []byte) (frame gridFrame, ok bool) {
	var w gridWire
	if err := json.Unmarshal(msg, &w); err != nil || len(w.Data) == 0 {
		return gridFrame{}, false
	}
	g.mu.Lock()
	defer g.mu.Unlock()

	sameShape := g.cells != nil && len(w.Data) == len(g.cells) &&
		w.Info.Width == g.width && w.Info.Height == g.height && w.Info.Resolution == g.res &&
		bytes.Equal(w.Info.Origin, g.origin)
	if sameShape && g.sinceFull < gridDeltaFullEvery {
		var gaps, vals []int
		prev := 0
		for i, v := range w.Data {
			if v != g.cells[i] {
				gaps = append(gaps, i-prev)
				vals = append(vals, int(v))
				prev = i
			}
		}
		if len(vals)*gridDeltaMaxFraction <= len(w.Data) {
			base := g.seq
			g.seq++
			g.sinceFull++
			g.remember(&w, msg)
			return gridFrame{Seq: g.seq, Base: base, Header: w.Header, Gaps: gaps, Vals: vals}, true
		}
	}
	g.seq++
	g.sinceFull = 0
	g.remember(&w, msg)
	return gridFrame{Full: true, Seq: g.seq, Raw: msg}, true
}

// resync re-sends the last grid in full, under a new seq, after the browser reported a gap.
func (g *gridDelta) resync() (frame gridFrame, ok bool) {
	g.mu.Lock()
	defer g.mu.Unlock()
	if g.last == nil {
		return gridFrame{}, false
	}
	g.seq++
	g.sinceFull = 0
	return gridFrame{Full: true, Seq: g.seq, Raw: g.last}, true
}
