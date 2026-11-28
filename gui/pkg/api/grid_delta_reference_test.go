package api

import (
	"bytes"
	"encoding/json"
	"sync"
)

// Frozen gridDelta implementation from 3695bd7a1, before parse reuse.
type referenceGridDelta struct {
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

func (g *referenceGridDelta) remember(w *gridWire, raw []byte) {
	g.cells = w.Data
	g.width, g.height, g.res = w.Info.Width, w.Info.Height, w.Info.Resolution
	g.origin = append(g.origin[:0], w.Info.Origin...)
	g.last = raw
}

// next decides how to send msg. ok is false when msg is not a decodable grid; the caller then
// sends it as an ordinary frame.
func (g *referenceGridDelta) next(msg []byte) (frame gridFrame, ok bool) {
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
		gaps, vals := make([]int, 0), make([]int, 0)
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
func (g *referenceGridDelta) resync() (frame gridFrame, ok bool) {
	g.mu.Lock()
	defer g.mu.Unlock()
	if g.last == nil {
		return gridFrame{}, false
	}
	g.seq++
	g.sinceFull = 0
	return gridFrame{Full: true, Seq: g.seq, Raw: g.last}, true
}
