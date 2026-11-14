package api

import (
	"bytes"
	"encoding/binary"
	"encoding/json"
	"strings"

	"github.com/vmihailenco/msgpack/v5"
)

// parsedGrid is immutable after decoding. Cells and metadata are reused for a
// full frame or resync without reinterpreting the cell JSON. Metadata slices
// refer to the same original payload already retained by gridDelta.last.
type parsedGrid struct {
	fields map[string]json.RawMessage
	cells  []int8
}

func (p *parsedGrid) object() (any, error) {
	return gridFieldsObject(p.fields, packInt8Cells(p.cells))
}

// multiplexObject shares the grid decoder with delta subscriptions. Everything
// outside the canonical grid domain keeps the original generic JSON path.
func multiplexObject(topic string, data []byte) (any, error) {
	if len(data) >= 4096 && gridDeltaTopics[topic] {
		if _, parsed, ok := decodeCanonicalGrid(data); ok {
			return parsed.object()
		}
	}
	var obj any
	err := json.Unmarshal(data, &obj)
	return obj, err
}

func decodeGrid(raw []byte) (gridWire, *parsedGrid, error) {
	if len(raw) >= 4096 {
		if w, parsed, ok := decodeCanonicalGrid(raw); ok {
			return w, parsed, nil
		}
	}
	var w gridWire
	err := json.Unmarshal(raw, &w)
	return w, nil, err
}

// gridValueEnd finds a top-level value's boundary in an already validated
// JSON document. It does not decode values or copy the large data array.
func gridValueEnd(raw []byte, start int) int {
	depth := 0
	quoted := false
	for i := start; i < len(raw); i++ {
		c := raw[i]
		if quoted {
			if c == '\\' {
				i++
			} else if c == '"' {
				quoted = false
			}
			continue
		}
		switch c {
		case '"':
			quoted = true
		case '[', '{':
			depth++
		case ']', '}':
			if depth == 0 {
				return i
			}
			depth--
		case ',':
			if depth == 0 {
				return i
			}
		}
	}
	return len(raw)
}

// decodeGridCells decodes a validated JSON array of canonical decimal int8s.
// Keep the direct MessagePack encoder's loop separate so ordinary full-frame
// subscriptions do not pay a per-cell mode branch.
func decodeGridCells(raw []byte) ([]int8, bool) {
	if len(raw) < 2 || raw[0] != '[' || raw[len(raw)-1] != ']' {
		return nil, false
	}
	i := 1
	space := func() {
		for i < len(raw) && (raw[i] == ' ' || raw[i] == '\n' || raw[i] == '\r' || raw[i] == '\t') {
			i++
		}
	}
	space()
	capacity := 0
	if raw[i] != ']' {
		capacity = bytes.Count(raw, []byte{','}) + 1
	}
	cells := make([]int8, 0, capacity)
	if raw[i] == ']' {
		return cells, true
	}
	for {
		space()
		negative := raw[i] == '-'
		if negative {
			i++
		}
		if i == len(raw) || raw[i] < '0' || raw[i] > '9' || (negative && raw[i] == '0') {
			return nil, false
		}
		first := raw[i]
		value := int(first - '0')
		i++
		for i < len(raw) && raw[i] >= '0' && raw[i] <= '9' {
			value = value*10 + int(raw[i]-'0')
			if first == '0' || value > 128 {
				return nil, false
			}
			i++
		}
		if negative {
			value = -value
		}
		if value < -128 || value > 127 {
			return nil, false
		}
		cells = append(cells, int8(value))
		space()
		if i == len(raw) {
			return nil, false
		}
		if raw[i] == ']' {
			return cells, true
		}
		if raw[i] != ',' {
			return nil, false
		}
		i++
	}
}

func decodeCanonicalGrid(raw []byte) (gridWire, *parsedGrid, bool) {
	var w gridWire
	// Whole-document validation preserves encoding/json's nesting limit.
	if !json.Valid(raw) {
		return w, nil, false
	}
	i := 0
	space := func() {
		for i < len(raw) && (raw[i] == ' ' || raw[i] == '\n' || raw[i] == '\r' || raw[i] == '\t') {
			i++
		}
	}
	space()
	if i == len(raw) || raw[i] != '{' {
		return w, nil, false
	}
	i++
	p := &parsedGrid{fields: make(map[string]json.RawMessage)}
	for {
		space()
		if raw[i] == '}' {
			break
		}
		// json.Valid guarantees a quoted key followed by a colon and a value.
		start := i
		i++
		for raw[i] != '"' {
			if raw[i] == '\\' {
				i++
			}
			i++
		}
		i++
		var name string
		if err := json.Unmarshal(raw[start:i], &name); err != nil {
			return w, nil, false
		}
		if _, duplicate := p.fields[name]; duplicate {
			return w, nil, false
		}
		// Struct decoding accepts case aliases and merges duplicate Info
		// objects. Leave every such document on the original decoder.
		for _, known := range []string{"data", "header", "info"} {
			if name != known && strings.EqualFold(name, known) {
				return w, nil, false
			}
		}
		space()
		i++ // colon
		space()
		end := gridValueEnd(raw, i)
		value := bytes.TrimSpace(raw[i:end])
		if name == "data" {
			cells, ok := decodeGridCells(value)
			if !ok {
				return w, nil, false
			}
			p.cells = cells
			p.fields[name] = nil // retain the key, not another copy of the array
		} else {
			p.fields[name] = value
		}
		i = end
		if raw[i] == '}' {
			break
		}
		i++ // comma
	}
	if _, hasData := p.fields["data"]; !hasData {
		return w, nil, false
	}
	w.Data, w.Header = p.cells, p.fields["header"]
	if info, ok := p.fields["info"]; ok {
		if err := json.Unmarshal(info, &w.Info); err != nil {
			return w, nil, false
		}
	}
	return w, p, true
}

// gridFieldsObject interprets metadata only when a complete frame is needed.
func gridFieldsObject(fields map[string]json.RawMessage, cells msgpack.RawMessage) (any, error) {
	obj := make(map[string]any, len(fields))
	for key, value := range fields {
		if key == "data" {
			obj[key] = cells
			continue
		}
		var decoded any
		if err := json.Unmarshal(value, &decoded); err != nil {
			return nil, err
		}
		obj[key] = decoded
	}
	return obj, nil
}

func packInt8Cells(cells []int8) msgpack.RawMessage {
	size := len(cells)
	for _, cell := range cells {
		if cell < -32 {
			size++
		}
	}
	out := make([]byte, 5, 5+size)
	for _, cell := range cells {
		if cell < -32 {
			out = append(out, 0xd0)
		}
		out = append(out, byte(cell))
	}
	return finishGridArray(out, len(cells))
}

func finishGridArray(out []byte, n int) msgpack.RawMessage {
	var header int
	switch {
	case n < 16:
		header = 1
	case n <= 65535:
		header = 3
	default:
		header = 5
	}
	prefix := out[5-header : 5]
	switch header {
	case 1:
		prefix[0] = 0x90 | byte(n)
	case 3:
		prefix[0] = 0xdc
		binary.BigEndian.PutUint16(prefix[1:], uint16(n))
	case 5:
		prefix[0] = 0xdd
		binary.BigEndian.PutUint32(prefix[1:], uint32(n))
	}
	return out[5-header:]
}
