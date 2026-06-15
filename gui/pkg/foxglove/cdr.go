package foxglove

import (
	"encoding/binary"
	"fmt"
	"math"
	"strings"
)

// ---------------------------------------------------------------------------
// ROS2 .msg schema parser
// ---------------------------------------------------------------------------

// fieldKind classifies a schema field.
type fieldKind int

const (
	kindPrimitive fieldKind = iota
	kindString
	kindMessage    // nested message
	kindFixedArray // e.g. float64[36]
	kindDynArray   // e.g. float64[]
)

// schemaField describes one field in a ROS2 message definition.
type schemaField struct {
	Name      string
	Kind      fieldKind
	Primitive string       // for kindPrimitive / kindFixedArray / kindDynArray base type
	ArrayLen  int          // for kindFixedArray
	SubFields []schemaField // for kindMessage (inline or referenced)
}

// msgSchema is a parsed ROS2 message definition.
type msgSchema struct {
	Fields []schemaField
}

// ParseSchema parses a multi-message ROS2 .msg definition (as provided by
// foxglove_bridge in the channel advertise message). The format supports
// separator lines of "=" characters followed by "MSG: pkg/MsgName" for nested
// types.
func ParseSchema(schemaText string) (*msgSchema, error) {
	// Split into sub-message definitions. The first block is the root message.
	// Subsequent blocks are preceded by "MSG: <type>" lines.
	blocks := splitMsgBlocks(schemaText)
	if len(blocks) == 0 {
		return &msgSchema{}, nil
	}

	// Build a map of type name → fields for referenced sub-messages.
	// Process in reverse order since dependencies (e.g. builtin_interfaces/Time)
	// are listed after the types that reference them (e.g. std_msgs/Header).
	subTypes := make(map[string][]schemaField)
	for i := len(blocks) - 1; i >= 1; i-- {
		_, fields, err := parseMsgBlock(blocks[i].body, subTypes)
		if err != nil {
			return nil, fmt.Errorf("parse sub-message %q: %w", blocks[i].typeName, err)
		}
		subTypes[blocks[i].typeName] = fields
		// Also store short name (without package prefix)
		if idx := strings.Index(blocks[i].typeName, "/"); idx >= 0 {
			short := blocks[i].typeName[idx+1:]
			subTypes[short] = fields
		}
	}

	// Parse the root message.
	_, fields, err := parseMsgBlock(blocks[0].body, subTypes)
	if err != nil {
		return nil, fmt.Errorf("parse root message: %w", err)
	}
	return &msgSchema{Fields: fields}, nil
}

type msgBlock struct {
	typeName string
	body     string
}

func splitMsgBlocks(text string) []msgBlock {
	// foxglove_bridge separates nested definitions with a line of "=" characters.
	// Split on lines that consist entirely of "=" (at least 3).
	var parts []string
	current := ""
	for _, line := range strings.Split(text, "\n") {
		trimmed := strings.TrimSpace(line)
		if len(trimmed) >= 3 && trimmed == strings.Repeat("=", len(trimmed)) {
			parts = append(parts, current)
			current = ""
		} else {
			current += line + "\n"
		}
	}
	if current != "" {
		parts = append(parts, current)
	}
	var blocks []msgBlock
	for i, part := range parts {
		part = strings.TrimSpace(part)
		if part == "" {
			continue
		}
		if i == 0 {
			blocks = append(blocks, msgBlock{typeName: "", body: part})
			continue
		}
		// Look for "MSG: type/Name" header line
		lines := strings.SplitN(part, "\n", 2)
		typeName := ""
		body := part
		if strings.HasPrefix(lines[0], "MSG: ") {
			typeName = strings.TrimPrefix(lines[0], "MSG: ")
			typeName = strings.TrimSpace(typeName)
			if len(lines) > 1 {
				body = lines[1]
			} else {
				body = ""
			}
		}
		blocks = append(blocks, msgBlock{typeName: typeName, body: body})
	}
	return blocks
}

func parseMsgBlock(body string, subTypes map[string][]schemaField) (string, []schemaField, error) {
	var fields []schemaField
	for _, line := range strings.Split(body, "\n") {
		line = strings.TrimSpace(line)
		if line == "" || strings.HasPrefix(line, "#") {
			continue
		}
		// Strip any trailing inline comment so it can't trip the
		// constant-detection check below (e.g. a field comment that
		// happens to contain "=" like "float64 duration_sec  # 0 = default").
		if hash := strings.Index(line, "#"); hash >= 0 {
			line = strings.TrimSpace(line[:hash])
			if line == "" {
				continue
			}
		}
		// Skip constant definitions (e.g. "uint8 FOO=1")
		if strings.Contains(line, "=") {
			continue
		}

		// Split into type and name.
		parts := strings.Fields(line)
		if len(parts) < 2 {
			continue
		}
		typStr := parts[0]
		name := parts[1]

		field, err := parseFieldType(typStr, name, subTypes)
		if err != nil {
			return "", nil, fmt.Errorf("field %s: %w", name, err)
		}
		fields = append(fields, field)
	}
	return "", fields, nil
}

func parseFieldType(typStr, name string, subTypes map[string][]schemaField) (schemaField, error) {
	// Check for fixed-size array: type[N]
	if idx := strings.Index(typStr, "["); idx >= 0 {
		baseType := typStr[:idx]
		sizeStr := typStr[idx+1 : len(typStr)-1] // remove ']'
		if sizeStr == "" {
			// Dynamic array: type[]
			if isPrimitive(baseType) || baseType == "string" {
				return schemaField{Name: name, Kind: kindDynArray, Primitive: baseType}, nil
			}
			// Dynamic array of messages
			sub := lookupSubType(baseType, subTypes)
			return schemaField{Name: name, Kind: kindDynArray, SubFields: sub, Primitive: baseType}, nil
		}
		// Fixed-size array: type[N]
		var size int
		if _, err := fmt.Sscanf(sizeStr, "%d", &size); err != nil {
			return schemaField{}, fmt.Errorf("bad array size %q", sizeStr)
		}
		if isPrimitive(baseType) || baseType == "string" {
			return schemaField{Name: name, Kind: kindFixedArray, Primitive: baseType, ArrayLen: size}, nil
		}
		sub := lookupSubType(baseType, subTypes)
		return schemaField{Name: name, Kind: kindFixedArray, SubFields: sub, Primitive: baseType, ArrayLen: size}, nil
	}

	if typStr == "string" {
		return schemaField{Name: name, Kind: kindString}, nil
	}
	if isPrimitive(typStr) {
		return schemaField{Name: name, Kind: kindPrimitive, Primitive: typStr}, nil
	}

	// Nested message type
	sub := lookupSubType(typStr, subTypes)
	return schemaField{Name: name, Kind: kindMessage, SubFields: sub, Primitive: typStr}, nil
}

func lookupSubType(typStr string, subTypes map[string][]schemaField) []schemaField {
	if fields, ok := subTypes[typStr]; ok {
		return fields
	}
	// Try without package prefix
	if idx := strings.LastIndex(typStr, "/"); idx >= 0 {
		short := typStr[idx+1:]
		if fields, ok := subTypes[short]; ok {
			return fields
		}
	}
	return nil
}

var primitiveTypes = map[string]bool{
	"bool": true, "byte": true, "char": true,
	"int8": true, "int16": true, "int32": true, "int64": true,
	"uint8": true, "uint16": true, "uint32": true, "uint64": true,
	"float32": true, "float64": true,
}

func isPrimitive(t string) bool {
	return primitiveTypes[t]
}

// primitiveSize returns the CDR wire size in bytes for a primitive type.
func primitiveSize(t string) int {
	switch t {
	case "bool", "byte", "char", "int8", "uint8":
		return 1
	case "int16", "uint16":
		return 2
	case "int32", "uint32", "float32":
		return 4
	case "int64", "uint64", "float64":
		return 8
	default:
		return 0
	}
}

// ---------------------------------------------------------------------------
// CDR reader — walks a schema to deserialize binary CDR data into a
// map[string]interface{} that can be marshalled to JSON.
// ---------------------------------------------------------------------------

// cdrReader reads CDR-encoded data using a parsed schema.
type cdrReader struct {
	data     []byte
	offset   int
	le       bool // little-endian
	maxAlign int  // max alignment (4 for XCDR2/PLAIN_CDR2, 8 for CDR1)
}

// DeserializeCDR deserializes CDR-encoded data using the given schema into a
// JSON-friendly map. The data must include the 4-byte CDR encapsulation header.
func DeserializeCDR(data []byte, schema *msgSchema) (map[string]interface{}, error) {
	if len(data) < 4 {
		return nil, fmt.Errorf("cdr: data too short (%d bytes)", len(data))
	}

	// Encapsulation header (OMG CDR §9.3.3):
	//   data[0]=0x00, data[1]=0x00 → CDR_BE  (XCDR1 big-endian,    maxAlign=8)
	//   data[0]=0x00, data[1]=0x01 → CDR_LE  (XCDR1 little-endian, maxAlign=8)
	//   data[0]=0x00, data[1]=0x06 → CDR2_BE (XCDR2 big-endian,    maxAlign=4)
	//   data[0]=0x00, data[1]=0x07 → CDR2_LE (XCDR2 little-endian, maxAlign=4)
	// The low bit of data[1] indicates little-endian for ALL variants.
	// The high nibble (0x0 vs 0x6/0x7) indicates XCDR1 vs XCDR2 alignment.
	enc := data[1]
	le := enc&0x01 != 0 // low bit = little-endian for both XCDR1 and CDR2
	maxAlign := 8       // XCDR1 default: 8-byte max alignment
	if enc == 0x06 || enc == 0x07 {
		maxAlign = 4 // PLAIN_CDR2: 4-byte max alignment
	}
	r := &cdrReader{data: data, offset: 4, le: le, maxAlign: maxAlign}

	return r.readMessage(schema.Fields)
}

func (r *cdrReader) readMessage(fields []schemaField) (map[string]interface{}, error) {
	out := make(map[string]interface{}, len(fields))
	for _, f := range fields {
		val, err := r.readField(f)
		if err != nil {
			return nil, fmt.Errorf("field %s: %w", f.Name, err)
		}
		out[f.Name] = val
	}
	return out, nil
}

func (r *cdrReader) readField(f schemaField) (interface{}, error) {
	switch f.Kind {
	case kindPrimitive:
		return r.readPrimitive(f.Primitive)
	case kindString:
		return r.readString()
	case kindMessage:
		return r.readMessage(f.SubFields)
	case kindFixedArray:
		return r.readArray(f, f.ArrayLen)
	case kindDynArray:
		r.align(4)
		length, err := r.readUint32()
		if err != nil {
			return nil, err
		}
		// Guard against a corrupt/hostile length triggering a huge make().
		// Every element consumes at least one byte on the wire, so the count
		// can never exceed the bytes remaining in the buffer.
		if int(length) > len(r.data)-r.offset {
			return nil, fmt.Errorf("cdr: dynamic array length %d exceeds %d remaining bytes",
				length, len(r.data)-r.offset)
		}
		return r.readArray(f, int(length))
	default:
		return nil, fmt.Errorf("unknown field kind %d", f.Kind)
	}
}

func (r *cdrReader) readArray(f schemaField, count int) (interface{}, error) {
	baseType := f.Primitive
	if baseType == "string" {
		arr := make([]interface{}, count)
		for i := 0; i < count; i++ {
			s, err := r.readString()
			if err != nil {
				return nil, err
			}
			arr[i] = s
		}
		return arr, nil
	}
	if isPrimitive(baseType) {
		return r.readPrimitiveArray(baseType, count)
	}
	// Array of messages
	arr := make([]interface{}, count)
	for i := 0; i < count; i++ {
		msg, err := r.readMessage(f.SubFields)
		if err != nil {
			return nil, err
		}
		arr[i] = msg
	}
	return arr, nil
}

func (r *cdrReader) readPrimitiveArray(typ string, count int) (interface{}, error) {
	size := primitiveSize(typ)
	r.align(size)

	switch typ {
	case "uint8", "byte":
		// Return as []int to match JSON number semantics
		arr := make([]int, count)
		for i := 0; i < count; i++ {
			if r.offset >= len(r.data) {
				return nil, fmt.Errorf("cdr: unexpected end of data")
			}
			arr[i] = int(r.data[r.offset])
			r.offset++
		}
		return arr, nil
	case "int8":
		arr := make([]int, count)
		for i := 0; i < count; i++ {
			if r.offset >= len(r.data) {
				return nil, fmt.Errorf("cdr: unexpected end of data")
			}
			arr[i] = int(int8(r.data[r.offset]))
			r.offset++
		}
		return arr, nil
	case "float64":
		// Array elements are tightly packed after initial alignment.
		arr := make([]float64, count)
		for i := 0; i < count; i++ {
			if r.offset+8 > len(r.data) {
				return nil, fmt.Errorf("cdr: unexpected end of data")
			}
			var v uint64
			if r.le {
				v = binary.LittleEndian.Uint64(r.data[r.offset:])
			} else {
				v = binary.BigEndian.Uint64(r.data[r.offset:])
			}
			arr[i] = math.Float64frombits(v)
			r.offset += 8
		}
		return arr, nil
	case "float32":
		arr := make([]float64, count)
		for i := 0; i < count; i++ {
			if r.offset+4 > len(r.data) {
				return nil, fmt.Errorf("cdr: unexpected end of data")
			}
			var v uint32
			if r.le {
				v = binary.LittleEndian.Uint32(r.data[r.offset:])
			} else {
				v = binary.BigEndian.Uint32(r.data[r.offset:])
			}
			arr[i] = float64(math.Float32frombits(v))
			r.offset += 4
		}
		return arr, nil
	default:
		// Generic: read each element. Only the first needs alignment
		// (array elements are tightly packed).
		arr := make([]interface{}, count)
		for i := 0; i < count; i++ {
			sz := primitiveSize(typ)
			if r.offset+sz > len(r.data) {
				return nil, fmt.Errorf("cdr: unexpected end of data")
			}
			switch typ {
			case "uint16":
				var v uint16
				if r.le { v = binary.LittleEndian.Uint16(r.data[r.offset:]) } else { v = binary.BigEndian.Uint16(r.data[r.offset:]) }
				arr[i] = int(v)
			case "int16":
				var v uint16
				if r.le { v = binary.LittleEndian.Uint16(r.data[r.offset:]) } else { v = binary.BigEndian.Uint16(r.data[r.offset:]) }
				arr[i] = int(int16(v))
			case "uint32":
				var v uint32
				if r.le { v = binary.LittleEndian.Uint32(r.data[r.offset:]) } else { v = binary.BigEndian.Uint32(r.data[r.offset:]) }
				arr[i] = v
			case "int32":
				var v uint32
				if r.le { v = binary.LittleEndian.Uint32(r.data[r.offset:]) } else { v = binary.BigEndian.Uint32(r.data[r.offset:]) }
				arr[i] = int32(v)
			case "uint64":
				var v uint64
				if r.le { v = binary.LittleEndian.Uint64(r.data[r.offset:]) } else { v = binary.BigEndian.Uint64(r.data[r.offset:]) }
				arr[i] = v
			case "int64":
				var v uint64
				if r.le { v = binary.LittleEndian.Uint64(r.data[r.offset:]) } else { v = binary.BigEndian.Uint64(r.data[r.offset:]) }
				arr[i] = int64(v)
			default:
				v, err := r.readPrimitive(typ)
				if err != nil { return nil, err }
				arr[i] = v
				continue
			}
			r.offset += sz
		}
		return arr, nil
	}
}

func (r *cdrReader) readPrimitive(typ string) (interface{}, error) {
	switch typ {
	case "bool":
		if r.offset >= len(r.data) {
			return nil, fmt.Errorf("cdr: unexpected end of data")
		}
		v := r.data[r.offset] != 0
		r.offset++
		return v, nil
	case "byte", "uint8":
		if r.offset >= len(r.data) {
			return nil, fmt.Errorf("cdr: unexpected end of data")
		}
		v := r.data[r.offset]
		r.offset++
		return int(v), nil
	case "char", "int8":
		if r.offset >= len(r.data) {
			return nil, fmt.Errorf("cdr: unexpected end of data")
		}
		v := int8(r.data[r.offset])
		r.offset++
		return int(v), nil
	case "uint16":
		r.align(2)
		v, err := r.readUint16()
		return int(v), err
	case "int16":
		r.align(2)
		v, err := r.readUint16()
		return int(int16(v)), err
	case "uint32":
		r.align(4)
		v, err := r.readUint32()
		return v, err
	case "int32":
		r.align(4)
		v, err := r.readUint32()
		return int32(v), err
	case "uint64":
		r.align(8)
		v, err := r.readUint64()
		return v, err
	case "int64":
		r.align(8)
		v, err := r.readUint64()
		return int64(v), err
	case "float32":
		return r.readFloat32()
	case "float64":
		return r.readFloat64()
	default:
		return nil, fmt.Errorf("cdr: unknown primitive %q", typ)
	}
}

func (r *cdrReader) readString() (string, error) {
	r.align(4)
	length, err := r.readUint32()
	if err != nil {
		return "", err
	}
	if length == 0 {
		return "", nil
	}
	end := r.offset + int(length)
	if end > len(r.data) {
		return "", fmt.Errorf("cdr: string overflows data (need %d, have %d)", end, len(r.data))
	}
	// CDR strings include a null terminator in the length count.
	s := string(r.data[r.offset : end-1])
	r.offset = end
	return s, nil
}

func (r *cdrReader) readFloat32() (float64, error) {
	r.align(4)
	v, err := r.readUint32()
	if err != nil {
		return 0, err
	}
	return float64(math.Float32frombits(v)), nil
}

func (r *cdrReader) readFloat64() (float64, error) {
	r.align(8)
	v, err := r.readUint64()
	if err != nil {
		return 0, err
	}
	return math.Float64frombits(v), nil
}

func (r *cdrReader) readUint16() (uint16, error) {
	if r.offset+2 > len(r.data) {
		return 0, fmt.Errorf("cdr: unexpected end of data")
	}
	var v uint16
	if r.le {
		v = binary.LittleEndian.Uint16(r.data[r.offset:])
	} else {
		v = binary.BigEndian.Uint16(r.data[r.offset:])
	}
	r.offset += 2
	return v, nil
}

func (r *cdrReader) readUint32() (uint32, error) {
	if r.offset+4 > len(r.data) {
		return 0, fmt.Errorf("cdr: unexpected end of data")
	}
	var v uint32
	if r.le {
		v = binary.LittleEndian.Uint32(r.data[r.offset:])
	} else {
		v = binary.BigEndian.Uint32(r.data[r.offset:])
	}
	r.offset += 4
	return v, nil
}

func (r *cdrReader) readUint64() (uint64, error) {
	if r.offset+8 > len(r.data) {
		return 0, fmt.Errorf("cdr: unexpected end of data")
	}
	var v uint64
	if r.le {
		v = binary.LittleEndian.Uint64(r.data[r.offset:])
	} else {
		v = binary.BigEndian.Uint64(r.data[r.offset:])
	}
	r.offset += 8
	return v, nil
}

// align advances offset to the next multiple of n (capped by maxAlign),
// where alignment is computed relative to the start of the encapsulation body
// — i.e., the byte just after the 4-byte CDR encapsulation header (OMG CORBA
// §15.3.3, "beginning of the message"). Aligning from the absolute buffer
// offset would land 4 bytes off for any field that follows a string in
// XCDR1/CDR2_LE wire frames produced by foxglove_bridge / FastDDS / Cyclone.
func (r *cdrReader) align(n int) {
	if n <= 1 {
		return
	}
	if n > r.maxAlign {
		n = r.maxAlign
	}
	bodyOff := r.offset - 4
	rem := bodyOff % n
	if rem != 0 {
		r.offset += n - rem
	}
}
