package foxglove

import (
	"encoding/json"
	"sort"
	"strconv"
)

// numericByteArray borrows a bounds-checked CDR array until its synchronous JSON
// conversion finishes. It is private to that conversion; the public decoder
// continues to return independent []int values.
type numericByteArray struct {
	cells  []byte
	signed bool
}

// A small byte field must not switch a large float-heavy message away from
// encoding/json. Select this optimization by the array's useful work, not
// merely by the size of the enclosing message.
const jsonByteArrayMinCells = 4096

// Every possible wire byte has one exact JSON integer spelling. Keeping the
// following separator in the table lets the hot loop append a whole token.
var unsignedByteJSON, signedByteJSON = func() ([256]string, [256]string) {
	var unsigned, signed [256]string
	for i := range unsigned {
		unsigned[i] = strconv.Itoa(i) + ","
		signed[i] = strconv.Itoa(int(int8(i))) + ","
	}
	return unsigned, signed
}()

// deserializeCDRJSON matches the public decoder followed by sanitizeJSONValue
// and json.Marshal byte-for-byte. Only numeric byte arrays bypass the standard
// JSON encoder: they need neither an intermediate []int nor per-cell reflection.
func deserializeCDRJSON(data []byte, schema *msgSchema) ([]byte, error) {
	// Keep ordinary telemetry on exactly the established encoder. The size
	// threshold chooses a compute algorithm; every message is still decoded.
	if len(data) < 4096 {
		decoded, err := DeserializeCDR(data, schema)
		if err != nil {
			return nil, err
		}
		return json.Marshal(sanitizeJSONValue(decoded))
	}
	return deserializeCDRJSONByteArrays(data, schema)
}

func deserializeCDRJSONByteArrays(data []byte, schema *msgSchema) ([]byte, error) {
	var hasByteArrays bool
	decoded, err := deserializeCDR(data, schema, &hasByteArrays)
	if err != nil {
		return nil, err
	}
	sanitized := sanitizeJSONValue(decoded)
	if !hasByteArrays {
		return json.Marshal(sanitized)
	}
	return appendCDRJSON(nil, sanitized)
}

func appendCDRJSON(dst []byte, value interface{}) ([]byte, error) {
	switch typed := value.(type) {
	case numericByteArray:
		tokens := &unsignedByteJSON
		if typed.signed {
			tokens = &signedByteJSON
		}
		// One byte needs at most five JSON bytes (signed -128 plus comma).
		// Reserve once instead of repeatedly growing a large output buffer.
		maxToken := 4
		if typed.signed {
			maxToken = 5
		}
		required := len(typed.cells)*maxToken + 2
		if cap(dst)-len(dst) < required {
			grown := make([]byte, len(dst), len(dst)+required)
			copy(grown, dst)
			dst = grown
		}
		dst = append(dst, '[')
		for _, cell := range typed.cells {
			dst = append(dst, tokens[cell]...)
		}
		if len(typed.cells) == 0 {
			return append(dst, ']'), nil
		}
		dst[len(dst)-1] = ']'
		return dst, nil
	case map[string]interface{}:
		keys := make([]string, 0, len(typed))
		for key := range typed {
			keys = append(keys, key)
		}
		sort.Strings(keys)
		dst = append(dst, '{')
		for i, key := range keys {
			if i != 0 {
				dst = append(dst, ',')
			}
			encodedKey, err := json.Marshal(key)
			if err != nil {
				return nil, err
			}
			dst = append(dst, encodedKey...)
			dst = append(dst, ':')
			dst, err = appendCDRJSON(dst, typed[key])
			if err != nil {
				return nil, err
			}
		}
		return append(dst, '}'), nil
	case []interface{}:
		// Primitive arrays cannot contain the private marker, so preserve the
		// standard encoder's efficient whole-array path (including floats).
		structured := false
		for _, item := range typed {
			switch item.(type) {
			case map[string]interface{}, []interface{}, numericByteArray:
				structured = true
			}
			if structured {
				break
			}
		}
		if structured {
			dst = append(dst, '[')
			for i, item := range typed {
				if i != 0 {
					dst = append(dst, ',')
				}
				var err error
				dst, err = appendCDRJSON(dst, item)
				if err != nil {
					return nil, err
				}
			}
			return append(dst, ']'), nil
		}
	}
	// Strings, scalars and all other primitive arrays retain encoding/json's
	// exact escaping and number formatting. Arbitrary RawMessage values also
	// go through its normal validation; only our private marker bypasses it.
	encoded, err := json.Marshal(value)
	if err != nil {
		return nil, err
	}
	return append(dst, encoded...), nil
}
