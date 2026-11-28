package foxglove

import (
	"bytes"
	"encoding/binary"
	"encoding/hex"
	"encoding/json"
	"fmt"
	"math"
	"math/rand"
	"testing"
)

const cdrJSONTestSchema = `uint8 mode
string frame_id
int8[] cells
uint8[256] lookup
byte[] blob
float64 scalar
uint64 correlation_id
int32 sec
uint32 nanosec
example/Child[] children
================================================================================
MSG: example/Child
string label
int8[3] signed
uint8[] unsigned
float64[] values
`

// This is the unchanged public decoder and JSON path used by dev. The candidate
// must match every output byte, including escaping, sorted keys and signed zero.
func cdrJSONReference(wire []byte, schema *msgSchema) ([]byte, error) {
	decoded, err := DeserializeCDR(wire, schema)
	if err != nil {
		return nil, err
	}
	return json.Marshal(sanitizeJSONValue(decoded))
}

type cdrJSONWireBuilder struct {
	wire  []byte
	order binary.AppendByteOrder
	align int
}

func newCDRJSONWire(enc byte) *cdrJSONWireBuilder {
	var order binary.AppendByteOrder = binary.BigEndian
	if enc&1 != 0 {
		order = binary.LittleEndian
	}
	alignment := 8
	if enc == 6 || enc == 7 {
		alignment = 4
	}
	return &cdrJSONWireBuilder{wire: []byte{0, enc, 0, 0}, order: order, align: alignment}
}

func (w *cdrJSONWireBuilder) pad(n int) {
	if n > w.align {
		n = w.align
	}
	for (len(w.wire)-4)%n != 0 {
		w.wire = append(w.wire, 0)
	}
}

func (w *cdrJSONWireBuilder) u32(v uint32) {
	w.pad(4)
	w.wire = w.order.AppendUint32(w.wire, v)
}

func (w *cdrJSONWireBuilder) u64(v uint64) {
	w.pad(8)
	w.wire = w.order.AppendUint64(w.wire, v)
}

func (w *cdrJSONWireBuilder) str(s string) {
	w.u32(uint32(len(s) + 1))
	w.wire = append(w.wire, s...)
	w.wire = append(w.wire, 0)
}

func (w *cdrJSONWireBuilder) sequence(data []byte) {
	w.u32(uint32(len(data)))
	w.wire = append(w.wire, data...)
}

func cdrJSONFixture(enc byte, cells []byte, seed int64) []byte {
	w := newCDRJSONWire(enc)
	w.wire = append(w.wire, 17)
	w.str("map/<>&\"\n\u2028\u2029\xff")
	w.sequence(cells)
	for i := 0; i < 256; i++ {
		w.wire = append(w.wire, byte(i))
	}
	w.sequence([]byte{0, 1, 127, 128, 255})
	w.u64(math.Float64bits(math.Copysign(0, -1)))
	w.u64(math.MaxUint64)
	w.u32(0x80000001)
	w.u32(999999999)
	w.u32(2)
	rng := rand.New(rand.NewSource(seed))
	for _, label := range []string{"nested-0", "nested-1"} {
		w.str(label)
		w.wire = append(w.wire, 0, 128, 255)
		w.sequence([]byte{255, 128, 0, 99})
		bits := []uint64{0, 1 << 63, 1, math.Float64bits(math.MaxFloat64),
			0x7ff0000000000000, 0xfff0000000000000, 0x7ff8000000001234, rng.Uint64()}
		w.u32(uint32(len(bits)))
		for _, v := range bits {
			w.u64(v)
		}
	}
	return w.wire
}

func requireCDRJSONExact(t *testing.T, wire []byte, schema *msgSchema) {
	t.Helper()
	want, wantErr := cdrJSONReference(wire, schema)
	for _, decode := range []func([]byte, *msgSchema) ([]byte, error){deserializeCDRJSON, deserializeCDRJSONByteArrays} {
		got, gotErr := decode(wire, schema)
		if (wantErr == nil) != (gotErr == nil) {
			t.Fatalf("error mismatch: baseline=%v candidate=%v", wantErr, gotErr)
		}
		if wantErr != nil {
			if wantErr.Error() != gotErr.Error() {
				t.Fatalf("error text mismatch: baseline=%v candidate=%v", wantErr, gotErr)
			}
			continue
		}
		if !bytes.Equal(want, got) {
			t.Fatalf("JSON differs\nbaseline: %s\ncandidate: %s", want, got)
		}
	}
}

func TestCDRJSONExactNumericByteArrays(t *testing.T) {
	schema := mustParseSchema(t, cdrJSONTestSchema)
	cells := make([]byte, 256)
	for i := range cells {
		cells[i] = byte(i)
	}
	for _, enc := range []byte{0, 1, 6, 7} {
		for _, n := range []int{0, 1, 255, 256} {
			t.Run(fmt.Sprintf("encoding_%d/cells_%d", enc, n), func(t *testing.T) {
				wire := cdrJSONFixture(enc, cells[:n], 719)
				requireCDRJSONExact(t, wire, schema)
				// Public DeserializeCDR must retain its []int API and signed values.
				decoded, err := DeserializeCDR(wire, schema)
				if err != nil {
					t.Fatal(err)
				}
				gotCells, ok := decoded["cells"].([]int)
				if !ok || len(gotCells) != n {
					t.Fatalf("public cells type/length changed: %T", decoded["cells"])
				}
				for i, v := range gotCells {
					if v != int(int8(cells[i])) {
						t.Fatalf("cell %d: got %d", i, v)
					}
				}
			})
		}
	}
	for _, typ := range []string{"byte", "uint8", "int8"} {
		for _, fixed := range []bool{false, true} {
			arrayType := typ + "[] values"
			if fixed {
				arrayType = typ + "[256] values"
			}
			arraySchema := mustParseSchema(t, arrayType)
			for _, enc := range []byte{0, 1, 6, 7} {
				w := newCDRJSONWire(enc)
				if !fixed {
					w.u32(256)
				}
				w.wire = append(w.wire, cells...)
				requireCDRJSONExact(t, w.wire, arraySchema)
			}
		}
		empty := mustParseSchema(t, typ+"[0] values")
		requireCDRJSONExact(t, []byte{0, 1, 0, 0}, empty)
	}
}

func TestCDRJSONSelectsOnlyLargeByteArrays(t *testing.T) {
	for _, typ := range []string{"int8", "uint8", "byte"} {
		schema := mustParseSchema(t, typ+"[] values")
		for _, n := range []int{0, 1, 256, 4095, 4096, 4097, 8192} {
			for _, enc := range []byte{0, 1, 6, 7} {
				w := newCDRJSONWire(enc)
				cells := make([]byte, n)
				for i := range cells {
					cells[i] = byte(i)
				}
				w.sequence(cells)
				var optimized bool
				_, err := deserializeCDR(w.wire, schema, &optimized)
				if err != nil {
					t.Fatal(err)
				}
				if optimized != (n >= jsonByteArrayMinCells) {
					t.Fatalf("%s count=%d optimized=%v", typ, n, optimized)
				}
				requireCDRJSONExact(t, w.wire, schema)
			}
		}
	}
	for _, enc := range []byte{0, 1, 6, 7} {
		wire, schema, err := cdrJSONFloatFixture(enc, true)
		if err != nil {
			t.Fatal(err)
		}
		var optimized bool
		if _, err := deserializeCDR(wire, schema, &optimized); err != nil {
			t.Fatal(err)
		}
		if optimized {
			t.Fatal("tiny byte array must retain ordinary JSON encoder")
		}
		requireCDRJSONExact(t, wire, schema)
		for _, typ := range []string{"int8", "uint8", "byte"} {
			fixed := mustParseSchema(t, typ+"[4096] values")
			w := newCDRJSONWire(enc)
			w.wire = append(w.wire, make([]byte, 4095)...)
			requireCDRJSONExact(t, w.wire, fixed)
		}
	}
}

func TestCDRJSONLargeFixedAndNestedByteArrays(t *testing.T) {
	for _, typ := range []string{"int8", "uint8", "byte"} {
		fixed := mustParseSchema(t, typ+"[4096] values")
		nested := mustParseSchema(t, "example/Chunk[] chunks\n"+
			"================================================================================\n"+
			"MSG: example/Chunk\nstring label\n"+typ+"[] values\nuint64 tail\n")
		for _, enc := range []byte{0, 1, 6, 7} {
			t.Run(fmt.Sprintf("%s/encoding_%d", typ, enc), func(t *testing.T) {
				cells := make([]byte, 4097)
				for i := range cells {
					cells[i] = byte(i)
				}
				w := newCDRJSONWire(enc)
				w.wire = append(w.wire, cells[:4096]...)
				var optimized bool
				if _, err := deserializeCDR(w.wire, fixed, &optimized); err != nil || !optimized {
					t.Fatalf("large fixed array did not select byte writer: optimized=%v err=%v", optimized, err)
				}
				requireCDRJSONExact(t, w.wire, fixed)

				w = newCDRJSONWire(enc)
				w.u32(2)
				for i := 0; i < 2; i++ {
					w.str(fmt.Sprintf("nested-%d/å<>&", i))
					w.sequence(cells[:4096+i])
					w.u64(math.MaxUint64 - uint64(i))
				}
				optimized = false
				if _, err := deserializeCDR(w.wire, nested, &optimized); err != nil || !optimized {
					t.Fatalf("large nested arrays did not select byte writer: optimized=%v err=%v", optimized, err)
				}
				requireCDRJSONExact(t, w.wire, nested)
			})
		}
	}
}

func TestCDRJSONExactTruncationAndMalformedLengths(t *testing.T) {
	schema := mustParseSchema(t, cdrJSONTestSchema)
	for _, enc := range []byte{0, 1, 6, 7} {
		wire := cdrJSONFixture(enc, []byte{0, 1, 128, 255}, 41)
		for end := 0; end <= len(wire); end++ {
			requireCDRJSONExact(t, wire[:end], schema)
		}
		for _, typ := range []string{"int8", "uint8", "byte"} {
			dynamic := mustParseSchema(t, typ+"[] cells")
			for _, length := range []uint32{1, 256, math.MaxUint32} {
				w := newCDRJSONWire(enc)
				w.u32(length)
				requireCDRJSONExact(t, w.wire, dynamic)
			}
			fixed := mustParseSchema(t, typ+"[256] cells")
			w := newCDRJSONWire(enc)
			w.wire = append(w.wire, make([]byte, 255)...)
			requireCDRJSONExact(t, w.wire, fixed)
		}
	}
	// Zero-length arrays may leave the old decoder aligned beyond EOF. Keep
	// even this accepted malformed-input behavior byte-for-byte compatible.
	zero := mustParseSchema(t, "uint8 flag\nfloat64[0] floats\nint8[0] cells")
	requireCDRJSONExact(t, []byte{0, 1, 0, 0, 7}, zero)
}

func TestCDRJSONSubscriptionPreservesEveryCallback(t *testing.T) {
	schema := mustParseSchema(t, cdrJSONTestSchema)
	client := NewClient("ws://unused")
	client.channels["/grid"] = &channelState{def: channelDef{Topic: "/grid"}, schema: schema, subscriptionID: 81}
	var received [2][][]byte
	for i := range received {
		i := i
		client.subscribers["/grid"] = append(client.subscribers["/grid"], subscriberEntry{
			id: fmt.Sprint(i), callback: func(raw json.RawMessage) {
				received[i] = append(received[i], append([]byte(nil), raw...))
			},
		})
	}
	var reference [][]byte
	for i := 0; i < 32; i++ {
		cells := []byte{byte(i), 255, 0, 128}
		if i%3 == 0 {
			cells = bytes.Repeat(cells, 1500)
		}
		wire := cdrJSONFixture([]byte{0, 1, 6, 7}[i%4], cells, int64(i))
		want, err := cdrJSONReference(wire, schema)
		if err != nil {
			t.Fatal(err)
		}
		reference = append(reference, want)
		frame := make([]byte, 12, 12+len(wire))
		binary.LittleEndian.PutUint32(frame, 81)
		binary.LittleEndian.PutUint64(frame[4:], uint64(i)+1700000000000000000)
		client.handleMessageData(append(frame, wire...))
	}
	for subscriber, stream := range received {
		if len(stream) != len(reference) {
			t.Fatalf("subscriber %d received %d messages; want %d", subscriber, len(stream), len(reference))
		}
		for i := range reference {
			if !bytes.Equal(stream[i], reference[i]) {
				t.Fatalf("subscriber %d message %d differs", subscriber, i)
			}
		}
	}
}

func TestCDRJSONRealIMUWireExact(t *testing.T) {
	wire, err := hex.DecodeString(realImuPayloadHex)
	if err != nil {
		t.Fatal(err)
	}
	requireCDRJSONExact(t, wire, mustParseSchema(t, realImuSchema))
}

func FuzzCDRJSONExact(f *testing.F) {
	schema, err := ParseSchema(cdrJSONTestSchema)
	if err != nil {
		f.Fatal(err)
	}
	for _, enc := range []byte{0, 1, 6, 7} {
		f.Add(cdrJSONFixture(enc, []byte{0, 255, 128, 1}, 89))
		for _, n := range []int{4095, 4096, 4097} {
			cells := make([]byte, n)
			for i := range cells {
				cells[i] = byte(i)
			}
			f.Add(cdrJSONFixture(enc, cells, 97))
		}
	}
	f.Add([]byte{})
	f.Add([]byte{0, 1, 0, 0})
	f.Fuzz(func(t *testing.T, wire []byte) {
		if len(wire) > 32<<10 {
			t.Skip()
		}
		requireCDRJSONExact(t, wire, schema)
	})
}

var cdrJSONBenchmarkOutput []byte

func BenchmarkCDRJSON(b *testing.B) {
	for _, n := range []int{16, 4095, 4096, 300000} {
		cells := make([]byte, n)
		rng := rand.New(rand.NewSource(181719))
		_, _ = rng.Read(cells)
		wire := cdrJSONFixture(1, cells, 419)
		schema, err := ParseSchema(cdrJSONTestSchema)
		if err != nil {
			b.Fatal(err)
		}
		for _, arm := range []struct {
			name string
			fn   func([]byte, *msgSchema) ([]byte, error)
		}{{"baseline", cdrJSONReference}, {"candidate", deserializeCDRJSON}} {
			b.Run(fmt.Sprintf("int8_grid_%d/%s", n, arm.name), func(b *testing.B) {
				b.SetBytes(int64(len(wire)))
				b.ReportAllocs()
				for b.Loop() {
					var err error
					cdrJSONBenchmarkOutput, err = arm.fn(wire, schema)
					if err != nil {
						b.Fatal(err)
					}
				}
			})
		}
	}
	controlSchema, err := ParseSchema("uint64 sequence\nfloat64 value\nstring frame_id")
	if err != nil {
		b.Fatal(err)
	}
	control := newCDRJSONWire(1)
	control.u64(math.MaxUint64)
	control.u64(math.Float64bits(math.Copysign(0, -1)))
	control.str("map")
	for _, arm := range []struct {
		name string
		fn   func([]byte, *msgSchema) ([]byte, error)
	}{{"baseline", cdrJSONReference}, {"candidate", deserializeCDRJSON}} {
		b.Run("control/"+arm.name, func(b *testing.B) {
			b.SetBytes(int64(len(control.wire)))
			b.ReportAllocs()
			for b.Loop() {
				cdrJSONBenchmarkOutput, err = arm.fn(control.wire, controlSchema)
				if err != nil {
					b.Fatal(err)
				}
			}
		})
	}
}
