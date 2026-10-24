package foxglove

import (
	"encoding/binary"
	"encoding/json"
	"math"
	"os"
	"testing"

	"github.com/stretchr/testify/require"
)

// Share the frontend fixture: a supported channel can have no current value.
// Exercise the real CDR decoder and JSON sanitizer, rather than assuming the
// browser receives the producer's NaN unchanged. This is a message-field subset.
func TestGnssUnavailableMeasurementsMatchFrontendWireFixture(t *testing.T) {
	raw, err := os.ReadFile("../../web/src/test/fixtures/gnss-mavros-sanitized.json")
	require.NoError(t, err)
	var values map[string]any
	require.NoError(t, json.Unmarshal(raw, &values))
	values["vertical_accuracy_m"] = 0
	values["msm_summary_age_s"] = 0
	schema := mustParseSchema(t, "float32 vertical_accuracy_m\nfloat32 msm_summary_age_s\nstring backend\nstring receiver_vendor\nstring receiver_model\nuint8 fix_type\nbool fix_valid\nuint8 rtk_mode\nuint32 capability_flags\nuint32 value_flags\nfloat32 horizontal_accuracy_m\nuint64 position_observation_sequence")
	wire := mustSerialize(t, values, schema)
	binary.LittleEndian.PutUint32(wire[4:8], math.Float32bits(float32(math.NaN())))
	binary.LittleEndian.PutUint32(wire[8:12], math.Float32bits(float32(math.NaN())))
	decoded, err := DeserializeCDR(wire, schema)
	require.NoError(t, err)
	sanitized, err := json.Marshal(sanitizeJSONValue(decoded))
	require.NoError(t, err)
	require.JSONEq(t, string(raw), string(sanitized))
}

func TestGnssSessionTupleWireIntegrity(t *testing.T) {
	values := map[string]any{
		"source_id": "/dev/serial/by-id/receiver", "source_incarnation": "session-a",
		"position_observation_sequence": float64(42), "fix_type": float64(6), "fix_valid": true,
	}
	schema := mustParseSchema(t, "string source_id\nstring source_incarnation\nuint64 position_observation_sequence\nuint8 fix_type\nbool fix_valid")
	wire := mustSerialize(t, values, schema)
	decoded, err := DeserializeCDR(wire, schema)
	require.NoError(t, err)
	encoded, err := json.Marshal(sanitizeJSONValue(decoded))
	require.NoError(t, err)
	var received map[string]any
	require.NoError(t, json.Unmarshal(encoded, &received))
	require.Equal(t, values, received)
}
