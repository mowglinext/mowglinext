package providers

import (
	"encoding/json"
	"os"
	"path/filepath"
	"testing"

	"github.com/mowglinext/mowglinext/pkg/types"
	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
	"github.com/vmihailenco/msgpack/v5"
)

func writeRobotYaml(t *testing.T, db types.IDBProvider, body string) {
	t.Helper()
	path := filepath.Join(t.TempDir(), "mowgli_robot.yaml")
	require.NoError(t, os.WriteFile(path, []byte(body), 0o644))
	require.NoError(t, db.Set("system.mower.yamlConfigFile", []byte(path)))
}

func TestReadRobotName_FallsBackToTemplateDefault(t *testing.T) {
	db := types.NewMockDBProvider()
	writeRobotYaml(t, db, "mowgli:\n  ros__parameters:\n    mower_model: YardForce500\n")

	assert.Equal(t, DefaultRobotName, ReadRobotName(db))
}

func TestReadRobotName_ReadsSparseYaml(t *testing.T) {
	db := types.NewMockDBProvider()
	writeRobotYaml(t, db, "mowgli:\n  ros__parameters:\n    robot_name: \"north\"\n    datum_lat: 48.1\n    datum_lon: 2.5\n")

	assert.Equal(t, "north", ReadRobotName(db))
	lat, lon := ReadDatum(db)
	assert.Equal(t, 48.1, lat)
	assert.Equal(t, 2.5, lon)
}

func TestReadRobotName_MissingFileIsDefault(t *testing.T) {
	db := types.NewMockDBProvider()
	require.NoError(t, db.Set("system.mower.yamlConfigFile", []byte("/nonexistent/mowgli_robot.yaml")))

	assert.Equal(t, DefaultRobotName, ReadRobotName(db))
}

func TestEnsureRobotID_IsStable(t *testing.T) {
	db := types.NewMockDBProvider()

	first, err := EnsureRobotID(db)
	require.NoError(t, err)
	second, err := EnsureRobotID(db)
	require.NoError(t, err)

	assert.NotEmpty(t, first)
	assert.Equal(t, first, second)
}

func TestLocalAPIPort(t *testing.T) {
	db := types.NewMockDBProvider()
	assert.Equal(t, defaultAPIPort, LocalAPIPort(db), "no key → default")

	require.NoError(t, db.Set("system.api.addr", []byte(":4123")))
	assert.Equal(t, 4123, LocalAPIPort(db))

	require.NoError(t, db.Set("system.api.addr", []byte("garbage")))
	assert.Equal(t, defaultAPIPort, LocalAPIPort(db))
}

func TestNormalizePeerAddress(t *testing.T) {
	cases := map[string]string{
		"10.0.0.5":                   "10.0.0.5:4006",
		"10.0.0.5:4200":              "10.0.0.5:4200",
		"http://mower-b.local/":      "mower-b.local:4006",
		"https://mower-b:4006/#/map": "mower-b:4006",
		"  mower-c  ":                "mower-c:4006",
	}
	for in, want := range cases {
		got, err := normalizePeerAddress(in)
		require.NoError(t, err, in)
		assert.Equal(t, want, got, in)
	}
	for _, bad := range []string{"", "   ", "http://", "host:notaport"} {
		_, err := normalizePeerAddress(bad)
		assert.Error(t, err, bad)
	}
}

func TestDecodeMultiplexFrame_RoundTripsNestedMaps(t *testing.T) {
	payload, err := msgpack.Marshal(map[string]any{
		"topic": "highLevelStatus",
		"data": map[string]any{
			"state":        2,
			"state_name":   "MOWING",
			"current_area": 3,
			"nested":       map[string]any{"a": []any{1, "x"}},
		},
	})
	require.NoError(t, err)

	topic, data, err := decodeMultiplexFrame(payload)
	require.NoError(t, err)
	assert.Equal(t, "highLevelStatus", topic)

	var decoded map[string]any
	require.NoError(t, json.Unmarshal(data, &decoded))
	assert.Equal(t, "MOWING", decoded["state_name"])
	assert.Equal(t, float64(3), decoded["current_area"])
	assert.Equal(t, map[string]any{"a": []any{float64(1), "x"}}, decoded["nested"])
}

func TestDecodeMultiplexFrame_RejectsTopiclessFrame(t *testing.T) {
	payload, err := msgpack.Marshal(map[string]any{"data": 1})
	require.NoError(t, err)

	_, _, err = decodeMultiplexFrame(payload)
	assert.Error(t, err)
}

func TestFleetProvider_LoadsPersistedPeers(t *testing.T) {
	db := types.NewMockDBProvider()
	writeRobotYaml(t, db, "mowgli:\n  ros__parameters: {}\n")
	require.NoError(t, db.Set(fleetPeersKey, []byte(`[{"id":"b","name":"bravo","address":"127.0.0.1:1"},{"id":"","name":"x","address":"y"}]`)))
	ros := types.NewMockRosProvider()

	f := NewFleetProvider(db, ros)
	defer f.Close()

	peers := f.Peers()
	require.Len(t, peers, 1, "entries without id are dropped")
	assert.Equal(t, "bravo", peers[0].Name)

	rows, err := f.Robots()
	require.NoError(t, err)
	require.Len(t, rows, 2)
	assert.True(t, rows[0].Self)
	assert.False(t, rows[1].Online, "an unreachable peer is offline")
}

func TestFleetProvider_SelfSnapshotMirrorsLocalTopics(t *testing.T) {
	db := types.NewMockDBProvider()
	writeRobotYaml(t, db, "mowgli:\n  ros__parameters:\n    robot_name: alpha\n")
	ros := types.NewMockRosProvider()
	f := NewFleetProvider(db, ros)
	defer f.Close()

	ros.Dispatch("highLevelStatus", []byte(`{"state":1,"state_name":"IDLE"}`))
	ros.Dispatch("power", []byte(`{"v_battery":28.1}`))

	rows, err := f.Robots()
	require.NoError(t, err)
	self := rows[0]
	assert.Equal(t, "alpha", self.Identity.Name)
	assert.JSONEq(t, `{"state":1,"state_name":"IDLE"}`, string(self.Topics["highLevelStatus"]))
	assert.JSONEq(t, `{"v_battery":28.1}`, string(self.Topics["power"]))
	assert.NotNil(t, self.LastSeen)
}

func TestFleetProvider_UnregisterPersists(t *testing.T) {
	db := types.NewMockDBProvider()
	writeRobotYaml(t, db, "mowgli:\n  ros__parameters: {}\n")
	require.NoError(t, db.Set(fleetPeersKey, []byte(`[{"id":"b","name":"bravo","address":"127.0.0.1:1"}]`)))
	f := NewFleetProvider(db, types.NewMockRosProvider())
	defer f.Close()

	f.UnregisterPeer("b")

	assert.Empty(t, f.Peers())
	raw, err := db.Get(fleetPeersKey)
	require.NoError(t, err)
	assert.JSONEq(t, `[]`, string(raw))
}
