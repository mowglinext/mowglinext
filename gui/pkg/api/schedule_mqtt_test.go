package api

import (
	"encoding/json"
	"os"
	"path/filepath"
	"sync"
	"testing"
	"time"

	"github.com/mowglinext/mowglinext/pkg/types"
	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
)

// fakeMqttClient is the test double for mqttClient — records every publish and
// lets a test invoke a subscribed handler directly, with no real broker.
type fakeMqttClient struct {
	mu           sync.Mutex
	published    []fakePublication
	handlers     map[string]func(payload []byte, retained bool)
	disconnected bool
}

type fakePublication struct {
	topic    string
	payload  []byte
	retained bool
}

func newFakeMqttClient() *fakeMqttClient {
	return &fakeMqttClient{handlers: map[string]func(payload []byte, retained bool){}}
}

func (f *fakeMqttClient) Publish(topic string, payload []byte, retained bool) error {
	f.mu.Lock()
	defer f.mu.Unlock()
	f.published = append(f.published, fakePublication{topic, payload, retained})
	return nil
}

func (f *fakeMqttClient) Subscribe(topic string, handler func(payload []byte, retained bool)) error {
	f.mu.Lock()
	defer f.mu.Unlock()
	f.handlers[topic] = handler
	return nil
}

func (f *fakeMqttClient) Disconnect() { f.disconnected = true }

func (f *fakeMqttClient) fire(t *testing.T, topic string, payload []byte) {
	t.Helper()
	f.fireRetained(t, topic, payload, false)
}

// fireRetained delivers a message the way the broker replays a RETAINED one on
// (re)subscribe, as opposed to a live publish.
func (f *fakeMqttClient) fireRetained(t *testing.T, topic string, payload []byte, retained bool) {
	t.Helper()
	f.mu.Lock()
	h, ok := f.handlers[topic]
	f.mu.Unlock()
	require.True(t, ok, "no handler subscribed for %s", topic)
	h(payload, retained)
}

func (f *fakeMqttClient) lastPublished(topic string) (fakePublication, bool) {
	f.mu.Lock()
	defer f.mu.Unlock()
	for i := len(f.published) - 1; i >= 0; i-- {
		if f.published[i].topic == topic {
			return f.published[i], true
		}
	}
	return fakePublication{}, false
}

func (f *fakeMqttClient) publishCount() int {
	f.mu.Lock()
	defer f.mu.Unlock()
	return len(f.published)
}

// ===========================================================================
// Publishing the schedule list
// ===========================================================================

func TestScheduleMqttBridge_PublishesCurrentSchedulesOnStart(t *testing.T) {
	db := types.NewMockDBProvider()
	require.NoError(t, saveSchedule(db, &Schedule{ID: "1", AreaID: 2, Time: "07:30", DaysOfWeek: []int{1, 3}, Enabled: true}))

	client := newFakeMqttClient()
	newScheduleMqttBridgeWithClient(db, "mowgli", client)

	pub, ok := client.lastPublished("mowgli/schedules")
	require.True(t, ok)
	assert.True(t, pub.retained, "the schedule mirror must be retained, like <prefix>/area_boundary")

	var resp ScheduleListResponse
	require.NoError(t, json.Unmarshal(pub.payload, &resp))
	require.Len(t, resp.Schedules, 1)
	assert.Equal(t, "1", resp.Schedules[0].ID)
	assert.Equal(t, uint32(2), resp.Schedules[0].AreaID)
}

func TestScheduleMqttBridge_PublishesEmptyArrayNotNullWhenThereAreNone(t *testing.T) {
	db := types.NewMockDBProvider()
	client := newFakeMqttClient()
	newScheduleMqttBridgeWithClient(db, "mowgli", client)

	pub, ok := client.lastPublished("mowgli/schedules")
	require.True(t, ok)
	assert.JSONEq(t, `{"schedules":[]}`, string(pub.payload))
}

// ===========================================================================
// schedules/set: create and update
// ===========================================================================

func TestScheduleMqttBridge_SetWithNoIdCreatesASchedule(t *testing.T) {
	db := types.NewMockDBProvider()
	client := newFakeMqttClient()
	newScheduleMqttBridgeWithClient(db, "mowgli", client)

	client.fire(t, "mowgli/schedules/set", []byte(`{"areaId":0,"time":"06:00","daysOfWeek":[1,2,3,4,5],"enabled":true}`))

	schedules, err := getAllSchedules(db)
	require.NoError(t, err)
	require.Len(t, schedules, 1)
	assert.NotEmpty(t, schedules[0].ID, "a generated id, same as the HTTP POST path")
	assert.Equal(t, "06:00", schedules[0].Time)
	assert.False(t, schedules[0].CreatedAt.IsZero())
}

func TestScheduleMqttBridge_SetWithAnExistingIdUpdatesAndKeepsHistory(t *testing.T) {
	db := types.NewMockDBProvider()
	require.NoError(t, saveSchedule(db, &Schedule{
		ID: "42", AreaID: 1, Time: "06:00", DaysOfWeek: []int{1}, Enabled: true,
		LastSkipReason: "soil wet",
	}))
	client := newFakeMqttClient()
	newScheduleMqttBridgeWithClient(db, "mowgli", client)

	client.fire(t, "mowgli/schedules/set", []byte(`{"id":"42","areaId":1,"time":"08:00","daysOfWeek":[1,2],"enabled":false}`))

	updated, err := getSchedule(db, "42")
	require.NoError(t, err)
	assert.Equal(t, "08:00", updated.Time)
	assert.False(t, updated.Enabled)
	assert.Equal(t, "soil wet", updated.LastSkipReason, "history fields survive an MQTT update, like the HTTP PUT path")
}

func TestScheduleMqttBridge_SetRejectsAnInvalidScheduleAndDoesNotSaveIt(t *testing.T) {
	db := types.NewMockDBProvider()
	client := newFakeMqttClient()
	newScheduleMqttBridgeWithClient(db, "mowgli", client)
	before := client.publishCount()

	client.fire(t, "mowgli/schedules/set", []byte(`{"areaId":0,"time":"25:99","daysOfWeek":[1]}`))

	schedules, err := getAllSchedules(db)
	require.NoError(t, err)
	assert.Empty(t, schedules, "the same validateSchedule the HTTP API uses must reject this")
	assert.Equal(t, before, client.publishCount(), "a rejected write must not republish")
}

func TestScheduleMqttBridge_SetPublishesTheUpdatedList(t *testing.T) {
	db := types.NewMockDBProvider()
	client := newFakeMqttClient()
	newScheduleMqttBridgeWithClient(db, "mowgli", client)

	client.fire(t, "mowgli/schedules/set", []byte(`{"areaId":0,"time":"06:00","daysOfWeek":[1]}`))

	pub, ok := client.lastPublished("mowgli/schedules")
	require.True(t, ok)
	var resp ScheduleListResponse
	require.NoError(t, json.Unmarshal(pub.payload, &resp))
	require.Len(t, resp.Schedules, 1)
}

// ===========================================================================
// schedules/delete
// ===========================================================================

func TestScheduleMqttBridge_DeleteAcceptsAPlainIdPayload(t *testing.T) {
	db := types.NewMockDBProvider()
	require.NoError(t, saveSchedule(db, &Schedule{ID: "7", AreaID: 0, Time: "06:00", DaysOfWeek: []int{1}}))
	client := newFakeMqttClient()
	newScheduleMqttBridgeWithClient(db, "mowgli", client)

	client.fire(t, "mowgli/schedules/delete", []byte("7"))

	_, err := getSchedule(db, "7")
	assert.Error(t, err)
}

func TestScheduleMqttBridge_DeleteAcceptsAJsonIdPayload(t *testing.T) {
	db := types.NewMockDBProvider()
	require.NoError(t, saveSchedule(db, &Schedule{ID: "7", AreaID: 0, Time: "06:00", DaysOfWeek: []int{1}}))
	client := newFakeMqttClient()
	newScheduleMqttBridgeWithClient(db, "mowgli", client)

	client.fire(t, "mowgli/schedules/delete", []byte(`{"id":"7"}`))

	_, err := getSchedule(db, "7")
	assert.Error(t, err)
}

func TestScheduleMqttBridge_DeleteWithNoIdIsIgnored(t *testing.T) {
	db := types.NewMockDBProvider()
	require.NoError(t, saveSchedule(db, &Schedule{ID: "7", AreaID: 0, Time: "06:00", DaysOfWeek: []int{1}}))
	client := newFakeMqttClient()
	newScheduleMqttBridgeWithClient(db, "mowgli", client)

	client.fire(t, "mowgli/schedules/delete", []byte(`{}`))

	_, err := getSchedule(db, "7")
	assert.NoError(t, err, "an empty payload must not be treated as \"delete everything\"")
}

// ===========================================================================
// The HTTP API (the GUI's own Schedules page) also reaches MQTT
// ===========================================================================

func TestScheduleMqttBridge_AnHttpCreateAlsoRepublishesToMqtt(t *testing.T) {
	// registerScheduleChangeListener is process-global (mirrors how schedules.go
	// has no import of this file); clear it so an earlier test's bridge isn't
	// still registered and asserting against the wrong fake client.
	scheduleChangeListenersMu.Lock()
	scheduleChangeListeners = nil
	scheduleChangeListenersMu.Unlock()

	db := types.NewMockDBProvider()
	client := newFakeMqttClient()
	newScheduleMqttBridgeWithClient(db, "mowgli", client)
	before := client.publishCount()

	sched := Schedule{AreaID: 0, Time: "06:00", DaysOfWeek: []int{1}, Enabled: true}
	require.NoError(t, validateSchedule(&sched))
	sched.ID = "999"
	require.NoError(t, saveSchedule(db, &sched))
	notifyScheduleChanged() // what createSchedule's HTTP handler calls after saving

	assert.Greater(t, client.publishCount(), before)
	pub, _ := client.lastPublished("mowgli/schedules")
	var resp ScheduleListResponse
	require.NoError(t, json.Unmarshal(pub.payload, &resp))
	require.Len(t, resp.Schedules, 1)
	assert.Equal(t, "999", resp.Schedules[0].ID)
}

// ===========================================================================
// Broker settings resolution
// ===========================================================================

func writeYamlConfig(t *testing.T, db *types.MockDBProvider, body string) {
	t.Helper()
	path := filepath.Join(t.TempDir(), "mowgli_robot.yaml")
	require.NoError(t, os.WriteFile(path, []byte(body), 0o600))
	require.NoError(t, db.Set("system.mower.yamlConfigFile", []byte(path)))
}

func TestLoadMqttBrokerSettings_ReadsExplicitValues(t *testing.T) {
	db := types.NewMockDBProvider()
	writeYamlConfig(t, db, `
mowgli:
  ros__parameters:
    mqtt_enabled: true
    mqtt_host: "broker.lan"
    mqtt_port: 8883
    mqtt_username: "arco"
    mqtt_password: "secret"
    mqtt_topic_prefix: "garden"
`)

	s, err := loadMqttBrokerSettings(db)
	require.NoError(t, err)
	assert.Equal(t, mqttBrokerSettings{
		enabled: true, host: "broker.lan", port: 8883,
		username: "arco", password: "secret", topicPrefix: "garden",
	}, s)
}

func TestLoadMqttBrokerSettings_AbsentKeysFallBackToTheSameDefaultsAsTheTemplate(t *testing.T) {
	// Isolate from any schema cached by another test in this package (getSchema
	// caches process-wide); this test only exercises the hardcoded fallback
	// that applies when the schema itself carries no default either.
	schemaCacheMu.Lock()
	savedSchema, savedTime := schemaCache, schemaCacheTime
	schemaCache, schemaCacheTime = map[string]any{}, savedTime
	schemaCacheMu.Unlock()
	t.Cleanup(func() {
		schemaCacheMu.Lock()
		schemaCache, schemaCacheTime = savedSchema, savedTime
		schemaCacheMu.Unlock()
	})

	db := types.NewMockDBProvider()
	writeYamlConfig(t, db, "mowgli:\n  ros__parameters: {}\n")

	s, err := loadMqttBrokerSettings(db)
	require.NoError(t, err)
	assert.Equal(t, mqttBrokerSettings{
		enabled: false, host: "localhost", port: 1883, topicPrefix: "mowgli",
	}, s)
}

func TestLoadMqttBrokerSettings_MissingFileIsNotAnError(t *testing.T) {
	db := types.NewMockDBProvider()
	require.NoError(t, db.Set("system.mower.yamlConfigFile", []byte("/does/not/exist.yaml")))

	s, err := loadMqttBrokerSettings(db)
	require.NoError(t, err)
	assert.False(t, s.enabled)
}

// ===========================================================================
// mqtt_enabled: false means the bridge touches no network
// ===========================================================================

func TestNewScheduleMqttBridge_DisabledIsInertAndDoesNotPanic(t *testing.T) {
	db := types.NewMockDBProvider()
	writeYamlConfig(t, db, "mowgli:\n  ros__parameters:\n    mqtt_enabled: false\n")

	assert.NotPanics(t, func() {
		b := NewScheduleMqttBridge(db)
		assert.Nil(t, b.client)
	})
}

// ===========================================================================
// Retained inbound commands are stale operator intent, not new instructions
// ===========================================================================

func TestScheduleMqttBridge_RetainedSetIsIgnored(t *testing.T) {
	db := types.NewMockDBProvider()
	client := newFakeMqttClient()
	newScheduleMqttBridgeWithClient(db, "mowgli", client)

	// A client published schedules/set with retain=1 once; the broker replays
	// it on every GUI start. Since an absent id CREATES, honouring the replay
	// silently adds another enabled mowing schedule per boot — a schedule the
	// operator deleted would come back, and multiply.
	payload, err := json.Marshal(Schedule{AreaID: 1, Time: "06:00", DaysOfWeek: []int{1}, Enabled: true})
	require.NoError(t, err)
	client.fireRetained(t, "mowgli/schedules/set", payload, true)

	all, err := getAllSchedules(db)
	require.NoError(t, err)
	assert.Empty(t, all, "a retained schedules/set must not create a schedule")
}

func TestScheduleMqttBridge_RetainedDeleteIsIgnored(t *testing.T) {
	db := types.NewMockDBProvider()
	require.NoError(t, saveSchedule(db, &Schedule{ID: "keep-me", AreaID: 1, Time: "06:00", DaysOfWeek: []int{1}, Enabled: true}))
	client := newFakeMqttClient()
	newScheduleMqttBridgeWithClient(db, "mowgli", client)

	client.fireRetained(t, "mowgli/schedules/delete", []byte("keep-me"), true)

	all, err := getAllSchedules(db)
	require.NoError(t, err)
	require.Len(t, all, 1, "a retained schedules/delete must not delete a schedule")
	assert.Equal(t, "keep-me", all[0].ID)
}

func TestScheduleMqttBridge_LiveSetIsStillHonoured(t *testing.T) {
	db := types.NewMockDBProvider()
	client := newFakeMqttClient()
	newScheduleMqttBridgeWithClient(db, "mowgli", client)

	payload, err := json.Marshal(Schedule{AreaID: 1, Time: "06:00", DaysOfWeek: []int{1}, Enabled: true})
	require.NoError(t, err)
	client.fireRetained(t, "mowgli/schedules/set", payload, false)

	all, err := getAllSchedules(db)
	require.NoError(t, err)
	assert.Len(t, all, 1, "rejecting retained deliveries must not reject live ones")
}

// ===========================================================================
// Broker settings: mqtt_use_ssl selects the scheme
// ===========================================================================

func TestNewPahoMqttClient_DoesNotBlockWhenTheBrokerIsUnreachable(t *testing.T) {
	// main.go builds this bridge BEFORE api.NewAPI, so a blocking connect
	// against an unreachable broker means the GUI never serves — and the
	// mqtt_enabled toggle that would undo it only exists in that GUI.
	// Port 1 is reserved and refuses immediately; with ConnectRetry paho
	// retries forever, which is exactly the case an unbounded Wait() hung on.
	done := make(chan struct{})
	go func() {
		defer close(done)
		c, err := newPahoMqttClient(mqttBrokerSettings{host: "127.0.0.1", port: 1, topicPrefix: "mowgli"})
		if err == nil && c != nil {
			c.Disconnect()
		}
	}()
	select {
	case <-done:
	case <-time.After(5 * time.Second):
		t.Fatal("newPahoMqttClient blocked on an unreachable broker; GUI startup would hang")
	}
}

func TestLoadMqttBrokerSettings_ReadsUseSSL(t *testing.T) {
	dir := t.TempDir()
	yamlPath := filepath.Join(dir, "mowgli_robot.yaml")
	require.NoError(t, os.WriteFile(yamlPath, []byte(`mowgli:
  ros__parameters:
    mqtt_enabled: true
    mqtt_host: "broker.example"
    mqtt_port: 8883
    mqtt_use_ssl: true
`), 0o644))

	db := types.NewMockDBProvider()
	db.Set("system.mower.yamlConfigFile", []byte(yamlPath))

	settings, err := loadMqttBrokerSettings(db)
	require.NoError(t, err)
	assert.True(t, settings.enabled)
	assert.True(t, settings.useSSL, "mqtt_use_ssl must be read, or the password goes out in the clear")
	assert.Equal(t, 8883, settings.port)
}
