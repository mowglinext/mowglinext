package api

import (
	"crypto/tls"
	"encoding/json"
	"fmt"
	"os"
	"sync"
	"time"

	mqtt "github.com/eclipse/paho.mqtt.golang"
	"github.com/mowglinext/mowglinext/pkg/types"
	"github.com/sirupsen/logrus"
	"gopkg.in/yaml.v3"
)

// ScheduleMqttBridge mirrors the mower's mowing schedules onto the SAME external
// MQTT broker mqtt_bridge_node connects to (mqtt_host/port/username/password/
// mqtt_topic_prefix in mowgli_robot.yaml — one master switch, mqtt_enabled,
// gates both). This is deliberately NOT the GUI's own embedded broker
// (providers.MqttProvider, an unrelated mochi-mqtt server on its own port/
// prefix for a different, undocumented purpose) — it publishes on the SAME
// documented contract (docs/MQTT_CONTROL.md) an external tool such as Home
// Assistant already talks to.
//
// Schedules live only in the GUI's own database; ROS2 (and mqtt_bridge_node)
// has no notion of them, so this bridge is the GUI backend acting as its own
// MQTT client rather than a detour through a ROS2 topic.
//
//   - <prefix>/schedules        (out, retained): the current schedule list
//   - <prefix>/schedules/set    (in): create (no "id") or update (existing "id")
//   - <prefix>/schedules/delete (in): the schedule id, as plain text or {"id":...}
//
// Every write goes through validateSchedule/saveSchedule, the exact functions
// the HTTP API uses — there is no second, divergent validation path, and a
// schedule created over MQTT is executed by the same SchedulerProvider poll
// loop as one created in the GUI.
type ScheduleMqttBridge struct {
	dbProvider types.IDBProvider
	client     mqttClient
	prefix     string
	mu         sync.Mutex // serialises writes so a rapid set+delete can't interleave
}

// mqttClient is the minimal surface this bridge needs, small enough to fake in
// tests without a real broker — mirrors the ros2/mowgli_monitoring IMqttClient
// pattern (RecordingMqttClient / StubMqttClient) already used in this repo.
type mqttClient interface {
	Publish(topic string, payload []byte, retained bool) error
	Subscribe(topic string, handler func(payload []byte, retained bool)) error
	Disconnect()
}

// reconnectNotifier is implemented by the real paho client only: it lets the
// bridge re-publish its retained snapshot after the client has re-established
// a dropped connection. The test fake is always "connected" and does not
// implement it.
type reconnectNotifier interface {
	SetOnReconnect(func())
}

// NewScheduleMqttBridge connects if (and only if) mqtt_enabled is true in
// mowgli_robot.yaml; otherwise it returns an inert bridge that touches no
// network, matching mqtt_bridge_node's own "false = not launched" gating.
func NewScheduleMqttBridge(dbProvider types.IDBProvider) *ScheduleMqttBridge {
	b := &ScheduleMqttBridge{dbProvider: dbProvider}
	settings, err := loadMqttBrokerSettings(dbProvider)
	if err != nil {
		logrus.Error(fmt.Errorf("schedule mqtt bridge: reading mqtt settings: %w", err))
		return b
	}
	if !settings.enabled {
		return b
	}
	b.prefix = settings.topicPrefix
	client, err := newPahoMqttClient(settings)
	if err != nil {
		logrus.Error(fmt.Errorf("schedule mqtt bridge: connecting: %w", err))
		return b
	}
	b.client = client
	b.start()
	return b
}

// newScheduleMqttBridgeWithClient is the test seam: build the bridge around an
// already-"connected" fake client instead of a real broker.
func newScheduleMqttBridgeWithClient(dbProvider types.IDBProvider, prefix string, client mqttClient) *ScheduleMqttBridge {
	b := &ScheduleMqttBridge{dbProvider: dbProvider, prefix: prefix, client: client}
	b.start()
	return b
}

// start wires the subscriptions, the HTTP-side listener and the first publish.
// It must stay non-blocking: main.go calls NewScheduleMqttBridge BEFORE
// api.NewAPI, so anything that waits here delays the HTTP server — and an
// operator whose broker is unreachable would get no GUI at all, hence no way
// to switch mqtt_enabled back off (it only exists in the GUI).
func (b *ScheduleMqttBridge) start() {
	if err := b.client.Subscribe(b.prefix+"/schedules/set", b.handleSet); err != nil {
		logrus.Error(fmt.Errorf("schedule mqtt bridge: subscribing to schedules/set: %w", err))
	}
	if err := b.client.Subscribe(b.prefix+"/schedules/delete", b.handleDelete); err != nil {
		logrus.Error(fmt.Errorf("schedule mqtt bridge: subscribing to schedules/delete: %w", err))
	}
	// A schedule created/edited/deleted through the HTTP API (the GUI's own
	// Schedules page) must also reach MQTT — registerScheduleChangeListener is
	// schedules.go's only awareness that this bridge exists.
	registerScheduleChangeListener(b.publish)
	// The schedule list is a RETAINED snapshot. A reconnect starts a fresh
	// session with the broker, so re-publish it rather than leaving whatever
	// the broker retained from the previous connection standing.
	if rn, ok := b.client.(reconnectNotifier); ok {
		rn.SetOnReconnect(b.publish)
	}
	b.publish()
}

// publish is the single place that republishes the schedule list, called after
// every mutation regardless of whether it came from MQTT or the HTTP API
// (registerScheduleChangeListener wires the latter).
func (b *ScheduleMqttBridge) publish() {
	if b.client == nil {
		return
	}
	schedules, err := getAllSchedules(b.dbProvider)
	if err != nil {
		logrus.Error(fmt.Errorf("schedule mqtt bridge: listing schedules: %w", err))
		return
	}
	if schedules == nil {
		schedules = []Schedule{}
	}
	payload, err := json.Marshal(ScheduleListResponse{Schedules: schedules})
	if err != nil {
		logrus.Error(fmt.Errorf("schedule mqtt bridge: marshalling schedules: %w", err))
		return
	}
	if err := b.client.Publish(b.prefix+"/schedules", payload, true); err != nil {
		logrus.Error(fmt.Errorf("schedule mqtt bridge: publishing schedules: %w", err))
	}
}

// handleSet creates a schedule (no "id", or an "id" that does not exist yet)
// or updates one (an "id" that does), exactly like POST/PUT /schedules.
func (b *ScheduleMqttBridge) handleSet(payload []byte, retained bool) {
	// Same rule as mqtt_bridge_node's inbound control topics: a retained
	// command is the broker replaying old operator intent, not a new
	// instruction. Without this, one `schedules/set` published with retain=1
	// is re-delivered on every GUI start and — since an absent/unknown id
	// CREATES — silently adds another enabled mowing schedule each time, so a
	// schedule the operator deleted comes back, and multiplies.
	if retained {
		logrus.Warn("schedule mqtt bridge: retained schedules/set ignored as stale operator intent")
		return
	}
	var sched Schedule
	if err := json.Unmarshal(payload, &sched); err != nil {
		logrus.Error(fmt.Errorf("schedule mqtt bridge: invalid schedules/set payload: %w", err))
		return
	}
	// Which area keys the client actually sent. An integration written before
	// per-area schedules knows neither and would otherwise reset a per-area
	// schedule to "all areas" the moment it toggles `enabled`.
	var sent map[string]json.RawMessage
	_ = json.Unmarshal(payload, &sent)
	_, sentAreaID := sent["areaId"]
	_, sentAreaName := sent["areaName"]
	b.mu.Lock()
	defer b.mu.Unlock()

	if err := validateSchedule(&sched); err != nil {
		logrus.Error(fmt.Errorf("schedule mqtt bridge: rejected schedule: %w", err))
		return
	}
	if existing, err := getSchedule(b.dbProvider, sched.ID); err == nil && existing != nil {
		// Omitted means "unchanged"; an explicit areaId (including 0 = all areas)
		// always wins. A name is only carried over for the SAME area.
		if !sentAreaID {
			sched.AreaID = existing.AreaID
			sched.AreaName = existing.AreaName
		} else if !sentAreaName && sched.AreaID == existing.AreaID {
			sched.AreaName = existing.AreaName
		}
		sched.CreatedAt = existing.CreatedAt
		sched.LastRun = existing.LastRun
		sched.LastSkipReason = existing.LastSkipReason
		sched.LastSkippedAt = existing.LastSkippedAt
	} else {
		sched.ID = fmt.Sprintf("%d", time.Now().UnixNano())
		sched.CreatedAt = time.Now()
	}
	if err := saveScheduleChecked(b.dbProvider, &sched); err != nil {
		logrus.Error(fmt.Errorf("schedule mqtt bridge: saving schedule: %w", err))
		return
	}
	b.publish()
}

// handleDelete accepts either a bare id ("1758...") or {"id":"1758..."}, so a
// simple MQTT client can publish plain text without building JSON for it.
func (b *ScheduleMqttBridge) handleDelete(payload []byte, retained bool) {
	if retained {
		logrus.Warn("schedule mqtt bridge: retained schedules/delete ignored as stale operator intent")
		return
	}
	id := parseScheduleID(payload)
	if id == "" {
		logrus.Error("schedule mqtt bridge: schedules/delete payload has no id")
		return
	}
	b.mu.Lock()
	defer b.mu.Unlock()

	if err := b.dbProvider.Delete(scheduleKeyPrefix + id); err != nil {
		logrus.Error(fmt.Errorf("schedule mqtt bridge: deleting schedule %s: %w", id, err))
		return
	}
	b.publish()
}

func parseScheduleID(payload []byte) string {
	var withID struct {
		ID string `json:"id"`
	}
	if json.Unmarshal(payload, &withID) == nil && withID.ID != "" {
		return withID.ID
	}
	return string(payload)
}

// --- broker settings --------------------------------------------------------

type mqttBrokerSettings struct {
	enabled     bool
	host        string
	port        int
	username    string
	password    string
	topicPrefix string
	useSSL      bool
}

// loadMqttBrokerSettings reads the same mqtt_* keys mqtt_bridge_node's launch
// injection reads (full_system.launch.py), via the same sparse-installed-
// yaml-over-schema-defaults resolution every other GUI settings reader in this
// package uses (see gnss_runtime_config.go) — never an independent default.
func loadMqttBrokerSettings(dbProvider types.IDBProvider) (mqttBrokerSettings, error) {
	configFilePath, err := dbProvider.Get("system.mower.yamlConfigFile")
	if err != nil {
		return mqttBrokerSettings{}, fmt.Errorf("reading yaml config path: %w", err)
	}
	existingYAML := map[string]any{}
	if data, readErr := os.ReadFile(string(configFilePath)); readErr == nil {
		if err := yaml.Unmarshal(data, &existingYAML); err != nil {
			return mqttBrokerSettings{}, fmt.Errorf("invalid mowgli_robot.yaml: %w", err)
		}
	} else if !os.IsNotExist(readErr) {
		return mqttBrokerSettings{}, fmt.Errorf("reading mowgli_robot.yaml: %w", readErr)
	}
	flat := flattenROS2YAML(existingYAML)
	defaults := loadSchemaDefaults(dbProvider)
	get := func(key string) any {
		if v, ok := flat[key]; ok {
			return v
		}
		return defaults[key]
	}
	return mqttBrokerSettings{
		enabled:     asBool(get("mqtt_enabled"), false),
		host:        asString(get("mqtt_host"), "localhost"),
		port:        asInt(get("mqtt_port"), 1883),
		username:    asString(get("mqtt_username"), ""),
		password:    asString(get("mqtt_password"), ""),
		topicPrefix: asString(get("mqtt_topic_prefix"), "mowgli"),
		useSSL:      asBool(get("mqtt_use_ssl"), false),
	}, nil
}

func asBool(v any, fallback bool) bool {
	if b, ok := v.(bool); ok {
		return b
	}
	return fallback
}

func asString(v any, fallback string) string {
	if s, ok := v.(string); ok && s != "" {
		return s
	}
	return fallback
}

// asInt accepts int (yaml.v3's native decode for a plain integer) and float64
// (encoding/json's native decode for a JSON-schema default number), since
// `get()` above can return either depending on which of the two sources
// answered.
func asInt(v any, fallback int) int {
	switch n := v.(type) {
	case int:
		return n
	case float64:
		return int(n)
	default:
		return fallback
	}
}

// --- the real client, wrapping paho.mqtt.golang -----------------------------

// brokerOpTimeout bounds every broker round-trip. paho's bare token.Wait()
// is unbounded, and with AutoReconnect IsConnected() stays true while the
// client is reconnecting, so a QoS-1 publish issued during a broker outage
// parks until the broker returns. publish() runs synchronously inside the
// gin handlers for the Schedules page (notifyScheduleChanged), so an
// unbounded wait there hangs create/update/delete in the browser.
const brokerOpTimeout = 2 * time.Second

type pahoMqttClient struct {
	client mqtt.Client

	mu          sync.Mutex
	subs        map[string]func(payload []byte, retained bool)
	onReconnect func()
}

// newPahoMqttClient never blocks. With SetConnectRetry(true) paho's Connect
// token does not complete until a connection actually succeeds, so waiting on
// it against an unreachable broker waits forever — and this is called before
// the HTTP server starts. Instead the connection is left to paho's own retry
// loop and everything session-scoped (subscriptions, the retained snapshot) is
// (re-)established from the OnConnect handler, which also covers reconnects:
// a resubscribe is required because each new session starts with none.
func newPahoMqttClient(s mqttBrokerSettings) (*pahoMqttClient, error) {
	c := &pahoMqttClient{subs: map[string]func(payload []byte, retained bool){}}

	// mqtt_use_ssl must be honoured here exactly as mqtt_bridge_node honours
	// it (mosquitto_tls_set against the system CA store, refusing to fall back
	// to plaintext). Ignoring it would send mqtt_password in the clear against
	// the operator's explicit setting, and against a TLS-only listener the
	// plaintext connect can never succeed.
	scheme := "tcp"
	if s.useSSL {
		scheme = "ssl"
	}
	opts := mqtt.NewClientOptions().
		AddBroker(fmt.Sprintf("%s://%s:%d", scheme, s.host, s.port)).
		SetClientID("mowgli-gui-schedules").
		SetAutoReconnect(true).
		SetConnectRetry(true).
		SetOnConnectHandler(c.onConnected)
	if s.useSSL {
		// System CA store (nil RootCAs), same trust model as the C++ bridge.
		opts.SetTLSConfig(&tls.Config{MinVersion: tls.VersionTLS12})
	}
	if s.username != "" {
		opts.SetUsername(s.username)
		opts.SetPassword(s.password)
	}
	c.client = mqtt.NewClient(opts)
	c.client.Connect()
	return c, nil
}

// onConnected re-establishes everything that is scoped to a broker session.
func (c *pahoMqttClient) onConnected(_ mqtt.Client) {
	c.mu.Lock()
	subs := make(map[string]func(payload []byte, retained bool), len(c.subs))
	for topic, handler := range c.subs {
		subs[topic] = handler
	}
	onReconnect := c.onReconnect
	c.mu.Unlock()

	for topic, handler := range subs {
		if err := c.subscribeNow(topic, handler); err != nil {
			logrus.Error(fmt.Errorf("schedule mqtt bridge: resubscribing to %s: %w", topic, err))
		}
	}
	if onReconnect != nil {
		// Off the paho callback goroutine: onReconnect publishes, and paho
		// documents that a callback must not call back into the package.
		go onReconnect()
	}
}

func (c *pahoMqttClient) SetOnReconnect(f func()) {
	c.mu.Lock()
	c.onReconnect = f
	c.mu.Unlock()
}

func (c *pahoMqttClient) Publish(topic string, payload []byte, retained bool) error {
	token := c.client.Publish(topic, 1, retained, payload)
	if !token.WaitTimeout(brokerOpTimeout) {
		return fmt.Errorf("publish to %s timed out after %s", topic, brokerOpTimeout)
	}
	return token.Error()
}

// Subscribe records the handler and establishes it now when the client is
// already connected; otherwise onConnected does it. A not-yet-connected
// client is the normal case at startup and is not an error.
func (c *pahoMqttClient) Subscribe(topic string, handler func(payload []byte, retained bool)) error {
	c.mu.Lock()
	c.subs[topic] = handler
	c.mu.Unlock()
	if !c.client.IsConnected() {
		return nil
	}
	return c.subscribeNow(topic, handler)
}

func (c *pahoMqttClient) subscribeNow(topic string, handler func(payload []byte, retained bool)) error {
	token := c.client.Subscribe(topic, 1, func(_ mqtt.Client, msg mqtt.Message) {
		// paho routes messages on a goroutine that must not block, so the
		// handler (which publishes) runs off it.
		go handler(msg.Payload(), msg.Retained())
	})
	if !token.WaitTimeout(brokerOpTimeout) {
		return fmt.Errorf("subscribe to %s timed out after %s", topic, brokerOpTimeout)
	}
	return token.Error()
}

func (c *pahoMqttClient) Disconnect() {
	c.client.Disconnect(250)
}

// --- notify-on-HTTP-write hook ----------------------------------------------

// scheduleChangeListeners lets the HTTP handlers in schedules.go (create/
// update/delete) also republish the MQTT mirror, without schedules.go needing
// to know this bridge exists (no import in either direction beyond this one
// package-level hook).
var (
	scheduleChangeListenersMu sync.Mutex
	scheduleChangeListeners   []func()
)

func registerScheduleChangeListener(f func()) {
	scheduleChangeListenersMu.Lock()
	defer scheduleChangeListenersMu.Unlock()
	scheduleChangeListeners = append(scheduleChangeListeners, f)
}

func notifyScheduleChanged() {
	scheduleChangeListenersMu.Lock()
	listeners := append([]func(){}, scheduleChangeListeners...)
	scheduleChangeListenersMu.Unlock()
	for _, f := range listeners {
		f()
	}
}
