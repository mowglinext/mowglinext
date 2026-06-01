package foxglove

import (
	"context"
	"encoding/binary"
	"encoding/json"
	"fmt"
	"sync"
	"sync/atomic"
	"time"

	"github.com/gorilla/websocket"
	"github.com/sirupsen/logrus"
)

const (
	defaultReconnectDelay = 1 * time.Second
	maxReconnectDelay     = 30 * time.Second
	callServiceTimeout    = 10 * time.Second
)

// subscriberEntry associates a caller-supplied ID with its callback.
type subscriberEntry struct {
	id       string
	callback func(json.RawMessage)
}

// topicDecimator rate-limits a topic's inbound frames BEFORE the (expensive)
// CDR→JSON deserialization. High-rate topics (/imu ~100 Hz, /scan ~40 Hz) are
// deserialized and marshalled on the read-pump goroutine for every frame; with
// the GUI only consuming ~10 Hz the surplus is pure wasted CPU. Dropping the
// surplus frame at the wire keeps the read pump responsive. lastNano is updated
// with atomics so handleMessageData can throttle while holding only chanMu.RLock.
type topicDecimator struct {
	intervalNano int64 // minimum spacing between dispatched frames; 0 = unlimited
	lastNano     atomic.Int64
}

// channelState holds the parsed schema and subscription info for a channel.
type channelState struct {
	def    channelDef
	schema *msgSchema // parsed from def.Schema

	// subscriptionID is the client-assigned subscription ID (0 = not subscribed).
	subscriptionID uint32
}

// serviceState holds info about an advertised service.
type serviceState struct {
	def serviceDef
}

// pendingServiceCall tracks a single in-flight service call.
type pendingServiceCall struct {
	ch chan serviceCallResult
}

type serviceCallResult struct {
	success bool
	data    []byte
}

// Client connects to a foxglove_bridge via WebSocket and exposes a
// topic-oriented API for subscribing, publishing, and calling services.
//
// All exported methods are safe for concurrent use.
type Client struct {
	url string

	conn   *websocket.Conn
	connMu sync.Mutex

	// channels maps topic name → channel state.
	channels   map[string]*channelState
	channelsByID map[uint32]*channelState
	chanMu     sync.RWMutex

	// services maps service name → service state.
	services     map[string]*serviceState
	servicesByID map[uint32]*serviceState
	svcDefMu     sync.RWMutex

	// subscribers maps topic name → ordered list of (id, callback) pairs.
	subscribers map[string][]subscriberEntry
	subMu       sync.RWMutex

	// decimators maps topic name → *topicDecimator (upstream rate cap applied
	// before CDR deserialization). Populated lazily from Subscribe's optional
	// decimation argument. sync.Map: written rarely (once per topic), read on
	// every inbound frame.
	decimators sync.Map

	// pendingTopics tracks topics that should be subscribed once the channel
	// is advertised by the server.
	pendingTopics map[string]bool
	pendingMu     sync.Mutex

	// pendingSvc maps callID → response channel.
	pendingSvc map[uint32]*pendingServiceCall
	svcMu      sync.Mutex

	// advertised tracks client-publish channels.
	advertised map[string]uint32 // topic → client channel ID
	advMu      sync.Mutex

	connected atomic.Bool
	done      chan struct{}

	reconnectDelay time.Duration
	maxReconnect   time.Duration

	subIDCounter  atomic.Uint32
	callIDCounter atomic.Uint32
	chanIDCounter atomic.Uint32

	// serverCapabilities is set after receiving serverInfo.
	serverCapabilities []string

	// dialer is the WebSocket dialer with the required subprotocol.
	dialer websocket.Dialer
}

// NewClient creates a Client that will connect to the foxglove_bridge
// WebSocket at url (e.g. "ws://localhost:8765").
func NewClient(url string) *Client {
	return &Client{
		url:            url,
		channels:       make(map[string]*channelState),
		channelsByID:   make(map[uint32]*channelState),
		services:       make(map[string]*serviceState),
		servicesByID:   make(map[uint32]*serviceState),
		subscribers:    make(map[string][]subscriberEntry),
		pendingTopics:  make(map[string]bool),
		pendingSvc:     make(map[uint32]*pendingServiceCall),
		advertised:     make(map[string]uint32),
		done:           make(chan struct{}),
		reconnectDelay: defaultReconnectDelay,
		maxReconnect:   maxReconnectDelay,
		dialer: websocket.Dialer{
			Subprotocols: []string{"foxglove.sdk.v1"},
		},
	}
}

// Connected reports whether the client currently has an active WebSocket
// connection.
func (c *Client) Connected() bool {
	return c.connected.Load()
}

// Connect starts the reconnect loop. It never returns an error — callers can
// watch Connected() for state changes.
func (c *Client) Connect(ctx context.Context) error {
	conn, _, err := c.dialer.DialContext(ctx, c.url, nil)
	if err != nil {
		logrus.WithError(err).WithField("url", c.url).
			Warn("foxglove: initial dial failed, will keep retrying")
		go c.reconnectLoop(ctx)
		return nil
	}

	c.connMu.Lock()
	c.conn = conn
	c.connMu.Unlock()
	c.connected.Store(true)

	logrus.WithField("url", c.url).Info("foxglove: connected")

	go c.readPump()
	go c.reconnectLoop(ctx)

	return nil
}

// Close shuts down the connection and stops all background goroutines.
func (c *Client) Close() error {
	select {
	case <-c.done:
		return nil
	default:
		close(c.done)
	}

	c.connMu.Lock()
	defer c.connMu.Unlock()

	if c.conn == nil {
		return nil
	}

	err := c.conn.WriteMessage(
		websocket.CloseMessage,
		websocket.FormatCloseMessage(websocket.CloseNormalClosure, ""),
	)
	closeErr := c.conn.Close()
	c.connected.Store(false)

	if closeErr != nil {
		return fmt.Errorf("foxglove: close: %w", closeErr)
	}
	return err
}

// Subscribe registers cb to receive every message published on topic.
// id is a caller-supplied identifier for deduplication. msgType is ignored
// (foxglove_bridge provides type info); it is accepted for API compatibility.
// An optional first opt is the upstream decimation interval in milliseconds:
// inbound frames closer together than this are dropped before deserialization.
func (c *Client) Subscribe(topic, msgType, id string, cb func(json.RawMessage), opts ...int) error {
	if len(opts) > 0 && opts[0] > 0 {
		c.decimators.Store(topic, &topicDecimator{
			intervalNano: int64(opts[0]) * int64(time.Millisecond),
		})
	}

	c.subMu.Lock()
	defer c.subMu.Unlock()

	entries := c.subscribers[topic]
	for i, e := range entries {
		if e.id == id {
			entries[i].callback = cb
			c.subscribers[topic] = entries
			return nil
		}
	}

	firstSubscriber := len(entries) == 0
	c.subscribers[topic] = append(entries, subscriberEntry{id: id, callback: cb})

	if firstSubscriber {
		c.subscribeTopic(topic)
	}
	return nil
}

// subscribeTopic sends the subscribe message if the channel is known,
// otherwise marks the topic as pending.
func (c *Client) subscribeTopic(topic string) {
	c.chanMu.Lock()
	ch, ok := c.channels[topic]
	if ok && c.connected.Load() {
		subID := c.subIDCounter.Add(1)
		ch.subscriptionID = subID
		c.chanMu.Unlock()

		msg := clientSubscribe{
			Op: "subscribe",
			Subscriptions: []subscriptionDef{
				{ID: subID, ChannelID: ch.def.ID},
			},
		}
		if err := c.writeJSON(msg); err != nil {
			logrus.WithError(err).WithField("topic", topic).
				Warn("foxglove: failed to subscribe")
		}
	} else {
		c.chanMu.Unlock()
		c.pendingMu.Lock()
		c.pendingTopics[topic] = true
		c.pendingMu.Unlock()
	}
}

// Unsubscribe removes the callback identified by id from topic.
func (c *Client) Unsubscribe(topic, id string) {
	c.subMu.Lock()
	defer c.subMu.Unlock()

	entries, ok := c.subscribers[topic]
	if !ok {
		return
	}

	filtered := entries[:0]
	for _, e := range entries {
		if e.id != id {
			filtered = append(filtered, e)
		}
	}

	if len(filtered) == 0 {
		delete(c.subscribers, topic)

		c.chanMu.RLock()
		ch, chOK := c.channels[topic]
		c.chanMu.RUnlock()

		if chOK && ch.subscriptionID != 0 && c.connected.Load() {
			msg := clientUnsubscribe{
				Op:              "unsubscribe",
				SubscriptionIDs: []uint32{ch.subscriptionID},
			}
			if err := c.writeJSON(msg); err != nil {
				logrus.WithError(err).WithField("topic", topic).
					Warn("foxglove: failed to unsubscribe")
			}
			ch.subscriptionID = 0
		}
	} else {
		c.subscribers[topic] = filtered
	}
}

// Advertise is a no-op kept for API compatibility; foxglove handles this
// via the clientAdvertise message sent by Publish.
func (c *Client) Advertise(topic, msgType string) error {
	return nil
}

// Publish sends a message on topic. The message is JSON-encoded and sent as a
// client publish message. An optional schemaName (e.g. "geometry_msgs/msg/Twist")
// tells the bridge how to convert JSON to CDR for ROS2 publishing.
func (c *Client) Publish(topic string, msg interface{}, schemaName ...string) error {
	if !c.connected.Load() {
		return fmt.Errorf("foxglove: Publish %s: not connected", topic)
	}

	data, err := json.Marshal(msg)
	if err != nil {
		return fmt.Errorf("foxglove: Publish marshal: %w", err)
	}

	c.advMu.Lock()
	chanID, ok := c.advertised[topic]
	if !ok {
		chanID = c.chanIDCounter.Add(1)
		sn := ""
		if len(schemaName) > 0 {
			sn = schemaName[0]
		}
		advMsg := clientAdvertise{
			Op: "advertise",
			Channels: []clientChannelDef{
				{
					ID:         chanID,
					Topic:      topic,
					Encoding:   "json",
					SchemaName: sn,
				},
			},
		}
		if err := c.writeJSON(advMsg); err != nil {
			c.advMu.Unlock()
			return fmt.Errorf("foxglove: Publish advertise: %w", err)
		}
		c.advertised[topic] = chanID
	}
	c.advMu.Unlock()

	// Binary frame: opcode(1) + channelID(4) + data
	buf := make([]byte, 5+len(data))
	buf[0] = clientBinMessageData
	binary.LittleEndian.PutUint32(buf[1:5], chanID)
	copy(buf[5:], data)

	c.connMu.Lock()
	defer c.connMu.Unlock()
	if c.conn == nil {
		return fmt.Errorf("foxglove: Publish: no connection")
	}
	return c.conn.WriteMessage(websocket.BinaryMessage, buf)
}

// CallService invokes a ROS2 service and blocks until a response arrives or
// ctx is cancelled. The request is JSON-marshalled, then CDR-encoded using the
// service's request schema. The CDR response is deserialized back to JSON.
func (c *Client) CallService(ctx context.Context, service string, args interface{}, serviceType ...string) (json.RawMessage, error) {
	if !c.connected.Load() {
		return nil, fmt.Errorf("foxglove: CallService %s: not connected", service)
	}

	c.svcDefMu.RLock()
	svc, ok := c.services[service]
	c.svcDefMu.RUnlock()
	if !ok {
		return nil, fmt.Errorf("foxglove: CallService %s: service not advertised", service)
	}

	// Parse request schema and serialize args to CDR.
	reqSchema, err := ParseSchema(svc.def.Request.Schema)
	if err != nil {
		return nil, fmt.Errorf("foxglove: CallService %s: parse request schema: %w", service, err)
	}

	jsonData, err := json.Marshal(args)
	if err != nil {
		return nil, fmt.Errorf("foxglove: CallService %s: marshal args: %w", service, err)
	}

	cdrData, err := SerializeCDR(jsonData, reqSchema)
	if err != nil {
		return nil, fmt.Errorf("foxglove: CallService %s: CDR serialize: %w", service, err)
	}

	callID := c.callIDCounter.Add(1)
	respCh := make(chan serviceCallResult, 1)

	c.svcMu.Lock()
	c.pendingSvc[callID] = &pendingServiceCall{ch: respCh}
	c.svcMu.Unlock()

	defer func() {
		c.svcMu.Lock()
		delete(c.pendingSvc, callID)
		c.svcMu.Unlock()
	}()

	// Build binary service call request:
	// opcode(1) + serviceID(4) + callID(4) + encodingLen(4) + encoding + cdrData
	encoding := []byte("cdr")
	buf := make([]byte, 1+4+4+4+len(encoding)+len(cdrData))
	buf[0] = clientBinServiceCallRequest
	binary.LittleEndian.PutUint32(buf[1:5], svc.def.ID)
	binary.LittleEndian.PutUint32(buf[5:9], callID)
	binary.LittleEndian.PutUint32(buf[9:13], uint32(len(encoding)))
	copy(buf[13:13+len(encoding)], encoding)
	copy(buf[13+len(encoding):], cdrData)

	c.connMu.Lock()
	if c.conn == nil {
		c.connMu.Unlock()
		return nil, fmt.Errorf("foxglove: CallService: no connection")
	}
	err = c.conn.WriteMessage(websocket.BinaryMessage, buf)
	c.connMu.Unlock()
	if err != nil {
		return nil, fmt.Errorf("foxglove: CallService send: %w", err)
	}

	// Parse response schema for CDR deserialization.
	respSchema, parseErr := ParseSchema(svc.def.Response.Schema)

	select {
	case <-ctx.Done():
		return nil, fmt.Errorf("foxglove: CallService %s: %w", service, ctx.Err())
	case result := <-respCh:
		if !result.success {
			return nil, fmt.Errorf("foxglove: CallService %s: service returned failure: %s", service, string(result.data))
		}
		// Deserialize CDR response to JSON.
		if parseErr != nil {
			// Can't parse schema — return raw data
			return json.RawMessage(result.data), nil
		}
		decoded, err := DeserializeCDR(result.data, respSchema)
		if err != nil {
			return nil, fmt.Errorf("foxglove: CallService %s: CDR deserialize response: %w", service, err)
		}
		jsonResult, err := json.Marshal(decoded)
		if err != nil {
			return nil, fmt.Errorf("foxglove: CallService %s: marshal response: %w", service, err)
		}
		return json.RawMessage(jsonResult), nil
	}
}

// ---------------------------------------------------------------------------
// Internal: write, read, dispatch
// ---------------------------------------------------------------------------

func (c *Client) writeJSON(v interface{}) error {
	data, err := json.Marshal(v)
	if err != nil {
		return fmt.Errorf("foxglove: marshal: %w", err)
	}

	c.connMu.Lock()
	defer c.connMu.Unlock()
	if c.conn == nil {
		return fmt.Errorf("foxglove: writeJSON: no connection")
	}
	return c.conn.WriteMessage(websocket.TextMessage, data)
}

func (c *Client) readPump() {
	defer func() {
		c.connected.Store(false)
		c.connMu.Lock()
		if c.conn != nil {
			_ = c.conn.Close()
			c.conn = nil
		}
		c.connMu.Unlock()
		logrus.Info("foxglove: readPump exiting")
	}()

	c.connMu.Lock()
	conn := c.conn
	c.connMu.Unlock()
	if conn == nil {
		return
	}

	for {
		select {
		case <-c.done:
			return
		default:
		}

		msgType, data, err := conn.ReadMessage()
		if err != nil {
			select {
			case <-c.done:
				return
			default:
				logrus.WithError(err).Warn("foxglove: read error, triggering reconnect")
				return
			}
		}

		switch msgType {
		case websocket.TextMessage:
			c.dispatchText(data)
		case websocket.BinaryMessage:
			c.dispatchBinary(data)
		}
	}
}

func (c *Client) dispatchText(data []byte) {
	var env jsonEnvelope
	if err := json.Unmarshal(data, &env); err != nil {
		logrus.WithError(err).Warn("foxglove: failed to unmarshal text frame")
		return
	}

	switch env.Op {
	case "serverInfo":
		var info serverInfo
		if err := json.Unmarshal(data, &info); err != nil {
			logrus.WithError(err).Warn("foxglove: bad serverInfo")
			return
		}
		c.serverCapabilities = info.Capabilities
		logrus.WithFields(logrus.Fields{
			"name":         info.Name,
			"capabilities": info.Capabilities,
		}).Info("foxglove: serverInfo received")

	case "advertise":
		var adv serverAdvertise
		if err := json.Unmarshal(data, &adv); err != nil {
			logrus.WithError(err).Warn("foxglove: bad advertise")
			return
		}
		c.handleAdvertise(adv)

	case "unadvertise":
		var unadv serverUnadvertise
		if err := json.Unmarshal(data, &unadv); err != nil {
			return
		}
		c.handleUnadvertise(unadv)

	case "advertiseServices":
		var svcAdv serverAdvertiseServices
		if err := json.Unmarshal(data, &svcAdv); err != nil {
			logrus.WithError(err).Warn("foxglove: bad advertiseServices")
			return
		}
		c.handleAdvertiseServices(svcAdv)

	case "unadvertiseServices":
		var svcUnadv serverUnadvertiseServices
		if err := json.Unmarshal(data, &svcUnadv); err != nil {
			return
		}
		c.handleUnadvertiseServices(svcUnadv)

	case "serviceCallFailure":
		var failure struct {
			Op      string `json:"op"`
			CallID  uint32 `json:"callId"`
			Message string `json:"message"`
		}
		if err := json.Unmarshal(data, &failure); err != nil {
			return
		}
		c.svcMu.Lock()
		pending, ok := c.pendingSvc[failure.CallID]
		c.svcMu.Unlock()
		if ok {
			select {
			case pending.ch <- serviceCallResult{success: false, data: []byte(failure.Message)}:
			default:
			}
		}

	case "status":
		var status serverStatus
		if err := json.Unmarshal(data, &status); err != nil {
			return
		}
		logrus.WithFields(logrus.Fields{
			"level": status.Level,
			"msg":   status.Message,
		}).Info("foxglove: server status")

	default:
		logrus.WithField("op", env.Op).Debug("foxglove: unhandled text op")
	}
}

func (c *Client) dispatchBinary(data []byte) {
	if len(data) < 1 {
		return
	}

	switch data[0] {
	case serverBinMessageData:
		c.handleMessageData(data[1:])
	case serverBinTime:
		// Ignore server time messages.
	case serverBinServiceCallResponse:
		c.handleServiceCallResponse(data[1:])
	}
}

// handleAdvertise processes newly advertised channels and subscribes to any
// pending topics.
func (c *Client) handleAdvertise(adv serverAdvertise) {
	c.chanMu.Lock()
	for _, ch := range adv.Channels {
		schema, err := ParseSchema(ch.Schema)
		if err != nil {
			logrus.WithError(err).WithField("topic", ch.Topic).
				Warn("foxglove: failed to parse channel schema")
			schema = &msgSchema{}
		}
		state := &channelState{def: ch, schema: schema}
		c.channels[ch.Topic] = state
		c.channelsByID[ch.ID] = state
	}
	c.chanMu.Unlock()

	// Check for pending subscriptions.
	c.pendingMu.Lock()
	pending := make([]string, 0, len(c.pendingTopics))
	for topic := range c.pendingTopics {
		pending = append(pending, topic)
	}
	c.pendingMu.Unlock()

	for _, topic := range pending {
		c.chanMu.RLock()
		_, ok := c.channels[topic]
		c.chanMu.RUnlock()
		if !ok {
			continue
		}

		c.subMu.RLock()
		hasSubs := len(c.subscribers[topic]) > 0
		c.subMu.RUnlock()

		if hasSubs {
			c.subscribeTopic(topic)
			c.pendingMu.Lock()
			delete(c.pendingTopics, topic)
			c.pendingMu.Unlock()
		}
	}
}

func (c *Client) handleUnadvertise(unadv serverUnadvertise) {
	c.chanMu.Lock()
	defer c.chanMu.Unlock()
	for _, id := range unadv.ChannelIDs {
		if state, ok := c.channelsByID[id]; ok {
			delete(c.channels, state.def.Topic)
			delete(c.channelsByID, id)
		}
	}
}

func (c *Client) handleAdvertiseServices(adv serverAdvertiseServices) {
	c.svcDefMu.Lock()
	defer c.svcDefMu.Unlock()
	for _, svc := range adv.Services {
		state := &serviceState{def: svc}
		c.services[svc.Name] = state
		c.servicesByID[svc.ID] = state
	}
	logrus.WithField("count", len(adv.Services)).Debug("foxglove: services advertised")
}

func (c *Client) handleUnadvertiseServices(unadv serverUnadvertiseServices) {
	c.svcDefMu.Lock()
	defer c.svcDefMu.Unlock()
	for _, id := range unadv.ServiceIDs {
		if state, ok := c.servicesByID[id]; ok {
			delete(c.services, state.def.Name)
			delete(c.servicesByID, id)
		}
	}
}

// handleMessageData processes a binary subscription data frame:
// subscriptionID(4) + timestamp(8) + payload
func (c *Client) handleMessageData(data []byte) {
	if len(data) < 12 {
		return
	}
	subID := binary.LittleEndian.Uint32(data[0:4])
	// timestamp := binary.LittleEndian.Uint64(data[4:12])
	payload := data[12:]

	// Find the channel for this subscription.
	c.chanMu.RLock()
	var topic string
	var ch *channelState
	for _, state := range c.channels {
		if state.subscriptionID == subID {
			topic = state.def.Topic
			ch = state
			break
		}
	}
	c.chanMu.RUnlock()

	if ch == nil {
		return
	}

	// Upstream decimation: drop this frame BEFORE the costly CDR→JSON when the
	// topic is rate-capped and the previous frame was dispatched too recently.
	// Keeps the read pump off the floor on /imu, /scan, etc.
	if d, ok := c.decimators.Load(topic); ok {
		dec := d.(*topicDecimator)
		if dec.intervalNano > 0 {
			now := time.Now().UnixNano()
			last := dec.lastNano.Load()
			if now-last < dec.intervalNano {
				return
			}
			dec.lastNano.Store(now)
		}
	}

	// Deserialize CDR → JSON.
	result, err := DeserializeCDR(payload, ch.schema)
	if err != nil {
		logrus.WithError(err).WithField("topic", topic).
			Debug("foxglove: CDR deserialization failed")
		return
	}

	jsonData, err := json.Marshal(result)
	if err != nil {
		logrus.WithError(err).WithField("topic", topic).
			Debug("foxglove: JSON marshal failed")
		return
	}

	c.dispatchToSubscribers(topic, json.RawMessage(jsonData))
}

func (c *Client) dispatchToSubscribers(topic string, msg json.RawMessage) {
	c.subMu.RLock()
	entries := make([]subscriberEntry, len(c.subscribers[topic]))
	copy(entries, c.subscribers[topic])
	c.subMu.RUnlock()

	// Dispatch synchronously on the read-pump goroutine. The GUI's callback is
	// a non-blocking mailbox publish (RosSubscriber.Publish), so this cannot
	// stall the pump, and it preserves message order + avoids spawning a
	// goroutine per frame (which, at /scan + /imu rates with several
	// subscribers, churned the scheduler and reordered fan-out).
	for _, e := range entries {
		e.callback(msg)
	}
}

// handleServiceCallResponse processes a binary service response frame
// (foxglove SDK v1 format):
// serviceID(4) + callID(4) + encodingLen(4) + encoding + responseData
func (c *Client) handleServiceCallResponse(data []byte) {
	if len(data) < 12 {
		return
	}
	// serviceID := binary.LittleEndian.Uint32(data[0:4])
	callID := binary.LittleEndian.Uint32(data[4:8])
	encLen := binary.LittleEndian.Uint32(data[8:12])

	offset := 12 + int(encLen)
	if offset > len(data) {
		return
	}
	// encoding := string(data[12:offset])
	respData := data[offset:]

	c.svcMu.Lock()
	pending, ok := c.pendingSvc[callID]
	c.svcMu.Unlock()

	if !ok {
		logrus.WithField("callID", callID).Warn("foxglove: service response for unknown call")
		return
	}

	select {
	case pending.ch <- serviceCallResult{success: true, data: respData}:
	default:
		logrus.WithField("callID", callID).Warn("foxglove: service response channel full")
	}
}

// ---------------------------------------------------------------------------
// Reconnect loop
// ---------------------------------------------------------------------------

func (c *Client) reconnectLoop(ctx context.Context) {
	delay := c.reconnectDelay

	for {
		for c.connected.Load() {
			select {
			case <-c.done:
				return
			case <-ctx.Done():
				return
			case <-time.After(200 * time.Millisecond):
			}
		}

		select {
		case <-c.done:
			return
		case <-ctx.Done():
			return
		default:
		}

		logrus.WithFields(logrus.Fields{
			"url":   c.url,
			"delay": delay,
		}).Info("foxglove: attempting reconnect")

		conn, _, err := c.dialer.DialContext(ctx, c.url, nil)
		if err != nil {
			logrus.WithError(err).WithField("retry_in", delay).
				Warn("foxglove: reconnect failed")
			select {
			case <-c.done:
				return
			case <-ctx.Done():
				return
			case <-time.After(delay):
			}
			delay *= 2
			if delay > c.maxReconnect {
				delay = c.maxReconnect
			}
			continue
		}

		c.connMu.Lock()
		c.conn = conn
		c.connMu.Unlock()
		c.connected.Store(true)

		// Clear stale state — server will re-advertise channels and services.
		c.chanMu.Lock()
		c.channels = make(map[string]*channelState)
		c.channelsByID = make(map[uint32]*channelState)
		c.chanMu.Unlock()

		c.svcDefMu.Lock()
		c.services = make(map[string]*serviceState)
		c.servicesByID = make(map[uint32]*serviceState)
		c.svcDefMu.Unlock()

		c.advMu.Lock()
		c.advertised = make(map[string]uint32)
		c.advMu.Unlock()

		// Mark all currently subscribed topics as pending so they get
		// re-subscribed when channels are re-advertised.
		c.subMu.RLock()
		c.pendingMu.Lock()
		for topic, entries := range c.subscribers {
			if len(entries) > 0 {
				c.pendingTopics[topic] = true
			}
		}
		c.pendingMu.Unlock()
		c.subMu.RUnlock()

		logrus.WithField("url", c.url).Info("foxglove: reconnected")

		go c.readPump()
		delay = c.reconnectDelay
	}
}
