package providers

import (
	"bytes"
	"context"
	"encoding/json"
	"errors"
	"fmt"
	"io"
	"net/http"
	"strings"
	"sync"
	"time"

	"github.com/gorilla/websocket"
	"github.com/sirupsen/logrus"
	"github.com/vmihailenco/msgpack/v5"
)

// FleetTopics are the logical topic keys a fleet member mirrors from every
// peer. They are the multiplex keys of the peer's own GUI backend, so the
// payloads are the same snake_case JSON the browser already understands.
var FleetTopics = []string{
	"highLevelStatus", "status", "power", "pose", "gps", "gnssStatus",
	"emergency", "coverageSession",
}

const (
	// Peers re-read each other's identity (name, datum, API version) at this
	// cadence: the datum is what places a peer's fused pose in OUR map frame.
	peerIdentityRefresh = 60 * time.Second
	peerHTTPTimeout     = 5 * time.Second
	peerDialTimeout     = 5 * time.Second
	peerReconnectMin    = 2 * time.Second
	peerReconnectMax    = 30 * time.Second
	peerReadDeadline    = 20 * time.Second
	peerPingPeriod      = 8 * time.Second
	peerOnlineMaxAge    = 10 * time.Second
	peerWriteDeadline   = 5 * time.Second
	peerMaxFrameBytes   = 4 << 20
	peerRegisterPath    = "/api/fleet/peers/register"
	peerUnregisterPath  = "/api/fleet/peers/unregister"
	peerIdentityPath    = "/api/fleet/identity"
	peerMultiplexPath   = "/api/mowglinext/multiplex"
	peerCallPathPrefix  = "/api/mowglinext/call/"
)

// peerCache is the last message seen per logical topic for one peer.
type peerCache struct {
	mu         sync.RWMutex
	topics     map[string]json.RawMessage
	stamps     map[string]time.Time
	lastSeen   time.Time
	socketUp   bool
	identity   *RobotIdentity
	expectedID string // empty only for the local robot's cache
	coverage   *coverageObservation
	// Not reset on disconnect: order deliveries even with a fixed test clock.
	coverageSequence uint64
}

// One topic delivery, not one read of the cache. The pointer remains stable
// across snapshots; a new delivery gets a new identity even at the same clock
// instant. This is receipt provenance, not a ROS acquisition timestamp.
type coverageObservation struct {
	receivedAt time.Time
	sequence   uint64
}

func newPeerCache() *peerCache {
	return &peerCache{topics: map[string]json.RawMessage{}, stamps: map[string]time.Time{}}
}

func (c *peerCache) store(topic string, data json.RawMessage, now time.Time) {
	c.mu.Lock()
	defer c.mu.Unlock()
	if c.expectedID != "" && (c.identity == nil || c.identity.ID != c.expectedID || c.identity.APIVersion != FleetAPIVersion) {
		return
	}
	c.topics[topic] = data
	c.stamps[topic] = now
	c.lastSeen = now
	if topic == "coverageSession" {
		c.coverageSequence++
		c.coverage = &coverageObservation{receivedAt: now, sequence: c.coverageSequence}
	}
}

func (c *peerCache) setSocket(up bool) {
	c.mu.Lock()
	defer c.mu.Unlock()
	c.socketUp = up
}

func (c *peerCache) setIdentity(id RobotIdentity) {
	c.mu.Lock()
	defer c.mu.Unlock()
	if c.expectedID != "" && (id.ID != c.expectedID || id.APIVersion != FleetAPIVersion) {
		c.clearLocked()
		return
	}
	copyID := id
	c.identity = &copyID
}

func (c *peerCache) clearLocked() {
	c.identity = nil
	c.topics = map[string]json.RawMessage{}
	c.stamps = map[string]time.Time{}
	c.lastSeen = time.Time{}
	c.socketUp = false
	c.coverage = nil
}

func (c *peerCache) invalidateIdentity() {
	c.mu.Lock()
	defer c.mu.Unlock()
	c.clearLocked()
}

// Identity and topics must be qualified under one lock; separate reads could
// combine an old identity with a newly replaced peer's data.
func (c *peerCache) snapshotForIdentity(expectedID string, now time.Time) (RobotIdentity, map[string]json.RawMessage, map[string]float64, time.Time, bool) {
	r := c.robotSnapshot(expectedID, now)
	var seen time.Time
	if r.LastSeen != nil {
		seen = *r.LastSeen
	}
	return r.Identity, r.Topics, r.TopicAgeS, seen, r.Online
}

// Include receipt identity under the SAME qualification lock as FLT-04. An
// invalidated/reconnected peer cannot attach an old receipt to new topics.
func (c *peerCache) robotSnapshot(expectedID string, now time.Time) FleetRobot {
	c.mu.RLock()
	defer c.mu.RUnlock()
	r := FleetRobot{Topics: map[string]json.RawMessage{}, TopicAgeS: map[string]float64{}}
	if expectedID != "" && (c.identity == nil || c.identity.ID != expectedID || c.identity.APIVersion != FleetAPIVersion) {
		return r
	}
	topics, ages, seen, online := c.snapshotLocked(now)
	if c.identity != nil {
		r.Identity = *c.identity
	}
	r.Topics, r.TopicAgeS, r.LastSeen, r.Online = topics, ages, timePtr(seen), online
	r.coverageObservation = c.coverage
	return r
}

// identitySnapshot returns the last identity the peer advertised, if any.
func (c *peerCache) identitySnapshot() (RobotIdentity, bool) {
	c.mu.RLock()
	defer c.mu.RUnlock()
	if c.identity == nil {
		return RobotIdentity{}, false
	}
	return *c.identity, true
}

// snapshot returns an immutable copy of the cached topics, each topic's age
// in seconds, the last receipt time and liveness.
func (c *peerCache) snapshot(now time.Time) (map[string]json.RawMessage, map[string]float64, time.Time, bool) {
	c.mu.RLock()
	defer c.mu.RUnlock()
	return c.snapshotLocked(now)
}

func (c *peerCache) snapshotLocked(now time.Time) (map[string]json.RawMessage, map[string]float64, time.Time, bool) {
	out := make(map[string]json.RawMessage, len(c.topics))
	ages := make(map[string]float64, len(c.topics))
	for k, v := range c.topics {
		out[k] = v
		ages[k] = now.Sub(c.stamps[k]).Seconds()
	}
	hlAge := now.Sub(c.stamps["highLevelStatus"])
	online := c.socketUp && !c.stamps["highLevelStatus"].IsZero() && hlAge <= peerOnlineMaxAge
	return out, ages, c.lastSeen, online
}

// peerClient keeps one WebSocket to a peer GUI's multiplex endpoint and
// mirrors FleetTopics into a peerCache. It reconnects with capped backoff
// until Close is called.
type peerClient struct {
	address               string
	expectedID            string
	cache                 *peerCache
	http                  *http.Client
	dialer                *websocket.Dialer
	onIdentity            func(RobotIdentity)
	onIdentityUnavailable func()
	ctx                   context.Context
	cancel                context.CancelFunc
	done                  chan struct{}
	now                   func() time.Time
	sessionOpened         bool
}

func newPeerClient(address string, now func() time.Time) *peerClient {
	ctx, cancel := context.WithCancel(context.Background())
	return &peerClient{
		address: address,
		cache:   newPeerCache(),
		http:    &http.Client{Timeout: peerHTTPTimeout},
		dialer:  &websocket.Dialer{HandshakeTimeout: peerDialTimeout},
		ctx:     ctx,
		cancel:  cancel,
		done:    make(chan struct{}),
		now:     now,
	}
}

func (p *peerClient) start() {
	go p.run()
}

// Identity refresh belongs to one socket lifetime. Its in-flight HTTP request
// is canceled and joined before reconnecting, so it cannot requalify old data.
func (p *peerClient) identityLoop(ctx context.Context, conn *websocket.Conn) {
	ticker := time.NewTicker(peerIdentityRefresh)
	defer ticker.Stop()
	for {
		select {
		case <-ctx.Done():
			return
		case <-ticker.C:
			if err := p.refreshIdentity(ctx); err != nil {
				_ = conn.Close()
				return
			}
		}
	}
}

func (p *peerClient) refreshIdentity(ctx context.Context) error {
	id, err := fetchPeerIdentity(ctx, p.http, p.address)
	if err == nil && p.expectedID != "" && id.ID != p.expectedID {
		err = fmt.Errorf("peer identity differs: expected %s, received %s", p.expectedID, id.ID)
	}
	if err != nil {
		p.cache.invalidateIdentity()
		if p.onIdentityUnavailable != nil {
			p.onIdentityUnavailable()
		}
		return err
	}
	if p.onIdentity != nil {
		p.onIdentity(id)
	}
	if id.APIVersion != FleetAPIVersion {
		p.cache.invalidateIdentity()
		return fmt.Errorf("peer fleet API v%d is incompatible with ours (v%d)", id.APIVersion, FleetAPIVersion)
	}
	p.cache.setIdentity(id)
	return nil
}

func (p *peerClient) Close() {
	p.cancel()
	<-p.done
}

func (p *peerClient) run() {
	defer close(p.done)
	backoff := peerReconnectMin
	for {
		err := p.session()
		p.cache.setSocket(false)
		if p.ctx.Err() != nil {
			return
		}
		if err != nil {
			logrus.WithField("peer", p.address).Debugf("fleet peer socket: %v", err)
		}
		select {
		case <-p.ctx.Done():
			return
		case <-time.After(backoff):
		}
		backoff *= 2
		if backoff > peerReconnectMax {
			backoff = peerReconnectMax
		}
	}
}

// session runs one connected WebSocket until it fails.
func (p *peerClient) session() error {
	p.cache.invalidateIdentity()
	conn, _, err := p.dialer.DialContext(p.ctx, "ws://"+p.address+peerMultiplexPath, nil)
	if err != nil {
		return err
	}
	defer conn.Close()
	defer p.cache.invalidateIdentity()
	wasConnected := p.sessionOpened
	if wasConnected && p.onIdentityUnavailable != nil {
		p.onIdentityUnavailable()
	}
	p.sessionOpened = true
	if err := p.refreshIdentity(p.ctx); err != nil {
		return err
	}
	conn.SetReadLimit(peerMaxFrameBytes)
	for _, topic := range FleetTopics {
		op := map[string]string{"op": "subscribe", "topic": topic}
		_ = conn.SetWriteDeadline(p.now().Add(peerWriteDeadline))
		if err := conn.WriteJSON(op); err != nil {
			return err
		}
	}
	p.cache.setSocket(true)
	identityCtx, cancelIdentity := context.WithCancel(p.ctx)
	identityDone := make(chan struct{})
	go func() { defer close(identityDone); p.identityLoop(identityCtx, conn) }()
	defer func() { cancelIdentity(); <-identityDone }()

	// Ping keeps the peer's read loop alive and detects a dead link fast;
	// the read deadline is refreshed by every pong or data frame.
	_ = conn.SetReadDeadline(p.now().Add(peerReadDeadline))
	conn.SetPongHandler(func(string) error {
		return conn.SetReadDeadline(p.now().Add(peerReadDeadline))
	})
	stop := make(chan struct{})
	defer close(stop)
	go func() {
		ticker := time.NewTicker(peerPingPeriod)
		defer ticker.Stop()
		for {
			select {
			case <-stop:
				return
			case <-p.ctx.Done():
				_ = conn.Close()
				return
			case <-ticker.C:
				_ = conn.SetWriteDeadline(p.now().Add(peerWriteDeadline))
				if err := conn.WriteMessage(websocket.PingMessage, nil); err != nil {
					_ = conn.Close()
					return
				}
			}
		}
	}()

	for {
		kind, payload, err := conn.ReadMessage()
		if err != nil {
			return err
		}
		_ = conn.SetReadDeadline(p.now().Add(peerReadDeadline))
		if kind != websocket.BinaryMessage {
			continue
		}
		topic, data, err := decodeMultiplexFrame(payload)
		if err != nil {
			continue
		}
		p.cache.store(topic, data, p.now())
	}
}

// decodeMultiplexFrame turns a msgpack {topic, data} frame from a GUI
// multiplex socket back into (topic, JSON payload).
func decodeMultiplexFrame(payload []byte) (string, json.RawMessage, error) {
	var frame struct {
		Topic string `msgpack:"topic"`
		Data  any    `msgpack:"data"`
	}
	if err := msgpack.Unmarshal(payload, &frame); err != nil {
		return "", nil, err
	}
	if frame.Topic == "" {
		return "", nil, errors.New("frame without topic")
	}
	data, err := json.Marshal(normalizeMsgpackValue(frame.Data))
	if err != nil {
		return "", nil, err
	}
	return frame.Topic, data, nil
}

// normalizeMsgpackValue converts map[any]any (which msgpack may produce for
// nested maps) into map[string]any so encoding/json accepts it.
func normalizeMsgpackValue(v any) any {
	switch t := v.(type) {
	case map[string]any:
		out := make(map[string]any, len(t))
		for k, val := range t {
			out[k] = normalizeMsgpackValue(val)
		}
		return out
	case map[any]any:
		out := make(map[string]any, len(t))
		for k, val := range t {
			out[fmt.Sprint(k)] = normalizeMsgpackValue(val)
		}
		return out
	case []any:
		out := make([]any, len(t))
		for i, val := range t {
			out[i] = normalizeMsgpackValue(val)
		}
		return out
	default:
		return v
	}
}

// ---------------------------------------------------------------------------
// HTTP helpers used both by peerClient and by the registry handshake.
// ---------------------------------------------------------------------------

// fetchPeerIdentity asks a GUI at address who it is.
func fetchPeerIdentity(ctx context.Context, client *http.Client, address string) (RobotIdentity, error) {
	req, err := http.NewRequestWithContext(ctx, http.MethodGet, "http://"+address+peerIdentityPath, nil)
	if err != nil {
		return RobotIdentity{}, err
	}
	resp, err := client.Do(req)
	if err != nil {
		return RobotIdentity{}, err
	}
	defer resp.Body.Close()
	if resp.StatusCode != http.StatusOK {
		return RobotIdentity{}, fmt.Errorf("peer %s answered HTTP %d to identity", address, resp.StatusCode)
	}
	var id RobotIdentity
	decoder := json.NewDecoder(io.LimitReader(resp.Body, 64<<10))
	if err := decoder.Decode(&id); err != nil {
		return RobotIdentity{}, err
	}
	if strings.TrimSpace(id.ID) == "" {
		return RobotIdentity{}, fmt.Errorf("peer %s returned an identity without id", address)
	}
	if err := decoder.Decode(new(any)); err != io.EOF {
		return RobotIdentity{}, fmt.Errorf("peer %s returned invalid trailing identity data", address)
	}
	return id, nil
}

// postPeerJSON posts a JSON body to a peer GUI and returns status + body.
func postPeerJSON(ctx context.Context, client *http.Client, address, path string, body any) (int, []byte, error) {
	return requestPeerJSON(ctx, client, http.MethodPost, address, path, body)
}

// requestPeerJSON sends a JSON body to a peer GUI with the given method.
func requestPeerJSON(ctx context.Context, client *http.Client, method, address, path string, body any) (int, []byte, error) {
	buf, err := json.Marshal(body)
	if err != nil {
		return 0, nil, err
	}
	req, err := http.NewRequestWithContext(ctx, method, "http://"+address+path, bytes.NewReader(buf))
	if err != nil {
		return 0, nil, err
	}
	req.Header.Set("Content-Type", "application/json")
	resp, err := client.Do(req)
	if err != nil {
		return 0, nil, err
	}
	defer resp.Body.Close()
	out, err := io.ReadAll(io.LimitReader(resp.Body, 256<<10))
	if err != nil {
		return resp.StatusCode, nil, err
	}
	return resp.StatusCode, out, nil
}
