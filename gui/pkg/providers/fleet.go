package providers

import (
	"context"
	"encoding/json"
	"errors"
	"fmt"
	"net"
	"net/http"
	"sort"
	"strconv"
	"strings"
	"sync"
	"time"

	"github.com/mowglinext/mowglinext/pkg/msgs/mowgli"
	"github.com/mowglinext/mowglinext/pkg/types"
	"github.com/sirupsen/logrus"
)

// Fleet: this robot's view of the other mowers on the network. Every robot
// keeps its own peer list (DB key fleet.peers) and mirrors each peer's live
// state over the peer GUI's multiplex WebSocket, so any robot's GUI can act
// as the fleet console. See docs/MULTI_ROBOT.md.

const (
	fleetPeersKey       = "fleet.peers"
	fleetSelfSubscriber = "fleet:self"
)

// FleetCommands lists the only GUI service commands that may be sent to a
// peer (or to self) through the fleet routes. Everything else stays local.
var FleetCommands = map[string]bool{
	"high_level_control":    true,
	"emergency":             true,
	"coverage_clear_resume": true,
}

// FleetPeer is a persisted registry entry.
type FleetPeer struct {
	ID         string `json:"id"`
	Name       string `json:"name"`
	Address    string `json:"address"` // host:port of the peer's GUI backend
	APIVersion int    `json:"api_version"`
}

// FleetRobot is one row of the fleet snapshot the GUI renders.
type FleetRobot struct {
	Identity RobotIdentity              `json:"identity"`
	Self     bool                       `json:"self"`
	Address  string                     `json:"address,omitempty"`
	Online   bool                       `json:"online"`
	LastSeen *time.Time                 `json:"last_seen,omitempty"`
	Topics   map[string]json.RawMessage `json:"topics"`
}

// FleetProvider owns the peer registry and the per-peer mirrors.
type FleetProvider struct {
	db   types.IDBProvider
	ros  types.IRosProvider
	http *http.Client
	now  func() time.Time

	mu      sync.Mutex
	peers   map[string]FleetPeer
	clients map[string]*peerClient
	self    *peerCache
}

// NewFleetProvider loads the registry, starts a mirror per peer and starts
// mirroring the local robot's own topics.
func NewFleetProvider(db types.IDBProvider, ros types.IRosProvider) *FleetProvider {
	f := &FleetProvider{
		db:      db,
		ros:     ros,
		http:    &http.Client{Timeout: peerHTTPTimeout},
		now:     time.Now,
		peers:   map[string]FleetPeer{},
		clients: map[string]*peerClient{},
		self:    newPeerCache(),
	}
	if _, err := EnsureRobotID(db); err != nil {
		logrus.Errorf("fleet: cannot persist robot id: %v", err)
	}
	f.loadPeers()
	for _, p := range f.peers {
		f.startClient(p)
	}
	f.subscribeSelf()
	return f
}

func (f *FleetProvider) subscribeSelf() {
	f.self.setSocket(true)
	for _, topic := range FleetTopics {
		t := topic
		err := f.ros.Subscribe(t, fleetSelfSubscriber, 0, func(msg []byte) {
			f.self.store(t, json.RawMessage(append([]byte(nil), msg...)), f.now())
		})
		if err != nil {
			logrus.Warnf("fleet: subscribe self %s: %v", t, err)
		}
	}
}

// Close stops every peer mirror. The registry stays persisted.
func (f *FleetProvider) Close() {
	f.mu.Lock()
	clients := f.clients
	f.clients = map[string]*peerClient{}
	f.mu.Unlock()
	for _, c := range clients {
		c.Close()
	}
}

// Identity is this robot's own identity.
func (f *FleetProvider) Identity() (RobotIdentity, error) {
	return CurrentIdentity(f.db)
}

// Peers returns the registry sorted by name.
func (f *FleetProvider) Peers() []FleetPeer {
	f.mu.Lock()
	defer f.mu.Unlock()
	return f.sortedPeersLocked()
}

func (f *FleetProvider) sortedPeersLocked() []FleetPeer {
	out := make([]FleetPeer, 0, len(f.peers))
	for _, p := range f.peers {
		out = append(out, p)
	}
	sort.Slice(out, func(i, j int) bool {
		if out[i].Name != out[j].Name {
			return out[i].Name < out[j].Name
		}
		return out[i].ID < out[j].ID
	})
	return out
}

// AddPeerResult reports how a peer was added.
type AddPeerResult struct {
	Peer       FleetPeer `json:"peer"`
	Reciprocal bool      `json:"reciprocal"`
	Warning    string    `json:"warning,omitempty"`
}

// AddPeer dials the GUI at address, stores it, starts mirroring it and asks
// it to register us in return so the fleet is symmetric.
func (f *FleetProvider) AddPeer(ctx context.Context, address string) (AddPeerResult, error) {
	address, err := normalizePeerAddress(address)
	if err != nil {
		return AddPeerResult{}, err
	}
	self, err := f.Identity()
	if err != nil {
		return AddPeerResult{}, err
	}
	remote, err := fetchPeerIdentity(ctx, f.http, address)
	if err != nil {
		return AddPeerResult{}, fmt.Errorf("cannot reach a MowgliNext GUI at %s: %w", address, err)
	}
	if remote.ID == self.ID {
		return AddPeerResult{}, errors.New("that address is this robot")
	}
	if strings.EqualFold(remote.Name, self.Name) {
		return AddPeerResult{}, fmt.Errorf("peer is also named %q; give each mower a distinct name in Settings → Hardware", remote.Name)
	}
	peer := FleetPeer{ID: remote.ID, Name: remote.Name, Address: address, APIVersion: remote.APIVersion}
	if err := f.upsertPeer(peer); err != nil {
		return AddPeerResult{}, err
	}
	result := AddPeerResult{Peer: peer}
	status, body, err := postPeerJSON(ctx, f.http, address, peerRegisterPath, registerBody{
		ID: self.ID, Name: self.Name, Port: LocalAPIPort(f.db), APIVersion: self.APIVersion,
	})
	switch {
	case err != nil:
		result.Warning = "registered one-way: the peer could not be asked to add this robot (" + err.Error() + ")"
	case status != http.StatusOK:
		result.Warning = fmt.Sprintf("registered one-way: the peer refused the reverse registration (HTTP %d %s)", status, strings.TrimSpace(string(body)))
	default:
		result.Reciprocal = true
	}
	if remote.APIVersion != FleetAPIVersion {
		result.Warning = strings.TrimSpace(result.Warning + fmt.Sprintf(" peer fleet API v%d differs from ours (v%d); update both robots", remote.APIVersion, FleetAPIVersion))
	}
	return result, nil
}

type registerBody struct {
	ID         string `json:"id"`
	Name       string `json:"name"`
	Port       int    `json:"port"`
	APIVersion int    `json:"api_version"`
}

// RegisterPeer is the receiving side of AddPeer: a peer told us its id, name
// and API port, and the transport told us its IP.
func (f *FleetProvider) RegisterPeer(id, name, host string, port int, apiVersions ...int) (FleetPeer, error) {
	if id == "" || name == "" || host == "" || port <= 0 {
		return FleetPeer{}, errors.New("id, name, host and port are required")
	}
	apiVersion := 0
	if len(apiVersions) > 0 {
		apiVersion = apiVersions[0]
	}
	self, err := f.Identity()
	if err != nil {
		return FleetPeer{}, err
	}
	if id == self.ID {
		return FleetPeer{}, errors.New("a robot cannot register itself")
	}
	peer := FleetPeer{ID: id, Name: name, Address: net.JoinHostPort(host, strconv.Itoa(port)), APIVersion: apiVersion}
	if err := f.upsertPeer(peer); err != nil {
		return FleetPeer{}, err
	}
	return peer, nil
}

// RemovePeer forgets a peer and asks it (best effort) to forget us.
func (f *FleetProvider) RemovePeer(ctx context.Context, id string) error {
	f.mu.Lock()
	peer, ok := f.peers[id]
	f.mu.Unlock()
	if !ok {
		return fmt.Errorf("unknown peer %s", id)
	}
	f.UnregisterPeer(id)
	self, err := f.Identity()
	if err != nil {
		return nil
	}
	if _, _, err := postPeerJSON(ctx, f.http, peer.Address, peerUnregisterPath, map[string]string{"id": self.ID}); err != nil {
		logrus.WithField("peer", peer.Address).Warnf("fleet: peer not told about removal: %v", err)
	}
	return nil
}

// UnregisterPeer removes a peer locally without contacting it.
func (f *FleetProvider) UnregisterPeer(id string) {
	f.mu.Lock()
	client := f.clients[id]
	delete(f.clients, id)
	delete(f.peers, id)
	f.savePeersLocked()
	f.mu.Unlock()
	if client != nil {
		client.Close()
	}
}

func (f *FleetProvider) upsertPeer(peer FleetPeer) error {
	f.mu.Lock()
	defer f.mu.Unlock()
	if old, ok := f.peers[peer.ID]; ok && old.Address != peer.Address {
		if c := f.clients[peer.ID]; c != nil {
			delete(f.clients, peer.ID)
			go c.Close()
		}
	}
	f.peers[peer.ID] = peer
	if _, running := f.clients[peer.ID]; !running {
		f.startClientLocked(peer)
	}
	return f.savePeersLocked()
}

func (f *FleetProvider) startClient(peer FleetPeer) {
	f.mu.Lock()
	defer f.mu.Unlock()
	f.startClientLocked(peer)
}

func (f *FleetProvider) startClientLocked(peer FleetPeer) {
	c := newPeerClient(peer.Address, f.now)
	c.onIdentityUnavailable = func() {
		f.updatePeerAPIVersion(peer.ID, peer.Address, 0)
	}
	c.onIdentity = func(identity RobotIdentity) {
		if identity.ID != peer.ID {
			f.updatePeerAPIVersion(peer.ID, peer.Address, 0)
			logrus.WithField("peer", peer.Address).Warn("fleet: identity refresh returned a different robot id")
			return
		}
		f.updatePeerAPIVersion(peer.ID, peer.Address, identity.APIVersion)
	}
	f.clients[peer.ID] = c
	c.start()
}

// updatePeerAPIVersion persists the version reported by the same peer identity
// and address that owns this client. Reconnecting peers may have been upgraded
// since their registry entry was first written.
func (f *FleetProvider) updatePeerAPIVersion(id, address string, apiVersion int) {
	f.mu.Lock()
	defer f.mu.Unlock()
	peer, ok := f.peers[id]
	if !ok || peer.Address != address || peer.APIVersion == apiVersion {
		return
	}
	peer.APIVersion = apiVersion
	f.peers[id] = peer
	if err := f.savePeersLocked(); err != nil {
		logrus.WithField("peer", address).Warnf("fleet: persist refreshed API version: %v", err)
	}
}

func (f *FleetProvider) loadPeers() {
	raw, err := f.db.Get(fleetPeersKey)
	if err != nil || len(raw) == 0 {
		return
	}
	var list []FleetPeer
	if err := json.Unmarshal(raw, &list); err != nil {
		logrus.Warnf("fleet: ignoring unreadable %s: %v", fleetPeersKey, err)
		return
	}
	for _, p := range list {
		if p.ID == "" || p.Address == "" {
			continue
		}
		f.peers[p.ID] = p
	}
}

func (f *FleetProvider) savePeersLocked() error {
	buf, err := json.Marshal(f.sortedPeersLocked())
	if err != nil {
		return err
	}
	return f.db.Set(fleetPeersKey, buf)
}

// Robots is the fleet snapshot: self first, then peers by name.
func (f *FleetProvider) Robots() ([]FleetRobot, error) {
	self, err := f.Identity()
	if err != nil {
		return nil, err
	}
	now := f.now()
	topics, seen, _ := f.self.snapshot(now)
	rows := []FleetRobot{{
		Identity: self,
		Self:     true,
		Online:   true,
		LastSeen: timePtr(seen),
		Topics:   topics,
	}}
	f.mu.Lock()
	peers := f.sortedPeersLocked()
	clients := make(map[string]*peerClient, len(f.clients))
	for k, v := range f.clients {
		clients[k] = v
	}
	f.mu.Unlock()
	for _, p := range peers {
		row := FleetRobot{
			Identity: RobotIdentity{ID: p.ID, Name: p.Name, APIVersion: p.APIVersion},
			Address:  p.Address,
			Topics:   map[string]json.RawMessage{},
		}
		if c := clients[p.ID]; c != nil {
			row.Topics, row.LastSeen, row.Online = snapshotRow(c.cache, now)
		}
		rows = append(rows, row)
	}
	return rows, nil
}

func snapshotRow(c *peerCache, now time.Time) (map[string]json.RawMessage, *time.Time, bool) {
	topics, seen, online := c.snapshot(now)
	return topics, timePtr(seen), online
}

func timePtr(t time.Time) *time.Time {
	if t.IsZero() {
		return nil
	}
	return &t
}

// Call sends one of FleetCommands to the robot with the given id. Self is
// executed against the local ROS graph; a peer is proxied to its GUI.
func (f *FleetProvider) Call(ctx context.Context, id, command string, body json.RawMessage) (int, []byte, error) {
	if !FleetCommands[command] {
		return http.StatusBadRequest, nil, fmt.Errorf("command %q is not a fleet command", command)
	}
	self, err := f.Identity()
	if err != nil {
		return http.StatusInternalServerError, nil, err
	}
	if id == self.ID {
		if err := f.callLocal(ctx, command, body); err != nil {
			return http.StatusInternalServerError, nil, err
		}
		return http.StatusOK, []byte(`{}`), nil
	}
	f.mu.Lock()
	peer, ok := f.peers[id]
	f.mu.Unlock()
	if !ok {
		return http.StatusNotFound, nil, fmt.Errorf("unknown robot %s", id)
	}
	if peer.APIVersion != FleetAPIVersion {
		return http.StatusConflict, nil, fmt.Errorf("peer %s fleet API version %d is incompatible with ours (v%d)", peer.Name, peer.APIVersion, FleetAPIVersion)
	}
	var payload any = map[string]any{}
	if len(body) > 0 {
		payload = body
	}
	status, resp, err := postPeerJSON(ctx, f.http, peer.Address, peerCallPathPrefix+command, payload)
	if err != nil {
		return http.StatusBadGateway, nil, fmt.Errorf("peer %s unreachable: %w", peer.Name, err)
	}
	return status, resp, nil
}

func (f *FleetProvider) callLocal(ctx context.Context, command string, body json.RawMessage) error {
	ctx, cancel := context.WithTimeout(ctx, 10*time.Second)
	defer cancel()
	switch command {
	case "high_level_control":
		var req mowgli.HighLevelControlReq
		if err := json.Unmarshal(body, &req); err != nil {
			return err
		}
		return f.ros.CallService(ctx, "/behavior_tree_node/high_level_control", &req, &mowgli.HighLevelControlRes{}, "mowgli_interfaces/srv/HighLevelControl")
	case "emergency":
		var req mowgli.EmergencyStopReq
		if err := json.Unmarshal(body, &req); err != nil {
			return err
		}
		return f.ros.CallService(ctx, "/hardware_bridge/emergency_stop", &req, &mowgli.EmergencyStopRes{}, "mowgli_interfaces/srv/EmergencyStop")
	case "coverage_clear_resume":
		var res struct {
			Success bool   `json:"success"`
			Message string `json:"message"`
		}
		if err := f.ros.CallService(ctx, "/behavior_tree_node/clear_coverage_resume", &struct{}{}, &res, "std_srvs/srv/Trigger"); err != nil {
			return err
		}
		if !res.Success {
			return errors.New(res.Message)
		}
		return nil
	default:
		return fmt.Errorf("command %q is not a fleet command", command)
	}
}

// normalizePeerAddress accepts "host", "host:port", "http://host:port/" and
// returns host:port (default port 4006).
func normalizePeerAddress(raw string) (string, error) {
	s := strings.TrimSpace(raw)
	s = strings.TrimPrefix(strings.TrimPrefix(s, "http://"), "https://")
	if i := strings.Index(s, "/"); i >= 0 {
		s = s[:i]
	}
	if s == "" {
		return "", errors.New("address is required")
	}
	host, port, err := net.SplitHostPort(s)
	if err != nil {
		host, port = s, strconv.Itoa(defaultAPIPort)
	}
	if host == "" {
		return "", errors.New("address needs a host")
	}
	if _, err := strconv.Atoi(port); err != nil {
		return "", fmt.Errorf("invalid port %q", port)
	}
	return net.JoinHostPort(host, port), nil
}
