package providers

import (
	"context"
	"encoding/json"
	"net/http"
	"net/http/httptest"
	"strings"
	"sync/atomic"
	"testing"
	"time"

	"github.com/gorilla/websocket"
	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
	"github.com/vmihailenco/msgpack/v5"
)

const (
	identityModeOK = iota
	identityModeHang
	identityModeUnavailable
	identityModeMismatch
	identityModeIncompatible
)

type sessionResult struct {
	established bool
	err         error
}

// identityPeer is a fake peer GUI whose identity endpoint can be switched
// between a good answer, a hang (client timeout), HTTP 503, a different robot
// and an incompatible API, while its multiplex socket streams a pose.
type identityPeer struct {
	server   *httptest.Server
	mode     atomic.Int32
	hits     atomic.Int32
	socketUp chan struct{}
	closed   chan struct{}
	conn     atomic.Pointer[websocket.Conn]
}

func newIdentityPeer(t *testing.T) *identityPeer {
	t.Helper()
	peer := &identityPeer{socketUp: make(chan struct{}), closed: make(chan struct{})}
	mux := http.NewServeMux()
	mux.HandleFunc(peerIdentityPath, func(w http.ResponseWriter, r *http.Request) {
		peer.hits.Add(1)
		switch peer.mode.Load() {
		case identityModeHang:
			select {
			case <-r.Context().Done():
			case <-time.After(2 * time.Second):
			}
			return
		case identityModeUnavailable:
			w.WriteHeader(http.StatusServiceUnavailable)
			return
		case identityModeMismatch:
			_ = json.NewEncoder(w).Encode(RobotIdentity{ID: "beta", APIVersion: FleetAPIVersion})
			return
		case identityModeIncompatible:
			_ = json.NewEncoder(w).Encode(RobotIdentity{ID: "alpha", APIVersion: FleetAPIVersion + 1})
			return
		}
		_ = json.NewEncoder(w).Encode(RobotIdentity{ID: "alpha", APIVersion: FleetAPIVersion})
	})
	mux.HandleFunc(peerMultiplexPath, func(w http.ResponseWriter, r *http.Request) {
		conn, err := (&websocket.Upgrader{}).Upgrade(w, r, nil)
		if err != nil {
			return
		}
		peer.conn.Store(conn)
		defer close(peer.closed)
		defer conn.Close()
		for range FleetTopics {
			if _, _, err := conn.ReadMessage(); err != nil {
				return
			}
		}
		for _, topic := range []string{"highLevelStatus", "pose"} {
			frame, err := msgpack.Marshal(map[string]any{"topic": topic, "data": map[string]any{"marker": topic}})
			if err != nil {
				return
			}
			if err := conn.WriteMessage(websocket.BinaryMessage, frame); err != nil {
				return
			}
		}
		close(peer.socketUp)
		for {
			if _, _, err := conn.ReadMessage(); err != nil {
				return
			}
		}
	})
	peer.server = httptest.NewServer(mux)
	t.Cleanup(peer.server.Close)
	return peer
}

func startIdentitySession(t *testing.T, peer *identityPeer) (*peerClient, <-chan sessionResult) {
	t.Helper()
	p := newPeerClient(strings.TrimPrefix(peer.server.URL, "http://"), time.Now)
	p.expectedID = "alpha"
	p.cache.expectedID = "alpha"
	p.identityRefresh = 30 * time.Millisecond
	p.http = &http.Client{Timeout: 100 * time.Millisecond}
	t.Cleanup(p.cancel)
	done := make(chan sessionResult, 1)
	go func() {
		established, err := p.session()
		done <- sessionResult{established, err}
	}()
	select {
	case <-peer.socketUp:
	case <-time.After(3 * time.Second):
		t.Fatal("socket never came up")
	}
	require.Eventually(t, func() bool {
		topics, _, _, online := p.cache.snapshot(time.Now())
		_, hasPose := topics["pose"]
		return hasPose && online
	}, 3*time.Second, 5*time.Millisecond, "pose mirrored before the identity check fails")
	return p, done
}

func TestFleetPeerTransientIdentityFailureKeepsSocketAndPoses(t *testing.T) {
	for name, mode := range map[string]int32{"timeout": identityModeHang, "http-503": identityModeUnavailable} {
		t.Run(name, func(t *testing.T) {
			// Arrange
			peer := newIdentityPeer(t)
			var unavailable atomic.Int32
			p, done := startIdentitySession(t, peer)
			p.onIdentityUnavailable = func() { unavailable.Add(1) }

			// Act
			peer.mode.Store(mode)
			before := peer.hits.Load()
			require.Eventually(t, func() bool { return peer.hits.Load() >= before+3 }, 5*time.Second, 5*time.Millisecond)

			// Assert
			select {
			case res := <-done:
				t.Fatalf("a transient identity failure tore down the socket: %v", res.err)
			default:
			}
			_, topics, _, _, online := p.cache.snapshotForIdentity("alpha", time.Now())
			assert.Contains(t, topics, "pose", "cached pose survives")
			assert.True(t, online, "peer stays online")
			assert.Zero(t, unavailable.Load(), "registry is not told the peer vanished")
		})
	}
}

func TestFleetPeerDefinitiveIdentityMismatchClearsAndDisconnects(t *testing.T) {
	for name, mode := range map[string]int32{"other-robot": identityModeMismatch, "incompatible-api": identityModeIncompatible} {
		t.Run(name, func(t *testing.T) {
			// Arrange
			peer := newIdentityPeer(t)
			p, done := startIdentitySession(t, peer)

			// Act
			peer.mode.Store(mode)

			// Assert
			select {
			case res := <-done:
				assert.True(t, res.established)
			case <-time.After(3 * time.Second):
				t.Fatal("a definitive identity mismatch kept the socket")
			}
			_, topics, _, seen, online := p.cache.snapshotForIdentity("alpha", time.Now())
			assert.Empty(t, topics)
			assert.True(t, seen.IsZero())
			assert.False(t, online)
			select {
			case <-peer.closed:
			case <-time.After(3 * time.Second):
				t.Fatal("peer socket not closed")
			}
		})
	}
}

func TestFleetPeerRefreshIdentityTimeoutLeavesCacheUntouched(t *testing.T) {
	// Arrange
	peer := newIdentityPeer(t)
	peer.mode.Store(identityModeHang)
	p := newPeerClient(strings.TrimPrefix(peer.server.URL, "http://"), time.Now)
	defer p.cancel()
	p.http = &http.Client{Timeout: 50 * time.Millisecond}
	p.expectedID = "alpha"
	p.cache.expectedID = "alpha"
	p.cache.setIdentity(RobotIdentity{ID: "alpha", APIVersion: FleetAPIVersion})
	p.cache.setSocket(true)
	p.cache.store("pose", json.RawMessage(`{"x":1}`), time.Now())

	// Act
	err := p.refreshIdentity(context.Background())

	// Assert
	require.Error(t, err)
	assert.True(t, isTransientPeerError(err))
	_, topics, _, _, _ := p.cache.snapshotForIdentity("alpha", time.Now())
	assert.Contains(t, topics, "pose")
}

func TestNextPeerBackoffDoublesWhileFailingAndResetsAfterAGoodSession(t *testing.T) {
	assert.Equal(t, 2*peerReconnectMin, nextPeerBackoff(peerReconnectMin, false))
	assert.Equal(t, peerReconnectMax, nextPeerBackoff(peerReconnectMax*3/4, false), "capped")
	assert.Equal(t, peerReconnectMax, nextPeerBackoff(peerReconnectMax, false), "capped")
	assert.Equal(t, peerReconnectMin, nextPeerBackoff(peerReconnectMax, true), "a good session starts over")
}

func TestPeerSessionReportsWhetherItWasEstablished(t *testing.T) {
	// A subscribed session that later drops counts as established.
	peer := newIdentityPeer(t)
	_, done := startIdentitySession(t, peer)
	require.NoError(t, peer.conn.Load().Close())
	select {
	case res := <-done:
		assert.True(t, res.established)
	case <-time.After(3 * time.Second):
		t.Fatal("session did not end")
	}

	// A session refused at the identity check keeps backing off.
	failing := newIdentityPeer(t)
	failing.mode.Store(identityModeUnavailable)
	p := newPeerClient(strings.TrimPrefix(failing.server.URL, "http://"), time.Now)
	defer p.cancel()
	p.expectedID = "alpha"
	established, err := p.session()
	require.Error(t, err)
	assert.False(t, established)
}
