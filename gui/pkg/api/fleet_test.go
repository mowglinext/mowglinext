package api

import (
	"bytes"
	"encoding/json"
	"io"
	"net"
	"net/http"
	"net/http/httptest"
	"os"
	"path/filepath"
	"testing"
	"time"

	"github.com/gin-gonic/gin"
	"github.com/mowglinext/mowglinext/pkg/providers"
	"github.com/mowglinext/mowglinext/pkg/types"
	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
)

// fleetRobot is one in-process robot: its own DB, ROS mock, fleet provider
// and a real HTTP server, so two of them can talk exactly like two Pis.
type fleetRobot struct {
	name  string
	db    *types.MockDBProvider
	ros   *types.MockRosProvider
	fleet *providers.FleetProvider
	srv   *httptest.Server
	addr  string
	id    string
}

func newFleetRobot(t *testing.T, name string) *fleetRobot {
	t.Helper()
	gin.SetMode(gin.TestMode)
	db := types.NewMockDBProvider()
	yamlPath := filepath.Join(t.TempDir(), "mowgli_robot.yaml")
	require.NoError(t, os.WriteFile(yamlPath, []byte("mowgli:\n  ros__parameters:\n    robot_name: "+name+"\n    datum_lat: 48.0\n    datum_lon: 2.0\n"), 0o644))
	require.NoError(t, db.Set("system.mower.yamlConfigFile", []byte(yamlPath)))
	ros := types.NewMockRosProvider()
	fleet := providers.NewFleetProvider(db, ros)

	coord := providers.NewFleetCoordinator(db, ros, fleet)
	r := gin.New()
	group := r.Group("/api")
	MowgliNextRoutes(group, ros)
	FleetRoutes(group, fleet, coord)
	srv := httptest.NewServer(r)
	_, port, err := net.SplitHostPort(srv.Listener.Addr().String())
	require.NoError(t, err)
	require.NoError(t, db.Set("system.api.addr", []byte(":"+port)))

	id, err := fleet.Identity()
	require.NoError(t, err)
	rb := &fleetRobot{name: name, db: db, ros: ros, fleet: fleet, srv: srv, addr: srv.Listener.Addr().String(), id: id.ID}
	t.Cleanup(func() {
		coord.Close()
		fleet.Close()
		srv.Close()
	})
	return rb
}

func (rb *fleetRobot) post(t *testing.T, path string, body any) (*http.Response, []byte) {
	t.Helper()
	return httpJSON(t, http.MethodPost, rb.srv.URL+path, body)
}

func (rb *fleetRobot) get(t *testing.T, path string, out any) {
	t.Helper()
	resp, body := httpJSON(t, http.MethodGet, rb.srv.URL+path, nil)
	require.Equal(t, http.StatusOK, resp.StatusCode, string(body))
	require.NoError(t, json.Unmarshal(body, out))
}

func TestFleet_AddPeerIsSymmetric(t *testing.T) {
	a := newFleetRobot(t, "alpha")
	b := newFleetRobot(t, "bravo")

	resp, body := a.post(t, "/api/fleet/peers", map[string]string{"address": b.addr})
	require.Equal(t, http.StatusOK, resp.StatusCode, string(body))
	var res providers.AddPeerResult
	require.NoError(t, json.Unmarshal(body, &res))
	assert.True(t, res.Reciprocal, res.Warning)
	assert.Equal(t, "bravo", res.Peer.Name)

	var aPeers, bPeers []providers.FleetPeer
	a.get(t, "/api/fleet/peers", &aPeers)
	b.get(t, "/api/fleet/peers", &bPeers)
	require.Len(t, aPeers, 1)
	require.Len(t, bPeers, 1)
	assert.Equal(t, b.id, aPeers[0].ID)
	assert.Equal(t, a.id, bPeers[0].ID)
	assert.Equal(t, a.addr, bPeers[0].Address, "peer stored our source IP + API port")

	raw, err := a.db.Get("fleet.peers")
	require.NoError(t, err)
	assert.Contains(t, string(raw), b.addr, "registry persisted")
}

func TestFleet_RefusesSelfAndDuplicateName(t *testing.T) {
	a := newFleetRobot(t, "alpha")
	a2 := newFleetRobot(t, "alpha")

	resp, body := a.post(t, "/api/fleet/peers", map[string]string{"address": a.addr})
	assert.Equal(t, http.StatusBadRequest, resp.StatusCode)
	assert.Contains(t, string(body), "this robot")

	resp, body = a.post(t, "/api/fleet/peers", map[string]string{"address": a2.addr})
	assert.Equal(t, http.StatusBadRequest, resp.StatusCode)
	assert.Contains(t, string(body), "distinct name")

	resp, _ = a.post(t, "/api/fleet/peers", map[string]string{"address": "127.0.0.1:1"})
	assert.Equal(t, http.StatusBadRequest, resp.StatusCode, "unreachable address")
}

func TestFleet_SnapshotMirrorsPeerTopicsAndLiveness(t *testing.T) {
	a := newFleetRobot(t, "alpha")
	b := newFleetRobot(t, "bravo")
	resp, body := a.post(t, "/api/fleet/peers", map[string]string{"address": b.addr})
	require.Equal(t, http.StatusOK, resp.StatusCode, string(body))

	// The peer mirror subscribes over the multiplex socket; the mock only
	// delivers on Dispatch, so keep dispatching until the mirror is up.
	require.Eventually(t, func() bool {
		b.ros.Dispatch("highLevelStatus", []byte(`{"state":2,"state_name":"MOWING","current_area":1}`))
		b.ros.Dispatch("power", []byte(`{"v_battery":27.5}`))
		var rows []providers.FleetRobot
		a.get(t, "/api/fleet/robots", &rows)
		if len(rows) != 2 || !rows[1].Online {
			return false
		}
		return string(rows[1].Topics["power"]) != ""
	}, 10*time.Second, 100*time.Millisecond)

	var rows []providers.FleetRobot
	a.get(t, "/api/fleet/robots", &rows)
	assert.True(t, rows[0].Self)
	assert.Equal(t, "alpha", rows[0].Identity.Name)
	peer := rows[1]
	assert.False(t, peer.Self)
	assert.Equal(t, "bravo", peer.Identity.Name)
	assert.Equal(t, b.addr, peer.Address)
	assert.JSONEq(t, `{"state":2,"state_name":"MOWING","current_area":1}`, string(peer.Topics["highLevelStatus"]))
	assert.NotNil(t, peer.LastSeen)
}

func TestFleet_CommandsProxyToPeerAndRunLocallyForSelf(t *testing.T) {
	a := newFleetRobot(t, "alpha")
	b := newFleetRobot(t, "bravo")
	resp, body := a.post(t, "/api/fleet/peers", map[string]string{"address": b.addr})
	require.Equal(t, http.StatusOK, resp.StatusCode, string(body))

	resp, body = a.post(t, "/api/fleet/robots/"+b.id+"/call/high_level_control", map[string]any{"Command": 2})
	require.Equal(t, http.StatusOK, resp.StatusCode, string(body))
	require.Len(t, b.ros.ServiceCalls, 1, "peer's BT got the command")
	assert.Equal(t, "/behavior_tree_node/high_level_control", b.ros.ServiceCalls[0].Service)
	assert.Empty(t, a.ros.ServiceCalls, "our own BT was not touched")

	resp, body = a.post(t, "/api/fleet/robots/"+a.id+"/call/high_level_control", map[string]any{"Command": 1})
	require.Equal(t, http.StatusOK, resp.StatusCode, string(body))
	require.Len(t, a.ros.ServiceCalls, 1)

	resp, _ = a.post(t, "/api/fleet/robots/"+b.id+"/call/mow_enabled", map[string]any{"MowEnabled": 1})
	assert.Equal(t, http.StatusBadRequest, resp.StatusCode, "only fleet commands are proxied")

	resp, _ = a.post(t, "/api/fleet/robots/nobody/call/high_level_control", map[string]any{"Command": 1})
	assert.Equal(t, http.StatusNotFound, resp.StatusCode)
}

func TestFleet_CommandToOfflinePeerIs502(t *testing.T) {
	a := newFleetRobot(t, "alpha")
	b := newFleetRobot(t, "bravo")
	resp, body := a.post(t, "/api/fleet/peers", map[string]string{"address": b.addr})
	require.Equal(t, http.StatusOK, resp.StatusCode, string(body))
	b.srv.Close()

	resp, body = a.post(t, "/api/fleet/robots/"+b.id+"/call/high_level_control", map[string]any{"Command": 1})
	assert.Equal(t, http.StatusBadGateway, resp.StatusCode, string(body))
	assert.Contains(t, string(body), "unreachable")
}

func TestFleet_RemovePeerForgetsBothSides(t *testing.T) {
	a := newFleetRobot(t, "alpha")
	b := newFleetRobot(t, "bravo")
	resp, body := a.post(t, "/api/fleet/peers", map[string]string{"address": b.addr})
	require.Equal(t, http.StatusOK, resp.StatusCode, string(body))

	req, err := http.NewRequest(http.MethodDelete, a.srv.URL+"/api/fleet/peers/"+b.id, nil)
	require.NoError(t, err)
	del, err := http.DefaultClient.Do(req)
	require.NoError(t, err)
	del.Body.Close()
	require.Equal(t, http.StatusOK, del.StatusCode)

	var aPeers, bPeers []providers.FleetPeer
	a.get(t, "/api/fleet/peers", &aPeers)
	b.get(t, "/api/fleet/peers", &bPeers)
	assert.Empty(t, aPeers)
	assert.Empty(t, bPeers, "the peer was told to forget us")

	req, _ = http.NewRequest(http.MethodDelete, a.srv.URL+"/api/fleet/peers/"+b.id, nil)
	del, err = http.DefaultClient.Do(req)
	require.NoError(t, err)
	del.Body.Close()
	assert.Equal(t, http.StatusNotFound, del.StatusCode)
}

func TestFleet_RegisterRejectsBadInput(t *testing.T) {
	a := newFleetRobot(t, "alpha")

	resp, _ := a.post(t, "/api/fleet/peers/register", map[string]any{"id": "", "name": "x", "port": 4006})
	assert.Equal(t, http.StatusBadRequest, resp.StatusCode)

	resp, _ = a.post(t, "/api/fleet/peers/register", map[string]any{"id": a.id, "name": "alpha", "port": 4006})
	assert.Equal(t, http.StatusBadRequest, resp.StatusCode, "cannot register itself")

	resp, _ = a.post(t, "/api/fleet/peers/unregister", map[string]any{"id": ""})
	assert.Equal(t, http.StatusBadRequest, resp.StatusCode)
}

func TestFleet_IdentityRoute(t *testing.T) {
	a := newFleetRobot(t, "alpha")

	var id providers.RobotIdentity
	a.get(t, "/api/fleet/identity", &id)
	assert.Equal(t, "alpha", id.Name)
	assert.Equal(t, a.id, id.ID)
	assert.Equal(t, providers.FleetAPIVersion, id.APIVersion)
	assert.Equal(t, 48.0, id.DatumLat)
}

// httpJSON performs a real HTTP request against a test server and returns the
// response plus its body.
func httpJSON(t *testing.T, method, url string, body any) (*http.Response, []byte) {
	t.Helper()
	var reader io.Reader
	if body != nil {
		buf, err := json.Marshal(body)
		require.NoError(t, err)
		reader = bytes.NewReader(buf)
	}
	req, err := http.NewRequest(method, url, reader)
	require.NoError(t, err)
	req.Header.Set("Content-Type", "application/json")
	resp, err := http.DefaultClient.Do(req)
	require.NoError(t, err)
	defer resp.Body.Close()
	out, err := io.ReadAll(resp.Body)
	require.NoError(t, err)
	return resp, out
}
