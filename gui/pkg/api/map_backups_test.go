package api

import (
	"context"
	"errors"
	"fmt"
	"net/http"
	"net/http/httptest"
	"os"
	"path/filepath"
	"testing"
	"time"

	"github.com/gin-gonic/gin"
	"github.com/mowglinext/mowglinext/pkg/msgs/mowgli"
	"github.com/mowglinext/mowglinext/pkg/types"
	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
)

// areasFile builds a minimal areas.dat with the given area names (and one obstacle
// and one ignore line on the first area, to exercise the summary).
func areasFile(names ...string) []byte {
	s := fmt.Sprintf("# Mowgli\narea_count: %d\nnext_area_id: %d\n", len(names), len(names)+1)
	for i, n := range names {
		s += fmt.Sprintf("area_%d_name: %s\narea_%d_polygon: 0,0;1,0;1,1\n", i, n, i)
		if i == 0 {
			s += fmt.Sprintf("area_%d_obstacle_count: 1\n", i)
		}
	}
	s += "lidar_corridor_count: 1\nlidar_corridor_0_name: hedge\n"
	return []byte(s)
}

type tickClock struct{ t time.Time }

func (c *tickClock) now() time.Time { c.t = c.t.Add(time.Second); return c.t }

func newBackupStore(t *testing.T) (*mapBackupStore, string) {
	t.Helper()
	dir := t.TempDir()
	clock := &tickClock{t: time.Date(2026, 10, 4, 12, 0, 0, 0, time.UTC)}
	return &mapBackupStore{
		areasFile: filepath.Join(dir, "areas.dat"),
		dir:       filepath.Join(dir, "backups"),
		keep:      3,
		now:       clock.now,
	}, dir
}

func TestMapBackupCreateSummarisesAndKeepsTheRawFile(t *testing.T) {
	store, _ := newBackupStore(t)
	require.NoError(t, os.WriteFile(store.areasFile, areasFile("Voor", "Achter"), 0o644))

	info, skipped, err := store.Create()
	require.NoError(t, err)
	require.False(t, skipped)
	assert.Equal(t, 2, info.Areas)
	assert.Equal(t, 1, info.Obstacles)
	assert.Equal(t, 1, info.IgnoreLines)
	assert.Equal(t, []string{"Voor", "Achter"}, info.AreaNames)
	assert.False(t, info.CreatedAt.IsZero())

	stored, err := os.ReadFile(filepath.Join(store.dir, info.ID))
	require.NoError(t, err)
	assert.Equal(t, areasFile("Voor", "Achter"), stored, "the backup is the raw areas.dat")
}

func TestMapBackupSkipsAMissingOrEmptyMapSoItNeverPushesAGoodOneOut(t *testing.T) {
	store, _ := newBackupStore(t)

	_, skipped, err := store.Create()
	require.NoError(t, err)
	assert.True(t, skipped, "no areas.dat yet")

	require.NoError(t, os.WriteFile(store.areasFile, areasFile("Voor"), 0o644))
	_, _, err = store.Create()
	require.NoError(t, err)

	require.NoError(t, os.WriteFile(store.areasFile, areasFile(), 0o644)) // only the ignore line is left
	_, skipped, err = store.Create()
	require.NoError(t, err)
	assert.True(t, skipped, "a map without areas is not worth a backup slot")

	list, err := store.List()
	require.NoError(t, err)
	assert.Len(t, list, 1)
}

func TestMapBackupDoesNotStoreTheSameMapTwice(t *testing.T) {
	store, _ := newBackupStore(t)
	require.NoError(t, os.WriteFile(store.areasFile, areasFile("Voor"), 0o644))

	first, _, err := store.Create()
	require.NoError(t, err)
	second, _, err := store.Create()
	require.NoError(t, err)

	assert.Equal(t, first.ID, second.ID)
	list, _ := store.List()
	assert.Len(t, list, 1, "opening the editor twice without a change must not use a second slot")
}

func TestMapBackupKeepsOnlyTheNewestAndListsNewestFirst(t *testing.T) {
	store, _ := newBackupStore(t) // keep = 3
	var ids []string
	for i := 0; i < 5; i++ {
		require.NoError(t, os.WriteFile(store.areasFile, areasFile(fmt.Sprintf("v%d", i)), 0o644))
		info, _, err := store.Create()
		require.NoError(t, err)
		ids = append(ids, info.ID)
	}

	list, err := store.List()
	require.NoError(t, err)
	require.Len(t, list, 3, "older backups are deleted when a new one is made")
	assert.Equal(t, []string{ids[4], ids[3], ids[2]}, []string{list[0].ID, list[1].ID, list[2].ID})
	assert.Equal(t, []string{"v4"}, list[0].AreaNames)
	for _, gone := range ids[:2] {
		_, statErr := os.Stat(filepath.Join(store.dir, gone))
		assert.True(t, errors.Is(statErr, os.ErrNotExist), "%s should have been pruned", gone)
	}
}

func TestMapBackupSameMillisecondStillGetsADistinctSortedID(t *testing.T) {
	store, _ := newBackupStore(t)
	frozen := time.Date(2026, 10, 4, 12, 0, 0, 0, time.UTC)
	store.now = func() time.Time { return frozen }

	require.NoError(t, os.WriteFile(store.areasFile, areasFile("a"), 0o644))
	a, _, err := store.Create()
	require.NoError(t, err)
	require.NoError(t, os.WriteFile(store.areasFile, areasFile("b"), 0o644))
	b, _, err := store.Create()
	require.NoError(t, err)

	assert.NotEqual(t, a.ID, b.ID)
	list, _ := store.List()
	assert.Equal(t, b.ID, list[0].ID, "the later backup lists first")
}

func TestMapBackupRestoreForwardsBytesKeepsUndoAndNeverWritesLiveFile(t *testing.T) {
	store, _ := newBackupStore(t)
	require.NoError(t, os.WriteFile(store.areasFile, areasFile("good"), 0o644))
	good, _, err := store.Create()
	require.NoError(t, err)

	require.NoError(t, os.WriteFile(store.areasFile, areasFile("newer"), 0o644))
	reloaded := 0
	require.NoError(t, store.Restore(context.Background(), good.ID, func(_ context.Context, content []byte) (mowgli.RestoreAreasRes, error) {
		assert.Equal(t, areasFile("good"), content)
		live, err := os.ReadFile(store.areasFile)
		require.NoError(t, err)
		assert.Equal(t, areasFile("newer"), live)
		reloaded++
		return restoredOK(), nil
	}))

	now, _ := os.ReadFile(store.areasFile)
	assert.Equal(t, areasFile("newer"), now)
	assert.Equal(t, 1, reloaded, "map_server is told to reload")
	list, _ := store.List()
	var names [][]string
	for _, b := range list {
		names = append(names, b.AreaNames)
	}
	assert.Contains(t, names, []string{"newer"}, "the map that was replaced stays available, so a restore can be undone")
}

func TestMapBackupRestoreRefusesBadIDsAndEmptyBackups(t *testing.T) {
	store, _ := newBackupStore(t)
	require.NoError(t, os.WriteFile(store.areasFile, areasFile("keep"), 0o644))
	noReload := func(context.Context, []byte) (mowgli.RestoreAreasRes, error) {
		t.Fatal("must not call restore")
		return mowgli.RestoreAreasRes{}, nil
	}

	for _, id := range []string{"../areas.dat", "areas-1.dat", "", "areas-20261004T120000.000Z.dat/../x"} {
		assert.Error(t, store.Restore(context.Background(), id, noReload), "id %q", id)
	}

	require.NoError(t, os.MkdirAll(store.dir, 0o755))
	emptyID := "areas-20261004T120000.000Z.dat"
	require.NoError(t, os.WriteFile(filepath.Join(store.dir, emptyID), areasFile(), 0o644))
	assert.Error(t, store.Restore(context.Background(), emptyID, noReload), "a backup without areas is never restored")

	now, _ := os.ReadFile(store.areasFile)
	assert.Equal(t, areasFile("keep"), now, "areas.dat is untouched by a refused restore")
}

func TestMapBackupRestoreReportsAFailedReload(t *testing.T) {
	store, _ := newBackupStore(t)
	require.NoError(t, os.WriteFile(store.areasFile, areasFile("good"), 0o644))
	good, _, err := store.Create()
	require.NoError(t, err)

	err = store.Restore(context.Background(), good.ID, func(context.Context, []byte) (mowgli.RestoreAreasRes, error) {
		return mowgli.RestoreAreasRes{}, errors.New("service not available")
	})
	require.Error(t, err)
	assert.Contains(t, err.Error(), "Restore outcome could not be confirmed")
	assert.NotContains(t, err.Error(), "unchanged")
	assert.Contains(t, err.Error(), "service not available")
}

func TestMapBackupRoutes(t *testing.T) {
	gin.SetMode(gin.TestMode)
	store, _ := newBackupStore(t)
	router := gin.New()
	registerMapBackupRoutes(router.Group("/api"), store, func(context.Context, []byte) (mowgli.RestoreAreasRes, error) { return restoredOK(), nil })
	do := func(method, path string) *httptest.ResponseRecorder {
		w := httptest.NewRecorder()
		router.ServeHTTP(w, httptest.NewRequest(method, path, nil))
		return w
	}

	require.NoError(t, os.WriteFile(store.areasFile, areasFile("Voor"), 0o644))
	assert.Equal(t, http.StatusOK, do(http.MethodPost, "/api/map-backups").Code)

	list := do(http.MethodGet, "/api/map-backups")
	require.Equal(t, http.StatusOK, list.Code)
	assert.Contains(t, list.Body.String(), `"keep":3`)
	assert.Contains(t, list.Body.String(), `"area_names":["Voor"]`)

	assert.Equal(t, http.StatusBadRequest, do(http.MethodPost, "/api/map-backups/evil.dat/restore").Code)
	assert.NotEqual(t, http.StatusOK, do(http.MethodPost, "/api/map-backups/..%2Fareas.dat/restore").Code, "a path-like id never reaches the file system")
	assert.Equal(t, http.StatusInternalServerError, do(http.MethodPost, "/api/map-backups/areas-20261004T120000.000Z.dat/restore").Code,
		"a well-formed id that does not exist is a server-side failure, not a path escape")
}

// An area's own mowing lines (angle, winding, start point) are optional keys in areas.dat. The
// backup is the raw file, so they ride along; the summary must not mistake them for areas,
// names or obstacles.
func areasFileWithLines() []byte {
	return []byte("# Mowgli\narea_count: 2\nnext_area_id: 3\n" +
		"area_0_name: Voor\narea_0_polygon: 0,0;10,0;10,10\narea_0_is_navigation: 0\narea_0_id: 1\n" +
		"area_0_mow_angle_deg: 35\narea_0_ring_direction: 2\narea_0_start_x: 9.500\narea_0_start_y: 4.000\n" +
		"area_0_obstacle_count: 1\narea_0_obstacle_0: 2,2;3,2;3,3\n" +
		"area_1_name: Achter\narea_1_polygon: 20,0;30,0;30,10\narea_1_is_navigation: 0\narea_1_id: 2\n" +
		"area_1_obstacle_count: 0\n" +
		"lidar_corridor_count: 0\n")
}

func TestMapBackupKeepsAnAreasOwnMowingLines(t *testing.T) {
	store, _ := newBackupStore(t)
	require.NoError(t, os.WriteFile(store.areasFile, areasFileWithLines(), 0o644))

	info, skipped, err := store.Create()
	require.NoError(t, err)
	require.False(t, skipped)

	// The new keys are not areas, names or obstacles.
	assert.Equal(t, 2, info.Areas)
	assert.Equal(t, 1, info.Obstacles)
	assert.Equal(t, []string{"Voor", "Achter"}, info.AreaNames)

	stored, err := os.ReadFile(filepath.Join(store.dir, info.ID))
	require.NoError(t, err)
	assert.Equal(t, areasFileWithLines(), stored, "the lines are in the backup, byte for byte")
}

func TestMapBackupRestoreBringsTheMowingLinesBack(t *testing.T) {
	store, _ := newBackupStore(t)
	require.NoError(t, os.WriteFile(store.areasFile, areasFileWithLines(), 0o644))
	good, _, err := store.Create()
	require.NoError(t, err)

	// The lines are removed from the live map afterwards (a map save that dropped them).
	require.NoError(t, os.WriteFile(store.areasFile, areasFile("Voor", "Achter"), 0o644))
	var forwarded []byte
	require.NoError(t, store.Restore(context.Background(), good.ID, func(_ context.Context, content []byte) (mowgli.RestoreAreasRes, error) {
		forwarded = content
		return restoredOK(), nil
	}))
	assert.Equal(t, areasFileWithLines(), forwarded)
	live, err := os.ReadFile(store.areasFile)
	require.NoError(t, err)
	assert.Equal(t, areasFile("Voor", "Achter"), live)
}

func restoredOK() mowgli.RestoreAreasRes {
	return mowgli.RestoreAreasRes{Success: true, Committed: true, Persisted: true, Published: true, Outcome: "COMMITTED"}
}

func TestMapBackupRestoreHTTPOutcomesHaveNoLegacyFallbackOrRollback(t *testing.T) {
	for _, tc := range []struct {
		name      string
		response  mowgli.RestoreAreasRes
		transport error
		status    int
		message   string
	}{
		{"success", restoredOK(), nil, 200, ""},
		{"mission", mowgli.RestoreAreasRes{Outcome: "REFUSED_MISSION"}, nil, 409, "Map backup not restored: a mission is running. Press Stop, restore the backup, then start or resume. Nothing was changed."},
		{"invalid", mowgli.RestoreAreasRes{Outcome: "INVALID_MAP"}, nil, 422, "invalid content"},
		{"write failure", mowgli.RestoreAreasRes{Outcome: "PERSISTENCE_FAILED_BEFORE_COMMIT"}, nil, 500, "did not commit"},
		{"directory sync", mowgli.RestoreAreasRes{Committed: true, Published: true, Outcome: "COMMITTED_DURABILITY_UNCONFIRMED"}, nil, 500, "restore committed"},
		{"publication", mowgli.RestoreAreasRes{Committed: true, Persisted: true, Outcome: "COMMITTED_PUBLICATION_PENDING"}, nil, 500, "restore committed"},
		{"unavailable", mowgli.RestoreAreasRes{}, errors.New("service not advertised"), 503, "Restore outcome could not be confirmed"},
		{"timeout after commit", mowgli.RestoreAreasRes{}, context.DeadlineExceeded, 503, "Restore outcome could not be confirmed"},
		{"unknown", mowgli.RestoreAreasRes{Outcome: "NEW_OUTCOME"}, nil, 503, "Restore outcome could not be confirmed"},
		{"inconsistent", mowgli.RestoreAreasRes{Outcome: "COMMITTED", Success: true}, nil, 503, "Restore outcome could not be confirmed"},
	} {
		t.Run(tc.name, func(t *testing.T) {
			store, _ := newBackupStore(t)
			backup := append([]byte("# exact bytes and backup datum\r\ndatum_lat: 48.123456789\r\ndatum_lon: 2.0\r\n"), areasFileWithLines()...)
			require.NoError(t, os.WriteFile(store.areasFile, backup, 0644))
			info, _, err := store.Create()
			require.NoError(t, err)
			live := areasFile("keep-current")
			require.NoError(t, os.WriteFile(store.areasFile, live, 0444))
			provider := types.NewMockRosProvider()
			provider.ServiceErr = tc.transport
			provider.ServiceResponder = func(service string, req any, res any) {
				assert.Equal(t, mapServiceRestoreAreas, service)
				assert.Equal(t, string(backup), req.(*mowgli.RestoreAreasReq).AreasDat)
				*res.(*mowgli.RestoreAreasRes) = tc.response
			}
			router := gin.New()
			registerMapBackupRoutes(router.Group("/api"), store, restoreAreasViaProvider(provider))
			w := httptest.NewRecorder()
			router.ServeHTTP(w, httptest.NewRequest("POST", "/api/map-backups/"+info.ID+"/restore", nil))
			assert.Equal(t, tc.status, w.Code, w.Body.String())
			assert.Contains(t, w.Body.String(), tc.message)
			if tc.status != 409 {
				assert.NotContains(t, w.Body.String(), "Nothing was changed")
				assert.NotContains(t, w.Body.String(), "unchanged")
			}
			require.Len(t, provider.ServiceCalls, 1)
			assert.Equal(t, mapServiceRestoreAreas, provider.ServiceCalls[0].Service)
			after, err := os.ReadFile(store.areasFile)
			require.NoError(t, err)
			assert.Equal(t, live, after, "GUI never writes live areas.dat or rolls it back")
		})
	}
}

func TestMapBackupLostResponseAfterCommitNeverRollsTheFileBack(t *testing.T) {
	store, _ := newBackupStore(t)
	backup := areasFileWithLines()
	require.NoError(t, os.WriteFile(store.areasFile, backup, 0o644))
	info, _, err := store.Create()
	require.NoError(t, err)
	require.NoError(t, os.WriteFile(store.areasFile, areasFile("current"), 0o644))
	router := gin.New()
	registerMapBackupRoutes(router.Group("/api"), store, func(_ context.Context, content []byte) (mowgli.RestoreAreasRes, error) {
		// Stand in for the server committing successfully before the response is lost.
		require.Equal(t, backup, content)
		require.NoError(t, os.WriteFile(store.areasFile, content, 0o644))
		return mowgli.RestoreAreasRes{}, context.DeadlineExceeded
	})
	w := httptest.NewRecorder()
	router.ServeHTTP(w, httptest.NewRequest("POST", "/api/map-backups/"+info.ID+"/restore", nil))
	require.Equal(t, http.StatusServiceUnavailable, w.Code)
	assert.Contains(t, w.Body.String(), "Restore outcome could not be confirmed")
	assert.NotContains(t, w.Body.String(), "unchanged")
	assert.NotContains(t, w.Body.String(), "Nothing was changed")
	after, err := os.ReadFile(store.areasFile)
	require.NoError(t, err)
	assert.Equal(t, backup, after, "GUI must leave the server's commit intact")
}
