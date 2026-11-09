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

func TestMapBackupRestoreWritesTheFileKeepsTheCurrentMapAndReloads(t *testing.T) {
	store, _ := newBackupStore(t)
	require.NoError(t, os.WriteFile(store.areasFile, areasFile("good"), 0o644))
	good, _, err := store.Create()
	require.NoError(t, err)

	require.NoError(t, os.WriteFile(store.areasFile, areasFile("newer"), 0o644))
	reloaded := 0
	require.NoError(t, store.Restore(context.Background(), good.ID, func(context.Context) error {
		reloaded++
		return nil
	}))

	now, _ := os.ReadFile(store.areasFile)
	assert.Equal(t, areasFile("good"), now)
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
	noReload := func(context.Context) error { t.Fatal("must not reload"); return nil }

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

	err = store.Restore(context.Background(), good.ID, func(context.Context) error { return errors.New("service not available") })
	require.Error(t, err)
	assert.Contains(t, err.Error(), "did not reload")
	assert.Contains(t, err.Error(), "service not available")
}

func TestMapBackupRoutes(t *testing.T) {
	gin.SetMode(gin.TestMode)
	store, _ := newBackupStore(t)
	router := gin.New()
	registerMapBackupRoutes(router.Group("/api"), store, func(context.Context) error { return nil })
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
	require.NoError(t, store.Restore(context.Background(), good.ID, func(context.Context) error { return nil }))

	now, err := os.ReadFile(store.areasFile)
	require.NoError(t, err)
	assert.Contains(t, string(now), "area_0_mow_angle_deg: 35")
	assert.Contains(t, string(now), "area_0_ring_direction: 2")
	assert.Contains(t, string(now), "area_0_start_x: 9.500")
	assert.Contains(t, string(now), "area_0_start_y: 4.000")
}
