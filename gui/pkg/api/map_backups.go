package api

import (
	"bytes"
	"context"
	"errors"
	"fmt"
	"net/http"
	"os"
	"path/filepath"
	"regexp"
	"sort"
	"strconv"
	"strings"
	"time"

	"github.com/gin-gonic/gin"
	"github.com/mowglinext/mowglinext/pkg/types"
)

// Map backups: a copy of areas.dat taken every time the operator opens the map
// editor, kept for the last mapBackupKeep edits.
//
// Why: areas.dat is the only copy of the operator's map and any implicit save in
// map_server writes whatever is in memory. A map save that stops half way (or a
// failed load) used to be able to leave only the LiDAR-ignore lines on disk, with no
// way back but a hand-made map.json. The copy is taken BEFORE the editor opens, so
// every edit has a known-good map behind it.
//
// The copy is the raw areas.dat, not the Map message: it carries everything the file
// does (areas and their stable ids, obstacles with name/source, ignore lines, datum
// stamp). Restoring writes it back and asks map_server to reload it.
const (
	mapBackupAreasFile = "/ros2_ws/maps/areas.dat"
	mapBackupDir       = "/ros2_ws/maps/backups"
	// MapBackupKeep is how many backups survive; older ones are deleted when a new one is made.
	MapBackupKeep = 20

	mapBackupPrefix = "areas-"
	mapBackupSuffix = ".dat"

	mapServiceLoadAreas = "/map_server_node/load_areas"
)

// mapBackupIDPattern is the only shape of id the API accepts, so an id can never
// name a file outside the backup directory.
var mapBackupIDPattern = regexp.MustCompile(`^areas-[0-9]{8}T[0-9]{6}\.[0-9]{3}Z\.dat$`)

// MapBackup describes one stored copy.
type MapBackup struct {
	ID          string    `json:"id"`
	CreatedAt   time.Time `json:"created_at"`
	SizeBytes   int64     `json:"size_bytes"`
	Areas       int       `json:"areas"`
	Obstacles   int       `json:"obstacles"`
	IgnoreLines int       `json:"ignore_lines"`
	AreaNames   []string  `json:"area_names"`
}

type mapBackupStore struct {
	areasFile string
	dir       string
	keep      int
	now       func() time.Time
}

func defaultMapBackupStore() *mapBackupStore {
	return &mapBackupStore{areasFile: mapBackupAreasFile, dir: mapBackupDir, keep: MapBackupKeep, now: time.Now}
}

// summariseAreasFile reads the few facts about a map that the list shows.
func summariseAreasFile(content []byte) (areas, obstacles, ignoreLines int, names []string) {
	names = []string{}
	for _, raw := range strings.Split(string(content), "\n") {
		line := strings.TrimSpace(raw)
		key, val, ok := strings.Cut(line, ":")
		if !ok || strings.HasPrefix(line, "#") {
			continue
		}
		val = strings.TrimSpace(val)
		switch {
		case key == "area_count":
			areas, _ = strconv.Atoi(val)
		case key == "lidar_corridor_count":
			ignoreLines, _ = strconv.Atoi(val)
		case strings.HasPrefix(key, "area_") && strings.HasSuffix(key, "_name"):
			names = append(names, val)
		case strings.HasPrefix(key, "area_") && strings.HasSuffix(key, "_obstacle_count"):
			n, _ := strconv.Atoi(val)
			obstacles += n
		}
	}
	return
}

func (s *mapBackupStore) idFor(t time.Time) string {
	return mapBackupPrefix + t.UTC().Format("20060102T150405.000Z") + mapBackupSuffix
}

func (s *mapBackupStore) createdAt(id string) time.Time {
	stamp := strings.TrimSuffix(strings.TrimPrefix(id, mapBackupPrefix), mapBackupSuffix)
	t, err := time.Parse("20060102T150405.000Z", stamp)
	if err != nil {
		return time.Time{}
	}
	return t
}

// ids returns the stored backup ids, newest first.
func (s *mapBackupStore) ids() ([]string, error) {
	entries, err := os.ReadDir(s.dir)
	if errors.Is(err, os.ErrNotExist) {
		return nil, nil
	}
	if err != nil {
		return nil, err
	}
	var ids []string
	for _, e := range entries {
		if !e.IsDir() && mapBackupIDPattern.MatchString(e.Name()) {
			ids = append(ids, e.Name())
		}
	}
	// The id starts with a fixed-width UTC stamp, so a plain sort is chronological.
	sort.Sort(sort.Reverse(sort.StringSlice(ids)))
	return ids, nil
}

// Create copies the current areas.dat into the backup directory and deletes the
// backups beyond the newest `keep`. It returns skipped=true (and no error) when
// there is nothing worth keeping: no areas.dat, or one without areas — an empty map
// must never push a good backup out of the list. A copy identical to the newest
// backup is not stored again (opening the editor twice without a change).
func (s *mapBackupStore) Create() (info *MapBackup, skipped bool, err error) {
	content, err := os.ReadFile(s.areasFile)
	if errors.Is(err, os.ErrNotExist) {
		return nil, true, nil
	}
	if err != nil {
		return nil, false, err
	}
	areas, _, _, _ := summariseAreasFile(content)
	if areas == 0 {
		return nil, true, nil
	}

	if err := os.MkdirAll(s.dir, 0o755); err != nil {
		return nil, false, err
	}
	existing, err := s.ids()
	if err != nil {
		return nil, false, err
	}
	if len(existing) > 0 {
		if newest, rerr := os.ReadFile(filepath.Join(s.dir, existing[0])); rerr == nil && bytes.Equal(newest, content) {
			b, derr := s.describe(existing[0])
			return b, false, derr
		}
	}

	// Same millisecond as an existing backup: step the stamp forward so ids stay unique and sorted.
	at := s.now()
	id := s.idFor(at)
	for {
		if _, serr := os.Stat(filepath.Join(s.dir, id)); errors.Is(serr, os.ErrNotExist) {
			break
		}
		at = at.Add(time.Millisecond)
		id = s.idFor(at)
	}
	if err := writeFileAtomically(filepath.Join(s.dir, id), content); err != nil {
		return nil, false, err
	}
	if err := s.prune(); err != nil {
		return nil, false, err
	}
	b, derr := s.describe(id)
	return b, false, derr
}

// prune deletes every backup beyond the newest `keep`.
func (s *mapBackupStore) prune() error {
	ids, err := s.ids()
	if err != nil {
		return err
	}
	for _, id := range ids[min(len(ids), s.keep):] {
		if err := os.Remove(filepath.Join(s.dir, id)); err != nil && !errors.Is(err, os.ErrNotExist) {
			return err
		}
	}
	return nil
}

func (s *mapBackupStore) describe(id string) (*MapBackup, error) {
	content, err := os.ReadFile(filepath.Join(s.dir, id))
	if err != nil {
		return nil, err
	}
	areas, obstacles, ignoreLines, names := summariseAreasFile(content)
	return &MapBackup{
		ID: id, CreatedAt: s.createdAt(id), SizeBytes: int64(len(content)),
		Areas: areas, Obstacles: obstacles, IgnoreLines: ignoreLines, AreaNames: names,
	}, nil
}

// List returns the stored backups, newest first.
func (s *mapBackupStore) List() ([]MapBackup, error) {
	ids, err := s.ids()
	if err != nil {
		return nil, err
	}
	out := make([]MapBackup, 0, len(ids))
	for _, id := range ids {
		if b, derr := s.describe(id); derr == nil {
			out = append(out, *b)
		}
	}
	return out, nil
}

// Restore puts a backup back as areas.dat and asks map_server to load it. The map
// being replaced is backed up first, so a restore can itself be undone.
func (s *mapBackupStore) Restore(ctx context.Context, id string, reload func(context.Context) error) error {
	if !mapBackupIDPattern.MatchString(id) {
		return fmt.Errorf("invalid backup id %q", id)
	}
	content, err := os.ReadFile(filepath.Join(s.dir, id))
	if err != nil {
		return fmt.Errorf("backup %s: %w", id, err)
	}
	if areas, _, _, _ := summariseAreasFile(content); areas == 0 {
		return fmt.Errorf("backup %s contains no areas; refusing to restore it", id)
	}
	if _, _, err := s.Create(); err != nil {
		return fmt.Errorf("could not keep the current map before restoring: %w", err)
	}
	if err := writePreservingPerms(s.areasFile, content); err != nil {
		return fmt.Errorf("writing %s: %w", s.areasFile, err)
	}
	if err := reload(ctx); err != nil {
		return fmt.Errorf("the backup was written to %s but map_server did not reload it (restart the ROS2 container to load it): %w", s.areasFile, err)
	}
	return nil
}

// writeFileAtomically writes next to the target and renames, so a reader (or a power
// cut) sees the old file or the complete new one.
func writeFileAtomically(path string, content []byte) error {
	tmp, err := os.CreateTemp(filepath.Dir(path), ".mowgli-mapbackup-*.tmp")
	if err != nil {
		return err
	}
	tmpPath := tmp.Name()
	defer func() { _ = os.Remove(tmpPath) }() // no-op after a successful rename
	if _, err := tmp.Write(content); err != nil {
		_ = tmp.Close()
		return err
	}
	if err := tmp.Sync(); err != nil {
		_ = tmp.Close()
		return err
	}
	if err := tmp.Close(); err != nil {
		return err
	}
	if err := os.Chmod(tmpPath, 0o644); err != nil {
		return err
	}
	return os.Rename(tmpPath, path)
}

// MapBackupRoutes registers the backup endpoints:
//
//	GET  /map-backups              the stored backups, newest first
//	POST /map-backups              take a backup now (the editor does this before it opens)
//	POST /map-backups/:id/restore  put a backup back and reload it in map_server
func MapBackupRoutes(r *gin.RouterGroup, provider types.IRosProvider) {
	registerMapBackupRoutes(r, defaultMapBackupStore(), func(ctx context.Context) error {
		var res triggerRes
		if err := callMapService(ctx, provider, mapServiceLoadAreas, &struct{}{}, &res, "std_srvs/srv/Trigger"); err != nil {
			return err
		}
		if !res.Success {
			return errors.New(res.Message)
		}
		return nil
	})
}

func registerMapBackupRoutes(r *gin.RouterGroup, store *mapBackupStore, reload func(context.Context) error) {
	r.GET("/map-backups", func(c *gin.Context) {
		list, err := store.List()
		if err != nil {
			c.JSON(http.StatusInternalServerError, ErrorResponse{Error: err.Error()})
			return
		}
		c.JSON(http.StatusOK, gin.H{"backups": list, "keep": store.keep})
	})

	r.POST("/map-backups", func(c *gin.Context) {
		// Serialise with a running map replace so the copy is a settled map, not a half-written one.
		if err := lockMapReplacement(c.Request.Context(), nil); err != nil {
			c.JSON(http.StatusServiceUnavailable, ErrorResponse{Error: err.Error()})
			return
		}
		defer unlockMapReplacement()
		info, skipped, err := store.Create()
		if err != nil {
			c.JSON(http.StatusInternalServerError, ErrorResponse{Error: err.Error()})
			return
		}
		c.JSON(http.StatusOK, gin.H{"skipped": skipped, "backup": info})
	})

	r.POST("/map-backups/:id/restore", func(c *gin.Context) {
		if err := lockMapReplacement(c.Request.Context(), nil); err != nil {
			c.JSON(http.StatusServiceUnavailable, ErrorResponse{Error: err.Error()})
			return
		}
		defer unlockMapReplacement()
		if err := store.Restore(c.Request.Context(), c.Param("id"), reload); err != nil {
			status := http.StatusInternalServerError
			if !mapBackupIDPattern.MatchString(c.Param("id")) {
				status = http.StatusBadRequest
			}
			c.JSON(status, ErrorResponse{Error: err.Error()})
			return
		}
		c.JSON(http.StatusOK, OkResponse{})
	})
}
