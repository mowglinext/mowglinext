package api

import (
	"archive/tar"
	"compress/gzip"
	"context"
	"fmt"
	"io"
	"net/http"
	"os"
	"path/filepath"
	"regexp"
	"sort"
	"strings"
	"time"

	"github.com/mowglinext/mowglinext/pkg/types"
	"github.com/gin-gonic/gin"
)

// ---------------------------------------------------------------------------
// Rosbag recording
//
// A one-click "record everything" facility for field debugging: an operator
// starts a `ros2 bag record -a` capture from the Diagnostics page, drives the
// robot to reproduce a problem, stops the recording, then downloads the bag
// as a .tar.gz to send back for offline analysis.
//
// The recorder runs INSIDE the mowgli-ros2 container (that is where ROS2, the
// DDS graph and the `ros2` CLI live — the GUI container has none of them). We
// reach it with the same `docker exec` mechanism the drive-tuning tool uses.
//
// State lives on disk, NOT in Go memory: each recording is a directory under
// the shared `mowgli_maps` volume (/ros2_ws/maps/rosbags/<name>) that carries a
// `.rosbag.pid` file while the capture is live. Because that volume is mounted
// into BOTH containers at the same path, the GUI lists/downloads bags straight
// from its own filesystem, and "is it still recording?" is answered by a cheap
// `kill -0` probe inside the container. Keeping the source of truth on disk
// means an in-progress capture survives a GUI restart (e.g. a watchtower
// auto-update) — the operator can still see it running and stop it afterwards.
// ---------------------------------------------------------------------------

const (
	rosbagRos2ContainerName = "mowgli-ros2"
	// rosbagDefaultDir is the recordings root on the shared mowgli_maps volume.
	// Mounted at the same path in both mowgli-ros2 and mowgli-gui, so the GUI
	// reads bags directly. Overridable via ROSBAG_DIR (used by tests).
	rosbagDefaultDir = "/ros2_ws/maps/rosbags"
	rosbagPidFile    = ".rosbag.pid"
	rosbagLogFile    = "record.log"
	// rosbagBagSubdir is the `-o` output directory ros2 bag writes into. It must
	// not pre-exist, so it sits one level below the (pre-created) recording dir.
	rosbagBagSubdir = "bag"
)

// rosbagNameRe guards every name that reaches the filesystem or a shell. Names
// are minted by the backend (a timestamp), but download/delete take one from
// the URL, so we validate defensively against path traversal / injection.
var rosbagNameRe = regexp.MustCompile(`^[A-Za-z0-9_.-]+$`)

// ---------------------------------------------------------------------------
// Response types
// ---------------------------------------------------------------------------

// RosbagRecording describes one capture on disk.
type RosbagRecording struct {
	Name       string `json:"name"`
	SizeBytes  int64  `json:"size_bytes"`
	ModifiedAt string `json:"modified_at"`
	Active     bool   `json:"active"`
}

// RosbagStatusResponse is the payload for GET /tools/rosbag/status.
type RosbagStatusResponse struct {
	Active     bool              `json:"active"`
	ActiveName string            `json:"active_name,omitempty"`
	Recordings []RosbagRecording `json:"recordings"`
	// Warning is a non-fatal note (e.g. the ROS2 container is down so live
	// state could not be probed) surfaced to the operator without failing.
	Warning string `json:"warning,omitempty"`
}

// RosbagStartResponse is the payload for POST /tools/rosbag/start.
type RosbagStartResponse struct {
	Name      string `json:"name"`
	StartedAt string `json:"started_at"`
}

// RosbagStopResponse is the payload for POST /tools/rosbag/stop.
type RosbagStopResponse struct {
	Stopped     bool   `json:"stopped"`
	StoppedName string `json:"stopped_name,omitempty"`
}

// ---------------------------------------------------------------------------
// Manager
// ---------------------------------------------------------------------------

type rosbagManager struct {
	docker        types.IDockerProvider
	containerName string
	recordingsDir string
}

func newRosbagManager(docker types.IDockerProvider) *rosbagManager {
	dir := strings.TrimSpace(os.Getenv("ROSBAG_DIR"))
	if dir == "" {
		dir = rosbagDefaultDir
	}
	return &rosbagManager{
		docker:        docker,
		containerName: rosbagRos2ContainerName,
		recordingsDir: dir,
	}
}

// RosbagRoutes registers the rosbag recording endpoints.
func RosbagRoutes(r *gin.RouterGroup, dockerProvider types.IDockerProvider) {
	m := newRosbagManager(dockerProvider)
	group := r.Group("/tools/rosbag")
	group.GET("/status", m.getStatus())
	group.POST("/start", m.postStart())
	group.POST("/stop", m.postStop())
	group.GET("/download/:name", m.getDownload())
	group.DELETE("/:name", m.deleteRecording())
}

// ---------------------------------------------------------------------------
// Container plumbing
// ---------------------------------------------------------------------------

func (m *rosbagManager) containerID(ctx context.Context) (string, error) {
	containers, err := m.docker.ContainerList(ctx)
	if err != nil {
		return "", err
	}
	for _, c := range containers {
		if containerNameMatches(c, m.containerName) {
			return c.ID, nil
		}
	}
	return "", fmt.Errorf("%s container not found", m.containerName)
}

// activeRecordingNames returns the names of recordings whose recorder process
// is still alive, probed inside the ROS2 container. Any `.rosbag.pid` whose
// process is gone is pruned so a crashed recorder does not read as "recording"
// forever.
func (m *rosbagManager) activeRecordingNames(ctx context.Context) ([]string, error) {
	containerID, err := m.containerID(ctx)
	if err != nil {
		return nil, err
	}
	base := shellQuote(m.recordingsDir)
	script := strings.Join([]string{
		"shopt -s nullglob",
		"for d in " + base + "/*/; do",
		"  pf=\"${d%/}/" + rosbagPidFile + "\"",
		"  [ -f \"$pf\" ] || continue",
		"  p=$(cat \"$pf\" 2>/dev/null)",
		"  if [ -n \"$p\" ] && kill -0 \"$p\" 2>/dev/null; then",
		"    basename \"${d%/}\"",
		"  else",
		"    rm -f \"$pf\"",
		"  fi",
		"done",
	}, "\n")
	res, err := m.docker.ContainerExec(ctx, containerID, types.ContainerExecSpec{
		Cmd: []string{"bash", "-c", script},
	})
	if err != nil {
		return nil, err
	}
	if res.ExitCode != 0 {
		return nil, fmt.Errorf("failed to probe recordings: %s", strings.TrimSpace(res.Stdout+res.Stderr))
	}
	names := []string{}
	for _, line := range strings.Split(strings.TrimSpace(res.Stdout), "\n") {
		if trimmed := strings.TrimSpace(line); trimmed != "" {
			names = append(names, trimmed)
		}
	}
	return names, nil
}

// ---------------------------------------------------------------------------
// GET /tools/rosbag/status
// ---------------------------------------------------------------------------

func (m *rosbagManager) getStatus() gin.HandlerFunc {
	return func(c *gin.Context) {
		ctx, cancel := context.WithTimeout(c.Request.Context(), 15*time.Second)
		defer cancel()

		resp := RosbagStatusResponse{Recordings: []RosbagRecording{}}

		activeSet := map[string]bool{}
		if names, err := m.activeRecordingNames(ctx); err != nil {
			// The ROS2 container being down is a normal, recoverable state; we
			// still list what is on disk and just note the probe failure.
			resp.Warning = "could not query recording state: " + err.Error()
		} else {
			for _, n := range names {
				activeSet[n] = true
			}
		}

		entries, err := os.ReadDir(m.recordingsDir)
		if err != nil && !os.IsNotExist(err) {
			c.JSON(http.StatusInternalServerError, ErrorResponse{Error: "cannot read recordings dir: " + err.Error()})
			return
		}
		for _, entry := range entries {
			if !entry.IsDir() {
				continue
			}
			name := entry.Name()
			dir := filepath.Join(m.recordingsDir, name)
			info, statErr := entry.Info()
			modified := ""
			if statErr == nil {
				modified = info.ModTime().UTC().Format(time.RFC3339)
			}
			rec := RosbagRecording{
				Name:       name,
				SizeBytes:  dirSize(dir),
				ModifiedAt: modified,
				Active:     activeSet[name],
			}
			resp.Recordings = append(resp.Recordings, rec)
			if rec.Active {
				resp.Active = true
				resp.ActiveName = name
			}
		}

		// Newest first.
		sort.Slice(resp.Recordings, func(i, j int) bool {
			return resp.Recordings[i].Name > resp.Recordings[j].Name
		})

		c.JSON(http.StatusOK, resp)
	}
}

// ---------------------------------------------------------------------------
// POST /tools/rosbag/start
// ---------------------------------------------------------------------------

func (m *rosbagManager) postStart() gin.HandlerFunc {
	return func(c *gin.Context) {
		ctx, cancel := context.WithTimeout(c.Request.Context(), 30*time.Second)
		defer cancel()

		// Refuse if something is already recording — one capture at a time.
		if names, err := m.activeRecordingNames(ctx); err != nil {
			c.JSON(http.StatusServiceUnavailable, ErrorResponse{Error: err.Error()})
			return
		} else if len(names) > 0 {
			c.JSON(http.StatusConflict, ErrorResponse{Error: "a recording is already in progress: " + names[0]})
			return
		}

		containerID, err := m.containerID(ctx)
		if err != nil {
			c.JSON(http.StatusServiceUnavailable, ErrorResponse{Error: err.Error()})
			return
		}

		startedAt := time.Now().UTC()
		name := "rosbag_" + startedAt.Format("20060102T150405Z")
		dir := m.recordingsDir + "/" + name

		res, err := m.docker.ContainerExec(ctx, containerID, types.ContainerExecSpec{
			Cmd: buildRosbagStartCommand(dir),
			Env: []string{"RCUTILS_COLORIZED_OUTPUT=0"},
		})
		if err != nil {
			c.JSON(http.StatusInternalServerError, ErrorResponse{Error: err.Error()})
			return
		}
		if res.ExitCode != 0 {
			c.JSON(http.StatusInternalServerError, ErrorResponse{
				Error: "failed to start recording: " + strings.TrimSpace(res.Stdout+res.Stderr),
			})
			return
		}

		c.JSON(http.StatusOK, RosbagStartResponse{
			Name:      name,
			StartedAt: startedAt.Format(time.RFC3339),
		})
	}
}

// buildRosbagStartCommand launches `ros2 bag record -a` detached inside the
// container and verifies it actually came up. The recorder is setsid-detached
// with its stdio redirected to a log file so the exec call returns immediately
// while the capture keeps running; its PID is written to `.rosbag.pid`.
func buildRosbagStartCommand(dir string) []string {
	qdir := shellQuote(dir)
	script := strings.Join([]string{
		"set -e",
		"dir=" + qdir,
		"mkdir -p \"$dir\"",
		"source /opt/ros/*/setup.bash",
		"if [ -f /ros2_ws/install/setup.bash ]; then source /ros2_ws/install/setup.bash; fi",
		// $0 inside the single-quoted inner script expands to "$dir".
		"setsid bash -c 'ros2 bag record -a -o \"$0/" + rosbagBagSubdir + "\" > \"$0/" + rosbagLogFile + "\" 2>&1 & echo $! > \"$0/" + rosbagPidFile + "\"' \"$dir\" < /dev/null",
		"sleep 1.5",
		"pid=$(cat \"$dir/" + rosbagPidFile + "\" 2>/dev/null || true)",
		"if [ -z \"$pid\" ] || ! kill -0 \"$pid\" 2>/dev/null; then echo 'recorder exited immediately'; tail -n 40 \"$dir/" + rosbagLogFile + "\" 2>/dev/null; rm -f \"$dir/" + rosbagPidFile + "\"; exit 1; fi",
		"echo \"$pid\"",
	}, "\n")
	return []string{"bash", "-c", script}
}

// ---------------------------------------------------------------------------
// POST /tools/rosbag/stop
// ---------------------------------------------------------------------------

func (m *rosbagManager) postStop() gin.HandlerFunc {
	return func(c *gin.Context) {
		ctx, cancel := context.WithTimeout(c.Request.Context(), 40*time.Second)
		defer cancel()

		containerID, err := m.containerID(ctx)
		if err != nil {
			c.JSON(http.StatusServiceUnavailable, ErrorResponse{Error: err.Error()})
			return
		}

		res, err := m.docker.ContainerExec(ctx, containerID, types.ContainerExecSpec{
			Cmd: buildRosbagStopCommand(m.recordingsDir),
		})
		if err != nil {
			c.JSON(http.StatusInternalServerError, ErrorResponse{Error: err.Error()})
			return
		}
		if res.ExitCode != 0 {
			c.JSON(http.StatusInternalServerError, ErrorResponse{
				Error: "failed to stop recording: " + strings.TrimSpace(res.Stdout+res.Stderr),
			})
			return
		}

		stoppedName := strings.TrimSpace(res.Stdout)
		c.JSON(http.StatusOK, RosbagStopResponse{
			Stopped:     stoppedName != "",
			StoppedName: stoppedName,
		})
	}
}

// buildRosbagStopCommand sends SIGINT to every live recorder (Ctrl-C is the
// only clean shutdown for rosbag2 — a hard kill can truncate the .mcap), waits
// up to ~20s for the file to finalize, then removes the pidfile. It echoes the
// name of the recording it stopped (empty if none was live).
func buildRosbagStopCommand(recordingsDir string) []string {
	base := shellQuote(recordingsDir)
	script := strings.Join([]string{
		"shopt -s nullglob",
		"stopped=''",
		"for d in " + base + "/*/; do",
		"  pf=\"${d%/}/" + rosbagPidFile + "\"",
		"  [ -f \"$pf\" ] || continue",
		"  p=$(cat \"$pf\" 2>/dev/null)",
		"  if [ -n \"$p\" ] && kill -0 \"$p\" 2>/dev/null; then",
		"    kill -INT \"$p\" 2>/dev/null || true",
		"    for i in $(seq 1 100); do kill -0 \"$p\" 2>/dev/null || break; sleep 0.2; done",
		"    stopped=$(basename \"${d%/}\")",
		"  fi",
		"  rm -f \"$pf\"",
		"done",
		"echo \"$stopped\"",
	}, "\n")
	return []string{"bash", "-c", script}
}

// ---------------------------------------------------------------------------
// GET /tools/rosbag/download/:name
// ---------------------------------------------------------------------------

func (m *rosbagManager) getDownload() gin.HandlerFunc {
	return func(c *gin.Context) {
		name := c.Param("name")
		if !rosbagNameRe.MatchString(name) {
			c.JSON(http.StatusBadRequest, ErrorResponse{Error: "invalid recording name"})
			return
		}
		dir := filepath.Join(m.recordingsDir, name)
		info, err := os.Stat(dir)
		if err != nil || !info.IsDir() {
			c.JSON(http.StatusNotFound, ErrorResponse{Error: "recording not found"})
			return
		}

		c.Header("Content-Type", "application/gzip")
		c.Header("Content-Disposition", fmt.Sprintf("attachment; filename=%q", name+".tar.gz"))

		// Stream the tarball straight to the client — bags can be large and we
		// do not want to buffer them in memory. Once bytes are on the wire we
		// can no longer change the status code, so a mid-stream error is logged
		// and the connection is simply cut (a truncated download the operator
		// can retry).
		if err := writeRecordingTarGz(c.Writer, dir, name); err != nil {
			c.Error(err) //nolint:errcheck // best-effort; headers already sent
		}
	}
}

// writeRecordingTarGz tars+gzips every file under dir, prefixing entries with
// the recording name so the archive extracts into a self-named folder. The
// live pidfile is skipped — it is transient bookkeeping, not part of the bag.
func writeRecordingTarGz(w interface{ Write([]byte) (int, error) }, dir, name string) error {
	gz := gzip.NewWriter(w)
	defer gz.Close()
	tw := tar.NewWriter(gz)
	defer tw.Close()

	return filepath.Walk(dir, func(path string, info os.FileInfo, err error) error {
		if err != nil {
			return err
		}
		if info.IsDir() {
			return nil
		}
		if info.Name() == rosbagPidFile {
			return nil
		}
		rel, err := filepath.Rel(dir, path)
		if err != nil {
			return err
		}
		header, err := tar.FileInfoHeader(info, "")
		if err != nil {
			return err
		}
		header.Name = filepath.ToSlash(filepath.Join(name, rel))
		if err := tw.WriteHeader(header); err != nil {
			return err
		}
		f, err := os.Open(path)
		if err != nil {
			return err
		}
		defer f.Close()
		_, err = io.Copy(tw, f)
		return err
	})
}

// ---------------------------------------------------------------------------
// DELETE /tools/rosbag/:name
// ---------------------------------------------------------------------------

func (m *rosbagManager) deleteRecording() gin.HandlerFunc {
	return func(c *gin.Context) {
		name := c.Param("name")
		if !rosbagNameRe.MatchString(name) {
			c.JSON(http.StatusBadRequest, ErrorResponse{Error: "invalid recording name"})
			return
		}

		ctx, cancel := context.WithTimeout(c.Request.Context(), 15*time.Second)
		defer cancel()

		// Refuse to delete a recording that is still capturing.
		if names, err := m.activeRecordingNames(ctx); err == nil {
			for _, n := range names {
				if n == name {
					c.JSON(http.StatusConflict, ErrorResponse{Error: "recording is still in progress; stop it first"})
					return
				}
			}
		}

		dir := filepath.Join(m.recordingsDir, name)
		if info, err := os.Stat(dir); err != nil || !info.IsDir() {
			c.JSON(http.StatusNotFound, ErrorResponse{Error: "recording not found"})
			return
		}
		if err := os.RemoveAll(dir); err != nil {
			c.JSON(http.StatusInternalServerError, ErrorResponse{Error: "failed to delete recording: " + err.Error()})
			return
		}
		c.JSON(http.StatusOK, gin.H{"ok": true})
	}
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

// dirSize sums the size of every regular file under dir (0 on error).
func dirSize(dir string) int64 {
	var total int64
	_ = filepath.Walk(dir, func(_ string, info os.FileInfo, err error) error {
		if err != nil {
			return nil
		}
		if !info.IsDir() {
			total += info.Size()
		}
		return nil
	})
	return total
}
