package api

import (
	"encoding/json"
	"fmt"
	"net/http"
	"os"
	"path/filepath"
	"sync"
	"sync/atomic"
	"time"

	"github.com/gin-gonic/gin"
	"github.com/mowglinext/mowglinext/pkg/blackbox"
	"github.com/mowglinext/mowglinext/pkg/buildinfo"
	"github.com/mowglinext/mowglinext/pkg/providers"
	"github.com/mowglinext/mowglinext/pkg/types"
)

const blackboxConfigKey = "blackbox.config"

// Curated from mow_session_monitor.py and the provider's maintained topic map.
// No raw scans, maps, paths, camera frames, robot configuration or credentials.
var blackboxTopics = []string{
	"fusionRaw", "wheelOdom", "imu", "gpsRaw", "gnssStatus", "ticks",
	"cmdVel", "cmdVelApplied", "status", "highLevelStatus", "btLog", "power",
	"emergency", "diagnostics", "fusionDiag", "digEvent", "localizationMode", "collisionMonitor",
}

type blackboxTopicStatus struct {
	Topic          string `json:"topic"`
	LastReceivedAt string `json:"last_received_at,omitempty"`
}

type BlackboxStatusResponse struct {
	blackbox.Status
	Recordings []blackbox.Snapshot   `json:"recordings"`
	Topics     []blackboxTopicStatus `json:"topics"`
	Warning    string                `json:"warning,omitempty"`
}

type blackboxManager struct {
	mu        sync.Mutex
	recorder  *blackbox.Recorder
	db        types.IDBProvider
	ros       types.IRosProvider
	error     string
	reconcile chan struct{}
	received  []atomic.Int64
}

// BlackboxRoutes starts collection independent of any browser/Diagnostics tab.
// A recorder initialization failure is isolated to these endpoints.
func BlackboxRoutes(r *gin.RouterGroup, db types.IDBProvider, ros types.IRosProvider) *blackboxManager {
	m := &blackboxManager{db: db, ros: ros, reconcile: make(chan struct{}, 1), received: make([]atomic.Int64, len(blackboxTopics))}
	cfg := blackbox.DefaultConfig()
	if raw, err := db.Get(blackboxConfigKey); err == nil && len(raw) > 0 {
		if err := json.Unmarshal(raw, &cfg); err != nil || cfg.Validate() != nil {
			// Invalid stored settings fail closed; the operator can still see the error.
			cfg = blackbox.DefaultConfig()
			cfg.Enabled = false
			m.error = "invalid stored blackbox configuration; recording disabled"
		}
	}
	dir := os.Getenv("BLACKBOX_DIR")
	if dir == "" {
		dir = "/ros2_ws/maps/blackbox"
	}
	b := buildinfo.Current()
	var err error
	m.recorder, err = blackbox.New(filepath.Clean(dir), cfg, map[string]string{"gui_revision": b.Revision, "gui_version": b.Version})
	if err != nil {
		m.error = "blackbox unavailable: " + err.Error()
	}
	m.register(r)
	if m.recorder != nil && ros != nil {
		go m.subscribeLoop()
		m.reconcile <- struct{}{}
	}
	return m
}

func (m *blackboxManager) subscribeLoop() {
	attached := false
	for range m.reconcile {
		enabled := m.recorder.Status().Config.Enabled
		if enabled == attached {
			continue
		}
		for i, topic := range blackboxTopics {
			if !enabled {
				m.ros.UnSubscribe(topic, "blackbox")
				continue
			}
			index, key := i, topic
			lastState := ""
			lastEmergency := false
			// Dedicated provider mailboxes already coalesce at 10 Hz, before
			// the blackbox admission queue. Events are unthrottled.
			interval := 100
			if key == "emergency" || key == "diagnostics" || key == "fusionDiag" || key == "digEvent" {
				interval = 0
			}
			if err := m.ros.Subscribe(key, "blackbox", interval, func(data []byte) {
				m.received[index].Store(time.Now().UnixNano())
				m.recorder.Ingest(key, data)
				if key == "highLevelStatus" && len(data) <= 8192 {
					var status providers.NotifyStatus
					if json.Unmarshal(data, &status) == nil && len(status.StateName) <= 128 {
						if status.StateName != lastState {
							if reason := providers.TerminalFailureMessage(status.StateName); reason != "" {
								m.recorder.Trigger("behavior: " + reason + ": " + status.StateName)
							}
						}
						if status.Emergency && !lastEmergency {
							m.recorder.Trigger("behavior: emergency")
						}
						lastState, lastEmergency = status.StateName, status.Emergency
					}
				}
			}); err != nil {
				m.mu.Lock()
				m.error = fmt.Sprintf("blackbox subscription %s: %v", key, err)
				m.mu.Unlock()
			}
		}
		attached = enabled
	}
}

func (m *blackboxManager) available(c *gin.Context) bool {
	if m.recorder != nil {
		return true
	}
	c.JSON(http.StatusServiceUnavailable, gin.H{"error": m.error})
	return false
}

func (m *blackboxManager) register(r *gin.RouterGroup) {
	g := r.Group("/tools/blackbox")
	g.GET("/status", m.getStatus)
	g.PUT("/config", m.putConfig)
	g.POST("/save", m.save)
	g.GET("/download/:name", m.download)
	g.DELETE("/:name", m.remove)
}

// @Summary Passive blackbox status and completed recordings
// @Tags diagnostics
// @Produce json
// @Success 200 {object} BlackboxStatusResponse
// @Router /tools/blackbox/status [get]
func (m *blackboxManager) getStatus(c *gin.Context) {
	if !m.available(c) {
		return
	}
	resp := BlackboxStatusResponse{Status: m.recorder.Status(), Topics: make([]blackboxTopicStatus, len(blackboxTopics))}
	m.mu.Lock()
	resp.Warning = m.error
	m.mu.Unlock()
	for i, topic := range blackboxTopics {
		resp.Topics[i].Topic = topic
		if nanos := m.received[i].Load(); nanos != 0 {
			resp.Topics[i].LastReceivedAt = time.Unix(0, nanos).UTC().Format(time.RFC3339Nano)
		}
	}
	var err error
	resp.Recordings, err = m.recorder.List()
	if err != nil {
		resp.Warning = "cannot list snapshots: " + err.Error()
	}
	if resp.Recordings == nil {
		resp.Recordings = []blackbox.Snapshot{}
	}
	c.JSON(http.StatusOK, resp)
}

// @Summary Configure passive blackbox (complete configuration)
// @Tags diagnostics
// @Accept json
// @Param config body blackbox.Config true "Recorder limits"
// @Success 200 {object} blackbox.Status
// @Router /tools/blackbox/config [put]
func (m *blackboxManager) putConfig(c *gin.Context) {
	if !m.available(c) {
		return
	}
	c.Request.Body = http.MaxBytesReader(c.Writer, c.Request.Body, 4096)
	var cfg blackbox.Config
	decoder := json.NewDecoder(c.Request.Body)
	decoder.DisallowUnknownFields()
	if err := decoder.Decode(&cfg); err != nil {
		c.JSON(400, gin.H{"error": "invalid blackbox configuration"})
		return
	}
	if err := cfg.Validate(); err != nil {
		c.JSON(400, gin.H{"error": err.Error()})
		return
	}
	m.mu.Lock()
	defer m.mu.Unlock()
	raw, _ := json.Marshal(cfg)
	var saveErr error
	if err := m.recorder.ConfigurePersist(cfg, func() error {
		saveErr = m.db.Set(blackboxConfigKey, raw)
		return saveErr
	}); err != nil {
		if saveErr != nil {
			c.JSON(500, gin.H{"error": "could not persist blackbox configuration"})
			return
		}
		c.JSON(409, gin.H{"error": err.Error()})
		return
	}
	m.error = ""
	select {
	case m.reconcile <- struct{}{}:
	default:
	}
	c.JSON(200, m.recorder.Status())
}

// @Summary Save available pre-event history and collect post-event window
// @Tags diagnostics
// @Success 202 {object} map[string]interface{}
// @Router /tools/blackbox/save [post]
func (m *blackboxManager) save(c *gin.Context) {
	if !m.available(c) {
		return
	}
	if !m.recorder.Status().Config.Enabled {
		c.JSON(409, gin.H{"error": "blackbox is disabled"})
		return
	}
	id, accepted := m.recorder.Trigger("manual")
	if id == "" {
		c.JSON(409, gin.H{"error": "blackbox is writing or unavailable"})
		return
	}
	c.JSON(202, gin.H{"capture_id": id, "accepted": accepted})
}

// @Summary Download a completed timestamped blackbox timeline
// @Tags diagnostics
// @Param name path string true "Completed snapshot name"
// @Produce application/x-ndjson
// @Success 200 {file} file
// @Router /tools/blackbox/download/{name} [get]
func (m *blackboxManager) download(c *gin.Context) {
	if !m.available(c) {
		return
	}
	f, err := m.recorder.Open(c.Param("name"))
	if err != nil {
		c.JSON(404, gin.H{"error": "snapshot not found"})
		return
	}
	defer f.Close()
	st, err := f.Stat()
	if err != nil {
		c.JSON(500, gin.H{"error": "cannot read snapshot"})
		return
	}
	c.Header("Content-Disposition", fmt.Sprintf("attachment; filename=%q", c.Param("name")))
	c.Header("Content-Type", "application/x-ndjson")
	c.Header("X-Content-Type-Options", "nosniff")
	http.ServeContent(c.Writer, c.Request, st.Name(), st.ModTime(), f)
}

// @Summary Delete a completed blackbox recording
// @Tags diagnostics
// @Param name path string true "Completed snapshot name"
// @Success 200 {object} OkResponse
// @Router /tools/blackbox/{name} [delete]
func (m *blackboxManager) remove(c *gin.Context) {
	if !m.available(c) {
		return
	}
	if err := m.recorder.Delete(c.Param("name")); err != nil {
		c.JSON(404, gin.H{"error": "snapshot not found or cannot be deleted"})
		return
	}
	c.JSON(200, gin.H{"message": "snapshot deleted"})
}
