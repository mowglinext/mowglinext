package api

import (
	"context"
	"encoding/json"
	"fmt"
	"math"
	"os"
	"strings"
	"sync"
	"time"

	"github.com/gin-gonic/gin"
	"github.com/mowglinext/mowglinext/pkg/msgs/mowgli"
	"github.com/mowglinext/mowglinext/pkg/types"
	"gopkg.in/yaml.v3"
)

type mavrosCalibrationResult struct {
	State         string  `json:"state"`
	Error         string  `json:"error,omitempty"`
	Samples       int     `json:"samples"`
	Distance      float64 `json:"distance_m"`
	Left          float64 `json:"left_motor_revolutions"`
	Right         float64 `json:"right_motor_revolutions"`
	TicksPerMeter float64 `json:"ticks_per_meter"`
}
type calibrationPoint struct {
	lat, lon    float64
	left, right uint32
}
type mavrosWheelCalibration struct {
	mu                                 sync.Mutex
	control                            sync.Mutex
	timer                              *time.Timer
	ros                                types.IRosProvider
	db                                 types.IDBProvider
	result                             mavrosCalibrationResult
	started                            time.Time
	ticks                              mowgli.WheelTick
	tickAt, qualityAt, sourceAt, fixAt time.Time
	lastTickStamp, lastFixStamp        int64
	quality                            mowgli.GnssStatus
	source, identity                   string
	sourceValid                        bool
	scale                              float64
	first, last                        *calibrationPoint
	points                             []calibrationPoint
	direction                          *uint8
}

// Passive measurement uses the historical independent WheelTick + RTK input
// path. The operator controls a straight pass using existing teleop; this API
// sends no drive, arm, mode, PID or feed-forward command.
// @Summary records independent MAVROS wheel observations and RTK distance
// @Tags calibration
// @Produce json
// @Success 200 {object} mavrosCalibrationResult
// @Router /tools/drive/mavros-calibration [get]
// @Router /tools/drive/mavros-calibration/start [post]
// @Router /tools/drive/mavros-calibration/finish [post]
// @Router /tools/drive/mavros-calibration/apply [post]
func MavrosWheelCalibrationRoutes(r *gin.RouterGroup, db types.IDBProvider, ros types.IRosProvider) {
	m := &mavrosWheelCalibration{ros: ros, db: db, result: mavrosCalibrationResult{State: "idle"}}
	group := r.Group("/tools/drive/mavros-calibration", func(c *gin.Context) {
		if activeHardwareBackendForDB(db) != "mavros" {
			c.AbortWithStatusJSON(409, ErrorResponse{Error: "MAVROS calibration requires HARDWARE_BACKEND=mavros"})
			return
		}
		c.Next()
	})
	group.Use(func(c *gin.Context) {
		if c.Request.Method == "POST" {
			m.control.Lock()
			defer m.control.Unlock()
		}
		c.Next()
	})
	group.GET("", func(c *gin.Context) { m.mu.Lock(); defer m.mu.Unlock(); m.expire(); c.JSON(200, m.result) })
	group.POST("/start", func(c *gin.Context) {
		m.mu.Lock()
		if m.result.State == "recording" {
			m.mu.Unlock()
			c.JSON(409, ErrorResponse{Error: "calibration already recording"})
			return
		}
		m.result = mavrosCalibrationResult{State: "recording"}
		m.started = time.Now()
		started := m.started
		if m.timer != nil {
			m.timer.Stop()
		}
		m.timer = time.AfterFunc(2*time.Minute, func() {
			m.control.Lock()
			defer m.control.Unlock()
			m.mu.Lock()
			sameSession := m.started.Equal(started)
			if sameSession && m.result.State == "recording" {
				m.fail("calibration expired")
			}
			m.mu.Unlock()
			if sameSession {
				m.stopSubscriptions()
			}
		})
		m.first = nil
		m.last = nil
		m.direction = nil
		m.points = nil
		m.identity = ""
		m.source = ""
		m.lastFixStamp = 0
		m.lastTickStamp = 0
		m.tickAt = time.Time{}
		m.qualityAt = time.Time{}
		m.sourceAt = time.Time{}
		m.fixAt = time.Time{}
		m.sourceValid = false
		m.mu.Unlock()
		for topic, cb := range map[string]func([]byte){"ticks": m.onTicks, "gps": m.onGPS, "gnssStatus": m.onQuality, "diagnostics": m.onDiagnostics, "emergency": m.onEmergency} {
			if err := ros.Subscribe(topic, "mavros-wheel-calibration", 0, cb); err != nil {
				m.stopSubscriptions()
				m.mu.Lock()
				m.fail(err.Error())
				m.mu.Unlock()
				c.JSON(503, ErrorResponse{Error: err.Error()})
				return
			}
		}
		c.JSON(200, mavrosCalibrationResult{State: "recording"})
	})
	group.POST("/finish", func(c *gin.Context) {
		if m.timer != nil {
			m.timer.Stop()
		}
		m.stopSubscriptions()
		m.mu.Lock()
		defer m.mu.Unlock()
		m.expire()
		if m.result.State == "recording" {
			if m.first == nil || m.last == nil || m.result.Samples < 3 {
				m.fail("insufficient paired WheelTick/RTK observations")
			} else if time.Since(m.fixAt) > 2*time.Second || time.Since(m.tickAt) > 2*time.Second || time.Since(m.sourceAt) > 2*time.Second {
				m.fail("feedback stopped before calibration finished")
			} else {
				distance := rtkDistance(*m.first, *m.last)
				left := float64(uint32(m.last.left-m.first.left)) / m.scale
				right := float64(uint32(m.last.right-m.first.right)) / m.scale
				if distance < 2 || distance > 10 || left <= 0 || right <= 0 || math.Abs(left-right)/math.Max(left, right) > 0.1 || !straightRTKPass(m.points) {
					m.fail("require a straight 2–10 m RTK pass with consistent left/right wheels")
				} else {
					m.result.State = "ready"
					m.result.Distance = distance
					m.result.Left = left
					m.result.Right = right
					m.result.TicksPerMeter = (left + right) / (2 * distance)
				}
			}
		}
		c.JSON(200, m.result)
	})
	group.POST("/apply", func(c *gin.Context) {
		var req struct {
			Confirm bool `json:"confirm"`
		}
		if c.ShouldBindJSON(&req) != nil || !req.Confirm {
			c.JSON(400, ErrorResponse{Error: "apply requires confirm=true"})
			return
		}
		m.mu.Lock()
		defer m.mu.Unlock()
		if m.result.State != "ready" {
			c.JSON(409, ErrorResponse{Error: "no validated calibration to apply"})
			return
		}
		ctx, cancel := context.WithTimeout(c.Request.Context(), 12*time.Second)
		defer cancel()
		parameter := types.RosParameter{Name: "mavros/esc_wheel_odometry.ticks_per_meter", Value: m.result.TicksPerMeter}
		updated, err := ros.SetParameters(ctx, []types.RosParameter{parameter})
		if err != nil || !parameterWasApplied(updated, parameter) {
			c.JSON(503, ErrorResponse{Error: "MAVROS rejected or did not confirm ticks_per_meter"})
			return
		}
		if err = persistMavrosTicksPerMeter(db, m.result.TicksPerMeter); err != nil {
			c.JSON(500, ErrorResponse{Error: "calibration applied live; persistence failed: " + err.Error()})
			return
		}
		m.result.State = "applied"
		c.JSON(200, m.result)
	})
}
func (m *mavrosWheelCalibration) stopSubscriptions() {
	for _, topic := range []string{"ticks", "gps", "gnssStatus", "diagnostics", "emergency"} {
		m.ros.UnSubscribe(topic, "mavros-wheel-calibration")
	}
}
func (m *mavrosWheelCalibration) fail(reason string) {
	m.result.State = "failed"
	m.result.Error = reason
}
func (m *mavrosWheelCalibration) expire() {
	if m.result.State == "recording" && time.Since(m.started) > 2*time.Minute {
		m.fail("calibration expired")
	}
}
func (m *mavrosWheelCalibration) onTicks(raw []byte) {
	var t mowgli.WheelTick
	if json.Unmarshal(raw, &t) != nil {
		return
	}
	m.mu.Lock()
	defer m.mu.Unlock()
	m.expire()
	if m.result.State != "recording" {
		return
	}
	stamp := int64(t.Stamp.Sec)*1e9 + int64(t.Stamp.Nanosec)
	if stamp == m.lastTickStamp {
		return
	}
	if stamp < m.lastTickStamp {
		m.fail("WheelTick clock reset")
		return
	}
	if m.first != nil && stamp-m.lastTickStamp > int64(2*time.Second) {
		m.fail("WheelTick observation gap")
		return
	}
	m.lastTickStamp = stamp
	if t.ValidWheels&12 != 12 {
		if m.first != nil {
			m.fail("invalid driven wheels")
		}
		return
	}
	if m.first != nil {
		dl, dr := uint32(t.WheelTicksRl-m.ticks.WheelTicksRl), uint32(t.WheelTicksRr-m.ticks.WheelTicksRr)
		if dl > math.MaxInt32 || dr > math.MaxInt32 {
			m.fail("wheel counter reset")
			return
		}
		if dl > 0 || dr > 0 {
			if t.WheelDirectionRl != t.WheelDirectionRr || (m.direction != nil && *m.direction != t.WheelDirectionRl) {
				m.fail("wheel direction changed during straight calibration")
				return
			}
			direction := t.WheelDirectionRl
			m.direction = &direction
		}
	}
	m.ticks = t
	m.tickAt = time.Now()
}
func (m *mavrosWheelCalibration) onQuality(raw []byte) {
	var g mowgli.GnssStatus
	if json.Unmarshal(raw, &g) != nil {
		return
	}
	m.mu.Lock()
	defer m.mu.Unlock()
	m.quality = g
	m.qualityAt = time.Now()
	if m.result.State == "recording" && m.first != nil && !m.rtkValid() {
		m.fail("RTK Fixed lost")
	}
}
func (m *mavrosWheelCalibration) rtkValid() bool {
	g := m.quality
	// Canonical GnssStatus: RTK_MODE_FIXED=3, CAP_HORIZONTAL_ACCURACY=8.
	return g.RtkMode == 3 && g.FixValid && g.CorrectionsActive && !g.DeadReckoning && g.CapabilityFlags&g.ValueFlags&8 != 0 && g.HorizontalAccuracyM > 0 && g.HorizontalAccuracyM <= 0.05
}
func (m *mavrosWheelCalibration) onDiagnostics(raw []byte) {
	var d struct {
		Status []struct {
			Name   string
			Values []struct{ Key, Value string }
		}
	}
	if json.Unmarshal(raw, &d) != nil {
		return
	}
	m.mu.Lock()
	defer m.mu.Unlock()
	for _, entry := range d.Status {
		if entry.Name != "mavros_esc_wheel_odometry/source" {
			continue
		}
		fields := map[string]string{}
		for _, v := range entry.Values {
			fields[v.Key] = v.Value
		}
		source := fields["wheel_tick_source"]
		prefix := source + "/"
		valid := (source == "esc_status" || source == "ardupilot_legacy") && fields[prefix+"left_valid"] == "true" && fields[prefix+"right_valid"] == "true" && fields["wheel_tick_transport_scale"] == "1000"
		identity := strings.Join([]string{source, fields[prefix+"epoch"], fields[prefix+"left_segment"], fields[prefix+"right_segment"]}, ":")
		if m.result.State == "recording" && m.first != nil && (!valid || identity != m.identity) {
			m.fail("wheel source/epoch/segment changed")
		}
		m.sourceValid = valid
		m.source = source
		m.sourceAt = time.Now()
		m.scale = 1000
		if m.first == nil {
			m.identity = identity
		}
	}
}
func (m *mavrosWheelCalibration) onEmergency(raw []byte) {
	var e mowgli.Emergency
	if json.Unmarshal(raw, &e) != nil {
		return
	}
	m.mu.Lock()
	defer m.mu.Unlock()
	if m.result.State == "recording" && (e.ActiveEmergency || e.LatchedEmergency) {
		m.fail("emergency asserted")
	}
}
func (m *mavrosWheelCalibration) onGPS(raw []byte) {
	var g mowgli.AbsolutePose
	if json.Unmarshal(raw, &g) != nil {
		return
	}
	m.mu.Lock()
	defer m.mu.Unlock()
	m.expire()
	if m.result.State != "recording" {
		return
	}
	stamp := int64(g.Header.Stamp.Sec)*1e9 + int64(g.Header.Stamp.Nanosec)
	if stamp == m.lastFixStamp {
		return
	}
	if stamp < m.lastFixStamp {
		m.fail("RTK clock reset")
		return
	}
	if m.first != nil && stamp-m.lastFixStamp > int64(2*time.Second) {
		m.fail("RTK observation gap")
		return
	}
	m.lastFixStamp = stamp
	if !m.rtkValid() || !m.sourceValid || time.Since(m.tickAt) > time.Second || time.Since(m.qualityAt) > 2*time.Second || time.Since(m.sourceAt) > 2*time.Second || math.Abs(float64(stamp-m.lastTickStamp)) > 5e8 {
		if m.first != nil {
			m.fail("RTK/wheel observations are invalid, stale or unpaired")
		}
		return
	}
	// The provider's gps adapter preserves raw NavSatFix latitude/longitude in
	// AbsolutePose x/y. Never use the fused pose or calibrated /wheel_odom.
	point := calibrationPoint{g.Pose.Pose.Position.X, g.Pose.Pose.Position.Y, m.ticks.WheelTicksRl, m.ticks.WheelTicksRr}
	if math.IsNaN(point.lat) || math.IsNaN(point.lon) || math.IsInf(point.lat, 0) || math.IsInf(point.lon, 0) || math.Abs(point.lat) > 90 || math.Abs(point.lon) > 180 {
		m.fail("invalid RTK coordinates")
		return
	}
	if m.first == nil {
		m.first = &point
	}
	if len(m.points) >= 4096 {
		m.fail("calibration observation limit reached")
		return
	}
	m.points = append(m.points, point)
	m.last = &point
	m.fixAt = time.Now()
	m.result.Samples++
}
func rtkDistance(a, b calibrationPoint) float64 {
	dx, dy := rtkOffset(a, b)
	return math.Hypot(dx, dy)
}
func rtkOffset(a, b calibrationPoint) (float64, float64) {
	const metresPerDegree = 111319.49079327358
	dx := (b.lon - a.lon) * metresPerDegree * math.Cos((a.lat+b.lat)*math.Pi/360)
	dy := (b.lat - a.lat) * metresPerDegree
	return dx, dy
}
func straightRTKPass(points []calibrationPoint) bool {
	if len(points) < 3 {
		return false
	}
	start := points[0]
	dx, dy := rtkOffset(start, points[len(points)-1])
	distance := math.Hypot(dx, dy)
	if distance == 0 {
		return false
	}
	for _, point := range points {
		px, py := rtkOffset(start, point)
		if math.Abs(px*dy-py*dx)/distance > 0.15 {
			return false
		}
	}
	return true
}
func parameterWasApplied(updated []types.RosParameter, want types.RosParameter) bool {
	for _, p := range updated {
		if strings.TrimPrefix(p.Name, "/") == strings.TrimPrefix(want.Name, "/") && valuesEqual(p.Value, want.Value) {
			return true
		}
	}
	return false
}

// Change only the canonical scalar; preserve every existing PID/FF value and
// YAML scalar type. In particular do not populate template defaults here.
func persistMavrosTicksPerMeter(db types.IDBProvider, value float64) error {
	path, err := db.Get("system.mower.yamlConfigFile")
	if err != nil {
		return err
	}
	raw, err := os.ReadFile(string(path))
	if err != nil {
		return err
	}
	var doc yaml.Node
	if err = yaml.Unmarshal(raw, &doc); err != nil {
		return err
	}
	if len(doc.Content) != 1 {
		return fmt.Errorf("invalid robot YAML")
	}
	node := doc.Content[0]
	for _, key := range []string{"mowgli", "ros__parameters"} {
		var found *yaml.Node
		for i := 0; i+1 < len(node.Content); i += 2 {
			if node.Content[i].Value == key {
				found = node.Content[i+1]
				break
			}
		}
		if found == nil || found.Kind != yaml.MappingNode {
			return fmt.Errorf("missing %s mapping", key)
		}
		node = found
	}
	scalar := &yaml.Node{Kind: yaml.ScalarNode, Tag: "!!float", Value: formatYAMLFloat(value)}
	replaced := false
	for i := 0; i+1 < len(node.Content); i += 2 {
		if node.Content[i].Value == "ticks_per_meter" {
			node.Content[i+1].Tag = scalar.Tag
			node.Content[i+1].Value = scalar.Value
			replaced = true
			break
		}
	}
	if !replaced {
		node.Content = append(node.Content, &yaml.Node{Kind: yaml.ScalarNode, Tag: "!!str", Value: "ticks_per_meter"}, scalar)
	}
	out, err := yaml.Marshal(&doc)
	if err != nil {
		return err
	}
	if err = writePreservingPerms(string(path), out); err != nil {
		return err
	}
	return writeMavrosRuntimeConfig(db)
}
