package providers

import (
	"net"
	"os"
	"strconv"
	"strings"

	"github.com/docker/distribution/uuid"
	"github.com/mowglinext/mowglinext/pkg/buildinfo"
	"github.com/mowglinext/mowglinext/pkg/types"
	"gopkg.in/yaml.v3"
)

// FleetAPIVersion is bumped whenever the robot-to-robot fleet contract
// (identity, register, snapshot topics, proxied commands) changes
// incompatibly. Peers advertising a different value are still listed but the
// GUI flags them so the operator updates the older robot.
const FleetAPIVersion = 1

// DefaultRobotName mirrors the template default of `robot_name` in
// ros2/src/mowgli_bringup/config/mowgli_robot.yaml and the GUI schema default.
const DefaultRobotName = "mowgli"

const (
	robotIDKey     = "fleet.robot_id"
	defaultAPIPort = 4006
)

// RobotIdentity is what a robot tells its peers about itself.
type RobotIdentity struct {
	ID         string  `json:"id"`
	Name       string  `json:"name"`
	Version    string  `json:"version,omitempty"`
	Revision   string  `json:"revision,omitempty"`
	DatumLat   float64 `json:"datum_lat"`
	DatumLon   float64 `json:"datum_lon"`
	APIVersion int     `json:"api_version"`
}

// EnsureRobotID returns the stable per-robot UUID, generating and persisting
// it on first use. The id is the fleet key: renaming a robot keeps its peers'
// registrations intact.
func EnsureRobotID(db types.IDBProvider) (string, error) {
	if v, err := db.Get(robotIDKey); err == nil && len(v) > 0 {
		return string(v), nil
	}
	id := uuid.Generate().String()
	if err := db.Set(robotIDKey, []byte(id)); err != nil {
		return "", err
	}
	return id, nil
}

// robotYamlScalars reads the flat `mowgli.ros__parameters` map of the
// installed mowgli_robot.yaml. A missing or unreadable file yields an empty
// map so callers fall back to defaults instead of failing.
func robotYamlScalars(db types.IDBProvider) map[string]any {
	path, err := db.Get("system.mower.yamlConfigFile")
	if err != nil {
		return map[string]any{}
	}
	data, err := os.ReadFile(string(path))
	if err != nil {
		return map[string]any{}
	}
	var doc map[string]any
	if err := yaml.Unmarshal(data, &doc); err != nil {
		return map[string]any{}
	}
	node, _ := doc["mowgli"].(map[string]any)
	params, _ := node["ros__parameters"].(map[string]any)
	if params == nil {
		return map[string]any{}
	}
	return params
}

// ReadRobotName returns the operator-chosen robot name, or DefaultRobotName
// when the sparse installed yaml does not override it.
func ReadRobotName(db types.IDBProvider) string {
	if v, ok := robotYamlScalars(db)["robot_name"]; ok {
		if s := strings.TrimSpace(stringOf(v)); s != "" {
			return s
		}
	}
	return DefaultRobotName
}

// ReadDatum returns datum_lat/datum_lon from the installed yaml (0/0 = unset).
func ReadDatum(db types.IDBProvider) (lat, lon float64) {
	params := robotYamlScalars(db)
	lat, _ = floatOf(params["datum_lat"])
	lon, _ = floatOf(params["datum_lon"])
	return lat, lon
}

// CurrentIdentity assembles this robot's identity from the DB (id), the
// yaml (name, datum) and the binary build info (version).
func CurrentIdentity(db types.IDBProvider) (RobotIdentity, error) {
	id, err := EnsureRobotID(db)
	if err != nil {
		return RobotIdentity{}, err
	}
	lat, lon := ReadDatum(db)
	info := buildinfo.Current()
	return RobotIdentity{
		ID:         id,
		Name:       ReadRobotName(db),
		Version:    info.Version,
		Revision:   info.Revision,
		DatumLat:   lat,
		DatumLon:   lon,
		APIVersion: FleetAPIVersion,
	}, nil
}

// LocalAPIPort returns the TCP port the GUI API listens on (system.api.addr),
// which a peer combines with our source IP when we register with it.
func LocalAPIPort(db types.IDBProvider) int {
	addr, err := db.Get("system.api.addr")
	if err != nil {
		return defaultAPIPort
	}
	_, portStr, err := net.SplitHostPort(string(addr))
	if err != nil {
		return defaultAPIPort
	}
	port, err := strconv.Atoi(portStr)
	if err != nil || port <= 0 {
		return defaultAPIPort
	}
	return port
}

func stringOf(v any) string {
	if s, ok := v.(string); ok {
		return s
	}
	return fmtAny(v)
}

func fmtAny(v any) string {
	switch t := v.(type) {
	case int:
		return strconv.Itoa(t)
	case int64:
		return strconv.FormatInt(t, 10)
	case float64:
		return strconv.FormatFloat(t, 'f', -1, 64)
	case bool:
		return strconv.FormatBool(t)
	default:
		return ""
	}
}

func floatOf(v any) (float64, bool) {
	switch t := v.(type) {
	case float64:
		return t, true
	case float32:
		return float64(t), true
	case int:
		return float64(t), true
	case int64:
		return float64(t), true
	default:
		return 0, false
	}
}
