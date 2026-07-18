package foxglove

import (
	"encoding/hex"
	"encoding/json"
	"testing"
)

// A REAL rmw-encoded (CycloneDDS, XCDR1 little-endian) wire frame for one
// mowgli_interfaces/msg/ObstacleArray carrying a single PERSISTENT obstacle
// (id=10, status=1, observation_count=51, 20-point polygon), captured off the
// live robot via `ros2 bag record --storage sqlite3 /obstacle_tracker/obstacles`.
//
// ObstacleArray is the ONLY GUI-facing message that nests a dynamic array
// (Polygon.points) inside a message (TrackedObstacle) inside another dynamic
// array (obstacles) — followed by an 8-byte-aligned float64 (radius) and a
// trailing uint8 (status). If the CDR reader mishandles alignment after the
// nested array, `status` is read from the wrong offset and the frontend filter
// `o.status === 1` drops every promoted obstacle — the "promoted obstacles
// never show in the GUI" bug. This pins the whole publish→JSON path.
const obstacleArrayWireHex = "00010000bca25b6abf308e2d040000006d617000010000000a00000014000000dd4dbfbf784797400000000048c7bebf4965944000000000c18ebdbf81c28e4000000000571cb2bfa9d78b400000000056fc9bbfe37c8b4000000000a14fa0bed992834000000000bfc80bbedd7c86400000000022ca41bd865289400000000091de1bbd54ce8e400000000056e3e6bea6fea94000000000c0fbebbe49d2ac4000000000e514f0be018daf4000000000f85411bf7363b24000000000dca628bf5dedaf4000000000af7b29bf0a82b240000000006af341bf1f8fb2400000000091765abf0088b24000000000f4f473bf3bd2af400000000034a375bff22dad4000000000551287bf45e8ac4000000000000000008d3de1083bb4e8bf270ad7586ce713400000000000000000700d500314d4ec3fca9b5b6ac9f84b233300000001000000"

// obstacleArraySchema mirrors the concatenated definition foxglove_bridge
// advertises for the topic (root block first, referenced types after "=" rules).
const obstacleArraySchema = `std_msgs/Header header
TrackedObstacle[] obstacles
================================================================================
MSG: std_msgs/Header
builtin_interfaces/Time stamp
string frame_id
================================================================================
MSG: builtin_interfaces/Time
int32 sec
uint32 nanosec
================================================================================
MSG: mowgli_interfaces/TrackedObstacle
uint32 id
geometry_msgs/Polygon polygon
geometry_msgs/Point centroid
float64 radius
builtin_interfaces/Time first_seen
uint32 observation_count
uint8 status
================================================================================
MSG: geometry_msgs/Polygon
geometry_msgs/Point32[] points
================================================================================
MSG: geometry_msgs/Point32
float32 x
float32 y
float32 z
================================================================================
MSG: geometry_msgs/Point
float64 x
float64 y
float64 z`

func TestObstacleArrayPersistentStatusDeserializes(t *testing.T) {
	schema := mustParseSchema(t, obstacleArraySchema)

	data, err := hex.DecodeString(obstacleArrayWireHex)
	if err != nil {
		t.Fatalf("hex decode: %v", err)
	}

	msg := mustDeserialize(t, data, schema)

	// Marshal exactly as the Go backend does before forwarding over the
	// WebSocket, then decode into the shape the frontend consumes. This is the
	// real path: any casing/type drift that would defeat `o.status === 1` in
	// the browser shows up here.
	jsonBytes, err := json.Marshal(msg)
	if err != nil {
		t.Fatalf("json.Marshal: %v", err)
	}

	var parsed struct {
		Obstacles []struct {
			ID               int     `json:"id"`
			Radius           float64 `json:"radius"`
			ObservationCount int     `json:"observation_count"`
			Status           int     `json:"status"`
			Polygon          struct {
				Points []struct {
					X float64 `json:"x"`
					Y float64 `json:"y"`
				} `json:"points"`
			} `json:"polygon"`
		} `json:"obstacles"`
	}
	if err := json.Unmarshal(jsonBytes, &parsed); err != nil {
		t.Fatalf("json.Unmarshal: %v\npayload=%s", err, jsonBytes)
	}

	if len(parsed.Obstacles) != 1 {
		t.Fatalf("obstacles: got %d, want 1\npayload=%s", len(parsed.Obstacles), jsonBytes)
	}
	o := parsed.Obstacles[0]
	if o.ID != 10 {
		t.Errorf("id: got %d, want 10", o.ID)
	}
	if len(o.Polygon.Points) != 20 {
		t.Errorf("polygon points: got %d, want 20", len(o.Polygon.Points))
	}
	if o.ObservationCount != 51 {
		t.Errorf("observation_count: got %d, want 51", o.ObservationCount)
	}
	if o.Radius < 0.9 || o.Radius > 0.902 {
		t.Errorf("radius: got %v, want ~0.9008", o.Radius)
	}
	// The crux: a promoted obstacle MUST deserialize with status==1 (PERSISTENT)
	// or the frontend `filter(o => o.status === 1)` hides it.
	if o.Status != 1 {
		t.Errorf("status: got %d, want 1 (PERSISTENT) — frontend would hide this obstacle", o.Status)
	}
	// First polygon vertex sanity-check (from the live echo): x≈-1.4946, y≈4.7275.
	if len(o.Polygon.Points) > 0 {
		p := o.Polygon.Points[0]
		if p.X < -1.5 || p.X > -1.49 || p.Y < 4.72 || p.Y > 4.73 {
			t.Errorf("point[0]: got (%v,%v), want ~(-1.4946,4.7275)", p.X, p.Y)
		}
	}
}
