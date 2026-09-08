package providers

import (
	"encoding/json"
	"testing"

	"github.com/mowglinext/mowglinext/pkg/msgs/mowgli"
)

func TestMapDockPresenceAndSnapshot(t *testing.T) {
	r := &RosProvider{}
	var unknown mowgli.Map
	r.addDockPose(&unknown)
	encoded, err := json.Marshal(unknown)
	if err != nil {
		t.Fatal(err)
	}
	var fields map[string]json.RawMessage
	if err := json.Unmarshal(encoded, &fields); err != nil {
		t.Fatal(err)
	}
	for _, key := range []string{"dock_x", "dock_y", "dock_heading"} {
		if _, exists := fields[key]; exists {
			t.Fatalf("unknown dock emitted %s", key)
		}
	}
	// Origin is valid once supplied by the map server, including zero heading.
	r.dockPoseSet = true
	var origin mowgli.Map
	r.addDockPose(&origin)
	encoded, err = json.Marshal(origin)
	if err != nil {
		t.Fatal(err)
	}
	if err := json.Unmarshal(encoded, &fields); err != nil {
		t.Fatal(err)
	}
	for _, key := range []string{"dock_x", "dock_y", "dock_heading"} {
		if string(fields[key]) != "0" {
			t.Fatalf("origin %s = %s", key, fields[key])
		}
	}
	r.dockX, r.dockY, r.dockHeading = -12.5, 8, 1.5
	var moved mowgli.Map
	r.addDockPose(&moved)
	if *origin.DockX != 0 || *origin.DockY != 0 || *origin.DockHeading != 0 {
		t.Fatal("previous map aliases mutable dock cache")
	}
	if *moved.DockX != -12.5 || *moved.DockY != 8 || *moved.DockHeading != 1.5 {
		t.Fatalf("updated dock not included: %+v", moved)
	}
}
