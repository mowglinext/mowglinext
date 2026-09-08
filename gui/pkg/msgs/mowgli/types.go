package mowgli

// Types in this file are GUI-internal (not generated from ROS2 .msg files).
// For ROS2 message types, see types_generated.go.

// Map is the internal map structure sent to the frontend via the virtual "map" topic.
// It is assembled by pollMap() from get_mowing_area service calls.
type Map struct {
	WorkingAreaIndices []uint32 `json:"working_area_indices"`
	MapWidth        float64   `json:"map_width"`
	MapHeight       float64   `json:"map_height"`
	MapCenterX      float64   `json:"map_center_x"`
	MapCenterY      float64   `json:"map_center_y"`
	NavigationAreas []MapArea `json:"navigation_areas"`
	WorkingArea     []MapArea `json:"working_area"`
	// Nil means no pose received; an explicitly received zero is a valid dock.
	DockX       *float64 `json:"dock_x,omitempty"`
	DockY       *float64 `json:"dock_y,omitempty"`
	DockHeading *float64 `json:"dock_heading,omitempty"`
}

// DockingSensor - placeholder, may not exist in ROS2 mowgli
type DockingSensor struct {
	DockPresent  bool    `json:"dock_present"`
	DockDistance float32 `json:"dock_distance"`
}
