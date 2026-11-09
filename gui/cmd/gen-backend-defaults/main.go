// Command gen-backend-defaults regenerates gui/asserts/backend_defaults.json
// from the ROS2 hardware-backend overlays
// (ros2/src/mowgli_bringup/config/backends/*.yaml).
//
// The GUI image ships only asserts/, so it cannot read the overlays at
// runtime; TestBackendDefaultsAssetMatchesOverlays fails in CI whenever the
// checked-in asset no longer matches them, and names this command as the fix.
//
// Usage:
//
//	cd gui && go run ./cmd/gen-backend-defaults
package main

import (
	"flag"
	"fmt"
	"os"

	"github.com/mowglinext/mowglinext/pkg/api"
)

func main() {
	overlays := flag.String("overlays", "../ros2/src/mowgli_bringup/config/backends",
		"directory of the ROS2 backend overlays, relative to the gui module root")
	output := flag.String("out", "asserts/backend_defaults.json",
		"path of the generated asset, relative to the gui module root")
	flag.Parse()

	if err := api.GenerateBackendDefaultsAsset(*overlays, *output); err != nil {
		fmt.Fprintf(os.Stderr, "gen-backend-defaults: %v\n", err)
		os.Exit(1)
	}
	fmt.Printf("wrote %s from %s\n", *output, *overlays)
}
