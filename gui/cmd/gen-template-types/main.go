// Command gen-template-types regenerates gui/asserts/ros2_template_types.json
// from the ROS2 package template.
//
// The GUI image ships only asserts/, so the settings writer cannot read the
// template at runtime; this bakes the template's number types (float vs int)
// into an asset it CAN read. TestTemplateTypesFileMatchesTemplate fails in CI
// whenever the checked-in asset no longer matches the template, and names this
// command as the fix.
//
// Usage:
//
//	cd gui && go run ./cmd/gen-template-types
package main

import (
	"flag"
	"fmt"
	"os"

	"github.com/mowglinext/mowglinext/pkg/api"
)

func main() {
	template := flag.String("template", "../ros2/src/mowgli_bringup/config/mowgli_robot.yaml",
		"path to the ROS2 package template, relative to the gui module root")
	output := flag.String("out", "asserts/ros2_template_types.json",
		"path of the generated asset, relative to the gui module root")
	flag.Parse()

	if err := api.GenerateTemplateTypesAsset(*template, *output); err != nil {
		fmt.Fprintf(os.Stderr, "gen-template-types: %v\n", err)
		os.Exit(1)
	}
	fmt.Printf("wrote %s from %s\n", *output, *template)
}
