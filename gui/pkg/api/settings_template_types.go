package api

import (
	"encoding/json"
	"fmt"
	"log"
	"os"

	"gopkg.in/yaml.v3"
)

// The ROS2 package template (ros2/src/mowgli_bringup/config/mowgli_robot.yaml)
// is the real source of every parameter default (Architecture Invariant 15),
// and therefore also of every parameter's intended NUMBER TYPE. The GUI needs
// that type when it rewrites the installed config: 19 template keys hold an
// integral float default and are absent from the JSON schema — tick_rate,
// imu_cal_auto_rest_sec, area_record_rate_hz, the loc_sigma_* family,
// dock_pose_x/y/yaw and the rest — and every one of them is consumed as a C++
// double, so writing "10" instead of "10.0" aborts its node exactly the way
// lidar_map_tile_size_m aborted the localizer on 2026-09-15.
//
// The GUI backend CANNOT read the template at runtime: the image is built with
// `context: ./gui` (.github/workflows/gui-docker.yml) and ships only
// /app/web, /app/mowglinext and /app/asserts, so ros2/ is outside both the
// build context and the container. That is exactly why
// GetSettingsYAMLDefaults' doc comment says the JSON schema "stands in for"
// the template.
//
// So the type table is GENERATED from the template into asserts/, the one
// directory the image does ship — the same shape as generate_ts_types.sh,
// which also reads ../ros2/src and writes a generated artifact into the gui
// tree. TestTemplateTypesFileMatchesTemplate re-derives it from the real
// template on every CI run, so the file cannot silently go stale, and no key
// name is ever hand-maintained.
//
// Regenerate with:  cd gui && go run ./cmd/gen-template-types

const (
	templateTypesAssetPath = "asserts/ros2_template_types.json"
	ros2TemplatePath       = "../ros2/src/mowgli_bringup/config/mowgli_robot.yaml"
	templateTypesComment   = "GENERATED from the ROS2 package template — do not edit by hand. " +
		"Regenerate with: cd gui && go run ./cmd/gen-template-types"
)

// templateTypesFile is the on-disk shape of the generated asset.
type templateTypesFile struct {
	Comment string            `json:"_comment"`
	Source  string            `json:"source"`
	Types   map[string]string `json:"types"`
}

// deriveTemplateNumberKinds maps every numeric parameter in a parsed template
// to its JSON-schema-style type name ("number" / "integer"). Booleans,
// strings and sequences are skipped: only numbers can be demoted by a
// marshal, and only numbers are retyped on write.
func deriveTemplateNumberKinds(templateYAML map[string]any) map[string]string {
	kinds := map[string]string{}
	for key, value := range flattenROS2YAML(templateYAML) {
		switch scalarNumberKind(value) {
		case yamlNumberFloat:
			kinds[key] = "number"
		case yamlNumberInt:
			kinds[key] = "integer"
		}
	}
	return kinds
}

// buildTemplateTypesFile parses the ROS2 template at path and returns the
// generated asset, ready to be marshalled.
func buildTemplateTypesFile(path string) (templateTypesFile, error) {
	raw, err := os.ReadFile(path)
	if err != nil {
		return templateTypesFile{}, fmt.Errorf("read ROS2 template: %w", err)
	}
	templateYAML := map[string]any{}
	if err := yaml.Unmarshal(raw, &templateYAML); err != nil {
		return templateTypesFile{}, fmt.Errorf("parse ROS2 template: %w", err)
	}
	kinds := deriveTemplateNumberKinds(templateYAML)
	if len(kinds) == 0 {
		return templateTypesFile{}, fmt.Errorf("ROS2 template %s yielded no numeric parameters", path)
	}
	return templateTypesFile{
		Comment: templateTypesComment,
		Source:  "ros2/src/mowgli_bringup/config/mowgli_robot.yaml",
		Types:   kinds,
	}, nil
}

// marshalTemplateTypesFile renders the asset exactly as the generator writes
// it, so the parity test can compare bytes. encoding/json sorts map keys, so
// the output is stable.
func marshalTemplateTypesFile(file templateTypesFile) ([]byte, error) {
	out, err := json.MarshalIndent(file, "", "  ")
	if err != nil {
		return nil, err
	}
	return append(out, '\n'), nil
}

// GenerateTemplateTypesAsset regenerates asserts/ros2_template_types.json from
// the ROS2 package template. Paths are relative to the gui module root, which
// is where the sibling generator scripts are run from.
func GenerateTemplateTypesAsset(templatePath, outputPath string) error {
	file, err := buildTemplateTypesFile(templatePath)
	if err != nil {
		return err
	}
	out, err := marshalTemplateTypesFile(file)
	if err != nil {
		return err
	}
	return os.WriteFile(outputPath, out, 0o644)
}

// loadTemplateNumberKinds reads the generated asset and returns the template's
// number types. A missing or unreadable asset is NOT fatal: the write falls
// through to the on-disk types, which is the pre-existing behaviour. It is
// logged because the asset missing at runtime means the image was built wrong.
func loadTemplateNumberKinds() map[string]yamlNumberKind {
	kinds := map[string]yamlNumberKind{}
	raw, err := os.ReadFile(templateTypesAssetPath)
	if err != nil {
		log.Printf("settings: %s unavailable (%v); YAML number types fall back to the on-disk document", templateTypesAssetPath, err)
		return kinds
	}
	var file templateTypesFile
	if err := json.Unmarshal(raw, &file); err != nil {
		log.Printf("settings: %s is not valid JSON (%v); YAML number types fall back to the on-disk document", templateTypesAssetPath, err)
		return kinds
	}
	for key, typeName := range file.Types {
		switch typeName {
		case "number":
			kinds[key] = yamlNumberFloat
		case "integer":
			kinds[key] = yamlNumberInt
		}
	}
	return kinds
}
