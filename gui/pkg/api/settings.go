package api

import (
	"encoding/json"
	"fmt"
	"log"
	"sync"
	"syscall"
	"time"

	"os"
	"path/filepath"
	"strconv"
	"strings"

	"github.com/cedbossneo/mowglinext/pkg/types"
	"github.com/gin-gonic/gin"
	"github.com/joho/godotenv"
	"gopkg.in/yaml.v3"
)

// writePreservingPerms writes content to path, preserving the existing
// file's mode and uid/gid when the file already exists. When the file
// is being created for the first time, it is written world-writable
// (0664) so other processes (ROS containers running as a different
// user) can still update it. The previous behavior would silently
// rewrite the file as owned by the GUI process with mode 0644, which
// locked out the calibration service and the line-splice writers in
// hardware_bridge / map_server.
func writePreservingPerms(path string, content []byte) error {
	mode := os.FileMode(0664)
	var uid, gid int = -1, -1
	if info, err := os.Stat(path); err == nil {
		mode = info.Mode().Perm()
		if stat, ok := info.Sys().(*syscall.Stat_t); ok {
			uid = int(stat.Uid)
			gid = int(stat.Gid)
		}
	}
	if err := os.WriteFile(path, content, mode); err != nil {
		return err
	}
	// os.WriteFile only applies the mode on creation; force it after
	// every write so an externally-changed mode does not stick.
	if err := os.Chmod(path, mode); err != nil {
		return err
	}
	if uid >= 0 && gid >= 0 {
		// Best-effort: chown can fail when the GUI process is not root
		// (e.g. running directly on the host). In that case the file
		// was opened in-place so ownership is already preserved.
		_ = os.Chown(path, uid, gid)
	}
	return nil
}

func SettingsRoutes(r *gin.RouterGroup, dbProvider types.IDBProvider) {
	GetSettings(r, dbProvider)
	PostSettings(r, dbProvider)
	GetSettingsSchema(r, dbProvider)
	GetSettingsYAML(r, dbProvider)
	PostSettingsYAML(r, dbProvider)
	GetSettingsStatus(r, dbProvider)
	PostSettingsStatus(r, dbProvider)
}

// GetSettingsStatus returns whether onboarding has been completed.
// Used by the frontend to decide whether to show the onboarding wizard.
//
// @Summary get settings onboarding status
// @Description returns whether onboarding has been completed
// @Tags settings
// @Produce json
// @Success 200 {object} SettingsStatusResponse
// @Router /settings/status [get]
func GetSettingsStatus(r *gin.RouterGroup, dbProvider types.IDBProvider) gin.IRoutes {
	return r.GET("/settings/status", func(c *gin.Context) {
		val, err := dbProvider.Get("onboarding.completed")
		completed := err == nil && string(val) == "true"
		c.JSON(200, gin.H{"onboarding_completed": completed})
	})
}

// PostSettingsStatus marks onboarding as completed.
//
// @Summary mark onboarding as completed
// @Description marks onboarding as completed so the wizard is not shown again
// @Tags settings
// @Produce json
// @Success 200 {object} OkResponse
// @Failure 500 {object} ErrorResponse
// @Router /settings/status [post]
func PostSettingsStatus(r *gin.RouterGroup, dbProvider types.IDBProvider) gin.IRoutes {
	return r.POST("/settings/status", func(c *gin.Context) {
		if err := dbProvider.Set("onboarding.completed", []byte("true")); err != nil {
			c.JSON(500, ErrorResponse{Error: err.Error()})
			return
		}
		c.JSON(200, OkResponse{})
	})
}

func extractDefaults(schema map[string]any, defaults map[string]any) {
	if props, ok := schema["properties"].(map[string]any); ok {
		for key, prop := range props {
			if propMap, ok := prop.(map[string]any); ok {
				if def, hasDef := propMap["default"]; hasDef {
					defaults[key] = def
				}
				extractDefaults(propMap, defaults)
			}
		}
	}
	if allOf, ok := schema["allOf"].([]any); ok {
		for _, cond := range allOf {
			if condMap, ok := cond.(map[string]any); ok {
				if thenBlock, ok := condMap["then"].(map[string]any); ok {
					extractDefaults(thenBlock, defaults)
				}
				if elseBlock, ok := condMap["else"].(map[string]any); ok {
					extractDefaults(elseBlock, defaults)
				}
			}
		}
	}
}

func extractAllKeys(schema map[string]any, keys map[string]bool) {
	if props, ok := schema["properties"].(map[string]any); ok {
		for key, prop := range props {
			if propMap, ok := prop.(map[string]any); ok {
				propType, _ := propMap["type"].(string)
				_, hasEnvVar := propMap["x-environment-variable"]
				if hasEnvVar || (propType != "" && propType != "object") {
					keys[key] = true
				}
				extractAllKeys(propMap, keys)
			}
		}
	}
	if allOf, ok := schema["allOf"].([]any); ok {
		for _, cond := range allOf {
			if condMap, ok := cond.(map[string]any); ok {
				if thenBlock, ok := condMap["then"].(map[string]any); ok {
					extractAllKeys(thenBlock, keys)
				}
				if elseBlock, ok := condMap["else"].(map[string]any); ok {
					extractAllKeys(elseBlock, keys)
				}
			}
		}
	}
}

func extractKeyTypes(schema map[string]any, types_ map[string]string) {
	if props, ok := schema["properties"].(map[string]any); ok {
		for key, prop := range props {
			if propMap, ok := prop.(map[string]any); ok {
				propType, _ := propMap["type"].(string)
				if propType != "" && propType != "object" {
					types_[key] = propType
				}
				extractKeyTypes(propMap, types_)
			}
		}
	}
	if allOf, ok := schema["allOf"].([]any); ok {
		for _, cond := range allOf {
			if condMap, ok := cond.(map[string]any); ok {
				if thenBlock, ok := condMap["then"].(map[string]any); ok {
					extractKeyTypes(thenBlock, types_)
				}
				if elseBlock, ok := condMap["else"].(map[string]any); ok {
					extractKeyTypes(elseBlock, types_)
				}
			}
		}
	}
}

// extractNodeMappings builds a map of paramKey -> ROS2 node name from
// x-yaml-node extensions in the schema. Keys without x-yaml-node default to "mowgli".
func extractNodeMappings(schema map[string]any) map[string]string {
	mappings := map[string]string{}
	allKeys := map[string]bool{}
	extractAllKeys(schema, allKeys)

	var walk func(s map[string]any)
	walk = func(s map[string]any) {
		if props, ok := s["properties"].(map[string]any); ok {
			for key, prop := range props {
				if propMap, ok := prop.(map[string]any); ok {
					if node, ok := propMap["x-yaml-node"].(string); ok {
						mappings[key] = node
					}
					walk(propMap)
				}
			}
		}
		if allOf, ok := s["allOf"].([]any); ok {
			for _, cond := range allOf {
				if condMap, ok := cond.(map[string]any); ok {
					if thenBlock, ok := condMap["then"].(map[string]any); ok {
						walk(thenBlock)
					}
					if elseBlock, ok := condMap["else"].(map[string]any); ok {
						walk(elseBlock)
					}
				}
			}
		}
	}
	walk(schema)

	// Default unmapped keys to "mowgli"
	for key := range allKeys {
		if _, exists := mappings[key]; !exists {
			mappings[key] = "mowgli"
		}
	}
	return mappings
}

// flattenROS2YAML reads nested ROS2 YAML (node: ros__parameters: {k: v}) and
// returns a flat map of all parameter key-value pairs across all nodes.
// It also duplicates datum_lat/datum_lon from mowgli node to keep them in sync.
func flattenROS2YAML(yamlData map[string]any) map[string]any {
	flat := map[string]any{}
	for _, nodeData := range yamlData {
		nodeMap, ok := nodeData.(map[string]any)
		if !ok {
			continue
		}
		rosParams, ok := nodeMap["ros__parameters"].(map[string]any)
		if !ok {
			continue
		}
		for k, v := range rosParams {
			flat[k] = v
		}
	}
	return flat
}

// nestToROS2YAML takes a flat param map and node mappings, and produces
// nested ROS2 YAML structure. It preserves any existing YAML content that
// is not covered by the schema.
func nestToROS2YAML(flat map[string]any, nodeMappings map[string]string, existingYAML map[string]any) map[string]any {
	result := map[string]any{}

	// Preserve existing top-level structure
	for nodeName, nodeData := range existingYAML {
		if nodeMap, ok := nodeData.(map[string]any); ok {
			cloned := map[string]any{}
			for k, v := range nodeMap {
				cloned[k] = v
			}
			result[nodeName] = cloned
		}
	}

	// Group flat params by node
	nodeParams := map[string]map[string]any{}
	for key, value := range flat {
		node := "mowgli" // default
		if n, ok := nodeMappings[key]; ok {
			node = n
		}
		if nodeParams[node] == nil {
			nodeParams[node] = map[string]any{}
		}
		nodeParams[node][key] = value
	}

	// Merge into result, preserving existing ros__parameters that aren't in flat
	for nodeName, params := range nodeParams {
		nodeMap, ok := result[nodeName].(map[string]any)
		if !ok {
			nodeMap = map[string]any{}
			result[nodeName] = nodeMap
		}
		rosParams, ok := nodeMap["ros__parameters"].(map[string]any)
		if !ok {
			rosParams = map[string]any{}
		}
		for k, v := range params {
			rosParams[k] = v
		}
		nodeMap["ros__parameters"] = rosParams
	}

	return result
}

func coerceValue(value string, schemaType string) any {
	switch schemaType {
	case "boolean":
		return strings.EqualFold(value, "true")
	case "number":
		if f, err := strconv.ParseFloat(value, 64); err == nil {
			return f
		}
	case "integer":
		if i, err := strconv.ParseInt(value, 10, 64); err == nil {
			return i
		}
	}
	return value
}

// applyMowgliOverlay modifies the settings schema to add Mowgli-specific
// options: adds "Mowgli" to the OM_MOWER enum, moves OM_HARDWARE_VERSION and
// OM_MOWER_ESC_TYPE behind conditional allOf rules, and adds OM_NO_COMMS for
// Mowgli boards.
func applyMowgliOverlay(schema map[string]any) map[string]any {
	props, ok := schema["properties"].(map[string]any)
	if !ok {
		return schema
	}
	hw, ok := props["important_settings"].(map[string]any)
	if !ok {
		return schema
	}
	hwProps, ok := hw["properties"].(map[string]any)
	if !ok {
		return schema
	}

	// Add "Mowgli" to OM_MOWER enum if not already present.
	if omMower, ok := hwProps["OM_MOWER"].(map[string]any); ok {
		if enum, ok := omMower["enum"].([]any); ok {
			found := false
			for _, v := range enum {
				if v == "Mowgli" {
					found = true
					break
				}
			}
			if !found {
				omMower["enum"] = append(enum, "Mowgli")
			}
		}
	}

	// Extract OM_HARDWARE_VERSION and OM_MOWER_ESC_TYPE from base properties.
	hwVersion := hwProps["OM_HARDWARE_VERSION"]
	escType := hwProps["OM_MOWER_ESC_TYPE"]
	delete(hwProps, "OM_HARDWARE_VERSION")
	delete(hwProps, "OM_MOWER_ESC_TYPE")

	// Build conditional allOf rules.
	nonMowgliThenProps := map[string]any{}
	if hwVersion != nil {
		nonMowgliThenProps["OM_HARDWARE_VERSION"] = hwVersion
	}
	if escType != nil {
		nonMowgliThenProps["OM_MOWER_ESC_TYPE"] = escType
	}

	nonMowgliCond := map[string]any{
		"if": map[string]any{
			"properties": map[string]any{
				"OM_MOWER": map[string]any{
					"not": map[string]any{"const": "Mowgli"},
				},
			},
		},
		"then": map[string]any{
			"properties": nonMowgliThenProps,
		},
	}

	mowgliCond := map[string]any{
		"if": map[string]any{
			"properties": map[string]any{
				"OM_MOWER": map[string]any{"const": "Mowgli"},
			},
		},
		"then": map[string]any{
			"properties": map[string]any{
				"OM_NO_COMMS": map[string]any{
					"type":    "boolean",
					"title":   "No Comms Mode",
					"default": true,
				},
			},
		},
	}

	hw["allOf"] = []any{nonMowgliCond, mowgliCond}

	return schema
}

func getSchema(dbProvider types.IDBProvider) (map[string]any, error) {
	schemaCacheMu.RLock()
	if schemaCache != nil && time.Since(schemaCacheTime) < schemaCacheTTL {
		cached := schemaCache
		schemaCacheMu.RUnlock()
		return cached, nil
	}
	schemaCacheMu.RUnlock()

	// Load local Mowgli schema (no upstream fetch)
	var schema map[string]any
	localFile, err := os.ReadFile("asserts/mower_config.schema.json")
	if err != nil {
		return nil, fmt.Errorf("failed to read local schema: %w", err)
	}
	if err := json.Unmarshal(localFile, &schema); err != nil {
		return nil, fmt.Errorf("invalid local schema JSON: %w", err)
	}

	schemaCacheMu.Lock()
	schemaCache = schema
	schemaCacheTime = time.Now()
	schemaCacheMu.Unlock()

	return schema, nil
}

// ============================================================================
// Legacy shell-config endpoints (kept for backward compatibility)
// ============================================================================

// PostSettings saves settings to the mower_config.sh file.
//
// @Summary saves the settings to the mower_config.sh file
// @Description saves the settings to the mower_config.sh file
// @Tags settings
// @Accept json
// @Produce json
// @Param settings body map[string]interface{} true "settings key-value map"
// @Success 200 {object} OkResponse
// @Failure 500 {object} ErrorResponse
// @Router /settings [post]
func PostSettings(r *gin.RouterGroup, dbProvider types.IDBProvider) gin.IRoutes {
	return r.POST("/settings", func(c *gin.Context) {
		var settingsPayload map[string]any
		err := c.BindJSON(&settingsPayload)
		if err != nil {
			c.JSON(500, ErrorResponse{Error: err.Error()})
			return
		}
		mowerConfigFile, err := dbProvider.Get("system.mower.configFile")
		if err != nil {
			c.JSON(500, ErrorResponse{Error: err.Error()})
			return
		}
		var settings = map[string]any{}
		configFileContent, err := os.ReadFile(string(mowerConfigFile))
		if err == nil {
			parse, err := godotenv.Parse(strings.NewReader(string(configFileContent)))
			if err == nil {
				for s, s2 := range parse {
					settings[s] = s2
				}
			}
		}

		schema, err := getSchema(dbProvider)
		defaults := map[string]any{}
		knownKeys := map[string]bool{}
		if err == nil {
			extractDefaults(schema, defaults)
			extractAllKeys(schema, knownKeys)
			for key, value := range defaults {
				if _, exists := settings[key]; !exists {
					settings[key] = value
				}
			}
		}

		customEnvVars := map[string]string{}
		for key, value := range settings {
			if !knownKeys[key] {
				if key != "custom_environment" {
					customEnvVars[key] = fmt.Sprintf("%v", value)
				}
				delete(settings, key)
			}
		}
		if customEnvObj, ok := settingsPayload["custom_environment"]; ok {
			customEnvVars = map[string]string{}
			if customEnv, ok := customEnvObj.(map[string]any); ok {
				for k, v := range customEnv {
					customEnvVars[k] = fmt.Sprintf("%v", v)
				}
			}
			delete(settingsPayload, "custom_environment")
		}
		for key, value := range settingsPayload {
			settings[key] = value
		}
		for k, v := range customEnvVars {
			settings[k] = v
		}

		var fileContent string
		for key, value := range settings {
			if key == "custom_environment" {
				continue
			}
			if value == true {
				value = "True"
			}
			if value == false {
				value = "False"
			}
			fileContent += "export " + key + "=" + fmt.Sprintf("%#v", value) + "\n"
		}
		if err = os.MkdirAll(filepath.Dir(string(mowerConfigFile)), 0755); err != nil {
			c.JSON(500, ErrorResponse{Error: err.Error()})
			return
		}
		err = writePreservingPerms(string(mowerConfigFile), []byte(fileContent))
		if err != nil {
			c.JSON(500, ErrorResponse{Error: err.Error()})
			return
		}
		c.JSON(200, OkResponse{})
	})
}

// GetSettings returns a JSON object with the settings.
//
// @Summary returns a JSON object with the settings
// @Description returns a JSON object with the settings
// @Tags settings
// @Produce json
// @Success 200 {object} GetSettingsResponse
// @Failure 500 {object} ErrorResponse
// @Router /settings [get]
func GetSettings(r *gin.RouterGroup, dbProvider types.IDBProvider) gin.IRoutes {
	return r.GET("/settings", func(c *gin.Context) {
		mowerConfigFilePath, err := dbProvider.Get("system.mower.configFile")
		if err != nil {
			c.JSON(500, ErrorResponse{Error: err.Error()})
			return
		}
		file, err := os.ReadFile(string(mowerConfigFilePath))
		if err != nil {
			if os.IsNotExist(err) {
				c.JSON(200, GetSettingsResponse{Settings: map[string]any{}})
				return
			}
			c.JSON(500, ErrorResponse{Error: err.Error()})
			return
		}
		settings, err := godotenv.Parse(strings.NewReader(string(file)))
		if err != nil {
			c.JSON(500, ErrorResponse{Error: err.Error()})
			return
		}
		schema, _ := getSchema(dbProvider)
		knownKeys := map[string]bool{}
		keyTypes := map[string]string{}
		if schema != nil {
			extractAllKeys(schema, knownKeys)
			extractKeyTypes(schema, keyTypes)
		}

		finalSettings := map[string]any{}
		customEnv := map[string]string{}
		for k, v := range settings {
			if knownKeys[k] || k == "custom_environment" {
				if t, ok := keyTypes[k]; ok {
					finalSettings[k] = coerceValue(v, t)
				} else {
					finalSettings[k] = v
				}
			} else {
				customEnv[k] = v
			}
		}
		if len(customEnv) > 0 {
			finalSettings["custom_environment"] = customEnv
		}

		c.JSON(200, GetSettingsResponse{Settings: finalSettings})
	})
}

// ============================================================================
// Schema endpoint
// ============================================================================

var (
	schemaCache     map[string]any
	schemaCacheMu   sync.RWMutex
	schemaCacheTime time.Time
	schemaCacheTTL  = 1 * time.Hour
)


// GetSettingsSchema returns the mower config JSON Schema.
//
// @Summary returns the mower config JSON Schema
// @Description returns the JSON Schema for mower configuration parameters
// @Tags settings
// @Produce json
// @Success 200 {object} map[string]interface{}
// @Failure 500 {object} ErrorResponse
// @Router /settings/schema [get]
func GetSettingsSchema(r *gin.RouterGroup, dbProvider types.IDBProvider) gin.IRoutes {
	return r.GET("/settings/schema", func(c *gin.Context) {
		schema, err := getSchema(dbProvider)
		if err != nil {
			c.JSON(500, ErrorResponse{Error: err.Error()})
			return
		}
		c.JSON(200, schema)
	})
}

// ============================================================================
// YAML settings endpoints — primary for Mowgli ROS2
// ============================================================================

// GetSettingsYAML reads mowgli_robot.yaml, flattens the nested ROS2 YAML,
// and returns a flat key-value JSON object for the settings form.
//
// @Summary returns the current YAML mower configuration
// @Description returns the current YAML mower configuration values as a flat key-value map
// @Tags settings
// @Produce json
// @Success 200 {object} map[string]interface{}
// @Failure 500 {object} ErrorResponse
// @Router /settings/yaml [get]
func GetSettingsYAML(r *gin.RouterGroup, dbProvider types.IDBProvider) gin.IRoutes {
	return r.GET("/settings/yaml", func(c *gin.Context) {
		configFilePath, err := dbProvider.Get("system.mower.yamlConfigFile")
		if err != nil {
			c.JSON(500, ErrorResponse{Error: err.Error()})
			return
		}
		file, err := os.ReadFile(string(configFilePath))
		if err != nil {
			if os.IsNotExist(err) {
				// Return schema defaults when file doesn't exist
				schema, schemaErr := getSchema(dbProvider)
				if schemaErr != nil {
					c.JSON(200, map[string]any{})
					return
				}
				defaults := map[string]any{}
				extractDefaults(schema, defaults)
				c.JSON(200, defaults)
				return
			}
			c.JSON(500, ErrorResponse{Error: err.Error()})
			return
		}

		var yamlData map[string]any
		if err := yaml.Unmarshal(file, &yamlData); err != nil {
			c.JSON(500, ErrorResponse{Error: "invalid YAML: " + err.Error()})
			return
		}

		flat := flattenROS2YAML(yamlData)

		// Merge schema defaults for missing keys
		schema, err := getSchema(dbProvider)
		if err == nil {
			defaults := map[string]any{}
			extractDefaults(schema, defaults)
			for key, value := range defaults {
				if _, exists := flat[key]; !exists {
					flat[key] = value
				}
			}
		}

		c.JSON(200, flat)
	})
}

// PostSettingsYAML receives flat key-value settings from the form,
// nests them into proper ROS2 YAML structure, and writes mowgli_robot.yaml.
//
// @Summary saves the mower configuration as YAML
// @Description saves the mower configuration as YAML to mowgli_robot.yaml
// @Tags settings
// @Accept json
// @Produce json
// @Param settings body map[string]interface{} true "flat key-value settings map"
// @Success 200 {object} OkResponse
// @Failure 400 {object} ErrorResponse
// @Failure 500 {object} ErrorResponse
// @Router /settings/yaml [post]
func PostSettingsYAML(r *gin.RouterGroup, dbProvider types.IDBProvider) gin.IRoutes {
	return r.POST("/settings/yaml", func(c *gin.Context) {
		var payload map[string]any
		if err := c.BindJSON(&payload); err != nil {
			c.JSON(400, ErrorResponse{Error: err.Error()})
			return
		}

		configFilePath, err := dbProvider.Get("system.mower.yamlConfigFile")
		if err != nil {
			c.JSON(500, ErrorResponse{Error: err.Error()})
			return
		}

		// Read existing YAML to preserve unknown parameters
		existingYAML := map[string]any{}
		file, err := os.ReadFile(string(configFilePath))
		if err == nil {
			_ = yaml.Unmarshal(file, &existingYAML)
		}

		// Flatten existing to get current values
		existing := flattenROS2YAML(existingYAML)

		// Merge schema defaults
		schema, err := getSchema(dbProvider)
		nodeMappings := map[string]string{}
		if err == nil {
			defaults := map[string]any{}
			extractDefaults(schema, defaults)
			for key, value := range defaults {
				if _, exists := existing[key]; !exists {
					existing[key] = value
				}
			}
			nodeMappings = extractNodeMappings(schema)
		}

		// Merge payload on top
		for key, value := range payload {
			existing[key] = value
		}

		// Nest back into ROS2 YAML structure
		nested := nestToROS2YAML(existing, nodeMappings, existingYAML)

		// Marshal with YAML comments header
		out, err := yaml.Marshal(nested)
		if err != nil {
			c.JSON(500, ErrorResponse{Error: "failed to marshal YAML: " + err.Error()})
			return
		}

		header := "# Mowgli Robot Configuration — managed by mowglinext-gui\n" +
			"# This file is the single source of truth for robot parameters.\n" +
			"# Changes made here are picked up on container restart.\n\n"

		if err := os.MkdirAll(filepath.Dir(string(configFilePath)), 0755); err != nil {
			c.JSON(500, ErrorResponse{Error: err.Error()})
			return
		}
		if err := writePreservingPerms(string(configFilePath), []byte(header+string(out))); err != nil {
			c.JSON(500, ErrorResponse{Error: err.Error()})
			return
		}

		log.Printf("Saved mowgli_robot.yaml (%d bytes)", len(out)+len(header))
		c.JSON(200, OkResponse{})
	})
}
