package main

import (
	"context"
	"encoding/json"
	"fmt"
	"os"
	"regexp"

	"github.com/mowglinext/mowglinext/pkg/updates"
)

// External components are approved in source control, never resolved from a
// floating tag during publication. Their original image metadata is retained.
type componentDefinition struct {
	Name    string `json:"name"`
	Type    string `json:"type"`
	Context string `json:"context"`
	File    string `json:"file"`
	Target  string `json:"target"`
	Image   string `json:"image"`
	Version string `json:"version"`
	Digest  string `json:"digest"`
}

func readComponents(path string) ([]componentDefinition, error) {
	data, err := os.ReadFile(path)
	if err != nil {
		return nil, err
	}
	var definition struct {
		Components []componentDefinition `json:"components"`
	}
	if err = json.Unmarshal(data, &definition); err != nil {
		return nil, err
	}
	if err = validateComponents(definition.Components); err != nil {
		return nil, err
	}
	return definition.Components, nil
}

func validateComponents(components []componentDefinition) error {
	namePattern := regexp.MustCompile(`^[a-z0-9]+(?:[._-][a-z0-9]+)*$`)
	versionPattern := regexp.MustCompile(`^[A-Za-z0-9][A-Za-z0-9_.-]{0,127}$`)
	seen := map[string]bool{}
	for _, c := range components {
		if !namePattern.MatchString(c.Name) || seen[c.Name] {
			return fmt.Errorf("invalid or duplicate component %s", c.Name)
		}
		seen[c.Name] = true
		switch c.Type {
		case "", "built":
			if c.Context == "" || c.File == "" || c.Image != "" || c.Version != "" || c.Digest != "" {
				return fmt.Errorf("invalid build definition for %s", c.Name)
			}
		case "external":
			if c.Name == "mowgli-ros2" || c.Name == "mowglinext-gui" || c.Context != "" || c.File != "" || c.Target != "" || !versionPattern.MatchString(c.Version) || !updates.DigestPattern.MatchString(c.Digest) {
				return fmt.Errorf("invalid external definition for %s", c.Name)
			}
			if _, _, _, err := updates.RegistryLocation(c.Image); err != nil {
				return err
			}
		default:
			return fmt.Errorf("unknown component type %q", c.Type)
		}
	}
	if !seen["mowgli-ros2"] || !seen["mowglinext-gui"] {
		return fmt.Errorf("build definition must include GUI and ROS2")
	}
	return nil
}

type imageResolver interface {
	Resolve(context.Context, string, string) (updates.Image, error)
}

func resolveExternal(ctx context.Context, registry imageResolver, c componentDefinition) (updates.Image, error) {
	image, err := registry.Resolve(ctx, c.Image, c.Digest)
	if err != nil {
		return image, err
	}
	if image.Repository != c.Image || image.Digest != c.Digest {
		return image, fmt.Errorf("external image identity mismatch for %s", c.Name)
	}
	for _, arch := range []string{"linux/amd64", "linux/arm64"} {
		p, ok := image.Platforms[arch]
		if !ok || !updates.DigestPattern.MatchString(p.Manifest) || !updates.DigestPattern.MatchString(p.Config) {
			return image, fmt.Errorf("external image %s lacks verified %s bytes", c.Name, arch)
		}
	}
	image.Type, image.Version = "external", c.Version
	return image, nil
}
