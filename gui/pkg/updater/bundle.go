package updater

import (
	"context"
	"encoding/json"
	"errors"
	"fmt"
	"net/http"
	"os"
	"path/filepath"
	"sort"
	"strings"
	"time"

	"github.com/joho/godotenv"
	"github.com/mowglinext/mowglinext/pkg/updates"
	"gopkg.in/yaml.v3"
)

// The same selection map is consumed by the installer and release publisher.
// New optional groups must provide an empty "none" choice; existing selections
// never fall back silently when a device is no longer supported.
type StackDefinition struct {
	Schema   int                            `json:"schema"`
	Required []string                       `json:"required"`
	Options  map[string]map[string][]string `json:"options"`
}
type ComposeBundle struct {
	StackDefinition
	Fragments map[string]json.RawMessage `json:"fragments"`
}
type BundleAsset struct {
	Asset  string `json:"asset"`
	SHA256 string `json:"sha256"`
}
type StackSelection struct {
	Options         map[string]string `json:"options"`
	InstallerConfig string            `json:"installer_config"`
}

func (d StackDefinition) Select(choices map[string]string) ([]string, error) {
	if d.Schema != 1 || len(d.Required) == 0 {
		return nil, errors.New("unsupported release stack definition")
	}
	for key, choice := range choices {
		if _, ok := d.Options[key]; !ok && choice != "none" {
			return nil, fmt.Errorf("release no longer supports selected %s: %s", key, choice)
		}
	}
	files := append([]string{}, d.Required...)
	keys := make([]string, 0, len(d.Options))
	for key := range d.Options {
		keys = append(keys, key)
	}
	sort.Strings(keys)
	for _, key := range keys {
		options := d.Options[key]
		if none, ok := options["none"]; !ok || len(none) != 0 {
			return nil, fmt.Errorf("optional group %s requires an empty none choice", key)
		}
		choice := choices[key]
		if choice == "" {
			choice = "none"
		}
		selected, ok := options[choice]
		if !ok {
			return nil, fmt.Errorf("release does not support selected %s: %s", key, choice)
		}
		files = append(files, selected...)
	}
	seen := map[string]bool{}
	for _, name := range files {
		if !idPattern.MatchString(name) || filepath.Base(name) != name || seen[name] {
			return nil, errors.New("invalid or duplicate Compose fragment")
		}
		seen[name] = true
	}
	return files, nil
}

func ReadComposeBundle(dir string) (ComposeBundle, error) {
	var b ComposeBundle
	data, err := os.ReadFile(filepath.Join(dir, "stack.json"))
	if err != nil {
		return b, err
	}
	if err = json.Unmarshal(data, &b.StackDefinition); err != nil {
		return b, err
	}
	b.Fragments = map[string]json.RawMessage{}
	names := append([]string{}, b.Required...)
	for _, options := range b.Options {
		for _, files := range options {
			names = append(names, files...)
		}
	}
	for _, name := range names {
		if !idPattern.MatchString(name) || filepath.Base(name) != name {
			return b, errors.New("invalid fragment path")
		}
		data, err = os.ReadFile(filepath.Join(dir, name))
		if err != nil {
			return b, err
		}
		var document map[string]any
		if err = yaml.Unmarshal(data, &document); err != nil {
			return b, err
		}
		b.Fragments[name], err = json.Marshal(document)
		if err != nil {
			return b, err
		}
	}
	return b, b.Validate()
}

func (b ComposeBundle) Validate() error {
	if _, err := b.Select(nil); err != nil {
		return err
	}
	for key, choices := range b.Options {
		for choice := range choices {
			if _, err := b.Select(map[string]string{key: choice}); err != nil {
				return err
			}
		}
	}
	names := append([]string{}, b.Required...)
	for _, choices := range b.Options {
		for _, files := range choices {
			names = append(names, files...)
		}
	}
	for _, name := range names {
		var doc map[string]any
		if err := json.Unmarshal(b.Fragments[name], &doc); err != nil {
			return fmt.Errorf("invalid or missing fragment %s", name)
		}
		if err := validateComposeDocument(doc); err != nil {
			return fmt.Errorf("%s: %w", name, err)
		}
	}
	return nil
}

// Publication must cover optional variants too, so switching installed hardware
// never discovers a missing image after a supposedly complete release shipped.
func (b ComposeBundle) ValidateImages(d Deployment) error {
	for name, data := range b.Fragments {
		var doc map[string]any
		if err := json.Unmarshal(data, &doc); err != nil {
			return err
		}
		for service, value := range object(doc["services"]) {
			family, _ := object(object(value)["labels"])[updateLabel+"image"].(string)
			if family == "" {
				continue
			} // maintenance overlay
			image, exists := d.Images[family]
			if !exists {
				return fmt.Errorf("%s: release is missing image %s for %s", name, family, service)
			}
			for _, arch := range []string{"linux/amd64", "linux/arm64"} {
				if _, ok := image.Platforms[arch]; !ok {
					return fmt.Errorf("%s: missing %s image", family, arch)
				}
			}
		}
	}
	return nil
}

// A release must be self-contained: config rendering must not fetch includes,
// build source, or read arbitrary host files through env_file/secrets/configs.
func validateComposeDocument(doc map[string]any) error {
	for _, key := range []string{"include", "secrets", "configs"} {
		if _, ok := doc[key]; ok {
			return fmt.Errorf("%s requires an explicit layout migration", key)
		}
	}
	services, ok := doc["services"].(map[string]any)
	if !ok {
		return errors.New("Compose fragment has no services")
	}
	for name, value := range services {
		s, ok := value.(map[string]any)
		if !ok {
			return errors.New("invalid service definition")
		}
		for _, key := range []string{"build", "extends", "env_file", "develop", "post_start", "pre_stop", "profiles", "scale"} {
			if _, exists := s[key]; exists {
				return fmt.Errorf("service %s: %s is unsupported in release bundles", name, key)
			}
		}
	}
	return nil
}

func (b DockerBackend) loadBundle(ctx context.Context, d Deployment) (ComposeBundle, error) {
	var bundle ComposeBundle
	if d.Bundle == nil {
		return bundle, errors.New("release has no Compose bundle")
	}
	data, err := updates.Read(ctx, &http.Client{Timeout: 30 * time.Second}, assetURL(d.Source.Repository, d.ReleaseTag, d.Bundle.Asset), "")
	if err != nil {
		return bundle, err
	}
	return decodeBundle(data, d.Bundle.SHA256)
}
func decodeBundle(data []byte, expected string) (ComposeBundle, error) {
	var b ComposeBundle
	if updates.Hash(data) != "sha256:"+expected {
		return b, errors.New("release Compose bundle checksum mismatch")
	}
	if err := json.Unmarshal(data, &b); err != nil {
		return b, err
	}
	return b, b.Validate()
}

// Membership-affecting installer choices have their own identity. Ordinary
// settings/env changes continue to apply without being mistaken for topology.
func installerConfigIdentity(dir string) (string, error) {
	env, err := godotenv.Read(filepath.Join(dir, ".env"))
	if err != nil {
		return "", err
	}
	var values []string
	for _, key := range []string{"HARDWARE_BACKEND", "GNSS_STACK", "GNSS_BACKEND", "LIDAR_ENABLED", "LIDAR_TYPE"} {
		values = append(values, key+"="+env[key])
	}
	return updates.Hash([]byte(strings.Join(values, "\n"))), nil
}
