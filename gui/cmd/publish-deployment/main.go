// CI-only publication helper. A descriptor is emitted only when every image
// resolves to the expected source on both supported architectures.
package main

import (
	"context"
	"crypto/sha256"
	"encoding/hex"
	"encoding/json"
	"flag"
	"fmt"
	"os"
	"path/filepath"
	"strings"
	"time"

	"github.com/mowglinext/mowglinext/pkg/updater"
	"github.com/mowglinext/mowglinext/pkg/updates"
)

func main() {
	if err := publish(); err != nil {
		fmt.Fprintln(os.Stderr, err)
		os.Exit(1)
	}
}
func publish() error {
	repo := flag.String("repository", "", "GitHub repository")
	branch := flag.String("branch", "", "full source branch")
	track := flag.String("track", "dev", "stable/dev/custom")
	revision := flag.String("revision", "", "source SHA")
	id := flag.String("id", "", "immutable deployment ID and image tag")
	release := flag.String("release", "", "release tag")
	dir := flag.String("assets", "", "binary asset directory")
	images := flag.String("images", strings.Join(updates.ImageNames, ","), "comma-separated published first-party image families")
	guiCompatibility := flag.String("gui-compatibility", "", "reviewed GUI/ROS API compatibility contract; empty disables mixed releases")
	composeDir := flag.String("compose-dir", "../install/compose", "release Compose fragments and shared selection map")
	contracts := flag.String("component-contracts", "", "JSON map of reviewed per-image drop-in compatibility contracts")
	protocol := flag.Int("protocol", 0, "firmware protocol")
	flag.Parse()
	d := updater.Deployment{Schema: 2, ID: *id, Source: updater.Source{Repository: *repo, Branch: *branch, Track: *track}, Revision: *revision, PublishedAt: time.Now().UTC(), ReleaseTag: *release, Layout: 1, DataSchema: updater.DataSchemaVersion, UpdaterAPI: 1, MaintenanceAPI: 1, GUICompatibility: *guiCompatibility, FirmwareProtocol: *protocol, Images: map[string]updates.Image{}, Updater: map[string]updater.Binary{}}
	if *contracts != "" {
		if err := json.Unmarshal([]byte(*contracts), &d.ComponentCompatibility); err != nil {
			return err
		}
	}
	registry := updates.NewRegistry()
	ctx, cancel := context.WithTimeout(context.Background(), 10*time.Minute)
	defer cancel()
	for _, name := range strings.Split(*images, ",") {
		image, err := registry.Resolve(ctx, "ghcr.io/"+strings.ToLower(*repo)+"/"+name, *id)
		if err != nil {
			return err
		}
		for _, platform := range []string{"linux/arm64", "linux/amd64"} {
			p, ok := image.Platforms[platform]
			if !ok || p.Revision != *revision {
				return fmt.Errorf("incomplete %s %s build", name, platform)
			}
		}
		d.Images[name] = image
	}
	for _, arch := range []string{"arm64", "amd64"} {
		asset := "mowgli-updater-linux-" + arch
		data, err := os.ReadFile(filepath.Join(*dir, asset))
		if err != nil {
			return err
		}
		sum := sha256.Sum256(data)
		d.Updater["linux/"+arch] = updater.Binary{Asset: asset, SHA256: hex.EncodeToString(sum[:]), Version: *id}
	}
	bundle, err := updater.ReadComposeBundle(*composeDir)
	if err != nil {
		return err
	}
	if err = bundle.ValidateImages(d); err != nil {
		return err
	}
	d.ServiceChoices, err = bundle.ServiceChoices()
	if err != nil {
		return err
	}
	bundleData, err := json.MarshalIndent(bundle, "", "  ")
	if err != nil {
		return err
	}
	sum := sha256.Sum256(bundleData)
	d.Bundle = &updater.BundleAsset{Asset: "mowgli-compose.json", SHA256: hex.EncodeToString(sum[:])}
	if err = os.WriteFile(filepath.Join(*dir, d.Bundle.Asset), bundleData, 0644); err != nil {
		return err
	}
	if err := d.Validate([]string{*repo}); err != nil {
		return err
	}
	data, err := json.MarshalIndent(d, "", "  ")
	if err != nil {
		return err
	}
	return os.WriteFile(filepath.Join(*dir, "mowgli-deployment.json"), data, 0644)
}
