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
	protocol := flag.Int("protocol", 0, "firmware protocol")
	flag.Parse()
	d := updater.Deployment{Schema: 1, ID: *id, Source: updater.Source{Repository: *repo, Branch: *branch, Track: *track}, Revision: *revision, PublishedAt: time.Now().UTC(), ReleaseTag: *release, Layout: 1, DataSchema: 1, UpdaterAPI: 1, MaintenanceAPI: 1, FirmwareProtocol: *protocol, Images: map[string]updates.Image{}, Updater: map[string]updater.Binary{}}
	registry := updates.NewRegistry()
	ctx, cancel := context.WithTimeout(context.Background(), 10*time.Minute)
	defer cancel()
	for _, name := range updates.ImageNames {
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
	if err := d.Validate([]string{*repo}); err != nil {
		return err
	}
	data, err := json.MarshalIndent(d, "", "  ")
	if err != nil {
		return err
	}
	return os.WriteFile(filepath.Join(*dir, "mowgli-deployment.json"), data, 0644)
}
