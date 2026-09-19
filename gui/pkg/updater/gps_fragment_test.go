package updater

import (
	"os"
	"path/filepath"
	"strings"
	"testing"

	"gopkg.in/yaml.v3"
)

// A robot that already runs the previous gps service must be able to take the
// release gps fragment through validateStackMounts: layout 1 refuses any NEW
// writable mount ("adds writable storage ...; explicit data/layout migration
// required"). Field, 2026-09-19: a named log volume in the fragment made every
// updater-managed robot unable to install the release.
func TestReleaseGPSFragmentAddsNoWritableStorage(t *testing.T) {
	data, err := os.ReadFile(filepath.Join("..", "..", "..", "install", "compose", "docker-compose.gps.yml"))
	if err != nil {
		t.Skip("install/compose not in this tree")
	}
	var doc struct {
		Services map[string]struct {
			ContainerName string   `yaml:"container_name"`
			Volumes       []string `yaml:"volumes"`
		} `yaml:"services"`
		Volumes map[string]any `yaml:"volumes"`
	}
	if err = yaml.Unmarshal(data, &doc); err != nil {
		t.Fatal(err)
	}
	if len(doc.Volumes) != 0 {
		t.Fatalf("fragment declares named volumes %v: a new resource needs a layout migration", doc.Volumes)
	}
	mounts := func(entries []string) []composeVolume {
		var out []composeVolume
		for _, entry := range entries {
			parts := strings.Split(entry, ":")
			if len(parts) < 2 {
				t.Fatalf("unsupported volume syntax %q", entry)
			}
			out = append(out, composeVolume{Type: "bind", Source: parts[0], Target: parts[1], ReadOnly: len(parts) > 2 && parts[2] == "ro"})
		}
		return out
	}
	// What every installed gps service has mounted since before the sidecar.
	current := composeConfig{Services: map[string]serviceConfig{"gps": {ContainerName: "mowgli-gps", Volumes: mounts([]string{
		"/dev:/dev", "./docker/config/mowgli:/config:ro", "./docker/config/cyclonedds.xml:/cyclonedds.xml:ro",
	})}}}
	gps := doc.Services["gps"]
	target := composeConfig{Services: map[string]serviceConfig{"gps": {ContainerName: gps.ContainerName, Volumes: mounts(gps.Volumes)}}}
	managed := map[string]managedService{"gps": {}}
	if err = validateStackMounts(current, target, managed, managed); err != nil {
		t.Fatalf("the release gps fragment cannot be installed by the updater: %v", err)
	}
	// And the rule itself still bites.
	target.Services["gps"] = serviceConfig{ContainerName: "mowgli-gps", Volumes: append(mounts(gps.Volumes), composeVolume{Type: "volume", Source: "logs", Target: "/var/log/universal_gnss"})}
	if err = validateStackMounts(current, target, managed, managed); err == nil || !strings.Contains(err.Error(), "adds writable storage") {
		t.Fatalf("a new writable volume must be refused, got %v", err)
	}
}
