package updater

import (
	"os"
	"path/filepath"
	"strings"
	"testing"

	"gopkg.in/yaml.v3"
)

// Field report 2026-10-10: every update of an OpenMower robot was refused with
// "managed service openmower has an unsupported writable mount /dev". A
// hardware sidecar binds the host's /dev for its serial ports; that holds no
// data, so it needs no backup contract. Every release sidecar must pass both
// mount gates, as installed (validateManagedMounts) and when a robot gains it
// (validateStackMounts), or the update is blocked on the robot.
func TestEveryReleaseSidecarPassesTheMountGates(t *testing.T) {
	dir := filepath.Join("..", "..", "..", "install", "compose")
	bundle, err := ReadComposeBundle(dir)
	if err != nil {
		t.Skip("install/compose not in this tree")
	}
	checked := 0
	for _, choices := range bundle.Options {
		for choice, files := range choices {
			for _, file := range files {
				data, err := os.ReadFile(filepath.Join(dir, file))
				if err != nil {
					t.Fatal(err)
				}
				var doc struct {
					Services map[string]struct {
						ContainerName string            `yaml:"container_name"`
						Labels        map[string]string `yaml:"labels"`
						Volumes       []string          `yaml:"volumes"`
					} `yaml:"services"`
				}
				if err = yaml.Unmarshal(data, &doc); err != nil {
					t.Fatal(err)
				}
				for name, s := range doc.Services {
					if s.Labels[updateLabel+"image"] == "" {
						continue
					}
					var info containerInfo
					var volumes []composeVolume
					for _, entry := range s.Volumes {
						parts := strings.Split(entry, ":")
						ro := len(parts) > 2 && parts[2] == "ro"
						info.Mounts = append(info.Mounts, struct {
							Type        string `json:"Type"`
							Source      string `json:"Source"`
							Destination string `json:"Destination"`
							RW          bool   `json:"RW"`
						}{"bind", parts[0], parts[1], !ro})
						volumes = append(volumes, composeVolume{Type: "bind", Source: parts[0], Target: parts[1], ReadOnly: ro})
					}
					if err = validateManagedMounts(name, info); err != nil {
						t.Errorf("%s (%s): installed service refused: %v", name, choice, err)
					}
					current := composeConfig{Services: map[string]serviceConfig{}}
					target := composeConfig{Services: map[string]serviceConfig{name: {ContainerName: s.ContainerName, Volumes: volumes}}}
					managed := map[string]managedService{name: {}}
					if err = validateStackMounts(current, target, map[string]managedService{}, managed); err != nil {
						t.Errorf("%s (%s): adding it refused: %v", name, choice, err)
					}
					checked++
				}
			}
		}
	}
	if checked == 0 {
		t.Fatal("no release sidecar checked")
	}
}

// The exemption is for the device tree only: a writable data directory, or a
// /dev target fed from elsewhere, still needs a backup contract.
func TestOnlyTheDeviceTreeSkipsTheBackupContract(t *testing.T) {
	for _, test := range []struct {
		source, target string
		covered        bool
	}{
		{"/dev", "/dev", true},
		{"/dev/ttyAMA0", "/dev/ttyAMA0", true},
		{"/dev/../dev", "/dev", true},
		{"/var/lib/data", "/dev", false},
		{"/dev", "/data", false},
		{"/devices", "/devices", false},
		{"./docker/config/db", "/db", true},
		{"./docker/logs", "/logs", false},
	} {
		if got := writableMountCovered(test.source, test.target); got != test.covered {
			t.Errorf("%s:%s covered=%v, want %v", test.source, test.target, got, test.covered)
		}
	}
}
