package updater

import (
	"os"
	"path/filepath"
	"testing"
)

// The same host directory reached through a symlink (a symlinked home, a
// moved checkout) or a differently spelled path is ONE mount: an installer
// run and the updater daemon must not see each other's file as adding
// writable storage.
func TestMountKeyCanonicalisesBindSources(t *testing.T) {
	root := t.TempDir()
	real := filepath.Join(root, "real", "docker", "config", "db")
	if err := os.MkdirAll(real, 0755); err != nil {
		t.Fatal(err)
	}
	link := filepath.Join(root, "home-link")
	if err := os.Symlink(filepath.Join(root, "real"), link); err != nil {
		t.Skip("symlinks unavailable")
	}
	viaLink := filepath.Join(link, "docker", "config", "db")
	spelled := filepath.Join(root, "real", "docker", ".", "config", "db")
	a := mountKey(composeVolume{Type: "bind", Source: real, Target: "/db"})
	if b := mountKey(composeVolume{Type: "bind", Source: viaLink, Target: "/db"}); b != a {
		t.Fatalf("symlinked source keyed differently: %s vs %s", a, b)
	}
	if b := mountKey(composeVolume{Type: "bind", Source: spelled, Target: "/db"}); b != a {
		t.Fatalf("dot-spelled source keyed differently: %s vs %s", a, b)
	}
	if mountKey(composeVolume{Type: "volume", Source: "mowgli_maps", Target: "/ros2_ws/maps"}) != "volume:mowgli_maps:/ros2_ws/maps" {
		t.Fatal("named volumes must key by name")
	}
}
