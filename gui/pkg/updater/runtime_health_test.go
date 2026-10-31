package updater

import (
	"context"
	"encoding/json"
	"os"
	"path/filepath"
	"testing"
)

func TestRuntimeDistinguishesHTTPHealthFromRunningWithoutAProbe(t *testing.T) {
	dir := t.TempDir()
	model := `{"services":{"gui":{"container_name":"mowgli-gui","image":"gui:test"},"mowgli":{"container_name":"mowgli-ros2","image":"ros:test"}}}`
	if err := os.WriteFile(filepath.Join(dir, "compose.json"), []byte(model), 0600); err != nil {
		t.Fatal(err)
	}
	// Exercise Observe's actual Docker output projection, without a daemon.
	script := `#!/bin/sh
case "$1" in
  inspect) cat "$TEST_HEALTH_DIR/inspect.json" ;;
  *) cat "$TEST_HEALTH_DIR/compose.json" ;;
esac
`
	if err := os.WriteFile(filepath.Join(dir, "docker"), []byte(script), 0755); err != nil {
		t.Fatal(err)
	}
	t.Setenv("PATH", dir+string(os.PathListSeparator)+os.Getenv("PATH"))
	t.Setenv("TEST_HEALTH_DIR", dir)
	b := DockerBackend{Config: HostConfig{Directory: dir, Project: "install"}}
	for _, test := range []struct {
		name, state         string
		healthy, configured bool
	}{
		{"healthy", `{"Running":true,"Health":{"Status":"healthy"}}`, true, true},
		{"unhealthy", `{"Running":true,"Health":{"Status":"unhealthy"}}`, false, true},
		{"starting", `{"Running":true,"Health":{"Status":"starting"}}`, false, true},
		{"not_configured", `{"Running":true}`, true, false},
		{"stopped", `{"Running":false}`, false, false},
	} {
		data := `[{"Image":"sha256:fixture","Config":{"Image":"gui:test","Labels":{"com.docker.compose.project":"install"}},"State":` + test.state + `}]`
		if !json.Valid([]byte(data)) {
			t.Fatal("invalid fixture")
		}
		if err := os.WriteFile(filepath.Join(dir, "inspect.json"), []byte(data), 0600); err != nil {
			t.Fatal(err)
		}
		observed, err := b.Observe(context.Background())
		if err != nil {
			t.Fatal(err)
		}
		gui := observed["gui"]
		if gui.HealthStatus != test.name || gui.Healthy != test.healthy || gui.Healthcheck != test.configured {
			t.Fatalf("incorrect health evidence for %s: %+v", test.name, gui)
		}
	}
}
