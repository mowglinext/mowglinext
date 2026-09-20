package updater

import (
	"bytes"
	"context"
	"errors"
	"os"
	"os/exec"
	"path/filepath"
	"reflect"
	"strings"
	"testing"

	"github.com/mowglinext/mowglinext/pkg/updates"
)

func TestLegacyDifferencesListsEveryChangedKeySorted(t *testing.T) {
	installed := []byte(`{"services":{
		"mowgli":{"image":"ros","environment":{"A":"1"},"command":"old"},
		"gps":{"image":"gps","environment":{"GNSS_DEVICE":"/dev/gps"}},
		"mqtt":{"image":"mosquitto","environment":{"LOCAL":"edit"}},
		"watchtower":{"image":"watchtower"}}}`)
	target := []byte(`{"services":{
		"mowgli":{"image":"ros","environment":{"A":"1","GNSS_STACK":"universal","MOWGLI_UPDATE_MAINTENANCE":"/var/lib/mowgli-updater/maintenance"},"command":"new",
			"volumes":[{"type":"bind","source":"/var/lib/mowgli-updater","target":"/var/lib/mowgli-updater"}]},
		"gps":{"image":"gps","environment":{}},
		"mqtt":{"image":"mosquitto"}}}`)
	got := legacyDifferences(installed, target)
	// mqtt is a local service and watchtower is retired by the installer:
	// neither may ever count. The updater's own gate/mount additions do not
	// count either.
	want := []string{"gps.environment", "mowgli.command", "mowgli.environment"}
	if !reflect.DeepEqual(got, want) {
		t.Fatalf("differences = %v, want %v", got, want)
	}
	if again := legacyDifferences(target, target); len(again) != 0 {
		t.Fatalf("identical definitions reported %v", again)
	}
}

// installStackFixture renders the real installer fragments the way the
// pre-updater installer did (base + gui + gps + mqtt + watchtower, no updater
// fragment) and returns the docker/ directory holding that legacy file.
func installStackFixture(t *testing.T) (DockerBackend, string, string) {
	t.Helper()
	if err := exec.Command("docker", "compose", "version").Run(); err != nil {
		t.Skip("docker compose is not available")
	}
	source, err := filepath.Abs(filepath.Join("..", "..", "..", "install", "compose"))
	if err != nil {
		t.Fatal(err)
	}
	if _, err = os.Stat(filepath.Join(source, "stack.json")); err != nil {
		t.Skip("install/compose not in this tree")
	}
	root := t.TempDir()
	dir := filepath.Join(root, "docker")
	if err = os.Mkdir(dir, 0755); err != nil {
		t.Fatal(err)
	}
	if err = os.WriteFile(filepath.Join(dir, ".env"), []byte("MOWGLI_ROS2_IMAGE=example/ros2:dev\nGUI_IMAGE=example/gui:dev\n"), 0600); err != nil {
		t.Fatal(err)
	}
	args := []string{"compose", "--project-name", "install", "--project-directory", root, "--env-file", filepath.Join(dir, ".env")}
	for _, name := range []string{"base", "gui", "gps", "mqtt", "watchtower"} {
		args = append(args, "-f", filepath.Join(source, "docker-compose."+name+".yml"))
	}
	var stderr bytes.Buffer
	render := exec.Command("docker", append(args, "config", "--no-interpolate")...)
	render.Stderr = &stderr
	generated, err := render.Output()
	if err != nil {
		t.Fatalf("render legacy compose: %v: %s", err, stderr.String())
	}
	return DockerBackend{Config: HostConfig{Directory: dir, Project: "install"}}, source, string(generated)
}

// What an untouched pre-#625 install has on disk: the same generated file,
// minus the variable the release added to the ROS2 container since.
func withoutGNSSStack(t *testing.T, generated string) string {
	t.Helper()
	lines := []string{}
	for _, line := range strings.Split(generated, "\n") {
		if !strings.HasPrefix(strings.TrimSpace(line), "GNSS_STACK:") {
			lines = append(lines, line)
		}
	}
	older := strings.Join(lines, "\n")
	if older == generated {
		t.Fatal("fixture no longer removes anything: GNSS_STACK left the fragments")
	}
	return older
}

var installChoices = map[string]string{"gnss": "universal", "lidar": "none"}

// Field report 2026-09-20: re-running the installer on an untouched install
// died with "legacy Compose differs in mowgli.environment". The file has no
// baseline, so the refusal stays — but it names every difference, leaves the
// file alone, and the operator's consent adopts it with the old file kept.
func TestInstallStackLegacyWithoutBaselineNeedsConsentAndKeepsTheOldFile(t *testing.T) {
	b, source, generated := installStackFixture(t)
	legacy := filepath.Join(b.Config.Directory, "docker-compose.yaml")
	older := withoutGNSSStack(t, generated)
	if err := os.WriteFile(legacy, []byte(older), 0600); err != nil {
		t.Fatal(err)
	}

	err := b.InstallStack(context.Background(), source, installChoices, false)
	var refused *LegacyComposeError
	if !errors.As(err, &refused) {
		t.Fatalf("want LegacyComposeError, got %v", err)
	}
	found := false
	for _, difference := range refused.Differences {
		found = found || difference == "mowgli.environment"
	}
	if !found {
		t.Fatalf("differences %v do not name mowgli.environment", refused.Differences)
	}
	if data, _ := os.ReadFile(legacy); string(data) != older {
		t.Fatal("refusal modified the installed Compose file")
	}
	if _, e := os.Stat(filepath.Join(b.Config.Directory, "stack-definition.sha256")); !os.IsNotExist(e) {
		t.Fatal("refusal recorded a baseline")
	}

	if err = b.InstallStack(context.Background(), source, installChoices, true); err != nil {
		t.Fatalf("consented adoption: %v", err)
	}
	backups, _ := filepath.Glob(legacy + ".legacy-*")
	if len(backups) != 1 {
		t.Fatalf("want exactly one preserved legacy file, got %v", backups)
	}
	if data, _ := os.ReadFile(backups[0]); string(data) != older {
		t.Fatal("preserved legacy file is not byte-identical to the replaced one")
	}
	adopted, _ := os.ReadFile(legacy)
	if !strings.Contains(string(adopted), "GNSS_STACK") || strings.Contains(string(adopted), "mowgli-watchtower") {
		t.Fatal("adopted file is not the current managed definition")
	}
	if !strings.Contains(string(adopted), "mowgli-mqtt") {
		t.Fatal("local MQTT service was not retained")
	}
	if err = b.checkStackBaseline(); err != nil {
		t.Fatalf("adoption left no valid baseline: %v", err)
	}
	// From here on the recorded baseline decides; reruns need no consent.
	if err = b.InstallStack(context.Background(), source, installChoices, false); err != nil {
		t.Fatalf("rerun after adoption: %v", err)
	}
}

// The correct comparison: against the baseline the file was GENERATED from.
// With one recorded, evolved fragments are adopted without any question —
// and a hand edit is refused even when consent to adopt a legacy file is given.
func TestInstallStackRecordedBaselineSeparatesEvolutionFromHandEdits(t *testing.T) {
	b, source, generated := installStackFixture(t)
	legacy := filepath.Join(b.Config.Directory, "docker-compose.yaml")
	baseline := filepath.Join(b.Config.Directory, "stack-definition.sha256")
	older := withoutGNSSStack(t, generated)
	if err := os.WriteFile(legacy, []byte(older), 0600); err != nil {
		t.Fatal(err)
	}
	if err := os.WriteFile(baseline, []byte(updates.Hash([]byte(older))+"\n"), 0600); err != nil {
		t.Fatal(err)
	}
	edited := strings.Replace(older, "container_name: mowgli-ros2", "container_name: mowgli-ros2\n    cpus: 2", 1)
	if edited == older {
		t.Fatal("fixture hand edit did not apply")
	}
	if err := os.WriteFile(legacy, []byte(edited), 0600); err != nil {
		t.Fatal(err)
	}
	err := b.InstallStack(context.Background(), source, installChoices, true)
	if err == nil || !strings.Contains(err.Error(), "edited manually") {
		t.Fatalf("a hand-edited generated file must stay refused, got %v", err)
	}
	if data, _ := os.ReadFile(legacy); string(data) != edited {
		t.Fatal("refusal modified the hand-edited file")
	}

	if err = os.WriteFile(legacy, []byte(older), 0600); err != nil {
		t.Fatal(err)
	}
	if err = b.InstallStack(context.Background(), source, installChoices, false); err != nil {
		t.Fatalf("untouched file generated by an older release: %v", err)
	}
	adopted, _ := os.ReadFile(legacy)
	if !strings.Contains(string(adopted), "GNSS_STACK") {
		t.Fatal("evolved fragments were not adopted")
	}
	if backups, _ := filepath.Glob(legacy + ".legacy-*"); len(backups) != 0 {
		t.Fatalf("an unedited generated file needs no legacy backup, got %v", backups)
	}
}
