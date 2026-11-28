package extensions

import (
	"context"
	"encoding/json"
	"os"
	"path/filepath"
	"strings"
	"testing"
)

type packRunner struct {
	raw   []byte
	calls []string
}

func (f *packRunner) Run(ctx context.Context, name string, args ...string) ([]byte, error) {
	f.calls = append([]string{name}, args...)
	return f.raw, nil
}

func TestPackNormalizesOnlyReadOnlyComposeOutput(t *testing.T) {
	release := adapterRelease(t)
	source := filepath.Join(t.TempDir(), "compose.yaml")
	if err := os.WriteFile(source, []byte("services: {}\n"), 0600); err != nil {
		t.Fatal(err)
	}
	manifest, _ := json.Marshal(release.Bundle.Manifest)
	runner := &packRunner{raw: release.Bundle.Compose}
	bundle, err := Pack(context.Background(), runner, manifest, source, "", release.Binding)
	if err != nil {
		t.Fatal(err)
	}
	if !strings.Contains(strings.Join(runner.calls, " "), "config --format json --resolve-image-digests") {
		t.Fatal("pack started services or did not lock image digests", runner.calls)
	}
	if bundle.Manifest.ID != release.Bundle.Manifest.ID {
		t.Fatal("manifest identity lost")
	}
	output := filepath.Join(t.TempDir(), "bundle.json")
	if err := WriteBundle(output, bundle); err != nil {
		t.Fatal(err)
	}
	info, _ := os.Stat(output)
	if info.Mode().Perm() != 0600 {
		t.Fatal("resolved config is not private")
	}
	data, _ := os.ReadFile(output)
	parsed, err := ParseBundle(data)
	if err != nil || BundleDigest(parsed) != BundleDigest(bundle) {
		t.Fatal("private bundle roundtrip failed", err)
	}
	if err = WriteBundle(output, bundle); err == nil {
		t.Fatal("pack overwrote an existing file")
	}
}

func TestPackCannotDropUnknownAuthorityOrFloatingImages(t *testing.T) {
	r := adapterRelease(t)
	manifest, _ := json.Marshal(r.Bundle.Manifest)
	source := filepath.Join(t.TempDir(), "compose.yaml")
	_ = os.WriteFile(source, []byte("services: {}\n"), 0600)
	for _, replacement := range []string{
		strings.Replace(string(r.Bundle.Compose), `"image":`, `"privileged":true,"image":`, 1),
		strings.Replace(string(r.Bundle.Compose), "@sha256:"+strings.Repeat("a", 64), ":latest", 1),
	} {
		if _, err := Pack(context.Background(), &packRunner{raw: []byte(replacement)}, manifest, source, "", r.Binding); err == nil {
			t.Fatal("pack accepted unsupported authority or a floating image")
		}
	}
}
