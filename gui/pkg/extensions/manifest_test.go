package extensions

import (
	"encoding/json"
	"strings"
	"testing"
)

func schemaTestBundle() Bundle {
	return Bundle{Manifest: Manifest{Schema: 1, ID: "test-service", Version: "1.2.3", Source: Source{Repository: "owner/service", Revision: strings.Repeat("a", 40)}, Runtime: Runtime{Kind: "compose"}, Requires: Requirements{ManagerAPI: 1, Platforms: []string{"linux/amd64"}, Contracts: []string{"gui:v1"}}, Grants: []string{"storage:extension-data"}, DataSchema: 1, Health: "process-only", Profile: "isolated"}, Compose: json.RawMessage(`{"services":{"agent":{"image":"example@sha256:aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"}}}`)}
}
func schemaTestHost() HostConfig {
	return HostConfig{Schema: 1, Installation: "robot", Platform: "linux/amd64", Contracts: []string{"gui:v1"}, TrustedRepositories: []string{"owner/service"}, ApprovedGrants: []string{"storage:extension-data"}, ExternalUnits: []string{}}
}
func schemaTestRelease() Release {
	b := schemaTestBundle()
	return Release{Bundle: b, Binding: Binding{Project: "robot-ext-test", ConfigDir: "/etc/mowgli-extensions/test", DataDir: "/var/lib/mowgli-extensions/test/data"}, Digest: BundleDigest(b)}
}

func TestStrictBundleParsing(t *testing.T) {
	raw, _ := json.Marshal(schemaTestBundle())
	if _, err := ParseBundle(raw); err != nil {
		t.Fatal(err)
	}
	cases := map[string]string{
		"case alias":        strings.Replace(string(raw), `"schema":1`, `"Schema":1`, 1),
		"case collision":    strings.Replace(string(raw), `"schema":1`, `"schema":1,"Schema":2`, 1),
		"unknown":           strings.Replace(string(raw), `"schema":1`, `"schema":1,"hooks":[]`, 1),
		"nested unknown":    strings.Replace(string(raw), `"kind":"compose"`, `"kind":"compose","script":"run"`, 1),
		"duplicate":         strings.Replace(string(raw), `"schema":1`, `"schema":1,"schema":2`, 1),
		"compose duplicate": strings.Replace(string(raw), `"services":`, `"services":{},"services":`, 1),
		"null":              `null`,
		"array":             `[]`,
		"null nested":       strings.Replace(string(raw), `"grants":["storage:extension-data"]`, `"grants":null`, 1),
		"trailing":          string(raw) + ` {}`,
		"oversize":          strings.Repeat(" ", inputLimit+1),
		"compose array":     strings.Replace(string(raw), string(schemaTestBundle().Compose), `[]`, 1),
	}
	for name, input := range cases {
		t.Run(name, func(t *testing.T) {
			if _, err := ParseBundle([]byte(input)); err == nil {
				t.Fatal("invalid bundle accepted")
			}
		})
	}
}

func TestManifestValidation(t *testing.T) {
	cases := map[string]func(*Manifest){
		"id":           func(m *Manifest) { m.ID = "../service" },
		"version":      func(m *Manifest) { m.Version = "1.2.3-latest" },
		"unpinned":     func(m *Manifest) { m.Source.Revision = "main" },
		"source":       func(m *Manifest) { m.Source.Repository = "https://github.com/owner/repo" },
		"grant":        func(m *Manifest) { m.Grants = []string{"privileged"} },
		"schema":       func(m *Manifest) { m.Schema = 2 },
		"platform":     func(m *Manifest) { m.Requires.Platforms = []string{"linux/arm/v7"} },
		"duplicate":    func(m *Manifest) { m.Grants = []string{"storage:extension-data", "storage:extension-data"} },
		"compose unit": func(m *Manifest) { m.Runtime.Units = []string{"test.service"} },
		"profile":      func(m *Manifest) { m.Profile = "host" },
		"data schema":  func(m *Manifest) { m.DataSchema = 0 },
		"health":       func(m *Manifest) { m.Health = "healthy" },
	}
	for name, mutate := range cases {
		t.Run(name, func(t *testing.T) {
			m := schemaTestBundle().Manifest
			mutate(&m)
			if ValidateManifest(m) == nil {
				t.Fatal("invalid manifest accepted")
			}
		})
	}
}

func TestHostAndCompatibility(t *testing.T) {
	host := schemaTestHost()
	raw, _ := json.Marshal(host)
	if _, err := ParseHost(raw); err != nil {
		t.Fatal(err)
	}
	if _, err := ParseHost(append(raw, []byte(` {}`)...)); err == nil {
		t.Fatal("trailing host accepted")
	}
	if _, err := ParseHost([]byte(strings.Replace(string(raw), `"schema":1`, `"schema":1,"wat":true`, 1))); err == nil {
		t.Fatal("unknown host field accepted")
	}
	m := schemaTestBundle().Manifest
	if got := Compatible(m, host); len(got) != 0 {
		t.Fatal(got)
	}
	host.Platform = "linux/arm64"
	host.Contracts = []string{}
	host.TrustedRepositories = []string{}
	host.ApprovedGrants = []string{}
	if got := Compatible(m, host); len(got) != 4 {
		t.Fatalf("want 4 independent blockers, got %v", got)
	}
	host = schemaTestHost()
	m.Runtime = Runtime{Kind: "external-systemd", Units: []string{"mowgli-lora-robot.service"}}
	m.Profile = "external"
	if got := Compatible(m, host); len(got) != 1 {
		t.Fatal(got)
	}
	host.ExternalUnits = []string{"mowgli-lora-robot.service"}
	if got := Compatible(m, host); len(got) != 0 {
		t.Fatal(got)
	}
}

func TestCanonicalBundleDigest(t *testing.T) {
	a := schemaTestBundle()
	b := a
	b.Compose = json.RawMessage(`{"services": {"agent": {"image": "example@sha256:aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"}}}`)
	if BundleDigest(a) != BundleDigest(b) || !strings.HasPrefix(BundleDigest(a), "sha256:") {
		t.Fatal("digest is not stable")
	}
	a.Compose = json.RawMessage(`{"services":{},"name":"example"}`)
	b.Compose = json.RawMessage(`{"name":"example","services":{}}`)
	if BundleDigest(a) != BundleDigest(b) {
		t.Fatal("object ordering changes digest")
	}
	b.Manifest.Version = "1.2.4"
	if BundleDigest(a) == BundleDigest(b) {
		t.Fatal("manifest identity ignored")
	}
}

func TestBindingValidation(t *testing.T) {
	b := schemaTestRelease().Binding
	if err := ValidateBinding(b); err != nil {
		t.Fatal(err)
	}
	for _, bad := range []Binding{{Project: "../bad", ConfigDir: "/config", DataDir: "/data"}, {Project: "good", ConfigDir: "../config", DataDir: "/data"}, {Project: "good", ConfigDir: "/x/../config", DataDir: "/data"}, {Project: "good", ConfigDir: "/data/config", DataDir: "/data"}} {
		if ValidateBinding(bad) == nil {
			t.Fatalf("accepted %+v", bad)
		}
	}
}
