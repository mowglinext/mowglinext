package main

import (
	"os"
	"os/exec"
	"path/filepath"
	"runtime"
	"strings"
	"testing"

	"gopkg.in/yaml.v3"
)

func TestWorkflowUsesVersionedIdentityAndReleaseMarkers(t *testing.T) {
	root := filepath.Join("..", "..", "..")
	data, err := os.ReadFile(filepath.Join(root, ".github", "workflows", "deployment-release.yml"))
	if err != nil {
		t.Fatal(err)
	}
	var workflow struct {
		Jobs map[string]struct {
			Steps []struct {
				ID   string            `yaml:"id"`
				Run  string            `yaml:"run"`
				With map[string]string `yaml:"with"`
			} `yaml:"steps"`
		} `yaml:"jobs"`
	}
	if err = yaml.Unmarshal(data, &workflow); err != nil {
		t.Fatal(err)
	}
	script := ""
	for _, step := range workflow.Jobs["identity"].Steps {
		if step.ID == "identity" {
			script = step.Run
		}
	}
	if script == "" {
		t.Fatal("missing release identity generator")
	}
	for _, step := range workflow.Jobs["build"].Steps {
		if step.ID != "image" {
			continue
		}
		for _, label := range []string{"garden.mowgli.image-contract=1", "garden.mowgli.image-family=", "garden.mowgli.firmware-protocol=", "garden.mowgli.release=", "garden.mowgli.deployment=", "org.opencontainers.image.source=", "org.opencontainers.image.created=", "org.opencontainers.image.version=${{ needs.identity.outputs.version }}"} {
			if !strings.Contains(step.With["labels"], label) {
				t.Fatal("missing publication label", label)
			}
		}
		if !strings.Contains(step.With["build-args"], "BUILD_VERSION=${{ needs.identity.outputs.version }}") {
			t.Fatal("GUI build does not receive the release version")
		}
	}
	if runtime.GOOS == "windows" {
		return
	}
	for _, tc := range []struct {
		ref, kind, want string
		valid           bool
	}{{"v1.3.0", "tag", "v1.3.0", true}, {"dev", "branch", "deployment-aaaaaaaaaaaa-42-1", true}, {"feat/gui-dashboard-improvements", "branch", "deployment-aaaaaaaaaaaa-42-1", true}, {"v1.3.0-preview", "tag", "", false}} {
		t.Run(tc.ref, func(t *testing.T) {
			out := filepath.Join(t.TempDir(), "outputs")
			cmd := exec.Command("bash", "-euo", "pipefail", "-c", script)
			cmd.Dir = root
			cmd.Env = append(os.Environ(), "REF="+tc.ref, "TYPE="+tc.kind, "SHA="+strings.Repeat("a", 40), "RUN=42", "ATTEMPT=1", "GITHUB_OUTPUT="+out)
			message, err := cmd.CombinedOutput()
			if (err == nil) != tc.valid {
				t.Fatalf("unexpected publisher identity result: %v %s", err, message)
			}
			if tc.valid {
				data, err := os.ReadFile(out)
				if err != nil || !strings.Contains(string(data), "version="+tc.want+"\n") || !strings.Contains(string(data), "protocol=") {
					t.Fatal("wrong publication identity", string(data), err)
				}
			}
		})
	}
}
