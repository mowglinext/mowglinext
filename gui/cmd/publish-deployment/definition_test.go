package main

import (
	"context"
	"encoding/json"
	"os"
	"os/exec"
	"path/filepath"
	"runtime"
	"strings"
	"testing"

	"gopkg.in/yaml.v3"

	"github.com/mowglinext/mowglinext/pkg/updates"
)

func TestRepositoryDefinitionAndWorkflowExcludeExternalBuilds(t *testing.T) {
	root := filepath.Join("..", "..", "..")
	if _, err := readComponents(filepath.Join(root, "install", "deployment.json")); err != nil {
		t.Fatal(err)
	}
	data, err := os.ReadFile(filepath.Join(root, ".github", "workflows", "deployment-release.yml"))
	if err != nil {
		t.Fatal(err)
	}
	var workflow struct {
		Jobs map[string]struct{ Steps []struct{ ID, Run string } }
	}
	if err = yaml.Unmarshal(data, &workflow); err != nil {
		t.Fatal(err)
	}
	script := ""
	for _, step := range workflow.Jobs["identity"].Steps {
		if step.ID == "components" {
			script = step.Run
		}
	}
	if script == "" {
		t.Fatal("missing component matrix")
	}
	if !strings.Contains(string(data), "--definition ../install/deployment.json") {
		t.Fatal("publisher does not read external definitions")
	}
	if _, err := exec.LookPath("jq"); err != nil {
		t.Skip("workflow execution requires jq")
	}
	if runtime.GOOS == "windows" {
		return
	}
	dir := t.TempDir()
	if err = os.Mkdir(filepath.Join(dir, "install"), 0700); err != nil {
		t.Fatal(err)
	}
	fixture, _ := json.Marshal(map[string]any{"components": definitions()})
	if err = os.WriteFile(filepath.Join(dir, "install", "deployment.json"), fixture, 0600); err != nil {
		t.Fatal(err)
	}
	output := filepath.Join(dir, "output")
	cmd := exec.Command("bash", "-euo", "pipefail", "-c", script)
	cmd.Dir = dir
	cmd.Env = append(os.Environ(), "GITHUB_OUTPUT="+output)
	if result, err := cmd.CombinedOutput(); err != nil || len(result) != 0 {
		t.Fatal(err, string(result))
	}
	result, err := os.ReadFile(output)
	if err != nil {
		t.Fatal(err)
	}
	var built []componentDefinition
	if err = json.Unmarshal([]byte(strings.TrimSpace(strings.TrimPrefix(string(result), "value="))), &built); err != nil {
		t.Fatal(err, string(result), script)
	}
	if len(built) != 2 || built[0].Name != "mowgli-ros2" || built[1].Name != "mowglinext-gui" {
		t.Fatal("external image entered build matrix", built)
	}
}

func definitions() []componentDefinition {
	return []componentDefinition{
		{Name: "mowgli-ros2", Context: ".", File: "ros2/Dockerfile"},
		{Name: "mowglinext-gui", Type: "built", Context: "gui", File: "gui/Dockerfile"},
		{Name: "helper", Type: "external", Image: "docker.io/library/example", Version: "2.0.22", Digest: "sha256:" + strings.Repeat("a", 64)},
	}
}

func TestExternalDefinitions(t *testing.T) {
	if err := validateComponents(definitions()); err != nil {
		t.Fatal(err)
	}
	for name, change := range map[string]func([]componentDefinition){
		"tag only":        func(c []componentDefinition) { c[2].Digest = "latest" },
		"missing version": func(c []componentDefinition) { c[2].Version = "" },
		"URL":             func(c []componentDefinition) { c[2].Image = "https://docker.io/library/example" },
		"unknown kind":    func(c []componentDefinition) { c[2].Type = "externl" },
		"ambiguous build": func(c []componentDefinition) { c[2].File = "Dockerfile" },
		"duplicate":       func(c []componentDefinition) { c[2].Name = c[0].Name },
		"external core":   func(c []componentDefinition) { c[0] = c[2]; c[0].Name = "mowgli-ros2"; c[2].Name = "other" },
	} {
		t.Run(name, func(t *testing.T) {
			c := definitions()
			change(c)
			if validateComponents(c) == nil {
				t.Fatal("accepted invalid definition")
			}
		})
	}
}

type externalResolver struct {
	t     *testing.T
	image updates.Image
}

func (r externalResolver) Resolve(_ context.Context, repo, ref string) (updates.Image, error) {
	if repo != r.image.Repository || !updates.DigestPattern.MatchString(ref) {
		r.t.Fatal("resolved a tag instead of the approved digest")
	}
	return r.image, nil
}
func TestExternalPublicationPreservesUpstreamIdentity(t *testing.T) {
	c := definitions()[2]
	image := updates.Image{Repository: c.Image, Digest: c.Digest, Platforms: map[string]updates.Platform{}}
	for _, arch := range []string{"linux/arm64", "linux/amd64"} {
		image.Platforms[arch] = updates.Platform{Manifest: "sha256:" + strings.Repeat("b", 64), Config: "sha256:" + strings.Repeat("c", 64), Revision: "upstream-revision", Version: "upstream-label"}
	}
	got, err := resolveExternal(context.Background(), externalResolver{t, image}, c)
	if err != nil || got.Type != "external" || got.Version != c.Version || got.Platforms["linux/arm64"].Revision != "upstream-revision" || got.Platforms["linux/arm64"].Version != "upstream-label" {
		t.Fatal(got, err)
	}
	image.Digest = "sha256:" + strings.Repeat("d", 64)
	if _, err = resolveExternal(context.Background(), externalResolver{t, image}, c); err == nil {
		t.Fatal("accepted substituted index")
	}
	image.Digest = c.Digest
	delete(image.Platforms, "linux/amd64")
	if _, err = resolveExternal(context.Background(), externalResolver{t, image}, c); err == nil {
		t.Fatal("accepted partial platform publication")
	}
}
