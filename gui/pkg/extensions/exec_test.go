package extensions

import (
	"context"
	"os"
	"path/filepath"
	"strings"
	"testing"
)

func TestExecRunnerUsesLocalDockerAndIgnoresInheritedContext(t *testing.T) {
	bin := t.TempDir()
	script := "#!/bin/sh\nprintf '%s\\n' \"$@\"\nprintf 'context=%s compose=%s platform=%s\\n' \"${DOCKER_CONTEXT-unset}\" \"${COMPOSE_PROJECT_NAME-unset}\" \"${DOCKER_DEFAULT_PLATFORM-unset}\"\n"
	if err := os.WriteFile(filepath.Join(bin, "docker"), []byte(script), 0700); err != nil {
		t.Fatal(err)
	}
	t.Setenv("PATH", bin)
	t.Setenv("DOCKER_HOST", "tcp://other-host:2376")
	t.Setenv("DOCKER_CONTEXT", "other-host")
	t.Setenv("DOCKER_DEFAULT_PLATFORM", "linux/arm/v7")
	t.Setenv("COMPOSE_PROJECT_NAME", "core-project")
	output, err := (ExecRunner{}).Run(context.Background(), "docker", "ps", "-aq")
	if err != nil {
		t.Fatal(err)
	}
	want := "--host=unix:///var/run/docker.sock\nps\n-aq\ncontext=unset compose=unset platform=unset\n"
	if string(output) != want {
		t.Fatalf("local runtime selection changed: %q", output)
	}
}

func TestImagePlatformIsCheckedDuringPreparation(t *testing.T) {
	r := adapterRelease(t)
	b, runner := adapterBackend(t, r)
	runner.image.Architecture = "arm"
	if err := b.Prepare(context.Background(), r); err == nil || !strings.Contains(err.Error(), "platform") {
		t.Fatal("foreign-architecture image accepted", err)
	}
	for _, call := range runner.calls {
		if strings.Contains(strings.Join(call, " "), " up ") || strings.Contains(strings.Join(call, " "), " stop") {
			t.Fatal("runtime was stopped before platform validation", call)
		}
	}
}
