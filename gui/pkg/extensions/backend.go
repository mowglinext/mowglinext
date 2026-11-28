package extensions

import (
	"bytes"
	"context"
	"errors"
	"fmt"
	"io"
	"net/http"
	"os"
	"os/exec"
	"strings"
	"time"
)

// CommandRunner permits deterministic adapter tests without a Docker daemon.
// Implementations must honor cancellation and must not log command output.
type CommandRunner interface {
	Run(context.Context, string, ...string) ([]byte, error)
}
type ExecRunner struct{}

// boundedOutput prevents an untrusted daemon response from exhausting memory.
type boundedOutput struct {
	bytes.Buffer
	overflow bool
}

func (b *boundedOutput) Write(p []byte) (int, error) {
	n := len(p)
	if b.Len()+n > 4<<20 {
		b.overflow = true
		return n, nil
	}
	_, _ = b.Buffer.Write(p)
	return n, nil
}
func (ExecRunner) Run(ctx context.Context, name string, args ...string) ([]byte, error) {
	if name == "docker" {
		// State/bind paths belong to this host. Inherited remote Docker contexts
		// must not redirect a local extension lifecycle operation.
		args = append([]string{"--host=unix:///var/run/docker.sock"}, args...)
	}
	cmd := exec.CommandContext(ctx, name, args...)
	if name == "docker" {
		cmd.Env = []string{}
		for _, value := range os.Environ() {
			key, _, _ := strings.Cut(value, "=")
			if strings.HasPrefix(key, "COMPOSE_") || key == "DOCKER_HOST" || key == "DOCKER_CONTEXT" || key == "DOCKER_TLS" || key == "DOCKER_TLS_VERIFY" || key == "DOCKER_CERT_PATH" || key == "DOCKER_DEFAULT_PLATFORM" {
				continue
			}
			cmd.Env = append(cmd.Env, value)
		}
	}
	var out boundedOutput
	cmd.Stdout = &out
	cmd.Stderr = io.Discard
	if err := cmd.Run(); err != nil {
		return nil, errors.New("runtime command failed")
	}
	if out.overflow {
		return nil, errors.New("runtime response exceeds limit")
	}
	return out.Bytes(), nil
}

type RuntimeBackend struct {
	Root   string
	Runner CommandRunner
	Client *http.Client
}

func NewBackend(stateRoot string) Backend {
	return &RuntimeBackend{Root: stateRoot, Runner: ExecRunner{}}
}
func (b *RuntimeBackend) run(ctx context.Context, name string, args ...string) ([]byte, error) {
	ctx, cancel := context.WithTimeout(ctx, 2*time.Minute)
	defer cancel()
	runner := b.Runner
	if runner == nil {
		runner = ExecRunner{}
	}
	out, err := runner.Run(ctx, name, args...)
	if err != nil {
		// Identify the fixed operation without leaking arguments, runtime output
		// or user configuration. This remains useful with real engine versions.
		operation := name
		if len(args) > 0 {
			operation += " " + args[0]
		}
		return nil, fmt.Errorf("extension runtime command failed (%s)", operation)
	}
	return out, nil
}
func (b *RuntimeBackend) Check(ctx context.Context, r Release, adopt bool) error {
	if r.Bundle.Manifest.Runtime.Kind == "external-systemd" {
		_, err := b.external(ctx, r)
		return err
	}
	spec, err := validatedCompose(r.Bundle, r.Binding)
	if err != nil {
		return err
	}
	return b.checkCompose(ctx, r, spec, adopt)
}
func (b *RuntimeBackend) Prepare(ctx context.Context, r Release) error {
	return b.mutate(ctx, r, "pull")
}
func (b *RuntimeBackend) Start(ctx context.Context, r Release) error {
	return b.mutate(ctx, r, "up", "-d", "--no-build", "--pull", "never")
}
func (b *RuntimeBackend) Stop(ctx context.Context, r Release) error { return b.mutate(ctx, r, "stop") }
func (b *RuntimeBackend) Remove(ctx context.Context, r Release) error {
	return b.mutate(ctx, r, "down")
}
func (b *RuntimeBackend) Observe(ctx context.Context, r Release) (Observation, error) {
	if r.Bundle.Manifest.Runtime.Kind == "external-systemd" {
		return b.external(ctx, r)
	}
	return b.observeCompose(ctx, r)
}
