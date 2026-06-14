package types

import (
	"context"
	"github.com/docker/docker/api/types"
	"io"
)

type ContainerDetails struct {
	ID         string
	Name       string
	Image      string
	State      string
	Status     string
	Running    bool
	Privileged bool
	Binds      []string
}

type ContainerRunSpec struct {
	Image      string
	Cmd        []string
	Env        []string
	Binds      []string
	Privileged bool
	AutoRemove bool
}

type ContainerRunResult struct {
	ContainerID string
	ExitCode    int64
	Stdout      string
	Stderr      string
}

type IDockerProvider interface {
	ContainerList(ctx context.Context) ([]types.Container, error)
	ContainerLogs(ctx context.Context, containerID string) (io.ReadCloser, error)
	ContainerStart(ctx context.Context, containerID string) error
	ContainerStop(ctx context.Context, containerID string) error
	ContainerRestart(ctx context.Context, containerID string) error
	ContainerInspect(ctx context.Context, containerID string) (ContainerDetails, error)
	ContainerRun(ctx context.Context, spec ContainerRunSpec) (ContainerRunResult, error)
}
