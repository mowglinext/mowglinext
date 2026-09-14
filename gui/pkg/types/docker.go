package types

import (
	"context"
	"github.com/docker/docker/api/types"
	"io"
)

type ContainerDetails struct {
	ID    string
	Name  string
	Image string
	State string
	// Tty reports whether the container was created with a TTY. Docker
	// multiplexes stdout/stderr into 8-byte-framed chunks for every container
	// WITHOUT one, so the log reader has to demultiplex (stdcopy) in that case
	// and read raw otherwise.
	Tty        bool
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

type ContainerExecSpec struct {
	Cmd     []string
	Env     []string
	User    string
	WorkDir string
}

type ContainerExecResult struct {
	ExecID   string
	ExitCode int64
	Stdout   string
	Stderr   string
}

type ContainerExecHandle struct {
	ExecID string
	Reader io.ReadCloser
}

type ContainerExecInspectResult struct {
	Running  bool
	ExitCode int64
}

type IDockerProvider interface {
	ContainerList(ctx context.Context) ([]types.Container, error)
	ContainerLogs(ctx context.Context, containerID string) (io.ReadCloser, error)
	ContainerStart(ctx context.Context, containerID string) error
	ContainerStop(ctx context.Context, containerID string) error
	ContainerRestart(ctx context.Context, containerID string) error
	ContainerInspect(ctx context.Context, containerID string) (ContainerDetails, error)
	ContainerRun(ctx context.Context, spec ContainerRunSpec) (ContainerRunResult, error)
	ContainerExec(ctx context.Context, containerID string, spec ContainerExecSpec) (ContainerExecResult, error)
	ContainerExecStart(ctx context.Context, containerID string, spec ContainerExecSpec) (ContainerExecHandle, error)
	ContainerExecInspect(ctx context.Context, execID string) (ContainerExecInspectResult, error)
}
