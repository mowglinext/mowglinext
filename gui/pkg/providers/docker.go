package providers

import (
	"bytes"
	"context"
	"errors"
	types2 "github.com/cedbossneo/mowglinext/pkg/types"
	"github.com/docker/docker/api/types"
	"github.com/docker/docker/api/types/container"
	docker "github.com/docker/docker/client"
	"github.com/docker/docker/pkg/stdcopy"
	"github.com/sirupsen/logrus"
	"io"
	"strings"
)

type DockerProvider struct {
	client *docker.Client
}

func NewDockerProvider() types2.IDockerProvider {
	client, err := docker.NewClientWithOpts(docker.FromEnv)
	if err != nil {
		logrus.Error(err)
	}
	return &DockerProvider{client: client}
}

func (i *DockerProvider) ContainerList(ctx context.Context) ([]types.Container, error) {
	if i.client == nil {
		return nil, errors.New("docker client is not initialized")
	}
	return i.client.ContainerList(ctx, types.ContainerListOptions{
		All: true,
	})
}

func (i *DockerProvider) ContainerLogs(ctx context.Context, containerID string) (io.ReadCloser, error) {
	if i.client == nil {
		return nil, errors.New("docker client is not initialized")
	}
	return i.client.ContainerLogs(ctx, containerID, types.ContainerLogsOptions{ShowStdout: true, ShowStderr: true, Follow: true, Tail: "100"})
}

func (i *DockerProvider) ContainerStart(ctx context.Context, containerID string) error {
	if i.client == nil {
		return errors.New("docker client is not initialized")
	}
	return i.client.ContainerStart(ctx, containerID, types.ContainerStartOptions{})
}

func (i *DockerProvider) ContainerStop(ctx context.Context, containerID string) error {
	if i.client == nil {
		return errors.New("docker client is not initialized")
	}
	return i.client.ContainerStop(ctx, containerID, nil)
}

func (i *DockerProvider) ContainerRestart(ctx context.Context, containerID string) error {
	if i.client == nil {
		return errors.New("docker client is not initialized")
	}
	return i.client.ContainerRestart(ctx, containerID, nil)
}

func (i *DockerProvider) ContainerInspect(ctx context.Context, containerID string) (types2.ContainerDetails, error) {
	if i.client == nil {
		return types2.ContainerDetails{}, errors.New("docker client is not initialized")
	}

	inspected, err := i.client.ContainerInspect(ctx, containerID)
	if err != nil {
		return types2.ContainerDetails{}, err
	}

	details := types2.ContainerDetails{
		ID:    inspected.ID,
		Name:  strings.TrimPrefix(inspected.Name, "/"),
		Image: "",
	}
	if inspected.Config != nil {
		details.Image = inspected.Config.Image
	}
	if inspected.ContainerJSONBase != nil && inspected.ContainerJSONBase.State != nil {
		details.State = inspected.ContainerJSONBase.State.Status
		details.Status = inspected.ContainerJSONBase.State.Status
		details.Running = inspected.ContainerJSONBase.State.Running
	}
	if inspected.HostConfig != nil {
		details.Privileged = inspected.HostConfig.Privileged
		details.Binds = append([]string(nil), inspected.HostConfig.Binds...)
	}

	return details, nil
}

func (i *DockerProvider) ContainerRun(ctx context.Context, spec types2.ContainerRunSpec) (types2.ContainerRunResult, error) {
	if i.client == nil {
		return types2.ContainerRunResult{}, errors.New("docker client is not initialized")
	}
	if strings.TrimSpace(spec.Image) == "" {
		return types2.ContainerRunResult{}, errors.New("container image is required")
	}
	if len(spec.Cmd) == 0 {
		return types2.ContainerRunResult{}, errors.New("container command is required")
	}

	created, err := i.client.ContainerCreate(
		ctx,
		&container.Config{
			Image:        spec.Image,
			Cmd:          append([]string(nil), spec.Cmd...),
			Env:          append([]string(nil), spec.Env...),
			AttachStdout: true,
			AttachStderr: true,
			Tty:          false,
		},
		&container.HostConfig{
			Binds:      append([]string(nil), spec.Binds...),
			Privileged: spec.Privileged,
			AutoRemove: spec.AutoRemove,
		},
		nil,
		nil,
		"",
	)
	if err != nil {
		return types2.ContainerRunResult{}, err
	}

	attached, err := i.client.ContainerAttach(ctx, created.ID, types.ContainerAttachOptions{
		Stream: true,
		Stdout: true,
		Stderr: true,
		Logs:   true,
	})
	if err != nil {
		_ = i.client.ContainerRemove(ctx, created.ID, types.ContainerRemoveOptions{Force: true})
		return types2.ContainerRunResult{}, err
	}
	defer attached.Close()

	var stdout bytes.Buffer
	var stderr bytes.Buffer
	copyDone := make(chan error, 1)
	go func() {
		_, copyErr := stdcopy.StdCopy(&stdout, &stderr, attached.Reader)
		copyDone <- copyErr
	}()

	if err := i.client.ContainerStart(ctx, created.ID, types.ContainerStartOptions{}); err != nil {
		_ = i.client.ContainerRemove(ctx, created.ID, types.ContainerRemoveOptions{Force: true})
		return types2.ContainerRunResult{}, err
	}

	statusCh, errCh := i.client.ContainerWait(ctx, created.ID, container.WaitConditionNotRunning)

	var exitCode int64
	select {
	case waitErr := <-errCh:
		if waitErr != nil {
			return types2.ContainerRunResult{}, waitErr
		}
	case waitResult := <-statusCh:
		if waitResult.Error != nil && waitResult.Error.Message != "" {
			return types2.ContainerRunResult{}, errors.New(waitResult.Error.Message)
		}
		exitCode = waitResult.StatusCode
	}

	if copyErr := <-copyDone; copyErr != nil {
		return types2.ContainerRunResult{}, copyErr
	}

	return types2.ContainerRunResult{
		ContainerID: created.ID,
		ExitCode:    exitCode,
		Stdout:      stdout.String(),
		Stderr:      stderr.String(),
	}, nil
}
