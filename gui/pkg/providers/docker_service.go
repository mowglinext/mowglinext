package providers

import (
	"archive/tar"
	"bytes"
	"context"
	"errors"
	"fmt"
	"io"
	"path"
	"sort"
	"strings"
	"time"

	"github.com/docker/docker/api/types"
	"github.com/docker/docker/api/types/container"
	"github.com/docker/docker/api/types/filters"
	types2 "github.com/mowglinext/mowglinext/pkg/types"
)

// Long-lived helper containers owned by the GUI (see
// types.IServiceContainerProvider). These sit next to the ContainerRun /
// ContainerExec helpers in docker.go: the difference is that a *service*
// container is created once and left running under a restart policy.

const serviceFileMode = 0o644

// ImagePull pulls an image reference, draining Docker's progress stream so
// the call returns only once the layers are on disk (or the pull failed).
func (i *DockerProvider) ImagePull(ctx context.Context, ref string) error {
	if i.client == nil {
		return errors.New("docker client is not initialized")
	}
	if strings.TrimSpace(ref) == "" {
		return errors.New("image reference is required")
	}
	reader, err := i.client.ImagePull(ctx, ref, types.ImagePullOptions{})
	if err != nil {
		return fmt.Errorf("pull %s: %w", ref, err)
	}
	defer reader.Close()
	if _, err := io.Copy(io.Discard, reader); err != nil {
		return fmt.Errorf("pull %s: %w", ref, err)
	}
	return nil
}

// FindContainerByName looks a container up by its exact name.
func (i *DockerProvider) FindContainerByName(ctx context.Context, name string) (types2.ContainerDetails, bool, error) {
	if i.client == nil {
		return types2.ContainerDetails{}, false, errors.New("docker client is not initialized")
	}
	if strings.TrimSpace(name) == "" {
		return types2.ContainerDetails{}, false, errors.New("container name is required")
	}
	// The name filter is a substring match on the daemon side, so the exact
	// comparison below is still needed ("mowgli-remote" vs "mowgli-remote2").
	list, err := i.client.ContainerList(ctx, types.ContainerListOptions{
		All:     true,
		Filters: filters.NewArgs(filters.Arg("name", name)),
	})
	if err != nil {
		return types2.ContainerDetails{}, false, err
	}
	for _, c := range list {
		for _, n := range c.Names {
			if strings.TrimPrefix(n, "/") == name {
				details, err := i.ContainerInspect(ctx, c.ID)
				if err != nil {
					return types2.ContainerDetails{}, false, err
				}
				return details, true, nil
			}
		}
	}
	return types2.ContainerDetails{}, false, nil
}

// ContainerCreateService creates (but does not start) a named service
// container and copies spec.Files into it.
func (i *DockerProvider) ContainerCreateService(ctx context.Context, spec types2.ServiceContainerSpec) (string, error) {
	if i.client == nil {
		return "", errors.New("docker client is not initialized")
	}
	if strings.TrimSpace(spec.Name) == "" {
		return "", errors.New("container name is required")
	}
	if strings.TrimSpace(spec.Image) == "" {
		return "", errors.New("container image is required")
	}
	hostConfig := &container.HostConfig{
		Binds:       append([]string(nil), spec.Binds...),
		NetworkMode: container.NetworkMode(spec.NetworkMode),
		CapDrop:     append([]string(nil), spec.CapDrop...),
	}
	if spec.RestartPolicy != "" {
		hostConfig.RestartPolicy = container.RestartPolicy{Name: spec.RestartPolicy}
	}
	created, err := i.client.ContainerCreate(
		ctx,
		&container.Config{
			Image:  spec.Image,
			Env:    append([]string(nil), spec.Env...),
			Labels: copyLabels(spec.Labels),
		},
		hostConfig,
		nil,
		nil,
		spec.Name,
	)
	if err != nil {
		return "", err
	}
	if len(spec.Files) > 0 {
		archive, err := tarFiles(spec.Files)
		if err != nil {
			_ = i.client.ContainerRemove(ctx, created.ID, types.ContainerRemoveOptions{Force: true})
			return "", err
		}
		if err := i.client.CopyToContainer(ctx, created.ID, "/", archive, types.CopyToContainerOptions{}); err != nil {
			_ = i.client.ContainerRemove(ctx, created.ID, types.ContainerRemoveOptions{Force: true})
			return "", fmt.Errorf("copy files into %s: %w", spec.Name, err)
		}
	}
	return created.ID, nil
}

// ContainerRemove deletes a container; force also kills a running one.
func (i *DockerProvider) ContainerRemove(ctx context.Context, containerID string, force bool) error {
	if i.client == nil {
		return errors.New("docker client is not initialized")
	}
	if strings.TrimSpace(containerID) == "" {
		return errors.New("container id is required")
	}
	return i.client.ContainerRemove(ctx, containerID, types.ContainerRemoveOptions{Force: force})
}

func copyLabels(in map[string]string) map[string]string {
	out := make(map[string]string, len(in))
	for k, v := range in {
		out[k] = v
	}
	return out
}

// tarFiles packs absolute in-container paths into a tar rooted at "/",
// emitting each parent directory first so extraction never depends on the
// image already having it.
func tarFiles(files map[string][]byte) (io.Reader, error) {
	var buf bytes.Buffer
	tw := tar.NewWriter(&buf)
	now := time.Now()
	seenDirs := map[string]bool{}
	paths := make([]string, 0, len(files))
	for p := range files {
		paths = append(paths, p)
	}
	sort.Strings(paths)
	for _, p := range paths {
		if !path.IsAbs(p) {
			return nil, fmt.Errorf("container file path must be absolute: %s", p)
		}
		rel := strings.TrimPrefix(path.Clean(p), "/")
		for _, dir := range parentDirs(rel) {
			if seenDirs[dir] {
				continue
			}
			seenDirs[dir] = true
			if err := tw.WriteHeader(&tar.Header{
				Typeflag: tar.TypeDir, Name: dir + "/", Mode: 0o755, ModTime: now,
			}); err != nil {
				return nil, err
			}
		}
		content := files[p]
		if err := tw.WriteHeader(&tar.Header{
			Typeflag: tar.TypeReg, Name: rel, Mode: serviceFileMode, Size: int64(len(content)), ModTime: now,
		}); err != nil {
			return nil, err
		}
		if _, err := tw.Write(content); err != nil {
			return nil, err
		}
	}
	if err := tw.Close(); err != nil {
		return nil, err
	}
	return &buf, nil
}

// parentDirs lists a relative path's ancestors shallowest first
// ("a/b/c.txt" → ["a", "a/b"]) so a tar extractor meets each directory
// before anything inside it.
func parentDirs(rel string) []string {
	var dirs []string
	for dir := path.Dir(rel); dir != "." && dir != "/"; dir = path.Dir(dir) {
		dirs = append([]string{dir}, dirs...)
	}
	return dirs
}
