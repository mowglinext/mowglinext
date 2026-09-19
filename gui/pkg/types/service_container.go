package types

import "context"

// ServiceContainerSpec describes a long-lived helper container the GUI owns
// outside the installer's Compose stack (today: the remote-access sidecar).
// Unlike ContainerRunSpec it is NOT waited on — it is created, started, and
// left running under a Docker restart policy.
type ServiceContainerSpec struct {
	// Name is the fixed container name (also how it is found again).
	Name  string
	Image string
	Env   []string
	// Binds are Docker "src:dst[:opts]" mounts; a bare name is a named volume.
	Binds  []string
	Labels map[string]string
	// NetworkMode is a Docker network mode ("host", "bridge", ...).
	NetworkMode string
	// RestartPolicy is a Docker restart policy name ("unless-stopped", ...).
	RestartPolicy string
	// CapDrop lists Linux capabilities to drop ("ALL" for an unprivileged
	// helper that needs none).
	CapDrop []string
	// Files are written into the created container BEFORE it starts, keyed by
	// absolute in-container path. This is how a config file reaches an image
	// without a host bind mount (the GUI does not know its own host paths).
	Files map[string][]byte
}

// IServiceContainerProvider is the subset of Docker operations needed to own a
// named helper container: pull its image, find/create/start/stop/remove it,
// and run commands inside it.
type IServiceContainerProvider interface {
	ImagePull(ctx context.Context, ref string) error
	// FindContainerByName returns the container details and true when a
	// container with exactly that name exists (running or not).
	FindContainerByName(ctx context.Context, name string) (ContainerDetails, bool, error)
	ContainerCreateService(ctx context.Context, spec ServiceContainerSpec) (string, error)
	ContainerRemove(ctx context.Context, containerID string, force bool) error
	ContainerStart(ctx context.Context, containerID string) error
	ContainerStop(ctx context.Context, containerID string) error
	ContainerInspect(ctx context.Context, containerID string) (ContainerDetails, error)
	ContainerExec(ctx context.Context, containerID string, spec ContainerExecSpec) (ContainerExecResult, error)
}
