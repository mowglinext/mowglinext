package providers

import (
	"context"
	"errors"
	pkgtypes "github.com/mowglinext/mowglinext/pkg/types"
)

func (i *DockerProvider) ImageMetadata(ctx context.Context, imageID string) (pkgtypes.ImageMetadata, error) {
	if i.client == nil {
		return pkgtypes.ImageMetadata{}, errors.New("docker client is not initialized")
	}
	image, _, err := i.client.ImageInspectWithRaw(ctx, imageID)
	if err != nil {
		return pkgtypes.ImageMetadata{}, err
	}
	result := pkgtypes.ImageMetadata{Digests: image.RepoDigests, Architecture: image.Architecture}
	if image.Config != nil {
		result.Labels = image.Config.Labels
	}
	return result, nil
}
