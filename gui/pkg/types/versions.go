package types

import "context"

// ImageMetadata is read by immutable image ID, never by a moving tag.
type ImageMetadata struct {
	Digests      []string
	Labels       map[string]string
	Architecture string
}

type IImageMetadataProvider interface {
	ImageMetadata(context.Context, string) (ImageMetadata, error)
}
