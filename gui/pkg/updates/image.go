// Package updates compares installed images with registry tags using immutable digests.
package updates

import (
	"regexp"
	"strings"
)

var RepositoryPattern = regexp.MustCompile(`^[A-Za-z0-9][A-Za-z0-9_.-]*/[A-Za-z0-9][A-Za-z0-9_.-]*$`)
var DigestPattern = regexp.MustCompile(`^sha256:[a-f0-9]{64}$`)

type Platform struct {
	Labels   map[string]string `json:"labels,omitempty"`
	Manifest string            `json:"manifest"`
	Config   string            `json:"config"`
	Revision string            `json:"revision"`
	Version  string            `json:"version,omitempty"`
	BuiltAt  string            `json:"built_at,omitempty"`
}

type Image struct {
	Repository string              `json:"repository"`
	Digest     string              `json:"digest"`
	Platforms  map[string]Platform `json:"platforms"`
}

var ImageNames = []string{"mowgli-ros2", "mowglinext-gui", "gps", "lidar-ldlidar", "lidar-rplidar", "lidar-stl27l"}

// Matches compares like-for-like immutable identities. A source revision alone
// is never sufficient: rebuilding the same commit can change the image bytes.
func Matches(image Image, platform, imageID string, digests []string) bool {
	p, ok := image.Platforms[platform]
	if !ok {
		return false
	}
	if imageID == p.Config || imageID == p.Manifest || imageID == image.Digest {
		return true
	}
	for _, d := range digests {
		parts := strings.Split(d, "@")
		if len(parts) == 2 && (parts[1] == image.Digest || parts[1] == p.Manifest) {
			return true
		}
	}
	return false
}
