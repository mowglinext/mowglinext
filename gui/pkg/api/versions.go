package api

import (
	"context"
	"github.com/gin-gonic/gin"
	"github.com/mowglinext/mowglinext/pkg/buildinfo"
	"github.com/mowglinext/mowglinext/pkg/types"
	"net/http"
	"sort"
	"strings"
	"time"
)

type InstalledComponent struct {
	Component         string   `json:"component"`
	Name              string   `json:"name"`
	State             string   `json:"state"`
	Image             string   `json:"image"`
	ImageID           string   `json:"image_id"`
	Digests           []string `json:"digests"`
	Revision          string   `json:"revision,omitempty"`
	Version           string   `json:"version,omitempty"`
	BuiltAt           string   `json:"built_at,omitempty"`
	Architecture      string   `json:"architecture,omitempty"`
	MetadataAvailable bool     `json:"metadata_available"`
}

type VersionsResponse struct {
	ObservedAt      string               `json:"observed_at"`
	Server          buildinfo.Info       `json:"server"`
	DockerAvailable bool                 `json:"docker_available"`
	Components      []InstalledComponent `json:"components"`
}

// Match the installer-owned names rather than exposing unrelated host containers.
var versionComponents = map[string]string{
	"mowgli-ros2": "robot", "mowgli-gui": "gui", "mowgli-gps": "gps",
	"mowgli-lidar": "lidar", "mowgli-mqtt": "mqtt", "mowgli-watchtower": "watchtower",
	"mowgli-mavros": "mavros", "mowgli-ntrip": "ntrip", "mowgli-vesc": "vesc",
	"mowgli-tfluna-front": "tfluna-front", "mowgli-tfluna-edge": "tfluna-edge",
}

func installedVersions(ctx context.Context, provider types.IDockerProvider) VersionsResponse {
	result := VersionsResponse{ObservedAt: time.Now().UTC().Format(time.RFC3339), Server: buildinfo.Current(), Components: []InstalledComponent{}}
	containers, err := provider.ContainerList(ctx)
	if err != nil {
		return result
	}
	result.DockerAvailable = true
	images, hasImages := provider.(types.IImageMetadataProvider)
	for _, c := range containers {
		name, component := "", ""
		for _, n := range c.Names {
			if key, ok := versionComponents[strings.TrimPrefix(n, "/")]; ok {
				name, component = strings.TrimPrefix(n, "/"), key
				break
			}
		}
		if component == "" {
			continue
		}
		item := InstalledComponent{Component: component, Name: name, State: c.State, Image: c.Image, ImageID: c.ImageID, Digests: []string{}}
		if hasImages && c.ImageID != "" {
			if m, err := images.ImageMetadata(ctx, c.ImageID); err == nil {
				item.MetadataAvailable = true
				item.Digests = append(item.Digests, m.Digests...)
				item.Architecture = m.Architecture
				item.Revision = m.Labels["org.opencontainers.image.revision"]
				item.Version = m.Labels["org.opencontainers.image.version"]
				item.BuiltAt = m.Labels["org.opencontainers.image.created"]
			}
		}
		result.Components = append(result.Components, item)
	}
	sort.Slice(result.Components, func(i, j int) bool { return result.Components[i].Name < result.Components[j].Name })
	return result
}

// VersionsRoutes reports installed versions without pulling images or contacting a registry.
// @Summary Installed software versions
// @Tags system
// @Produce json
// @Success 200 {object} VersionsResponse
// @Router /system/versions [get]
func VersionsRoutes(r *gin.RouterGroup, provider types.IDockerProvider) {
	r.GET("/system/versions", func(c *gin.Context) {
		ctx, cancel := context.WithTimeout(c.Request.Context(), 5*time.Second)
		defer cancel()
		c.Header("Cache-Control", "no-store")
		c.JSON(http.StatusOK, installedVersions(ctx, provider))
	})
}
