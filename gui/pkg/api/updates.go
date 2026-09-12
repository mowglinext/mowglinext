package api

import (
	"context"
	"encoding/json"
	"errors"
	"os"
	"strings"
	"sync"
	"time"

	"github.com/gin-gonic/gin"
	"github.com/mowglinext/mowglinext/pkg/types"
	"github.com/mowglinext/mowglinext/pkg/updates"
)

type UpdateComponent struct {
	Name              string `json:"name"`
	State             string `json:"state"`
	InstalledRevision string `json:"installed_revision,omitempty"`
	AvailableRevision string `json:"available_revision,omitempty"`
	AvailableImage    string `json:"available_image,omitempty"`
	SourceRelation    string `json:"source_relation,omitempty"`
	CustomImage       bool   `json:"custom_image"`
	DigestReference   bool   `json:"digest_reference"`
}

type UpdateCheck struct {
	Channel          string            `json:"channel"`
	State            string            `json:"state"`
	CheckedAt        string            `json:"checked_at,omitempty"`
	LastSuccessfulAt string            `json:"last_successful_at,omitempty"`
	Version          string            `json:"version,omitempty"`
	NotesURL         string            `json:"notes_url,omitempty"`
	Components       []UpdateComponent `json:"components"`
}

type updateSource interface {
	Resolve(context.Context, string, string) (updates.Image, error)
	Stable(context.Context, string) (string, string, error)
	Compare(context.Context, string, string, string) string
}
type remoteUpdates struct {
	*updates.Registry
	revisions map[string]string // Immutable pairs; accessed under the route lock.
}

func (r remoteUpdates) Compare(ctx context.Context, repo, installed, available string) string {
	key := repo + "/" + installed + "..." + available
	if relation, ok := r.revisions[key]; ok {
		return relation
	}
	relation := updates.CompareRevisions(ctx, r.Client, repo, installed, available)
	if relation != "unknown" && r.revisions != nil {
		if len(r.revisions) >= 128 {
			clear(r.revisions)
		}
		r.revisions[key] = relation
	}
	return relation
}

func (r remoteUpdates) Stable(ctx context.Context, repo string) (string, string, error) {
	body, err := updates.Read(ctx, r.Client, "https://api.github.com/repos/"+repo+"/releases/latest", "")
	if err != nil {
		return "", "", err
	}
	var release struct {
		Tag        string `json:"tag_name"`
		Draft      bool   `json:"draft"`
		Prerelease bool   `json:"prerelease"`
	}
	if err = json.Unmarshal(body, &release); err != nil || release.Draft || release.Prerelease || release.Tag == "" {
		return "", "", errors.New("stable release unavailable")
	}
	return strings.TrimPrefix(release.Tag, "v"), "https://github.com/" + repo + "/releases/tag/" + release.Tag, nil
}

func checkUpdates(ctx context.Context, inventory VersionsResponse, repo, channel string, source updateSource) UpdateCheck {
	result := UpdateCheck{Channel: channel, State: "unavailable", CheckedAt: time.Now().UTC().Format(time.RFC3339), Components: []UpdateComponent{}}
	if !inventory.DockerAvailable {
		return result
	}
	ref := "dev"
	result.NotesURL = "https://github.com/" + repo + "/commits/dev"
	if channel == "stable" {
		var err error
		ref, result.NotesURL, err = source.Stable(ctx, repo)
		if err != nil {
			return result
		}
	}
	result.Version = ref
	changed, incomplete, managed := false, false, 0
	for _, installed := range inventory.Components {
		item := UpdateComponent{Name: installed.Name, InstalledRevision: installed.Revision, DigestReference: strings.Contains(installed.Image, "@"), State: "unmanaged"}
		repository := strings.Split(strings.Split(installed.Image, "@")[0], ":")[0]
		name := repository[strings.LastIndex(repository, "/")+1:]
		known := false
		for _, candidate := range updates.ImageNames {
			if candidate == name {
				known = true
				break
			}
		}
		if !known {
			result.Components = append(result.Components, item)
			incomplete = true
			continue
		}
		managed++
		targetRepo := "ghcr.io/" + strings.ToLower(repo) + "/" + name
		item.CustomImage = repository != targetRepo
		target, err := source.Resolve(ctx, targetRepo, ref)
		if err != nil {
			item.State = "unavailable"
			incomplete = true
			result.Components = append(result.Components, item)
			continue
		}
		platform := "linux/" + installed.Architecture
		p, ok := target.Platforms[platform]
		item.AvailableImage = target.Repository + "@" + target.Digest
		item.AvailableRevision = p.Revision
		if !ok {
			item.State = "missing_platform"
			incomplete = true
		} else if updates.Matches(target, platform, installed.ImageID, installed.Digests) {
			item.State = "current"
		} else {
			// An older multiarch index may still contain the same platform image.
			matched, identified := false, installed.ImageID != "" && installed.MetadataAvailable
			for _, digest := range installed.Digests {
				parts := strings.Split(digest, "@")
				if len(parts) != 2 || !strings.HasPrefix(parts[0], "ghcr.io/") {
					continue
				}
				old, e := source.Resolve(ctx, parts[0], parts[1])
				if e != nil {
					identified = false
					continue
				}
				if previous, exists := old.Platforms[platform]; exists && previous.Manifest == p.Manifest {
					matched = true
					break
				}
			}
			if matched {
				item.State = "current"
			} else if !identified {
				item.State = "unknown"
				incomplete = true
			} else {
				item.State = "changed"
				changed = true
			}
		}
		result.Components = append(result.Components, item)
	}
	if managed == 0 {
		return result
	}
	if incomplete {
		result.State = "incomplete"
	} else if changed {
		result.State = "available"
	} else {
		result.State = "current"
	}
	if !incomplete {
		result.LastSuccessfulAt = result.CheckedAt
	}
	// Optional source ordering must not consume the image comparison budget or
	// turn a successful digest check into a failure if GitHub is unavailable.
	ancestryCtx, cancel := context.WithTimeout(ctx, 5*time.Second)
	defer cancel()
	compared := map[string]string{}
	for i := range result.Components {
		item := &result.Components[i]
		if item.State != "changed" {
			continue
		}
		key := item.InstalledRevision + "..." + item.AvailableRevision
		relation, exists := compared[key]
		if !exists {
			relation = source.Compare(ancestryCtx, repo, item.InstalledRevision, item.AvailableRevision)
			compared[key] = relation
		}
		item.SourceRelation = relation
	}
	return result
}

// UpdatesRoutes only reads installed and remote metadata. Checking never pulls,
// changes channels, rewrites Compose, or starts/stops a container.
// @Summary Check available software versions
// @Tags system
// @Produce json
// @Param channel query string true "Comparison channel" Enums(stable,dev)
// @Param check query boolean false "Check remote metadata (otherwise cached result)"
// @Success 200 {object} UpdateCheck
// @Router /system/updates [get]
func UpdatesRoutes(r *gin.RouterGroup, provider types.IDockerProvider) {
	repo := os.Getenv("UPDATES_REPOSITORY")
	if repo == "" {
		repo = "mowglinext/mowglinext"
	}
	source := remoteUpdates{Registry: updates.NewRegistry(), revisions: map[string]string{}}
	registerUpdatesRoutes(r, provider, repo, source)
}

func registerUpdatesRoutes(r *gin.RouterGroup, provider types.IDockerProvider, repo string, source updateSource) {
	var lock sync.Mutex
	cache := map[string]UpdateCheck{}
	r.GET("/system/updates", func(c *gin.Context) {
		channel := c.DefaultQuery("channel", "dev")
		if (channel != "dev" && channel != "stable") || !updates.RepositoryPattern.MatchString(repo) {
			c.JSON(400, ErrorResponse{Error: "invalid update channel or repository"})
			return
		}
		c.Header("Cache-Control", "no-store")
		lock.Lock()
		defer lock.Unlock()
		previous, exists := cache[channel]
		checked, _ := time.Parse(time.RFC3339, previous.CheckedAt)
		if c.Query("check") != "true" || (exists && time.Since(checked) < time.Minute) {
			if !exists {
				previous = UpdateCheck{Channel: channel, State: "not_checked", Components: []UpdateComponent{}}
			}
			c.JSON(200, previous)
			return
		}
		ctx, cancel := context.WithTimeout(c.Request.Context(), 45*time.Second)
		defer cancel()
		result := checkUpdates(ctx, installedVersions(ctx, provider), repo, channel, source)
		if c.Request.Context().Err() != nil {
			return // A disconnected client must not replace a usable cached result.
		}
		if result.LastSuccessfulAt == "" {
			result.LastSuccessfulAt = previous.LastSuccessfulAt
		}
		cache[channel] = result
		c.JSON(200, result)
	})
}
