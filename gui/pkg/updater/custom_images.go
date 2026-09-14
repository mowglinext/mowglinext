package updater

import (
	"context"
	"encoding/json"
	"errors"
	"fmt"
	"net/http"
	"regexp"
	"strconv"
	"strings"
	"time"

	"github.com/distribution/reference"
	"github.com/mowglinext/mowglinext/pkg/updates"
)

// Custom images have operator-approved provenance, not a release compatibility
// promise. Keep the entered reference beside the immutable installed identity.
type CustomImage struct {
	FirmwareProtocol int    `json:"firmware_protocol"`
	Repository       string `json:"repository"`
	ReleaseTag       string `json:"release_tag"`
	DeploymentID     string `json:"deployment_id"`
	Family           string `json:"family"`
	Requested        string `json:"requested"`
	Reference        string `json:"reference"`
	ImageID          string `json:"image_id"`
	Version          string `json:"version,omitempty"`
	Revision         string `json:"revision,omitempty"`
	BuiltAt          string `json:"built_at,omitempty"`
}

func customReference(input, current string) (string, error) {
	input = strings.TrimSpace(input)
	input = strings.TrimPrefix(input, "https://")
	input = strings.TrimPrefix(input, "docker://")
	if input == "" || len(input) > 512 || strings.ContainsAny(input, " \t\r\n?#") || strings.Contains(input, "://") {
		return "", errors.New("enter a Docker tag or registry/repository:tag (or @sha256:digest), without credentials or URL parameters")
	}
	if !strings.ContainsAny(input, "/:@") {
		base, err := reference.ParseNormalizedNamed(current)
		if err != nil {
			return "", errors.New("enter the complete image reference for this container")
		}
		input = reference.TrimNamed(base).Name() + ":" + input
	}
	named, err := reference.ParseNormalizedNamed(input)
	if err != nil {
		return "", errors.New("invalid Docker image reference; use the published image tag, not a GitHub branch URL")
	}
	if _, tagged := named.(reference.Tagged); !tagged {
		if _, pinned := named.(reference.Digested); !pinned {
			return "", errors.New("include an explicit tag or sha256 digest")
		}
	}
	if pinned, ok := named.(reference.Digested); ok && !updates.DigestPattern.MatchString(pinned.Digest().String()) {
		return "", errors.New("only sha256 image digests are supported")
	}
	return named.String(), nil
}

var pullDigest = regexp.MustCompile(`(?m)^Digest: (sha256:[a-f0-9]{64})\s*$`)

func (b DockerBackend) resolveCustomImage(ctx context.Context, input, current, service string, installed containerInfo) (CustomImage, error) {
	named, err := customReference(input, current)
	if err != nil {
		return CustomImage{}, err
	}
	// Download only, never start a candidate during review. Docker handles the
	// registry protocol/authentication using the updater account's configuration.
	output, err := command(ctx, "docker", "pull", "--platform", b.Config.Platform, named)
	if err != nil {
		return CustomImage{}, fmt.Errorf("cannot download %s; check the tag, platform and host registry access", named)
	}
	parsed, _ := reference.ParseNormalizedNamed(named)
	digest := ""
	if pinned, ok := parsed.(reference.Digested); ok {
		digest = pinned.Digest().String()
	} else {
		matches := pullDigest.FindSubmatch(output)
		if len(matches) != 2 {
			return CustomImage{}, errors.New("Docker did not return an immutable digest")
		}
		digest = string(matches[1])
	}
	resolved := reference.TrimNamed(parsed).Name() + "@" + digest
	// Inspect the digest returned by THIS pull, never the mutable tag, which
	// another process could repoint between download and inspection.
	data, err := command(ctx, "docker", "image", "inspect", resolved)
	if err != nil {
		return CustomImage{}, err
	}
	image, err := inspectCustomImage(data, input, resolved, service, b.Config.Platform, installed)
	if err != nil {
		return CustomImage{}, err
	}
	return image, verifyPublishedCustomImage(ctx, &http.Client{Timeout: 20 * time.Second}, image, b.Config.Platform)
}

func inspectCustomImage(data []byte, input, resolved, service, platform string, installed containerInfo) (CustomImage, error) {
	var images []struct {
		ID           string `json:"Id"`
		OS           string `json:"Os"`
		Architecture string `json:"Architecture"`
		Created      string `json:"Created"`
		Config       struct {
			Labels  map[string]string          `json:"Labels"`
			Volumes map[string]json.RawMessage `json:"Volumes"`
		} `json:"Config"`
	}
	if err := json.Unmarshal(data, &images); err != nil || len(images) != 1 {
		return CustomImage{}, errors.New("cannot inspect downloaded image")
	}
	i := images[0]
	if i.OS+"/"+i.Architecture != platform || !updates.DigestPattern.MatchString(i.ID) {
		return CustomImage{}, errors.New("image does not match this host platform")
	}
	if (service == "gui" || service == "mowgli") && i.Config.Labels["garden.mowgli.maintenance-api"] != "1" {
		return CustomImage{}, errors.New("custom GUI/robot image must support update maintenance API 1; older images cannot be installed safely")
	}
	for destination := range i.Config.Volumes {
		found := false
		for _, mount := range installed.Mounts {
			if mount.Destination == destination {
				found = true
			}
		}
		if !found {
			return CustomImage{}, errors.New("image introduces untracked volume storage; an explicit deployment migration is required")
		}
	}
	labels := i.Config.Labels
	protocol, err := strconv.Atoi(labels["garden.mowgli.firmware-protocol"])
	if err != nil || protocol < 1 {
		return CustomImage{}, errors.New("image does not declare its firmware protocol")
	}
	repo := strings.TrimPrefix(labels["org.opencontainers.image.source"], "https://github.com/")
	if !updates.RepositoryPattern.MatchString(repo) || labels["org.opencontainers.image.source"] != "https://github.com/"+repo || !revisionPattern.MatchString(labels["org.opencontainers.image.revision"]) || labels["garden.mowgli.image-contract"] != "1" || !idPattern.MatchString(labels["garden.mowgli.release"]) || !snapshotVersion.MatchString(labels["garden.mowgli.deployment"]) || !idPattern.MatchString(labels["garden.mowgli.image-family"]) {
		return CustomImage{}, errors.New("not a versioned MowgliNext release image: source, revision, release identity and image contract 1 are required")
	}
	if _, err := time.Parse(time.RFC3339, labels["org.opencontainers.image.created"]); err != nil {
		return CustomImage{}, errors.New("release image is missing a valid build date")
	}
	if service == "gui" && labels["garden.mowgli.updater-ui"] != "1" {
		return CustomImage{}, errors.New("GUI image predates the supported update interface")
	}
	return CustomImage{FirmwareProtocol: protocol, Repository: repo, ReleaseTag: labels["garden.mowgli.release"], DeploymentID: labels["garden.mowgli.deployment"], Family: labels["garden.mowgli.image-family"], Requested: strings.TrimSpace(input), Reference: resolved, ImageID: i.ID, Version: i.Config.Labels["org.opencontainers.image.version"], Revision: i.Config.Labels["org.opencontainers.image.revision"], BuiltAt: i.Created}, nil
}

func (b DockerBackend) PlanCustomImages(ctx context.Context, requested map[string]string) (map[string]CustomImage, int, error) {
	if pending, err := b.SelectionPending(); err != nil || pending {
		return nil, 0, errors.New("reconcile pending installer selections with a published deployment first")
	}
	ready, err := b.readiness(ctx)
	if err != nil || ready.FirmwareProtocol < 1 {
		return nil, 0, errors.New("mainboard firmware protocol is unavailable")
	}
	c, _, err := b.model(ctx)
	if err != nil {
		return nil, 0, err
	}
	managed, err := managedServices(c)
	if err != nil {
		return nil, 0, err
	}
	for name := range requested {
		if _, exists := managed[name]; !exists {
			return nil, 0, fmt.Errorf("%s is not an installed managed container", name)
		}
	}
	// Check the complete installed rollback baseline before any download.
	installed := map[string]containerInfo{}
	for name := range managed {
		ci, e := b.inspect(ctx, c.Services[name].ContainerName)
		if e != nil {
			return nil, 0, e
		}
		if e = validateManagedMounts(name, ci); e != nil {
			return nil, 0, e
		}
		if (name == "gui" || name == "mowgli") && ci.Config.Labels["garden.mowgli.maintenance-api"] != "1" {
			return nil, 0, errors.New("installed GUI/robot needs the installer maintenance upgrade first")
		}
		installed[name] = ci
	}
	free, err := freeBytes(b.Config.StateDir)
	if err != nil || free < 2*1024*1024*1024 {
		return nil, 0, errors.New("at least 2 GiB free is required before downloading custom images")
	}
	result := map[string]CustomImage{}
	for name, input := range requested {
		image, e := b.resolveCustomImage(ctx, input, c.Services[name].Image, name, installed[name])
		if e != nil {
			return nil, 0, fmt.Errorf("%s: %w", name, e)
		}
		if image.FirmwareProtocol != ready.FirmwareProtocol {
			return nil, 0, fmt.Errorf("%s: release requires a different mainboard firmware protocol", name)
		}
		if image.Family != managed[name].Image {
			return nil, 0, fmt.Errorf("%s: image belongs to another component family", name)
		}
		result[name] = image
	}
	return result, ready.FirmwareProtocol, nil
}

// Custom mode changes images in the CURRENT stack. It cannot smuggle in a
// different Compose definition, hardware selection, service or updater binary.
func (m *Manager) MakeCustomPlan(ctx context.Context, requested map[string]string, acknowledged bool) (Plan, error) {
	m.mu.Lock()
	locked, planning := true, false
	defer func() {
		if !locked {
			m.mu.Lock()
		}
		if planning {
			m.busy = false
		}
		m.mu.Unlock()
	}()
	if !acknowledged {
		return Plan{}, errors.New("acknowledge custom image compatibility and trust risks before downloading")
	}
	if m.busy || m.checking || m.state.Job.Pending() {
		return Plan{}, errors.New("update or check in progress")
	}
	if len(requested) == 0 || len(requested) > 32 {
		return Plan{}, errors.New("choose at least one installed container, up to 32")
	}
	b, ok := m.backend.(interface {
		PlanCustomImages(context.Context, map[string]string) (map[string]CustomImage, int, error)
	})
	if !ok {
		return Plan{}, errors.New("custom images are not supported by this updater")
	}
	// Keep status reads responsive during downloads; serialize all mutating
	// manager operations without holding the status mutex across Docker calls.
	m.busy, planning = true, true
	m.mu.Unlock()
	locked = false
	ctx, cancel := context.WithTimeout(ctx, 3*time.Minute)
	defer cancel()
	if locker, ok := m.backend.(interface{ Lock() (func(), error) }); ok {
		unlock, err := locker.Lock()
		if err != nil {
			return Plan{}, err
		}
		defer unlock()
	}
	fingerprint, previous, err := m.backend.Inventory(ctx)
	if err != nil {
		return Plan{}, err
	}
	custom, protocol, err := b.PlanCustomImages(ctx, requested)
	if err != nil {
		return Plan{}, err
	}
	after, _, err := m.backend.Inventory(ctx)
	if err != nil || after != fingerprint {
		return Plan{}, errors.New("installed stack changed during image download; review again")
	}
	images := map[string]string{}
	for name, image := range previous {
		images[name] = image
	}
	for name, image := range custom {
		if _, ok := previous[name]; !ok {
			return Plan{}, errors.New("custom image service is not installed")
		}
		images[name] = image.Reference
	}
	m.mu.Lock()
	locked = true
	target := Deployment{ID: fmt.Sprintf("custom-%d", m.now().UnixNano()), FirmwareProtocol: protocol}
	p := Plan{ID: fmt.Sprintf("plan-%d", m.now().UnixNano()), Target: target, Policy: m.state.Policy, Fingerprint: fingerprint, ExpiresAt: m.now().Add(15 * time.Minute), Images: images, Previous: previous, CustomImages: custom}
	m.state.Plans = []Plan{p}
	return p, m.save()
}
