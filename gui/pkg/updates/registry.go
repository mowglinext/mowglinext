package updates

import (
	"context"
	"crypto/sha256"
	"encoding/hex"
	"encoding/json"
	"fmt"
	"io"
	"net/http"
	"net/url"
	"regexp"
	"strings"
	"time"
)

type RemoteError struct{ Status int }

func (e RemoteError) Error() string { return fmt.Sprintf("remote HTTP status %d", e.Status) }

type Registry struct{ Client *http.Client }

func NewRegistry() *Registry { return &Registry{Client: &http.Client{Timeout: 15 * time.Second}} }

func Hash(data []byte) string {
	sum := sha256.Sum256(data)
	return "sha256:" + hex.EncodeToString(sum[:])
}

func Read(ctx context.Context, client *http.Client, address, token string) ([]byte, error) {
	req, err := http.NewRequestWithContext(ctx, "GET", address, nil)
	if err != nil {
		return nil, err
	}
	req.Header.Set("Accept", "application/vnd.oci.image.index.v1+json, application/vnd.docker.distribution.manifest.list.v2+json, application/vnd.oci.image.manifest.v1+json, application/vnd.docker.distribution.manifest.v2+json, application/json")
	if token != "" {
		req.Header.Set("Authorization", "Bearer "+token)
	}
	resp, err := client.Do(req)
	if err != nil {
		return nil, err
	}
	defer resp.Body.Close()
	if resp.StatusCode != 200 {
		return nil, RemoteError{resp.StatusCode}
	}
	data, err := io.ReadAll(io.LimitReader(resp.Body, 4*1024*1024+1))
	if len(data) > 4*1024*1024 {
		return nil, fmt.Errorf("remote document exceeds limit")
	}
	return data, err
}

var registryPathPattern = regexp.MustCompile(`^[a-z0-9]+(?:[._-][a-z0-9]+)*(?:/[a-z0-9]+(?:[._-][a-z0-9]+)*)+$`)

// Registry hosts are deliberately bounded; repository metadata cannot redirect authentication.
func RegistryLocation(repository string) (host, tokenURL, path string, err error) {
	host, path, ok := strings.Cut(repository, "/")
	if !ok || !registryPathPattern.MatchString(path) {
		return "", "", "", fmt.Errorf("unsupported image repository")
	}
	switch host {
	case "ghcr.io":
		tokenURL = "https://ghcr.io/token?service=ghcr.io"
	case "docker.io":
		host = "registry-1.docker.io"
		tokenURL = "https://auth.docker.io/token?service=registry.docker.io"
	default:
		return "", "", "", fmt.Errorf("unsupported image registry")
	}
	return host, tokenURL, path, nil
}

var tagPattern = regexp.MustCompile(`^[A-Za-z0-9_][A-Za-z0-9_.-]{0,127}$`)

// Resolve reads only registry manifests and image configuration, never layers.
// Registry and token hosts are fixed; callers cannot supply an arbitrary URL.
func (r *Registry) Resolve(ctx context.Context, repository, ref string) (Image, error) {
	image := Image{Repository: repository, Platforms: map[string]Platform{}}
	host, tokenURL, path, err := RegistryLocation(repository)
	if err != nil || (!tagPattern.MatchString(ref) && !DigestPattern.MatchString(ref)) {
		return image, fmt.Errorf("unsupported image reference")
	}
	auth, err := Read(ctx, r.Client, tokenURL+"&scope="+url.QueryEscape("repository:"+path+":pull"), "")
	if err != nil {
		return image, err
	}
	var credentials struct {
		Token string `json:"token"`
	}
	if err = json.Unmarshal(auth, &credentials); err != nil || credentials.Token == "" {
		return image, fmt.Errorf("registry authentication unavailable")
	}
	get := func(kind, reference string) ([]byte, error) {
		data, e := Read(ctx, r.Client, "https://"+host+"/v2/"+path+"/"+kind+"/"+reference, credentials.Token)
		if e == nil && DigestPattern.MatchString(reference) && Hash(data) != reference {
			e = fmt.Errorf("registry digest mismatch")
		}
		return data, e
	}
	data, err := get("manifests", ref)
	if err != nil {
		return image, err
	}
	image.Digest = Hash(data)
	type descriptor struct {
		Digest   string `json:"digest"`
		Platform struct {
			OS           string `json:"os"`
			Architecture string `json:"architecture"`
			Variant      string `json:"variant"`
		} `json:"platform"`
	}
	var index struct {
		Manifests []descriptor `json:"manifests"`
	}
	if err = json.Unmarshal(data, &index); err != nil {
		return image, err
	}
	readPlatform := func(manifest []byte, digest string) (string, Platform, error) {
		var m struct {
			Config descriptor `json:"config"`
		}
		if e := json.Unmarshal(manifest, &m); e != nil {
			return "", Platform{}, e
		}
		if !DigestPattern.MatchString(m.Config.Digest) {
			return "", Platform{}, fmt.Errorf("invalid image configuration digest")
		}
		config, e := get("blobs", m.Config.Digest)
		if e != nil {
			return "", Platform{}, e
		}
		var c struct {
			OS           string `json:"os"`
			Architecture string `json:"architecture"`
			Config       struct {
				Labels map[string]string `json:"Labels"`
			} `json:"config"`
		}
		if e = json.Unmarshal(config, &c); e != nil {
			return "", Platform{}, e
		}
		return c.OS + "/" + c.Architecture, Platform{Labels: c.Config.Labels, Manifest: digest, Config: m.Config.Digest, Revision: c.Config.Labels["org.opencontainers.image.revision"], Version: c.Config.Labels["org.opencontainers.image.version"], BuiltAt: c.Config.Labels["org.opencontainers.image.created"]}, nil
	}
	if len(index.Manifests) == 0 {
		arch, p, e := readPlatform(data, image.Digest)
		if e != nil {
			return image, e
		}
		image.Platforms[arch] = p
	} else {
		if len(index.Manifests) > 32 {
			return image, fmt.Errorf("too many image platforms")
		}
		for _, d := range index.Manifests {
			if d.Platform.OS != "linux" || (d.Platform.Architecture != "amd64" && d.Platform.Architecture != "arm64") {
				continue
			}
			if d.Platform.Variant != "" && !(d.Platform.Architecture == "arm64" && d.Platform.Variant == "v8") {
				continue
			}
			if !DigestPattern.MatchString(d.Digest) {
				return image, fmt.Errorf("invalid platform digest")
			}
			body, e := get("manifests", d.Digest)
			if e != nil {
				return image, e
			}
			arch, p, e := readPlatform(body, d.Digest)
			if e != nil {
				return image, e
			}
			if arch != d.Platform.OS+"/"+d.Platform.Architecture {
				return image, fmt.Errorf("platform identity mismatch")
			}
			if _, exists := image.Platforms[arch]; exists {
				return image, fmt.Errorf("ambiguous platform")
			}
			image.Platforms[arch] = p
		}
	}
	return image, nil
}
