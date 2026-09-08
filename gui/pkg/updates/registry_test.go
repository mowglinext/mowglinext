package updates

import (
	"context"
	"io"
	"net/http"
	"strings"
	"testing"
)

type transportFunc func(*http.Request) (*http.Response, error)

func (f transportFunc) RoundTrip(r *http.Request) (*http.Response, error) { return f(r) }
func response(body string) *http.Response {
	return &http.Response{StatusCode: 200, Body: io.NopCloser(strings.NewReader(body)), Header: http.Header{}}
}

func TestRegistryResolvesPlatformIdentityWithoutLayers(t *testing.T) {
	config := `{"architecture":"arm64","os":"linux","config":{"Labels":{"org.opencontainers.image.revision":"` + strings.Repeat("a", 40) + `"}}}`
	manifest := `{"config":{"digest":"` + Hash([]byte(config)) + `"},"layers":[{"digest":"never-download"}]}`
	index := `{"manifests":[{"digest":"` + Hash([]byte(manifest)) + `","platform":{"os":"linux","architecture":"arm64","variant":"v8"}},{"digest":"ignored","platform":{"os":"unknown","architecture":"unknown"}}]}`
	requests := 0
	tamperConfig := false
	r := Registry{Client: &http.Client{Transport: transportFunc(func(req *http.Request) (*http.Response, error) {
		requests++
		if req.URL.Host != "ghcr.io" {
			t.Fatal("unexpected host")
		}
		switch {
		case req.URL.Path == "/token":
			return response(`{"token":"read-only-token"}`), nil
		case strings.HasSuffix(req.URL.Path, "/manifests/dev"):
			return response(index), nil
		case strings.HasSuffix(req.URL.Path, "/manifests/"+Hash([]byte(manifest))):
			return response(manifest), nil
		case strings.HasSuffix(req.URL.Path, "/blobs/"+Hash([]byte(config))):
			if tamperConfig {
				return response(config + " "), nil
			}
			return response(config), nil
		default:
			t.Fatalf("unexpected registry request %s", req.URL)
			return nil, nil
		}
	})}}
	image, err := r.Resolve(context.Background(), "ghcr.io/owner/repo/gui", "dev")
	if err != nil {
		t.Fatal(err)
	}
	p := image.Platforms["linux/arm64"]
	if image.Digest != Hash([]byte(index)) || p.Manifest != Hash([]byte(manifest)) || p.Config != Hash([]byte(config)) || requests != 4 {
		t.Fatalf("wrong platform resolution: %+v (%d requests)", image, requests)
	}
	if _, err = r.Resolve(context.Background(), "http://127.0.0.1/private", "dev"); err == nil || requests != 4 {
		t.Fatal("accepted arbitrary URL")
	}
	tamperConfig = true
	if _, err = r.Resolve(context.Background(), "ghcr.io/owner/repo/gui", "dev"); err == nil {
		t.Fatal("accepted configuration bytes that do not match their digest")
	}
}

func TestImageComparisonNeverUsesSourceRevisionAsIdentity(t *testing.T) {
	i := Image{Repository: "ghcr.io/owner/repo/gps", Digest: "sha256:" + strings.Repeat("a", 64), Platforms: map[string]Platform{"linux/arm64": {Manifest: "sha256:" + strings.Repeat("b", 64), Config: "sha256:" + strings.Repeat("c", 64), Revision: strings.Repeat("d", 40)}}}
	if !Matches(i, "linux/arm64", i.Platforms["linux/arm64"].Config, nil) {
		t.Fatal("config identity did not match")
	}
	if !Matches(i, "linux/arm64", "", []string{i.Repository + "@" + i.Digest}) {
		t.Fatal("index identity did not match")
	}
	if Matches(i, "linux/arm64", i.Platforms["linux/arm64"].Revision, nil) || Matches(i, "linux/arm/v7", i.Digest, nil) {
		t.Fatal("compared unrelated identities/platforms")
	}
}

func TestDockerHubResolutionChecksEveryDigest(t *testing.T) {
	config := `{"architecture":"amd64","os":"linux","config":{}}`
	manifest := `{"config":{"digest":"` + Hash([]byte(config)) + `"}}`
	digest := Hash([]byte(manifest))
	tamper := false
	registry := Registry{Client: &http.Client{Transport: transportFunc(func(req *http.Request) (*http.Response, error) {
		if req.URL.Host == "auth.docker.io" && req.URL.Path == "/token" {
			if req.Header.Get("Authorization") != "" || req.URL.Query().Get("scope") != "repository:library/example:pull" {
				t.Fatal("incorrect authentication scope")
			}
			return response(`{"token":"example"}`), nil
		}
		if req.URL.Host != "registry-1.docker.io" || req.Header.Get("Authorization") != "Bearer example" {
			t.Fatal("unexpected authenticated endpoint", req.URL)
		}
		switch req.URL.Path {
		case "/v2/library/example/manifests/" + digest:
			if tamper {
				return response(manifest + " "), nil
			}
			return response(manifest), nil
		case "/v2/library/example/blobs/" + Hash([]byte(config)):
			return response(config), nil
		default:
			t.Fatal("unexpected request", req.URL)
			return nil, nil
		}
	})}}
	got, err := registry.Resolve(context.Background(), "docker.io/library/example", digest)
	if err != nil || got.Platforms["linux/amd64"].Config != Hash([]byte(config)) {
		t.Fatal(got, err)
	}
	tamper = true
	if _, err = registry.Resolve(context.Background(), "docker.io/library/example", digest); err == nil {
		t.Fatal("accepted changed index bytes")
	}
	for _, repo := range []string{"localhost/library/image", "docker.io/library/image:latest", "docker.io/library/image?x=1", "docker.io/../image", "docker.io/user@host/image", "https://docker.io/library/image"} {
		if _, _, _, err := RegistryLocation(repo); err == nil {
			t.Fatal("accepted invalid repository", repo)
		}
	}
}
