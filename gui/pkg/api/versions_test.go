package api

import (
	"context"
	"encoding/json"
	"errors"
	docker "github.com/docker/docker/api/types"
	"github.com/gin-gonic/gin"
	pkgtypes "github.com/mowglinext/mowglinext/pkg/types"
	"net/http/httptest"
	"strings"
	"testing"
)

type versionDocker struct {
	pkgtypes.IDockerProvider
	containers []docker.Container
	listErr    error
	metadata   map[string]pkgtypes.ImageMetadata
	inspected  []string
}

func (f *versionDocker) ContainerList(context.Context) ([]docker.Container, error) {
	return f.containers, f.listErr
}
func (f *versionDocker) ImageMetadata(_ context.Context, id string) (pkgtypes.ImageMetadata, error) {
	f.inspected = append(f.inspected, id)
	m, ok := f.metadata[id]
	if !ok {
		return m, errors.New("image unavailable")
	}
	return m, nil
}

func TestVersionsUseRunningImageAndAllowlistedMetadata(t *testing.T) {
	f := &versionDocker{containers: []docker.Container{
		{Names: []string{"/unrelated-db"}, ImageID: "private"},
		{Names: []string{"/mowgli-gui"}, Image: "ghcr.io/example/gui:dev", ImageID: "sha256:installed", State: "running", Labels: map[string]string{"org.opencontainers.image.revision": "overridden"}},
	}, metadata: map[string]pkgtypes.ImageMetadata{"sha256:installed": {Digests: []string{"ghcr.io/example/gui@sha256:manifest"}, Labels: map[string]string{"org.opencontainers.image.revision": "actual-build", "org.opencontainers.image.version": "dev", "SECRET": "do-not-expose"}, Architecture: "arm64"}}}
	got := installedVersions(context.Background(), f)
	if !got.DockerAvailable || len(got.Components) != 1 {
		t.Fatalf("unexpected inventory: %+v", got)
	}
	c := got.Components[0]
	if c.Revision != "actual-build" || c.ImageID != "sha256:installed" || !c.MetadataAvailable || c.Architecture != "arm64" {
		t.Fatalf("wrong identity: %+v", c)
	}
	if len(f.inspected) != 1 || f.inspected[0] != "sha256:installed" {
		t.Fatalf("inspected moving tag/unrelated image: %v", f.inspected)
	}
	b, _ := json.Marshal(got)
	if strings.Contains(string(b), "do-not-expose") || strings.Contains(string(b), "overridden") {
		t.Fatal("leaked non-authoritative labels")
	}
}

func TestVersionsPartialMetadataAndStoppedContainers(t *testing.T) {
	f := &versionDocker{containers: []docker.Container{{Names: []string{"/mowgli-ros2"}, ImageID: "removed", State: "exited"}, {Names: []string{"/mowgli-gps"}, ImageID: "unlabelled", State: "running"}}, metadata: map[string]pkgtypes.ImageMetadata{"unlabelled": {}}}
	got := installedVersions(context.Background(), f)
	if len(got.Components) != 2 || !got.DockerAvailable {
		t.Fatalf("lost partial inventory: %+v", got)
	}
	if got.Components[0].Revision != "" || !got.Components[0].MetadataAvailable || got.Components[1].MetadataAvailable || got.Components[1].State != "exited" {
		t.Fatalf("invented metadata/health: %+v", got.Components)
	}
}

func TestVersionsIncludeOptionalDistanceSensors(t *testing.T) {
	f := &versionDocker{containers: []docker.Container{
		{Names: []string{"/mowgli-tfluna-front"}, State: "running"},
		{Names: []string{"/mowgli-tfluna-edge"}, State: "running"},
	}}
	got := installedVersions(context.Background(), f)
	if len(got.Components) != 2 || got.Components[0].Component != "tfluna-edge" || got.Components[1].Component != "tfluna-front" {
		t.Fatalf("missing distance sensors: %+v", got.Components)
	}
}

func TestVersionsDockerUnavailableStillReturnsServerAndNoStore(t *testing.T) {
	gin.SetMode(gin.TestMode)
	r := gin.New()
	VersionsRoutes(r.Group("/api"), &versionDocker{listErr: errors.New("sensitive daemon detail")})
	w := httptest.NewRecorder()
	r.ServeHTTP(w, httptest.NewRequest("GET", "/api/system/versions", nil))
	if w.Code != 200 || w.Header().Get("Cache-Control") != "no-store" {
		t.Fatalf("response %d %s", w.Code, w.Body.String())
	}
	var got VersionsResponse
	if err := json.Unmarshal(w.Body.Bytes(), &got); err != nil {
		t.Fatal(err)
	}
	if got.DockerAvailable || got.Components == nil || got.ObservedAt == "" || strings.Contains(w.Body.String(), "sensitive") {
		t.Fatalf("incorrect unavailable response: %s", w.Body.String())
	}
	w = httptest.NewRecorder()
	r.ServeHTTP(w, httptest.NewRequest("POST", "/api/system/versions", nil))
	if w.Code != 404 {
		t.Fatal("inventory must be read-only")
	}
}
