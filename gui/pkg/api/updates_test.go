package api

import (
	"context"
	"errors"
	"net/http/httptest"
	"strings"
	"testing"

	"github.com/gin-gonic/gin"
	"github.com/mowglinext/mowglinext/pkg/updates"
)

type fakeUpdates struct {
	image       updates.Image
	old         updates.Image
	resolveErr  error
	calls       int
	stableErr   error
	refs        []string
	relation    string
	comparisons int
}

func (f *fakeUpdates) Compare(context.Context, string, string, string) string {
	f.comparisons++
	return f.relation
}

func (f *fakeUpdates) Stable(context.Context, string) (string, string, error) {
	return "1.1.0", "https://github.com/owner/repo/releases/tag/v1.1.0", f.stableErr
}
func (f *fakeUpdates) Resolve(_ context.Context, _ string, ref string) (updates.Image, error) {
	f.calls++
	f.refs = append(f.refs, ref)
	if strings.HasPrefix(ref, "sha256:") {
		return f.old, f.resolveErr
	}
	return f.image, f.resolveErr
}

func updateFixture() (VersionsResponse, *fakeUpdates) {
	image := updates.Image{Repository: "ghcr.io/owner/repo/gps", Digest: "sha256:" + strings.Repeat("a", 64), Platforms: map[string]updates.Platform{"linux/arm64": {Manifest: "sha256:" + strings.Repeat("b", 64), Config: "sha256:" + strings.Repeat("c", 64), Revision: strings.Repeat("d", 40)}}}
	return VersionsResponse{DockerAvailable: true, Components: []InstalledComponent{{Name: "mowgli-gps", Image: "ghcr.io/owner/repo/gps:dev", ImageID: image.Platforms["linux/arm64"].Config, Architecture: "arm64", Revision: strings.Repeat("d", 40), MetadataAvailable: true}}}, &fakeUpdates{image: image, old: image}
}

func TestUpdateCheckTagsAndSameCommitRebuild(t *testing.T) {
	inv, source := updateFixture()
	if got := checkUpdates(context.Background(), inv, "owner/repo", "dev", source); got.State != "current" || source.refs[0] != "dev" || got.LastSuccessfulAt == "" {
		t.Fatalf("%+v", got)
	}
	inv.Components[0].ImageID = "sha256:" + strings.Repeat("f", 64)
	got := checkUpdates(context.Background(), inv, "owner/repo", "dev", source)
	if got.State != "available" || got.Components[0].State != "changed" {
		t.Fatalf("same-SHA rebuild hidden: %+v", got)
	}
	inv.Components[0].Digests = []string{"ghcr.io/owner/repo/gps@sha256:" + strings.Repeat("f", 64)}
	if got = checkUpdates(context.Background(), inv, "owner/repo", "dev", source); got.State != "current" {
		t.Fatalf("different index with same platform: %+v", got)
	}
}

func TestUpdateCheckStableUsesReleaseTagAndMissingImagesStayIncomplete(t *testing.T) {
	inv, source := updateFixture()
	got := checkUpdates(context.Background(), inv, "owner/repo", "stable", source)
	if got.State != "current" || source.refs[0] != "1.1.0" || got.Version != "1.1.0" || !strings.HasSuffix(got.NotesURL, "/releases/tag/v1.1.0") {
		t.Fatalf("%+v", got)
	}
	for _, status := range []int{404, 429, 503} {
		source.resolveErr = updates.RemoteError{Status: status}
		got = checkUpdates(context.Background(), inv, "owner/repo", "stable", source)
		if got.State != "incomplete" || got.LastSuccessfulAt != "" || got.Components[0].State != "unavailable" {
			t.Fatalf("failed check marked current: %+v", got)
		}
	}
	source.stableErr = errors.New("release unavailable")
	calls := source.calls
	if got = checkUpdates(context.Background(), inv, "owner/repo", "stable", source); got.State != "unavailable" || got.Version != "" || calls != source.calls {
		t.Fatal("failed release lookup fell back to a moving tag")
	}
}

func TestUpdateCheckMissingPlatformAndUnknownIdentity(t *testing.T) {
	inv, source := updateFixture()
	inv.Components[0].Architecture = "arm"
	if got := checkUpdates(context.Background(), inv, "owner/repo", "dev", source); got.State != "incomplete" || got.Components[0].State != "missing_platform" {
		t.Fatalf("%+v", got)
	}
	inv.Components[0].Architecture = "arm64"
	inv.Components[0].ImageID = ""
	inv.Components[0].MetadataAvailable = false
	if got := checkUpdates(context.Background(), inv, "owner/repo", "dev", source); got.Components[0].State != "unknown" {
		t.Fatalf("%+v", got)
	}
}

func TestUpdateCheckCustomPinnedAndUnsupportedImages(t *testing.T) {
	inv, source := updateFixture()
	inv.Components[0].Image = "ghcr.io/custom/repo/gps@" + source.image.Digest
	got := checkUpdates(context.Background(), inv, "owner/repo", "dev", source)
	if got.State != "current" || !got.Components[0].CustomImage || !got.Components[0].DigestReference {
		t.Fatalf("custom pinned image was not identified: %+v", got)
	}
	inv.Components = append(inv.Components, InstalledComponent{Name: "mowgli-watchtower", Image: "containrrr/watchtower:latest"})
	got = checkUpdates(context.Background(), inv, "owner/repo", "dev", source)
	if got.State != "incomplete" || got.Components[1].State != "unmanaged" {
		t.Fatalf("unsupported image caused a blanket match: %+v", got)
	}
}

func TestUpdatesEndpointIsReadOnlyAndCacheDoesNotContactRegistry(t *testing.T) {
	gin.SetMode(gin.TestMode)
	r := gin.New()
	_, source := updateFixture()
	registerUpdatesRoutes(r.Group("/api"), &versionDocker{}, "owner/repo", source)
	for _, channel := range []string{"dev", "stable"} {
		w := httptest.NewRecorder()
		r.ServeHTTP(w, httptest.NewRequest("GET", "/api/system/updates?channel="+channel, nil))
		if w.Code != 200 || w.Header().Get("Cache-Control") != "no-store" || !strings.Contains(w.Body.String(), "not_checked") {
			t.Fatalf("%d %s", w.Code, w.Body.String())
		}
	}
	if source.calls != 0 {
		t.Fatal("cached read contacted registry")
	}
	for _, request := range []struct {
		method, path string
		code         int
	}{{"GET", "/api/system/updates?channel=http://localhost", 400}, {"POST", "/api/system/updates", 404}} {
		w := httptest.NewRecorder()
		r.ServeHTTP(w, httptest.NewRequest(request.method, request.path, nil))
		if w.Code != request.code {
			t.Fatalf("%s returned %d", request.path, w.Code)
		}
	}
}

func TestSourceOrderingEnrichesChangedImagesWithoutAffectingDigestResults(t *testing.T) {
	for _, relation := range []string{"newer", "older", "diverged", "same", "unknown"} {
		inv, source := updateFixture()
		source.relation = relation
		if got := checkUpdates(context.Background(), inv, "owner/repo", "dev", source); source.comparisons != 0 || got.Components[0].SourceRelation != "" {
			t.Fatal("matching images do not need an ancestry lookup")
		}
		inv.Components[0].ImageID = "sha256:" + strings.Repeat("f", 64)
		second := inv.Components[0]
		second.Name = "mowgli-lidar"
		second.Image = "ghcr.io/custom/repo/lidar-ldlidar:dev"
		inv.Components = append(inv.Components, second)
		got := checkUpdates(context.Background(), inv, "owner/repo", "dev", source)
		if got.State != "available" || got.LastSuccessfulAt == "" || source.comparisons != 1 {
			t.Fatalf("ordering must be deduplicated and optional: %+v (%d)", got, source.comparisons)
		}
		for _, item := range got.Components {
			if item.State != "changed" || item.SourceRelation != relation {
				t.Fatalf("digest result lost: %+v", item)
			}
		}
	}
}
