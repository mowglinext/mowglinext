package api

import (
	"context"
	"encoding/json"
	"net/http"
	"net/http/httptest"
	"sync/atomic"
	"testing"
	"time"

	"github.com/gin-gonic/gin"
)

func TestLatestDeploymentByBranchKeepsTheNewestPrerelease(t *testing.T) {
	got := latestDeploymentByBranch([]githubRelease{
		{Name: "fix/a deployment-aaaaaaaaaaaa-2-1", Prerelease: true},
		{Name: "fix/a deployment-bbbbbbbbbbbb-1-1", Prerelease: true},
		{Name: "dev deployment-cccccccccccc-9-1", Prerelease: true},
		{Name: "v1.6.0", Prerelease: false},
		{Name: "feat/x deployment-dddddddddddd-3-1", Prerelease: true, Draft: true},
		{Name: "no-deployment-suffix", Prerelease: true},
	})
	want := map[string]string{"fix/a": "deployment-aaaaaaaaaaaa-2-1", "dev": "deployment-cccccccccccc-9-1"}
	if len(got) != len(want) {
		t.Fatalf("got %v, want %v", got, want)
	}
	for branch, deployment := range want {
		if got[branch] != deployment {
			t.Errorf("%s: got %q, want %q", branch, got[branch], deployment)
		}
	}
}

func pull(number int, branch, headRepo string, labels ...string) githubPull {
	var p githubPull
	p.Number, p.Title, p.Head.Ref = number, "PR", branch
	if headRepo != "" {
		p.Head.Repo = &struct {
			FullName string `json:"full_name"`
		}{FullName: headRepo}
	}
	for _, l := range labels {
		p.Labels = append(p.Labels, githubLabel{Name: l})
	}
	return p
}

func TestPullRequestsForMarksOnlyPublishedSameRepoBranchesInstallable(t *testing.T) {
	const repo = "mowglinext/mowglinext"
	got := pullRequestsFor(repo, []githubPull{
		pull(1, "fix/built", repo, PreviewLabel),
		pull(2, "fix/unbuilt", repo),
		pull(3, "fix/built", "someone/mowglinext", PreviewLabel), // fork reusing the name
		pull(4, "feat/deleted-fork", ""),
	}, []githubRelease{{Name: "fix/built deployment-aaaaaaaaaaaa-1-1", Prerelease: true}})

	if got.Repository != repo || got.Label != PreviewLabel || len(got.PullRequests) != 4 {
		t.Fatalf("unexpected listing %+v", got)
	}
	built, unbuilt, fork, gone := got.PullRequests[0], got.PullRequests[1], got.PullRequests[2], got.PullRequests[3]
	if !built.Installable || built.Deployment != "deployment-aaaaaaaaaaaa-1-1" || !built.Labeled || built.FromFork {
		t.Errorf("same-repo published PR: %+v", built)
	}
	if unbuilt.Installable || unbuilt.Labeled {
		t.Errorf("unpublished PR must not be installable: %+v", unbuilt)
	}
	if fork.Installable || !fork.FromFork {
		t.Errorf("a fork PR must never match this repository's release: %+v", fork)
	}
	if gone.Installable || !gone.FromFork {
		t.Errorf("a PR without a head repository is treated as a fork: %+v", gone)
	}
}

func fakeGitHub(t *testing.T, calls *atomic.Int32) *httptest.Server {
	t.Helper()
	return httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		calls.Add(1)
		switch r.URL.Path {
		case "/repos/mowglinext/mowglinext/pulls":
			if r.URL.Query().Get("state") != "open" {
				t.Errorf("pulls queried without state=open: %s", r.URL.RawQuery)
			}
			_ = json.NewEncoder(w).Encode([]githubPull{pull(7, "fix/x", "mowglinext/mowglinext", PreviewLabel)})
		case "/repos/mowglinext/mowglinext/releases":
			_ = json.NewEncoder(w).Encode([]githubRelease{{Name: "fix/x deployment-abcdefabcdef-5-1", Prerelease: true}})
		default:
			http.NotFound(w, r)
		}
	}))
}

func routerWith(lister *pullRequestLister) *gin.Engine {
	gin.SetMode(gin.TestMode)
	r := gin.New()
	registerUpdaterPullRequests(r.Group("/api"), lister)
	return r
}

func TestPullRequestEndpointListsTrustedRepositoryAndCaches(t *testing.T) {
	var calls atomic.Int32
	server := fakeGitHub(t, &calls)
	defer server.Close()
	now := time.Unix(1000, 0)
	lister := &pullRequestLister{
		api: server.URL, github: server.Client(), now: func() time.Time { return now },
		trusted: func(context.Context) ([]string, error) { return []string{"mowglinext/mowglinext"}, nil },
		cache:   map[string]cachedPullRequests{},
	}
	r := routerWith(lister)

	for i := 0; i < 2; i++ {
		w := httptest.NewRecorder()
		r.ServeHTTP(w, httptest.NewRequest(http.MethodGet, "/api/system/updater/pull-requests?repository=mowglinext%2Fmowglinext", nil))
		if w.Code != http.StatusOK {
			t.Fatalf("status %d: %s", w.Code, w.Body.String())
		}
		var got UpdaterPullRequests
		if err := json.Unmarshal(w.Body.Bytes(), &got); err != nil {
			t.Fatal(err)
		}
		if len(got.PullRequests) != 1 || !got.PullRequests[0].Installable || got.PullRequests[0].Branch != "fix/x" {
			t.Fatalf("unexpected listing %+v", got)
		}
	}
	if calls.Load() != 2 {
		t.Fatalf("second request within the TTL must be cached: %d GitHub calls", calls.Load())
	}
	now = now.Add(pullRequestCacheTTL)
	w := httptest.NewRecorder()
	r.ServeHTTP(w, httptest.NewRequest(http.MethodGet, "/api/system/updater/pull-requests?repository=mowglinext/mowglinext", nil))
	if w.Code != http.StatusOK || calls.Load() != 4 {
		t.Fatalf("expired cache must refetch: status %d, %d calls", w.Code, calls.Load())
	}
}

func TestPullRequestEndpointRefusesUntrustedOrMalformedRepositories(t *testing.T) {
	var calls atomic.Int32
	server := fakeGitHub(t, &calls)
	defer server.Close()
	r := routerWith(&pullRequestLister{
		api: server.URL, github: server.Client(), now: time.Now,
		trusted: func(context.Context) ([]string, error) { return []string{"mowglinext/mowglinext"}, nil },
		cache:   map[string]cachedPullRequests{},
	})
	for query, status := range map[string]int{
		"repository=evil/repo":    http.StatusForbidden,
		"repository=":             http.StatusBadRequest,
		"repository=a/b/c":        http.StatusBadRequest,
		"repository=noslash":      http.StatusBadRequest,
		"other=mowglinext%2Fmowg": http.StatusBadRequest,
	} {
		w := httptest.NewRecorder()
		r.ServeHTTP(w, httptest.NewRequest(http.MethodGet, "/api/system/updater/pull-requests?"+query, nil))
		if w.Code != status {
			t.Errorf("%s: status %d, want %d", query, w.Code, status)
		}
	}
	if calls.Load() != 0 {
		t.Fatalf("GitHub was queried for a refused repository: %d calls", calls.Load())
	}
}

func TestPullRequestEndpointReportsGitHubFailure(t *testing.T) {
	server := httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, _ *http.Request) {
		w.WriteHeader(http.StatusForbidden) // rate limited
	}))
	defer server.Close()
	r := routerWith(&pullRequestLister{
		api: server.URL, github: server.Client(), now: time.Now,
		trusted: func(context.Context) ([]string, error) { return []string{"mowglinext/mowglinext"}, nil },
		cache:   map[string]cachedPullRequests{},
	})
	w := httptest.NewRecorder()
	r.ServeHTTP(w, httptest.NewRequest(http.MethodGet, "/api/system/updater/pull-requests?repository=mowglinext/mowglinext", nil))
	if w.Code != http.StatusBadGateway {
		t.Fatalf("status %d, want 502", w.Code)
	}
}
