package api

import (
	"context"
	"encoding/json"
	"errors"
	"io"
	"net/http"
	"net/url"
	"slices"
	"strings"
	"sync"
	"time"

	"github.com/gin-gonic/gin"
	"github.com/mowglinext/mowglinext/pkg/updates"
)

// PreviewLabel is the pull-request label that makes
// .github/workflows/deployment-release.yml publish a complete deployment of a
// same-repository PR, titled "<head branch> deployment-…" like any custom
// branch. The updater installs it through the ordinary custom-branch path.
const PreviewLabel = "deploy-preview"

// pullRequestCacheTTL bounds unauthenticated GitHub API use (60 requests/hour
// per address): one listing costs two requests.
const pullRequestCacheTTL = 2 * time.Minute

// UpdaterPullRequest is one open pull request as offered by the Advanced
// update settings.
type UpdaterPullRequest struct {
	Number     int    `json:"number"`
	Title      string `json:"title"`
	Branch     string `json:"branch"`
	Author     string `json:"author"`
	URL        string `json:"url"`
	UpdatedAt  string `json:"updated_at"`
	FromFork   bool   `json:"from_fork"`
	Labeled    bool   `json:"labeled"`
	Deployment string `json:"deployment,omitempty"`
	// Installable: a complete deployment of this PR's head branch is published.
	Installable bool `json:"installable"`
}

// UpdaterPullRequests is the GET /system/updater/pull-requests response.
type UpdaterPullRequests struct {
	Repository   string               `json:"repository"`
	Label        string               `json:"label"`
	PullRequests []UpdaterPullRequest `json:"pull_requests"`
}

type githubLabel struct {
	Name string `json:"name"`
}

type githubPull struct {
	Number    int    `json:"number"`
	Title     string `json:"title"`
	HTMLURL   string `json:"html_url"`
	UpdatedAt string `json:"updated_at"`
	User      struct {
		Login string `json:"login"`
	} `json:"user"`
	Labels []githubLabel `json:"labels"`
	Head   struct {
		Ref  string `json:"ref"`
		Repo *struct {
			FullName string `json:"full_name"`
		} `json:"repo"`
	} `json:"head"`
}

type githubRelease struct {
	Name       string `json:"name"`
	Draft      bool   `json:"draft"`
	Prerelease bool   `json:"prerelease"`
}

// latestDeploymentByBranch maps a branch to the newest deployment published for
// it. deployment-release.yml titles a custom deployment "<branch> <deployment>"
// and GitHub lists releases newest first, so the first title per branch wins.
func latestDeploymentByBranch(releases []githubRelease) map[string]string {
	out := map[string]string{}
	for _, r := range releases {
		if r.Draft || !r.Prerelease {
			continue
		}
		i := strings.LastIndex(r.Name, " deployment-")
		if i <= 0 {
			continue
		}
		branch := r.Name[:i]
		if _, seen := out[branch]; !seen {
			out[branch] = r.Name[i+1:]
		}
	}
	return out
}

// pullRequestsFor combines open pulls with the published deployments of their
// head branches. A fork's branch name may collide with an upstream branch, so a
// fork PR is never reported installable from this repository's releases.
func pullRequestsFor(repo string, pulls []githubPull, releases []githubRelease) UpdaterPullRequests {
	deployments := latestDeploymentByBranch(releases)
	result := UpdaterPullRequests{Repository: repo, Label: PreviewLabel, PullRequests: []UpdaterPullRequest{}}
	for _, p := range pulls {
		fork := p.Head.Repo == nil || !strings.EqualFold(p.Head.Repo.FullName, repo)
		labeled := slices.ContainsFunc(p.Labels, func(l githubLabel) bool { return l.Name == PreviewLabel })
		pr := UpdaterPullRequest{
			Number: p.Number, Title: p.Title, Branch: p.Head.Ref, Author: p.User.Login,
			URL: p.HTMLURL, UpdatedAt: p.UpdatedAt, FromFork: fork, Labeled: labeled,
		}
		if deployment, ok := deployments[p.Head.Ref]; ok && !fork {
			pr.Deployment = deployment
			pr.Installable = true
		}
		result.PullRequests = append(result.PullRequests, pr)
	}
	return result
}

type pullRequestLister struct {
	api     string // GitHub API base, https://api.github.com in production
	github  *http.Client
	trusted func(context.Context) ([]string, error)
	now     func() time.Time
	mu      sync.Mutex
	cache   map[string]cachedPullRequests
}

type cachedPullRequests struct {
	at   time.Time
	data UpdaterPullRequests
}

var errUntrustedRepository = errors.New("repository is not in the updater's trusted_repositories")

func (l *pullRequestLister) list(ctx context.Context, repo string) (UpdaterPullRequests, error) {
	trusted, err := l.trusted(ctx)
	if err != nil {
		return UpdaterPullRequests{}, err
	}
	if !slices.Contains(trusted, repo) {
		return UpdaterPullRequests{}, errUntrustedRepository
	}
	l.mu.Lock()
	if hit, ok := l.cache[repo]; ok && l.now().Sub(hit.at) < pullRequestCacheTTL {
		l.mu.Unlock()
		return hit.data, nil
	}
	l.mu.Unlock()

	base := l.api + "/repos/" + repo
	body, err := updates.Read(ctx, l.github, base+"/pulls?state=open&sort=updated&direction=desc&per_page=50", "")
	if err != nil {
		return UpdaterPullRequests{}, err
	}
	var pulls []githubPull
	if err = json.Unmarshal(body, &pulls); err != nil {
		return UpdaterPullRequests{}, err
	}
	body, err = updates.Read(ctx, l.github, base+"/releases?per_page=100", "")
	if err != nil {
		return UpdaterPullRequests{}, err
	}
	var releases []githubRelease
	if err = json.Unmarshal(body, &releases); err != nil {
		return UpdaterPullRequests{}, err
	}
	data := pullRequestsFor(repo, pulls, releases)
	l.mu.Lock()
	l.cache[repo] = cachedPullRequests{at: l.now(), data: data}
	l.mu.Unlock()
	return data, nil
}

// trustedFromUpdater reads trusted_repositories from the host updater state, so
// the browser can only list repositories the updater would install from.
func trustedFromUpdater(client *http.Client) func(context.Context) ([]string, error) {
	return func(ctx context.Context) ([]string, error) {
		req, err := http.NewRequestWithContext(ctx, http.MethodGet, "http://updater/v1/state", nil)
		if err != nil {
			return nil, err
		}
		response, err := client.Do(req)
		if err != nil {
			return nil, err
		}
		defer response.Body.Close()
		if response.StatusCode != http.StatusOK {
			return nil, errors.New("host updater state unavailable")
		}
		var state struct {
			Trusted []string `json:"trusted_repositories"`
		}
		if err = json.NewDecoder(io.LimitReader(response.Body, 8*1024*1024)).Decode(&state); err != nil {
			return nil, err
		}
		return state.Trusted, nil
	}
}

func registerUpdaterPullRequests(r *gin.RouterGroup, lister *pullRequestLister) {
	// @Summary List open pull requests of a trusted repository for the updater
	// @Tags updates
	// @Produce json
	// @Param repository query string true "owner/name, one of the updater's trusted repositories"
	// @Success 200 {object} UpdaterPullRequests
	// @Router /system/updater/pull-requests [get]
	r.GET("/system/updater/pull-requests", func(c *gin.Context) {
		repo, err := url.QueryUnescape(c.Query("repository"))
		if err != nil || repo == "" || strings.Count(repo, "/") != 1 {
			c.JSON(http.StatusBadRequest, ErrorResponse{Error: "repository must be owner/name"})
			return
		}
		data, err := lister.list(c.Request.Context(), repo)
		if errors.Is(err, errUntrustedRepository) {
			c.JSON(http.StatusForbidden, ErrorResponse{Error: err.Error()})
			return
		}
		if err != nil {
			c.JSON(http.StatusBadGateway, ErrorResponse{Error: "GitHub pull requests unavailable: " + err.Error()})
			return
		}
		c.Header("Cache-Control", "no-store")
		c.JSON(http.StatusOK, data)
	})
}
