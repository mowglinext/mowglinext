package updater

import (
	"context"
	"encoding/json"
	"fmt"
	"net/http"
	"net/url"
	"sort"
	"strings"

	"github.com/mowglinext/mowglinext/pkg/updates"
	"golang.org/x/mod/semver"
)

type GitHubSource struct {
	Client  *http.Client
	Trusted []string
}

func (g GitHubSource) Compare(ctx context.Context, repo, installed, available string) string {
	return updates.CompareRevisions(ctx, g.Client, repo, installed, available)
}

func (g GitHubSource) List(ctx context.Context, source Source) ([]Deployment, error) {
	if err := source.Validate(g.Trusted); err != nil {
		return nil, err
	}
	// Release metadata is bounded. Retention/publication keeps the most recent
	// deployment snapshots discoverable in this window.
	type releaseHeader struct {
		Tag        string `json:"tag_name"`
		Name       string `json:"name"`
		Draft      bool   `json:"draft"`
		Prerelease bool   `json:"prerelease"`
		Assets     []struct {
			Name string `json:"name"`
		} `json:"assets"`
	}
	result := []Deployment{}
	headers := []releaseHeader{}
	for page := 1; page <= 10; page++ {
		data, err := updates.Read(ctx, g.Client, fmt.Sprintf("https://api.github.com/repos/%s/releases?per_page=100&page=%d", source.Repository, page), "")
		if err != nil {
			return nil, err
		}
		var releases []releaseHeader
		if err = json.Unmarshal(data, &releases); err != nil {
			return nil, err
		}
		for _, r := range releases {
			if r.Draft || !idPattern.MatchString(r.Tag) || (source.Track == "stable") == r.Prerelease {
				continue
			}
			if source.Track != "stable" && !strings.HasPrefix(r.Name, source.Branch+" deployment-") {
				continue
			}
			if source.Track == "stable" && !stableVersion.MatchString(r.Tag) {
				continue
			}
			found := false
			for _, a := range r.Assets {
				if a.Name == "mowgli-deployment.json" {
					found = true
				}
			}
			if !found {
				continue
			}
			headers = append(headers, r)
		}
		if len(releases) < 100 || (source.Track != "stable" && len(headers) >= 30) {
			break
		}
		if page == 10 {
			return nil, fmt.Errorf("release scan limit reached; narrow or archive obsolete publications")
		}
	}
	// Backports may occupy newer pages than the highest production version.
	// Rank headers across the scan window before limiting descriptor downloads.
	if source.Track == "stable" {
		sort.Slice(headers, func(i, j int) bool { return semver.Compare(headers[i].Tag, headers[j].Tag) > 0 })
	}
	for _, r := range headers {
		body, err := updates.Read(ctx, g.Client, assetURL(source.Repository, r.Tag, "mowgli-deployment.json"), "")
		if err != nil {
			return nil, err
		}
		var d Deployment
		if err = json.Unmarshal(body, &d); err != nil {
			return nil, err
		}
		if d.Source != source {
			continue
		}
		if d.ReleaseTag != r.Tag {
			return nil, fmt.Errorf("release identity mismatch")
		}
		if err = d.Validate(g.Trusted); err != nil {
			return nil, err
		}
		result = append(result, d)
		if len(result) == 30 {
			break
		}
	}
	sort.Slice(result, func(i, j int) bool {
		if source.Track == "stable" {
			if order := semver.Compare(result[i].ReleaseTag, result[j].ReleaseTag); order != 0 {
				return order > 0
			}
		}
		return result[i].PublishedAt.After(result[j].PublishedAt)
	})
	if len(result) > 30 {
		result = result[:30]
	}
	return result, nil
}
func assetURL(repo, tag, asset string) string {
	return "https://github.com/" + repo + "/releases/download/" + url.PathEscape(tag) + "/" + url.PathEscape(asset)
}
