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
			found := false
			for _, a := range r.Assets {
				if a.Name == "mowgli-deployment.json" {
					found = true
				}
			}
			if !found {
				continue
			}
			body, e := updates.Read(ctx, g.Client, assetURL(source.Repository, r.Tag, "mowgli-deployment.json"), "")
			if e != nil {
				return nil, e
			}
			var d Deployment
			if e = json.Unmarshal(body, &d); e != nil {
				return nil, e
			}
			if d.Source != source {
				continue
			}
			if d.ReleaseTag != r.Tag {
				return nil, fmt.Errorf("release identity mismatch")
			}
			if e = d.Validate(g.Trusted); e != nil {
				return nil, e
			}
			result = append(result, d)
		}
		if len(releases) < 100 || len(result) >= 30 {
			break
		}
		if page == 10 {
			return nil, fmt.Errorf("release scan limit reached; narrow or archive obsolete publications")
		}
	}
	sort.Slice(result, func(i, j int) bool { return result[i].PublishedAt.After(result[j].PublishedAt) })
	if len(result) > 30 {
		result = result[:30]
	}
	return result, nil
}
func assetURL(repo, tag, asset string) string {
	return "https://github.com/" + repo + "/releases/download/" + url.PathEscape(tag) + "/" + url.PathEscape(asset)
}
