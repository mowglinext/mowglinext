package updates

import (
	"context"
	"encoding/json"
	"net/http"
	"strings"
	"testing"
)

func commitsJSON(t *testing.T, pairs ...string) string {
	t.Helper()
	type commit struct {
		SHA    string `json:"sha"`
		Commit struct {
			Message string `json:"message"`
		} `json:"commit"`
	}
	var list []commit
	for i := 0; i+1 < len(pairs); i += 2 {
		c := commit{SHA: pairs[i]}
		c.Commit.Message = pairs[i+1]
		list = append(list, c)
	}
	data, err := json.Marshal(list)
	if err != nil {
		t.Fatal(err)
	}
	return string(data)
}

func TestReadChangelogGroupsFeaturesAndFixesUpToTheInstalledRevision(t *testing.T) {
	installed, available := strings.Repeat("a", 40), strings.Repeat("b", 40)
	client := &http.Client{Transport: transportFunc(func(req *http.Request) (*http.Response, error) {
		if req.URL.Host != "api.github.com" || req.URL.Path != "/repos/owner/repo/commits" || req.URL.Query().Get("sha") != available {
			t.Fatalf("unexpected URL: %s", req.URL)
		}
		return response(commitsJSON(t,
			available, "feat(dig): operator-selectable sensitivity (#673)\n\nbody is ignored",
			strings.Repeat("c", 40), "fix(updater)!: parse stdout only (#677)",
			strings.Repeat("d", 40), "ci(sensors-gps): upgrade base packages (#670)",
			strings.Repeat("e", 40), "Merge branch 'dev' into feat/x",
			strings.Repeat("f", 40), "not a conventional subject",
			installed, "feat: already installed, must not be listed",
			strings.Repeat("9", 40), "fix: older than installed",
		)), nil
	})}
	log, err := ReadChangelog(context.Background(), client, "owner/repo", installed, available)
	if err != nil {
		t.Fatal(err)
	}
	if len(log.Features) != 1 || log.Features[0] != (ChangelogEntry{Scope: "dig", Title: "operator-selectable sensitivity", PR: 673}) {
		t.Fatalf("features: %+v", log.Features)
	}
	if len(log.Fixes) != 1 || log.Fixes[0] != (ChangelogEntry{Scope: "updater", Title: "parse stdout only", PR: 677, Breaking: true}) {
		t.Fatalf("fixes: %+v", log.Fixes)
	}
	if log.Other != 2 || log.Total != 4 || log.Truncated {
		t.Fatalf("other=%d total=%d truncated=%v", log.Other, log.Total, log.Truncated)
	}
	if log.URL != "https://github.com/owner/repo/compare/"+installed+"..."+available {
		t.Fatalf("url: %s", log.URL)
	}
}

func TestReadChangelogIsTruncatedWhenTheInstalledRevisionIsNotInThePage(t *testing.T) {
	installed, available := strings.Repeat("a", 40), strings.Repeat("b", 40)
	client := &http.Client{Transport: transportFunc(func(*http.Request) (*http.Response, error) {
		return response(commitsJSON(t, available, "fix: one")), nil
	})}
	log, err := ReadChangelog(context.Background(), client, "owner/repo", installed, available)
	if err != nil || !log.Truncated || len(log.Fixes) != 1 {
		t.Fatalf("log=%+v err=%v", log, err)
	}
}

func TestReadChangelogRejectsInvalidInputWithoutContactingGitHub(t *testing.T) {
	sha := strings.Repeat("a", 40)
	client := &http.Client{Transport: transportFunc(func(*http.Request) (*http.Response, error) {
		t.Fatal("must not contact GitHub")
		return nil, nil
	})}
	for _, tc := range []struct{ repo, base, head string }{
		{"http://localhost", sha, strings.Repeat("b", 40)},
		{"owner/repo", "dev", sha},
		{"owner/repo", sha, sha + "&per_page=1"},
	} {
		if _, err := ReadChangelog(context.Background(), client, tc.repo, tc.base, tc.head); err == nil {
			t.Fatalf("accepted %+v", tc)
		}
	}
	if log, err := ReadChangelog(context.Background(), client, "owner/repo", sha, sha); err != nil || log.Total != 0 {
		t.Fatalf("same revision: %+v %v", log, err)
	}
}

func TestClipKeepsLongTitlesBounded(t *testing.T) {
	if got := clip(strings.Repeat("é", 300), 160); len([]rune(got)) != 160 || !strings.HasSuffix(got, "…") {
		t.Fatalf("clip: %d runes", len([]rune(got)))
	}
}
