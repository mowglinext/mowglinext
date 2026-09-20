package updates

import (
	"context"
	"encoding/json"
	"errors"
	"net/http"
	"regexp"
	"strconv"
	"strings"
)

// ChangelogEntry is one squash-merged change, read from its commit subject.
type ChangelogEntry struct {
	Scope    string `json:"scope,omitempty"`
	Title    string `json:"title"`
	PR       int    `json:"pr,omitempty"`
	Breaking bool   `json:"breaking,omitempty"`
}

// Changelog is what an operator wants to know before installing: what is new
// and what got fixed between the installed source and the candidate. Image
// digests answer "which bytes", never "why would I take this update".
type Changelog struct {
	Features []ChangelogEntry `json:"features"`
	Fixes    []ChangelogEntry `json:"fixes"`
	// Commits that are neither a feature nor a fix (ci, docs, test, chore, ...).
	Other int `json:"other"`
	Total int `json:"total"`
	// The installed revision was not found in the first page of history, so
	// the lists cover the most recent changes only.
	Truncated bool   `json:"truncated"`
	URL       string `json:"url"`
}

const (
	changelogPageSize  = 100
	changelogTitleSize = 160
)

// Conventional-commit subject, with the "(#123)" GitHub appends on a squash merge.
var subjectPattern = regexp.MustCompile(`^([a-zA-Z]+)(?:\(([^)]{1,60})\))?(!)?:\s*(.+?)(?:\s*\(#(\d{1,7})\))?$`)

// ReadChangelog lists the changes reachable from `available` down to
// `installed`. It walks the commit list rather than GitHub's compare endpoint,
// whose first page carries the whole changed-files list with patches.
func ReadChangelog(ctx context.Context, client *http.Client, repo, installed, available string) (Changelog, error) {
	log := Changelog{Features: []ChangelogEntry{}, Fixes: []ChangelogEntry{}}
	if !RepositoryPattern.MatchString(repo) || !revisionPattern.MatchString(installed) || !revisionPattern.MatchString(available) {
		return log, errors.New("invalid repository or revision")
	}
	log.URL = "https://github.com/" + repo + "/compare/" + installed + "..." + available
	if installed == available {
		return log, nil
	}
	body, err := Read(ctx, client, "https://api.github.com/repos/"+repo+"/commits?sha="+available+"&per_page="+strconv.Itoa(changelogPageSize), "")
	if err != nil {
		return log, err
	}
	var commits []struct {
		SHA    string `json:"sha"`
		Commit struct {
			Message string `json:"message"`
		} `json:"commit"`
	}
	if err = json.Unmarshal(body, &commits); err != nil {
		return log, err
	}
	log.Truncated = true
	for _, c := range commits {
		if c.SHA == installed {
			log.Truncated = false
			break
		}
		subject, _, _ := strings.Cut(c.Commit.Message, "\n")
		subject = strings.TrimSpace(subject)
		if subject == "" || strings.HasPrefix(subject, "Merge ") {
			continue
		}
		log.Total++
		m := subjectPattern.FindStringSubmatch(subject)
		if m == nil {
			log.Other++
			continue
		}
		entry := ChangelogEntry{Scope: m[2], Title: clip(m[4], changelogTitleSize), Breaking: m[3] == "!"}
		entry.PR, _ = strconv.Atoi(m[5])
		switch strings.ToLower(m[1]) {
		case "feat":
			log.Features = append(log.Features, entry)
		case "fix", "perf":
			log.Fixes = append(log.Fixes, entry)
		default:
			log.Other++
		}
	}
	return log, nil
}

func clip(s string, limit int) string {
	r := []rune(s)
	if len(r) <= limit {
		return s
	}
	return string(r[:limit-1]) + "…"
}
