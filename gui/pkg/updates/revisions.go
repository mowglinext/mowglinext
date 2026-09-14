package updates

import (
	"context"
	"encoding/json"
	"net/http"
	"regexp"
)

var revisionPattern = regexp.MustCompile(`^[a-f0-9]{40}$`)

// CompareRevisions describes the candidate's source relative to the installed
// source. Dates and lexical SHA ordering cannot establish Git ancestry.
func CompareRevisions(ctx context.Context, client *http.Client, repo, installed, available string) string {
	if !RepositoryPattern.MatchString(repo) || !revisionPattern.MatchString(installed) || !revisionPattern.MatchString(available) {
		return "unknown"
	}
	if installed == available {
		return "same"
	}
	// Page 2 omits the potentially large changed-files list. Only the comparison
	// summary is needed, not the commit list or patches.
	body, err := Read(ctx, client, "https://api.github.com/repos/"+repo+"/compare/"+installed+"..."+available+"?per_page=1&page=2", "")
	if err != nil {
		return "unknown"
	}
	var comparison struct {
		Status string `json:"status"`
	}
	if json.Unmarshal(body, &comparison) != nil {
		return "unknown"
	}
	switch comparison.Status {
	case "ahead":
		return "newer"
	case "behind":
		return "older"
	case "diverged":
		return "diverged"
	default:
		return "unknown"
	}
}
