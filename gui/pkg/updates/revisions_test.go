package updates

import (
	"context"
	"errors"
	"net/http"
	"strings"
	"testing"
)

func TestCompareRevisionsUsesAncestryAndFixedValidatedURL(t *testing.T) {
	installed, available := strings.Repeat("a", 40), strings.Repeat("b", 40)
	for _, tc := range []struct{ status, want string }{
		{"ahead", "newer"}, {"behind", "older"}, {"diverged", "diverged"}, {"unexpected", "unknown"},
	} {
		t.Run(tc.status, func(t *testing.T) {
			client := &http.Client{Transport: transportFunc(func(req *http.Request) (*http.Response, error) {
				if req.Method != "GET" || req.URL.Host != "api.github.com" || req.URL.Path != "/repos/owner/repo/compare/"+installed+"..."+available || req.URL.Query().Get("page") != "2" {
					t.Fatalf("incorrect comparison direction or URL: %s", req.URL)
				}
				return response(`{"status":"` + tc.status + `"}`), nil
			})}
			if got := CompareRevisions(context.Background(), client, "owner/repo", installed, available); got != tc.want {
				t.Fatalf("got %s, want %s", got, tc.want)
			}
		})
	}
	client := &http.Client{Transport: transportFunc(func(*http.Request) (*http.Response, error) {
		t.Fatal("same or invalid revisions must not contact GitHub")
		return nil, nil
	})}
	for _, tc := range []struct{ repo, base, head, want string }{
		{"owner/repo", installed, installed, "same"},
		{"owner/repo", "", available, "unknown"},
		{"owner/repo", "abc1234", available, "unknown"},
		{"owner/repo", installed, "dev", "unknown"},
		{"owner/repo", installed + "?injected=1", available, "unknown"},
		{"http://localhost", installed, available, "unknown"},
	} {
		if got := CompareRevisions(context.Background(), client, tc.repo, tc.base, tc.head); got != tc.want {
			t.Fatalf("%+v: %s", tc, got)
		}
	}
}

func TestCompareRevisionsFailuresRemainUnknown(t *testing.T) {
	for _, status := range []int{404, 403, 429, 503, 200, 0} {
		client := &http.Client{Transport: transportFunc(func(req *http.Request) (*http.Response, error) {
			if status == 0 {
				return nil, errors.New("offline")
			}
			r := response(`not JSON`)
			r.StatusCode = status
			return r, nil
		})}
		if got := CompareRevisions(context.Background(), client, "owner/repo", strings.Repeat("a", 40), strings.Repeat("b", 40)); got != "unknown" {
			t.Fatalf("HTTP %d: %s", status, got)
		}
	}
}
