package updater

import (
	"context"
	"encoding/json"
	"fmt"
	"io"
	"net/http"
	"strings"
	"testing"
	"time"
)

type releaseTransport func(*http.Request) (*http.Response, error)

func (f releaseTransport) RoundTrip(r *http.Request) (*http.Response, error) { return f(r) }

func TestReleaseOrderingByTrack(t *testing.T) {
	for _, track := range []string{"stable", "dev", "custom"} {
		t.Run(track, func(t *testing.T) {
			first := fixture()
			first.Source.Track = track
			if track == "stable" {
				first.Source.Branch = "main"
			} else if track == "custom" {
				first.Source.Branch = "feat/example"
			}
			first.ReleaseTag = "v1.10.0"
			first.PublishedAt = time.Date(2026, 9, 1, 0, 0, 0, 0, time.UTC)
			second := first
			second.ID = "backport"
			second.ReleaseTag = "v1.9.1"
			second.PublishedAt = first.PublishedAt.Add(time.Hour)
			if track != "stable" {
				first.ReleaseTag = "deployment-old"
				second.ReleaseTag = "deployment-new"
			}
			client := &http.Client{Transport: releaseTransport(func(r *http.Request) (*http.Response, error) {
				var value any
				if r.URL.Host == "api.github.com" {
					headers := []map[string]any{}
					for _, d := range []Deployment{second, first} {
						headers = append(headers, map[string]any{"tag_name": d.ReleaseTag, "name": d.Source.Branch + " deployment-example", "prerelease": track != "stable", "assets": []map[string]string{{"name": "mowgli-deployment.json"}}})
					}
					value = headers
				} else if strings.Contains(r.URL.Path, second.ReleaseTag) {
					value = second
				} else {
					value = first
				}
				data, _ := json.Marshal(value)
				return &http.Response{StatusCode: 200, Body: io.NopCloser(strings.NewReader(string(data)))}, nil
			})}
			g := GitHubSource{Client: client, Trusted: []string{first.Source.Repository}}
			result, err := g.List(context.Background(), first.Source)
			if err != nil {
				t.Fatal(err)
			}
			expected := second.ReleaseTag
			if track == "stable" {
				expected = first.ReleaseTag
			}
			if len(result) != 2 || result[0].ReleaseTag != expected {
				t.Fatalf("unexpected ordering: %+v", result)
			}
		})
	}
}

func TestProductionVersionOrderingPrecedesPaginationAndRetention(t *testing.T) {
	base := fixture()
	base.Source = Source{Repository: base.Source.Repository, Track: "stable", Branch: "main"}
	downloads := 0
	client := &http.Client{Transport: releaseTransport(func(r *http.Request) (*http.Response, error) {
		var value any
		if r.URL.Host == "api.github.com" {
			headers := []map[string]any{}
			count := 100
			if r.URL.Query().Get("page") == "2" {
				count = 1
			}
			for i := 0; i < count; i++ {
				tag := fmt.Sprintf("v1.2.%d", i)
				if count == 1 {
					tag = "v2.0.0"
				}
				headers = append(headers, map[string]any{"tag_name": tag, "assets": []map[string]string{{"name": "mowgli-deployment.json"}}})
			}
			value = headers
		} else {
			downloads++
			d := base
			parts := strings.Split(r.URL.Path, "/")
			d.ReleaseTag = parts[len(parts)-2]
			d.ID = d.ReleaseTag
			value = d
		}
		data, _ := json.Marshal(value)
		return &http.Response{StatusCode: 200, Body: io.NopCloser(strings.NewReader(string(data)))}, nil
	})}
	g := GitHubSource{Client: client, Trusted: []string{base.Source.Repository}}
	result, err := g.List(context.Background(), base.Source)
	if err != nil {
		t.Fatal(err)
	}
	if len(result) != 30 || result[0].ReleaseTag != "v2.0.0" || downloads != 30 {
		t.Fatalf("wrong latest/retention: %s, %d results, %d downloads", result[0].ReleaseTag, len(result), downloads)
	}
}
