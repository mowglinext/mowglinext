package api

import (
	"io"
	"net/http"
	"net/http/httptest"
	"strings"
	"testing"

	"github.com/gin-gonic/gin"
)

type extensionTransport func(*http.Request) (*http.Response, error)

func (f extensionTransport) RoundTrip(r *http.Request) (*http.Response, error) { return f(r) }
func TestExtensionsProxyRestrictsOperationsAndSameOrigin(t *testing.T) {
	calls := 0
	client := &http.Client{Transport: extensionTransport(func(r *http.Request) (*http.Response, error) {
		calls++
		if r.URL.Host != "updater" || r.URL.Path != "/v1/extensions/review" {
			t.Fatal(r.URL)
		}
		data, _ := io.ReadAll(r.Body)
		if string(data) != `{"action":"install"}` {
			t.Fatal(string(data))
		}
		return &http.Response{StatusCode: 200, Header: http.Header{}, Body: io.NopCloser(strings.NewReader(`{"blockers":[]}`))}, nil
	})}
	r := gin.New()
	registerExtensionsProxy(r.Group("/api"), client)
	for _, tc := range []struct {
		path, origin, content, header string
		code                          int
	}{
		{"review", "http://mower:4006", "application/json", "1", 200},
		{"review", "http://evil.example", "application/json", "1", 403},
		{"review", "", "application/json", "1", 403},
		{"review", "http://mower:4006", "text/plain", "1", 403},
		{"review", "http://mower:4006", "application/json", "", 403},
		{"exec", "http://mower:4006", "application/json", "1", 404},
		{"review?url=http://evil.example", "http://mower:4006", "application/json", "1", 404},
	} {
		req := httptest.NewRequest("POST", "http://mower:4006/api/system/extensions/"+tc.path, strings.NewReader(`{"action":"install"}`))
		req.Header.Set("Origin", tc.origin)
		req.Header.Set("Content-Type", tc.content)
		req.Header.Set("X-Mowgli-Update", tc.header)
		w := httptest.NewRecorder()
		r.ServeHTTP(w, req)
		if w.Code != tc.code {
			t.Fatalf("%+v: %d %s", tc, w.Code, w.Body.String())
		}
	}
	if calls != 1 {
		t.Fatal("rejected requests reached host", calls)
	}
}
func TestExtensionsProxyHandlesOlderHostAndLimitsResponses(t *testing.T) {
	for _, tc := range []struct {
		code int
		body string
		want int
	}{
		{404, "not found", 503}, {200, strings.Repeat("x", (2<<20)+1), 502}, {202, `{"phase":"running"}`, 202},
	} {
		client := &http.Client{Transport: extensionTransport(func(r *http.Request) (*http.Response, error) {
			return &http.Response{StatusCode: tc.code, Header: http.Header{}, Body: io.NopCloser(strings.NewReader(tc.body))}, nil
		})}
		r := gin.New()
		registerExtensionsProxy(r.Group("/api"), client)
		w := httptest.NewRecorder()
		r.ServeHTTP(w, httptest.NewRequest("GET", "http://mower:4006/api/system/extensions/state", nil))
		if w.Code != tc.want {
			t.Fatal(w.Code, w.Body.String())
		}
	}
}
