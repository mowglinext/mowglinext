package extensions

import (
	"context"
	"errors"
	"io"
	"net/http"
	"net/http/httptest"
	"strings"
	"testing"
)

type unitRunner struct {
	calls     [][]string
	responses []string
}

func (f *unitRunner) Run(ctx context.Context, name string, args ...string) ([]byte, error) {
	f.calls = append(f.calls, append([]string{name}, args...))
	if name != "systemctl" || len(args) != 5 || args[0] != "show" || args[3] != "--" {
		return nil, errors.New("unapproved external operation")
	}
	if len(f.responses) == 0 {
		return nil, errors.New("no response")
	}
	out := f.responses[0]
	if len(f.responses) > 1 {
		f.responses = f.responses[1:]
	}
	return []byte(out), nil
}
func externalRelease() Release {
	b := schemaTestBundle()
	b.Manifest.ID = "mowgli-lora-link"
	b.Manifest.Profile = "external"
	b.Manifest.Health = "application"
	b.Manifest.Runtime = Runtime{Kind: "external-systemd", Units: []string{"mowgli-lora-robot.service"}}
	b.Compose = nil
	return Release{Bundle: b, Digest: BundleDigest(b)}
}

const activeUnit = "ActiveState=active\nSubState=running\nInvocationID=aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa\n"

type rewriteTransport struct {
	url  string
	base http.RoundTripper
	seen *[]string
}

func (r rewriteTransport) RoundTrip(req *http.Request) (*http.Response, error) {
	*r.seen = append(*r.seen, req.URL.String())
	copy := req.Clone(req.Context())
	target, _ := http.NewRequest(http.MethodGet, r.url, nil)
	copy.URL = target.URL
	return r.base.RoundTrip(copy)
}
func TestExternalOnlyObservesAndNeverMutates(t *testing.T) {
	r := externalRelease()
	f := &unitRunner{responses: []string{activeUnit}}
	b := &RuntimeBackend{Root: t.TempDir(), Runner: f}
	for _, mutate := range []func(context.Context, Release) error{b.Prepare, b.Start, b.Stop, b.Remove} {
		if mutate(context.Background(), r) == nil {
			t.Fatal("external mutation allowed")
		}
	}
	if len(f.calls) != 0 {
		t.Fatal("external mutation invoked systemctl")
	}
	r.Bundle.Manifest.Health = "process-only"
	o, err := b.Observe(context.Background(), r)
	if err != nil || o.Process != "running" || o.Application != "unknown" || o.Data != "unknown" {
		t.Fatalf("%+v %v", o, err)
	}
	r.Bundle.Manifest.Runtime.Units = []string{"--bad.service"}
	if _, err = b.Observe(context.Background(), r); err == nil {
		t.Fatal("unsafe unit accepted")
	}
}
func TestLoraHealthIsBoundToInvocationAndNeverMeansDataFreshness(t *testing.T) {
	cases := []struct {
		name, body  string
		status      int
		after       string
		application string
		wantErr     bool
	}{
		{"healthy", `{"invocation_id":"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa","status":"ok","role":"robot","checks":{"modem_connected":true,"modem_handshake_complete":true,"modem_radio_ready":true}}`, 200, activeUnit, "healthy", false},
		{"degraded", `{"invocation_id":"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa","status":"degraded","role":"robot","checks":{"modem_connected":false,"modem_handshake_complete":false,"modem_radio_ready":false}}`, 503, activeUnit, "unhealthy", false},
		{"missing checks", `{"invocation_id":"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa","status":"ok","role":"robot","checks":{}}`, 200, activeUnit, "unknown", false},
		{"wrong role", `{"invocation_id":"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa","status":"ok","role":"base","checks":{"modem_connected":true,"modem_handshake_complete":true,"modem_radio_ready":true}}`, 200, activeUnit, "unknown", false},
		{"restarted", `{"invocation_id":"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa","status":"ok","role":"robot","checks":{"modem_connected":true,"modem_handshake_complete":true,"modem_radio_ready":true}}`, 200, strings.ReplaceAll(activeUnit, "aaaaaaaa", "bbbbbbbb"), "unknown", true},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			server := httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, req *http.Request) {
				w.WriteHeader(tc.status)
				_, _ = io.WriteString(w, tc.body)
			}))
			defer server.Close()
			seen := []string{}
			f := &unitRunner{responses: []string{activeUnit, tc.after}}
			b := &RuntimeBackend{Runner: f, Client: &http.Client{Transport: rewriteTransport{server.URL, http.DefaultTransport, &seen}}}
			o, err := b.Observe(context.Background(), externalRelease())
			if (err != nil) != tc.wantErr || o.Application != tc.application || o.Data != "unknown" {
				t.Fatalf("%+v %v", o, err)
			}
			if len(f.calls) != 2 || len(seen) != 1 || seen[0] != "http://127.0.0.1:9609/healthz" {
				t.Fatalf("calls %v HTTP %v", f.calls, seen)
			}
		})
	}
}
func TestStoppedExternalDoesNotProbe(t *testing.T) {
	f := &unitRunner{responses: []string{"ActiveState=inactive\nSubState=dead\nInvocationID=\n"}}
	b := &RuntimeBackend{Runner: f}
	o, err := b.Observe(context.Background(), externalRelease())
	if err != nil || o.Process != "stopped" || o.Application != "unknown" {
		t.Fatalf("%+v %v", o, err)
	}
	if len(f.calls) != 1 {
		t.Fatal(f.calls)
	}
}

func TestPartialExternalRemainsSubjectToPreflight(t *testing.T) {
	r := externalRelease()
	r.Bundle.Manifest.Runtime.Units = append(r.Bundle.Manifest.Runtime.Units, "helper.service")
	r.Digest = BundleDigest(r.Bundle)
	f := &unitRunner{responses: []string{activeUnit, "ActiveState=inactive\nSubState=dead\nInvocationID=\n"}}
	b := &RuntimeBackend{Root: t.TempDir(), Runner: f}
	o, err := b.Observe(context.Background(), r)
	if err != nil || o.Process != "partial" || o.Services["mowgli-lora-robot.service"] != "running" || o.Application != "unknown" {
		t.Fatalf("partly running external service misreported: %+v %v", o, err)
	}
	r.Binding = schemaTestRelease().Binding
	store := Store{Root: b.Root}
	state := NewState()
	state.Entries[r.Bundle.Manifest.ID] = Entry{Active: r}
	if err := store.Save(state); err != nil {
		t.Fatal(err)
	}
	f.responses = []string{activeUnit, "ActiveState=inactive\nSubState=dead\nInvocationID=\n"}
	target := schemaTestHost()
	target.Contracts = []string{}
	target.ExternalUnits = r.Bundle.Manifest.Runtime.Units
	m := Manager{Store: store, Host: schemaTestHost(), Backend: b}
	conflicts, err := m.Preflight(context.Background(), target)
	if err != nil || len(conflicts[r.Bundle.Manifest.ID]) == 0 {
		t.Fatalf("running primary skipped required contract: %+v %v", conflicts, err)
	}
}

func TestTransitionalExternalStateIsNotStopped(t *testing.T) {
	for _, state := range []string{
		"ActiveState=activating\nSubState=start\nInvocationID=aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa\n",
		"ActiveState=deactivating\nSubState=stop-sigterm\nInvocationID=aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa\n",
		"ActiveState=active\nSubState=exited\nInvocationID=aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa\n",
	} {
		b := &RuntimeBackend{Runner: &unitRunner{responses: []string{state}}}
		o, err := b.Observe(context.Background(), externalRelease())
		if err != nil || o.Process != "unknown" || o.Application != "unknown" {
			t.Fatalf("transitional service reported stopped: %+v %v", o, err)
		}
	}
}

func TestLegacyLoraHealthHasUnknownApplication(t *testing.T) {
	server := httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		_, _ = io.WriteString(w, `{"status":"ok","role":"robot","checks":{"modem_connected":true,"modem_handshake_complete":true,"modem_radio_ready":true}}`)
	}))
	defer server.Close()
	seen := []string{}
	b := &RuntimeBackend{Runner: &unitRunner{responses: []string{activeUnit}}, Client: &http.Client{Transport: rewriteTransport{server.URL, http.DefaultTransport, &seen}}}
	o, err := b.Observe(context.Background(), externalRelease())
	if err != nil || o.Process != "running" || o.Application != "unknown" || o.Data != "unknown" {
		t.Fatalf("%+v %v", o, err)
	}
}
func TestLoraHealthRequiresExactProducerInvocation(t *testing.T) {
	server := httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		_, _ = io.WriteString(w, `{"invocation_id":"bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb","status":"ok","role":"robot","checks":{"modem_connected":true,"modem_handshake_complete":true,"modem_radio_ready":true}}`)
	}))
	defer server.Close()
	seen := []string{}
	b := &RuntimeBackend{Runner: &unitRunner{responses: []string{activeUnit}}, Client: &http.Client{Transport: rewriteTransport{server.URL, http.DefaultTransport, &seen}}}
	o, err := b.Observe(context.Background(), externalRelease())
	if err != nil || o.Application != "unknown" {
		t.Fatalf("%+v %v", o, err)
	}
	r := externalRelease()
	r.Bundle.Manifest.Runtime.Units = []string{"some-other.service"}
	o, err = b.Observe(context.Background(), r)
	if err != nil || o.Application != "unknown" || len(seen) != 1 {
		t.Fatalf("unexpected probe for unrelated unit %+v %v", o, err)
	}
}
