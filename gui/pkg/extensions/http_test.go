package extensions

import (
	"bytes"
	"context"
	"encoding/json"
	"net/http"
	"net/http/httptest"
	"path/filepath"
	"strings"
	"testing"
	"time"
)

func httpFixture(t *testing.T) (*HTTPService, Manager, *lifecycleRuntime, Bundle, Binding) {
	t.Helper()
	release := adapterRelease(t)
	old := release.Binding
	store := Store{Root: filepath.Join(t.TempDir(), "state")}
	release.Binding.ConfigDir = filepath.Join("/etc/mowgli-extensions", release.Bundle.Manifest.ID, "config")
	release.Binding.DataDir = filepath.Join(store.Root, release.Bundle.Manifest.ID, "data")
	release.Bundle.Compose = bytes.ReplaceAll(release.Bundle.Compose, []byte(old.ConfigDir), []byte(release.Binding.ConfigDir))
	release.Bundle.Compose = bytes.ReplaceAll(release.Bundle.Compose, []byte(old.DataDir), []byte(release.Binding.DataDir))
	var spec composeSpec
	if err := json.Unmarshal(release.Bundle.Compose, &spec); err != nil {
		t.Fatal(err)
	}
	configured := spec.Services["agent"]
	configured.Environment = map[string]string{"TOKEN": "SECRET_NOT_PUBLIC"}
	spec.Services["agent"] = configured
	release.Bundle.Compose, _ = json.Marshal(spec)
	backend := &lifecycleRuntime{t: t, store: store, failures: map[string]int{}, readiness: "healthy"}
	host := schemaTestHost()
	host.ApprovedGrants = append(host.ApprovedGrants, "network:egress")
	m := Manager{Store: store, Host: host, Backend: backend}
	service := &HTTPService{entries: []Status{}, load: func() (Manager, error) { return m, nil }}
	return service, m, backend, release.Bundle, release.Binding
}
func httpCall(t *testing.T, h http.Handler, method, path string, value any) *httptest.ResponseRecorder {
	t.Helper()
	var body []byte
	if value != nil {
		var err error
		body, err = json.Marshal(value)
		if err != nil {
			t.Fatal(err)
		}
	}
	return httpRaw(h, method, path, body)
}
func httpRaw(h http.Handler, method, path string, body []byte) *httptest.ResponseRecorder {
	w := httptest.NewRecorder()
	h.ServeHTTP(w, httptest.NewRequest(method, path, bytes.NewReader(body)))
	return w
}
func reviewed(t *testing.T, s *HTTPService, b Bundle, binding Binding, action string) httpRequest {
	t.Helper()
	raw, _ := json.Marshal(b)
	request := httpRequest{Action: action, ID: b.Manifest.ID, Bundle: raw, Binding: binding}
	response := httpCall(t, s, "POST", "/review", request)
	if response.Code != 200 {
		t.Fatal(response.Code, response.Body.String())
	}
	if strings.Contains(response.Body.String(), "SECRET_NOT_PUBLIC") {
		t.Fatal("review leaked resolved environment")
	}
	var review httpReview
	if err := json.Unmarshal(response.Body.Bytes(), &review); err != nil {
		t.Fatal(err)
	}
	if len(review.Services) != 1 || len(review.Services[0].Mounts) != 2 || len(review.Services[0].Networks) != 1 || len(review.Services[0].EnvironmentKeys) != 1 || review.Services[0].EnvironmentKeys[0] != "TOKEN" || !strings.Contains(review.Services[0].Image, "@sha256:") {
		t.Fatal("missing concrete service review", review.Services)
	}
	request.Digest = review.Digest
	request.Acknowledged = true
	return request
}
func waitHTTPJob(t *testing.T, s *HTTPService) HTTPOperation {
	t.Helper()
	deadline := time.Now().Add(3 * time.Second)
	for time.Now().Before(deadline) {
		s.mu.Lock()
		var operation HTTPOperation
		if s.operation != nil {
			operation = *s.operation
		}
		s.mu.Unlock()
		if operation.Phase != "running" && operation.Phase != "" {
			return operation
		}
		time.Sleep(time.Millisecond)
	}
	t.Fatal("operation did not settle")
	return HTTPOperation{}
}
func TestHTTPImportReviewLifecycleAndSecretBoundary(t *testing.T) {
	s, m, backend, bundle, binding := httpFixture(t)
	request := reviewed(t, s, bundle, binding, "install")
	response := httpCall(t, s, "POST", "/apply", request)
	if response.Code != 202 {
		t.Fatal(response.Code, response.Body.String())
	}
	if result := waitHTTPJob(t, s); result.Phase != "succeeded" {
		t.Fatal(result)
	}
	state, err := m.Store.Load()
	if err != nil {
		t.Fatal(err)
	}
	entry := state.Entries[bundle.Manifest.ID]
	if entry.Enabled || backend.running != nil {
		t.Fatal("GUI import started the service")
	}
	status := httpCall(t, s, "GET", "/state", nil)
	if status.Code != 200 || strings.Contains(status.Body.String(), "SECRET_NOT_PUBLIC") || !strings.Contains(status.Body.String(), `"manifest"`) {
		t.Fatal(status.Body.String())
	}
	action := httpRequest{Action: "enable", ID: bundle.Manifest.ID, Digest: entry.Active.Digest, Acknowledged: true}
	if w := httpCall(t, s, "POST", "/action", action); w.Code != 202 {
		t.Fatal(w.Code, w.Body.String())
	}
	if result := waitHTTPJob(t, s); result.Phase != "succeeded" || backend.running == nil {
		t.Fatal(result)
	}
}
func TestHTTPConfirmationBindsActionStorageAndCurrentRelease(t *testing.T) {
	s, m, _, bundle, binding := httpFixture(t)
	request := reviewed(t, s, bundle, binding, "install")
	changed := request
	changed.Binding.Project = "other-project"
	if w := httpCall(t, s, "POST", "/apply", changed); w.Code != 409 && w.Code != 400 {
		t.Fatal("changed binding accepted", w.Code)
	}
	changed = request
	changed.Action = "adopt"
	if w := httpCall(t, s, "POST", "/apply", changed); w.Code != 409 {
		t.Fatal("changed action accepted", w.Code)
	}
	if w := httpCall(t, s, "POST", "/apply", request); w.Code != 202 {
		t.Fatal(w.Body.String())
	}
	waitHTTPJob(t, s)
	target := candidateBundle(bundle)
	upgrade := reviewed(t, s, target, Binding{}, "upgrade")
	newer := bundle
	newer.Manifest.Version = "2.0.0"
	if err := m.Apply(context.Background(), "upgrade", bundle.Manifest.ID, &newer, Binding{}); err != nil {
		t.Fatal(err)
	}
	if w := httpCall(t, s, "POST", "/apply", upgrade); w.Code != 409 {
		t.Fatal("stale upgrade accepted", w.Code, w.Body.String())
	}
	state, _ := m.Store.Load()
	if err := m.apply(context.Background(), "enable", bundle.Manifest.ID, nil, Binding{}, BundleDigest(bundle), ""); err == nil {
		t.Fatal("stale active digest accepted under mutation lock")
	}
	if state.Entries[bundle.Manifest.ID].Enabled {
		t.Fatal("stale action enabled new release")
	}
}
func TestHTTPUntrustedCustomBundleIsReviewableButCannotExecute(t *testing.T) {
	s, _, backend, bundle, binding := httpFixture(t)
	bundle.Manifest.Source.Repository = "community/custom-integration"
	request := reviewed(t, s, bundle, binding, "install")
	w := httpCall(t, s, "POST", "/review", request)
	var view httpReview
	_ = json.Unmarshal(w.Body.Bytes(), &view)
	if len(view.Blockers) == 0 || view.Manifest.Source.Repository != bundle.Manifest.Source.Repository {
		t.Fatal(w.Body.String())
	}
	if w = httpCall(t, s, "POST", "/apply", request); w.Code != 409 {
		t.Fatal(w.Code)
	}
	if len(backend.events) != 0 {
		t.Fatal("untrusted import touched runtime")
	}
}
func TestHTTPRejectsAmbiguousRequestsTemporaryPathsAndMaintenance(t *testing.T) {
	s, _, backend, bundle, binding := httpFixture(t)
	for _, body := range []string{`{"action":"install","action":"adopt"}`, `{"Action":"install"}`, `{"action":"install"} {}`, `{"action":"install","hook":"bad"}`, strings.Repeat("x", 2*inputLimit+16385)} {
		if w := httpRaw(s, "POST", "/review", []byte(body)); w.Code != 400 {
			t.Fatal("ambiguous request accepted", w.Code)
		}
	}
	for _, path := range []string{"/tmp/private-extension", "/var/tmp/private-extension"} {
		bad := binding
		bad.DataDir = path
		raw, _ := json.Marshal(bundle)
		if w := httpCall(t, s, "POST", "/review", httpRequest{Action: "install", Bundle: raw, Binding: bad}); w.Code != 400 {
			t.Fatal("PrivateTmp path accepted")
		}
	}
	request := reviewed(t, s, bundle, binding, "install")
	s.blocked = func() bool { return true }
	if w := httpCall(t, s, "POST", "/apply", request); w.Code != 423 {
		t.Fatal(w.Code)
	}
	if len(backend.events) != 0 {
		t.Fatal("blocked operation touched runtime")
	}
}
func TestHTTPMissingPolicyCreatesNoRegister(t *testing.T) {
	root := t.TempDir()
	state := filepath.Join(root, "state")
	handler := NewHTTPHandler(filepath.Join(root, "missing.json"), state, nil)
	response := httpCall(t, handler, "GET", "/state", nil)
	if response.Code != 200 || !strings.Contains(response.Body.String(), `"configured":false`) {
		t.Fatal(response.Body.String())
	}
	if _, err := (Store{Root: state}).Load(); err != nil {
		t.Fatal(err)
	}
}

type heldStart struct {
	Backend
	entered, release chan struct{}
}

func (h heldStart) Start(ctx context.Context, r Release) error {
	close(h.entered)
	select {
	case <-h.release:
	case <-ctx.Done():
		return ctx.Err()
	}
	return h.Backend.Start(ctx, r)
}
func TestHTTPActiveJobSurvivesRequestAndReturnsCachedStatus(t *testing.T) {
	s, m, backend, bundle, binding := httpFixture(t)
	if err := m.Apply(context.Background(), "install", bundle.Manifest.ID, &bundle, binding); err != nil {
		t.Fatal(err)
	}
	if w := httpCall(t, s, "GET", "/state", nil); w.Code != 200 {
		t.Fatal(w.Body.String())
	}
	held := heldStart{Backend: backend, entered: make(chan struct{}), release: make(chan struct{})}
	m.Backend = held
	s.load = func() (Manager, error) { return m, nil }
	request := httpRequest{Action: "enable", ID: bundle.Manifest.ID, Digest: BundleDigest(bundle), Acknowledged: true}
	if w := httpCall(t, s, "POST", "/action", request); w.Code != 202 {
		t.Fatal(w.Code, w.Body.String())
	}
	<-held.entered
	w := httpCall(t, s, "GET", "/state", nil)
	if w.Code != 200 || !strings.Contains(w.Body.String(), `"phase":"running"`) {
		t.Fatal(w.Body.String())
	}
	if w = httpCall(t, s, "POST", "/action", request); w.Code != 409 {
		t.Fatal("duplicate job accepted")
	}
	close(held.release)
	if result := waitHTTPJob(t, s); result.Phase != "succeeded" {
		t.Fatal(result)
	}
}

func TestHTTPRollbackConfirmsPreviousDigestUnderLock(t *testing.T) {
	s, m, _, original, binding := httpFixture(t)
	if err := m.Apply(context.Background(), "install", original.Manifest.ID, &original, binding); err != nil {
		t.Fatal(err)
	}
	target := candidateBundle(original)
	if err := m.Apply(context.Background(), "upgrade", original.Manifest.ID, &target, Binding{}); err != nil {
		t.Fatal(err)
	}
	// Reviewed active target / previous original. Change Previous while keeping
	// Active target by upgrading again and rolling back through the CLI.
	third := original
	third.Manifest.Version = "3.0.0"
	if err := m.Apply(context.Background(), "upgrade", original.Manifest.ID, &third, Binding{}); err != nil {
		t.Fatal(err)
	}
	if err := m.Apply(context.Background(), "rollback", original.Manifest.ID, nil, Binding{}); err != nil {
		t.Fatal(err)
	}
	request := httpRequest{Action: "rollback", ID: original.Manifest.ID, Digest: BundleDigest(target), PreviousDigest: BundleDigest(original), Acknowledged: true}
	if w := httpCall(t, s, "POST", "/action", request); w.Code != 409 {
		t.Fatal("stale rollback target accepted", w.Code, w.Body.String())
	}
	if err := m.apply(context.Background(), "rollback", original.Manifest.ID, nil, Binding{}, request.Digest, request.PreviousDigest); err == nil {
		t.Fatal("stale rollback target passed locked guard")
	}
}
func TestHTTPOriginalBundleTextRejectsDuplicateKeys(t *testing.T) {
	s, _, _, bundle, binding := httpFixture(t)
	raw, _ := json.Marshal(bundle)
	original := strings.Replace(string(raw), `"schema":1`, `"schema":1,"schema":1`, 1)
	wrapped, _ := json.Marshal(original)
	if w := httpCall(t, s, "POST", "/review", httpRequest{Action: "install", Bundle: wrapped, Binding: binding}); w.Code != 400 {
		t.Fatal("duplicate original file keys accepted", w.Code)
	}
	wrapped, _ = json.Marshal(string(raw))
	if w := httpCall(t, s, "POST", "/review", httpRequest{Action: "install", Bundle: wrapped, Binding: binding}); w.Code != 200 {
		t.Fatal("raw uploaded bundle rejected", w.Code, w.Body.String())
	}
}

func TestGUIStoragePolicyKeepsBrowserWithinExtensionRoots(t *testing.T) {
	root := "/var/lib/mowgli-extensions"
	id := "example"
	good := Binding{Project: "example", ConfigDir: "/etc/mowgli-extensions/example/config", DataDir: root + "/example/data"}
	if err := guiBinding(root, id, good); err != nil {
		t.Fatal(err)
	}
	for _, path := range []string{"/root/private", "/home/mower/private", "/var/lib/mowgli-updater/jobs", "/home/mower/mowglinext/docker/config", "/var/lib/mowgli-extensions/example-other/data", "/tmp/example/data"} {
		bad := good
		bad.DataDir = path
		if guiBinding(root, id, bad) == nil {
			t.Fatalf("browser data escaped policy: %s", path)
		}
		bad = good
		bad.ConfigDir = path
		if guiBinding(root, id, bad) == nil {
			t.Fatalf("browser config escaped policy: %s", path)
		}
	}
	for _, path := range []string{"/etc/mowgli-extensions/example-other", "/etc/mowgli-extensions/example/../other"} {
		bad := good
		bad.ConfigDir = path
		if guiBinding(root, id, bad) == nil {
			t.Fatal("config sibling/traversal accepted")
		}
	}
}

func TestGUIStoragePolicyAlsoGuardsLockedMutationAndRecovery(t *testing.T) {
	m, backend, bundle, binding := lifecycleFixture(t)
	applyOK(t, m, "install", &bundle, binding) // Local CLI policy intentionally allows this layout.
	gui := m
	gui.bindingPolicy = func(id string, b Binding) error { return guiBinding(m.Store.Root, id, b) }
	if gui.Apply(context.Background(), "enable", "example", nil, Binding{}) == nil {
		t.Fatal("GUI mutation accepted CLI-only storage")
	}
	backend.crashAfter = "start"
	func() {
		defer func() {
			if recover() == nil {
				t.Fatal("crash injection missing")
			}
		}()
		_ = m.Apply(context.Background(), "enable", "example", nil, Binding{})
	}()
	backend.crashAfter = ""
	count := len(backend.events)
	if gui.Recover(context.Background()) == nil {
		t.Fatal("GUI recovery bypassed storage policy")
	}
	if len(backend.events) != count {
		t.Fatal("denied recovery touched runtime")
	}
	if err := m.Recover(context.Background()); err != nil {
		t.Fatal("local CLI could not recover its registration", err)
	}
}

func TestServiceSummaryUsesPhysicalVolumeIdentityAndNoEnvironmentValues(t *testing.T) {
	spec := composeSpec{Services: map[string]composeService{"agent": {
		Image:       "example/image@sha256:" + strings.Repeat("a", 64),
		Environment: map[string]string{"Z_KEY": "PRIVATE_SECRET", "A_KEY": "OTHER_SECRET"},
		Volumes:     []composeMount{{Type: "volume", Source: "data", Target: "/data"}},
		Networks:    map[string]json.RawMessage{"private": json.RawMessage(`{}`)},
	}}, Volumes: map[string]composeVolume{"data": {Name: "existing-identity-volume", External: true}}, Networks: map[string]composeNetwork{"private": {Name: "private-network", Internal: true}}}
	view := summarizeServices(spec)
	raw, _ := json.Marshal(view)
	if len(view) != 1 || view[0].Mounts[0].Source != "existing-identity-volume" || !view[0].Networks[0].Internal || strings.Contains(string(raw), "SECRET") || strings.Join(view[0].EnvironmentKeys, ",") != "A_KEY,Z_KEY" {
		t.Fatal(string(raw))
	}
}
