package extensions

import (
	"context"
	"crypto/sha256"
	"encoding/hex"
	"encoding/json"
	"errors"
	"io"
	"net/http"
	"os"
	"path/filepath"
	"runtime"
	"strings"
	"sync"
	"time"
)

// HTTPService lives on the existing host updater's Unix socket. It never runs
// inside the GUI container or exposes bundles, environment or arbitrary commands.
// Only one short-lived job is kept in memory; crash recovery belongs to Store.
type HTTPService struct {
	load      func() (Manager, error)
	blocked   func() bool
	mu        sync.Mutex
	operation *HTTPOperation
	entries   []Status
}
type HTTPOperation struct {
	Action string `json:"action"`
	ID     string `json:"id"`
	Phase  string `json:"phase"`
	Error  string `json:"error,omitempty"`
}
type httpRequest struct {
	Action         string          `json:"action"`
	ID             string          `json:"id,omitempty"`
	Bundle         json.RawMessage `json:"bundle,omitempty"`
	Binding        Binding         `json:"binding,omitempty"`
	Digest         string          `json:"digest,omitempty"`
	PreviousDigest string          `json:"previous_digest,omitempty"`
	Acknowledged   bool            `json:"acknowledged,omitempty"`
}
type httpReview struct {
	Manifest Manifest `json:"manifest"`
	Digest   string   `json:"digest"`
	Binding  Binding  `json:"binding"`
	Action   string   `json:"action"`
	Blockers []string `json:"blockers"`
	current  string
}

// NewHTTPHandler uses host-admin paths only. Missing policy leaves GUI setup
// disabled and creates no register. It never switches UID, root or ownership.
func NewHTTPHandler(policyPath, stateRoot string, blocked func() bool) http.Handler {
	return &HTTPService{blocked: blocked, entries: []Status{}, load: func() (Manager, error) {
		f, err := os.Open(policyPath)
		if err != nil {
			return Manager{}, err
		}
		defer f.Close()
		data, err := io.ReadAll(io.LimitReader(f, inputLimit+1))
		if err != nil {
			return Manager{}, err
		}
		host, err := ParseHost(data)
		if err != nil {
			return Manager{}, errors.New("invalid host extension policy")
		}
		if host.Platform != runtime.GOOS+"/"+runtime.GOARCH {
			return Manager{}, errors.New("host extension platform mismatch")
		}
		if !filepath.IsAbs(stateRoot) || temporaryPath(stateRoot) {
			return Manager{}, errors.New("extension register requires a persistent absolute path")
		}
		return Manager{Store: Store{Root: stateRoot}, Host: host, Backend: NewBackend(stateRoot)}, nil
	}}
}
func temporaryPath(path string) bool {
	for _, root := range []string{"/tmp", "/var/tmp"} {
		if path == root || strings.HasPrefix(path, root+"/") {
			return true
		}
	}
	return false
}
func persistentBinding(binding Binding) error {
	if temporaryPath(binding.ConfigDir) || temporaryPath(binding.DataDir) {
		return errors.New("GUI extensions require persistent config/data paths outside /tmp and /var/tmp")
	}
	return ValidateBinding(binding)
}
func extensionJSON(w http.ResponseWriter, code int, value any) {
	w.Header().Set("Content-Type", "application/json")
	w.Header().Set("Cache-Control", "no-store")
	w.WriteHeader(code)
	_ = json.NewEncoder(w).Encode(value)
}
func extensionError(w http.ResponseWriter, code int, err error) {
	extensionJSON(w, code, map[string]string{"error": err.Error()})
}
func (s *HTTPService) ServeHTTP(w http.ResponseWriter, r *http.Request) {
	if r.URL.RawQuery != "" {
		extensionError(w, 400, errors.New("query parameters are not supported"))
		return
	}
	op := strings.TrimPrefix(r.URL.Path, "/")
	if !(r.Method == http.MethodGet && op == "state") && !(r.Method == http.MethodPost && (op == "review" || op == "apply" || op == "action")) {
		http.NotFound(w, r)
		return
	}
	s.mu.Lock()
	defer s.mu.Unlock()
	m, err := s.load()
	if err != nil {
		if r.Method == http.MethodGet {
			extensionJSON(w, 200, map[string]any{"api": 1, "configured": false, "entries": []Status{}, "error": "Configure the host extension policy and its owning account before managing integrations"})
		} else {
			extensionError(w, 503, errors.New("host extension policy unavailable"))
		}
		return
	}
	if r.Method == http.MethodGet {
		// Polling an active job must not contend with its durable transaction lock.
		if s.operation == nil || s.operation.Phase != "running" {
			ctx, cancel := context.WithTimeout(r.Context(), 15*time.Second)
			entries, e := m.Status(ctx)
			cancel()
			if e != nil {
				extensionError(w, 409, errors.New("extension state unavailable; check register ownership or an active CLI operation"))
				return
			}
			s.entries = entries
		}
		extensionJSON(w, 200, map[string]any{"api": 1, "configured": true, "entries": s.entries, "operation": s.operation})
		return
	}
	data, err := io.ReadAll(http.MaxBytesReader(w, r.Body, 2*inputLimit+16384))
	var request httpRequest
	if err != nil || strictJSON(data, &request, 2*inputLimit+16384) != nil {
		extensionError(w, 400, errors.New("invalid or oversized extension request"))
		return
	}
	if s.operation != nil && s.operation.Phase == "running" {
		extensionError(w, 409, errors.New("an extension operation is already running"))
		return
	}
	var bundle *Bundle
	var expected string
	if op == "review" || op == "apply" {
		review, b, e := s.review(m, request)
		if e != nil {
			extensionError(w, 400, e)
			return
		}
		if op == "review" {
			extensionJSON(w, 200, review)
			return
		}
		if !request.Acknowledged || request.Digest != review.Digest || request.ID != review.Manifest.ID {
			extensionError(w, 409, errors.New("confirm the exact reviewed bundle, action and storage binding"))
			return
		}
		if len(review.Blockers) != 0 {
			extensionError(w, 409, errors.New(strings.Join(review.Blockers, "; ")))
			return
		}
		request.Binding = review.Binding
		bundle = &b
		if request.Action == "upgrade" {
			expected = review.current
		}
	} else {
		switch request.Action {
		case "enable", "disable", "rollback", "remove", "forget":
			state, e := m.Store.Load()
			if e != nil {
				extensionError(w, 409, errors.New("register unavailable"))
				return
			}
			entry, ok := state.Entries[request.ID]
			if !ok || request.Digest == "" || entry.Active.Digest != request.Digest {
				extensionError(w, 409, errors.New("the reviewed release changed; refresh and review again"))
				return
			}
			if e = persistentBinding(entry.Active.Binding); e != nil {
				extensionError(w, 409, e)
				return
			}
			expected = request.Digest
			if request.Action == "rollback" && (entry.Previous == nil || request.PreviousDigest == "" || request.PreviousDigest != entry.Previous.Digest) {
				extensionError(w, 409, errors.New("the reviewed rollback target changed; refresh and review again"))
				return
			}
		case "recover":
			if request.ID != "" || request.Digest != "" {
				extensionError(w, 400, errors.New("recovery applies to the pending journal"))
				return
			}
		default:
			extensionError(w, 400, errors.New("unsupported extension action"))
			return
		}
		if !request.Acknowledged || len(request.Bundle) != 0 || request.Binding != (Binding{}) {
			extensionError(w, 400, errors.New("explicit action confirmation required"))
			return
		}
	}
	if s.blocked != nil && s.blocked() {
		extensionError(w, 423, errors.New("core update is active; wait before changing extensions"))
		return
	}
	operation := &HTTPOperation{Action: request.Action, ID: request.ID, Phase: "running"}
	s.operation = operation
	go func() {
		// Browser disconnects do not cancel a confirmed operation halfway through.
		// A host restart still leaves the manager's explicit recovery journal.
		ctx, cancel := context.WithTimeout(context.Background(), 2*time.Minute)
		defer cancel()
		var e error
		if request.Action == "recover" {
			e = m.Recover(ctx)
		} else {
			e = m.apply(ctx, request.Action, request.ID, bundle, request.Binding, expected, request.PreviousDigest)
		}
		s.mu.Lock()
		defer s.mu.Unlock()
		operation.Phase = "succeeded"
		if e != nil {
			operation.Phase = "failed"
			operation.Error = e.Error()
		}
	}()
	extensionJSON(w, 202, operation)
}

func (s *HTTPService) review(m Manager, req httpRequest) (httpReview, Bundle, error) {
	result := httpReview{Action: req.Action, Blockers: []string{}}
	raw := req.Bundle
	// Preserve uploaded text until the strict parser sees the original keys.
	// Direct objects remain supported for local Unix clients.
	if len(raw) > 0 && raw[0] == '"' {
		var original string
		if json.Unmarshal(raw, &original) != nil {
			return result, Bundle{}, errors.New("invalid bundle text")
		}
		raw = []byte(original)
	}
	bundle, err := ParseBundle(raw)
	if err != nil {
		return result, bundle, errors.New("bundle does not match the supported manifest and Compose schema")
	}
	result.Manifest = bundle.Manifest
	binding := req.Binding
	current := ""
	switch req.Action {
	case "upgrade":
		state, e := m.Store.Load()
		if e != nil {
			return result, bundle, errors.New("register unavailable")
		}
		entry, ok := state.Entries[bundle.Manifest.ID]
		if !ok {
			return result, bundle, errors.New("upgrade requires a registered extension")
		}
		if binding != (Binding{}) && binding != entry.Active.Binding {
			return result, bundle, errors.New("upgrade cannot change storage ownership")
		}
		binding = entry.Active.Binding
		current = entry.Active.Digest
	case "install", "adopt":
		if req.Action == "adopt" && binding.Project == "" {
			return result, bundle, errors.New("adoption requires the explicit existing project")
		}
		if binding.Project == "" {
			binding.Project = m.Host.Installation + "-ext-" + bundle.Manifest.ID
		}
		if binding.ConfigDir == "" {
			binding.ConfigDir = filepath.Join("/etc/mowgli-extensions", bundle.Manifest.ID)
		}
		if binding.DataDir == "" {
			binding.DataDir = filepath.Join(m.Store.Root, bundle.Manifest.ID, "data")
		}
	default:
		return result, bundle, errors.New("review supports install, adopt or upgrade only")
	}
	if err = persistentBinding(binding); err != nil {
		return result, bundle, err
	}
	if bundle.Manifest.Runtime.Kind == "compose" {
		if err = ValidateCompose(bundle, binding); err != nil {
			return result, bundle, err
		}
	} else if req.Action != "adopt" {
		result.Blockers = append(result.Blockers, "external services support observation/adoption only")
	}
	result.Binding = binding
	result.current = current
	result.Blockers = append(result.Blockers, Compatible(bundle.Manifest, m.Host)...)
	// Bind confirmation to action, destination, target bundle and prior release.
	// A modified form or intervening CLI upgrade requires a new review.
	fingerprint, _ := json.Marshal(struct {
		Action  string
		Binding Binding
		Bundle  string
		Current string
	}{req.Action, binding, BundleDigest(bundle), current})
	sum := sha256.Sum256(fingerprint)
	result.Digest = "sha256:" + hex.EncodeToString(sum[:])
	return result, bundle, nil
}
