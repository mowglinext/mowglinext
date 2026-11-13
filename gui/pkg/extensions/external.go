package extensions

import (
	"context"
	"errors"
	"io"
	"net/http"
	"regexp"
	"strings"
	"time"
)

var safeUnit = regexp.MustCompile(`^[a-zA-Z0-9][a-zA-Z0-9_.@-]{0,127}\.service$`)
var invocationPattern = regexp.MustCompile(`^[a-fA-F0-9]{32}$`)

type unitState struct{ active, sub, invocation string }

func (b *RuntimeBackend) unit(ctx context.Context, name string) (unitState, error) {
	var u unitState
	if !safeUnit.MatchString(name) {
		return u, errors.New("invalid external unit name")
	}
	raw, err := b.run(ctx, "systemctl", "show", "--no-pager", "--property=ActiveState,SubState,InvocationID", "--", name)
	if err != nil {
		return u, err
	}
	fields := map[string]string{}
	for _, line := range strings.Split(strings.TrimSpace(string(raw)), "\n") {
		k, v, ok := strings.Cut(line, "=")
		if !ok || fields[k] != "" {
			return u, errors.New("invalid systemd observation")
		}
		switch k {
		case "ActiveState", "SubState", "InvocationID":
			fields[k] = v
		default:
			return u, errors.New("unexpected systemd observation")
		}
	}
	u = unitState{fields["ActiveState"], fields["SubState"], fields["InvocationID"]}
	if u.active == "" || u.sub == "" {
		return u, errors.New("incomplete systemd observation")
	}
	if u.active == "active" && !invocationPattern.MatchString(u.invocation) {
		return u, errors.New("missing external process instance")
	}
	return u, nil
}
func (b *RuntimeBackend) external(ctx context.Context, r Release) (Observation, error) {
	o := Observation{Process: "unknown", Application: "unknown", Data: "unknown", Services: map[string]string{}}
	m := r.Bundle.Manifest
	if err := ValidateManifest(m); err != nil {
		return o, err
	}
	if m.Runtime.Kind != "external-systemd" || len(m.Runtime.Units) == 0 {
		return o, errors.New("external runtime requires explicit units")
	}
	before := map[string]unitState{}
	allActive := true
	anyActive := false
	allStopped := true
	ids := []string{}
	for _, name := range m.Runtime.Units {
		if _, ok := before[name]; ok {
			return o, errors.New("duplicate external unit")
		}
		u, err := b.unit(ctx, name)
		if err != nil {
			return o, err
		}
		before[name] = u
		state := "unknown"
		if u.active == "active" && u.sub == "running" {
			state = "running"
			anyActive = true
		} else if u.active == "inactive" && u.sub == "dead" || u.active == "failed" && u.sub == "failed" {
			state = "stopped"
		}
		allActive = allActive && state == "running"
		allStopped = allStopped && state == "stopped"
		o.Services[name] = state
		ids = append(ids, name+"="+u.invocation)
	}
	o.Process = "unknown"
	if allActive {
		o.Process = "running"
	} else if anyActive {
		o.Process = "partial"
	} else if allStopped {
		o.Process = "stopped"
	}
	o.Identity = r.Digest + ":" + strings.Join(ids, ",")
	if !allActive || m.Health != "application" || m.ID != "mowgli-lora-link" {
		return o, nil
	}
	primary, ok := before["mowgli-lora-robot.service"]
	if !ok {
		return o, nil
	}
	healthy, err := b.loraHealth(ctx, primary.invocation)
	// Bind the HTTP response to the same live systemd invocation on both sides.
	for _, name := range m.Runtime.Units {
		after, e := b.unit(ctx, name)
		if e != nil {
			return Observation{Process: "unknown", Application: "unknown", Data: "unknown"}, e
		}
		if after != before[name] {
			return Observation{Process: "unknown", Application: "unknown", Data: "unknown"}, errors.New("external process changed during health observation")
		}
	}
	if err == nil {
		if healthy {
			o.Application = "healthy"
		} else {
			o.Application = "unhealthy"
		}
	}
	return o, nil
}
func (b *RuntimeBackend) loraHealth(ctx context.Context, expectedInvocation string) (bool, error) {
	ctx, cancel := context.WithTimeout(ctx, 3*time.Second)
	defer cancel()
	req, err := http.NewRequestWithContext(ctx, http.MethodGet, "http://127.0.0.1:9609/healthz", nil)
	if err != nil {
		return false, err
	}
	// Disable redirects and environment proxies. Tests can inject a transport,
	// while the request target remains fixed to the approved local endpoint.
	client := http.Client{Timeout: 3 * time.Second, CheckRedirect: func(*http.Request, []*http.Request) error { return http.ErrUseLastResponse }, Transport: &http.Transport{Proxy: nil, DisableKeepAlives: true}}
	if b.Client != nil {
		client = *b.Client
		client.CheckRedirect = func(*http.Request, []*http.Request) error { return http.ErrUseLastResponse }
		client.Timeout = 3 * time.Second
	}
	res, err := client.Do(req)
	if err != nil {
		return false, errors.New("local modem health unavailable")
	}
	defer res.Body.Close()
	if res.StatusCode != http.StatusOK && res.StatusCode != http.StatusServiceUnavailable {
		return false, errors.New("unexpected local modem health status")
	}
	raw, err := io.ReadAll(io.LimitReader(res.Body, 8193))
	if err != nil || len(raw) > 8192 {
		return false, errors.New("invalid local modem health response")
	}
	var body struct {
		InvocationID string `json:"invocation_id,omitempty"`
		Status       string `json:"status"`
		Role         string `json:"role"`
		Checks       struct {
			Connected *bool `json:"modem_connected"`
			Handshake *bool `json:"modem_handshake_complete"`
			Ready     *bool `json:"modem_radio_ready"`
		} `json:"checks"`
	}
	if strictJSON(raw, &body, 8192) != nil || !invocationPattern.MatchString(body.InvocationID) || body.InvocationID != expectedInvocation || body.Role != "robot" || body.Checks.Connected == nil || body.Checks.Handshake == nil || body.Checks.Ready == nil {
		return false, errors.New("unrecognized local modem health contract")
	}
	healthy := *body.Checks.Connected && *body.Checks.Handshake && *body.Checks.Ready
	if healthy && (body.Status != "ok" || res.StatusCode != http.StatusOK) || !healthy && (body.Status != "degraded" || res.StatusCode != http.StatusServiceUnavailable) {
		return false, errors.New("inconsistent local modem health contract")
	}
	return healthy, nil
}
