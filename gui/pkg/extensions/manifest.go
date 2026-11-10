package extensions

import (
	"bytes"
	"encoding/json"
	"errors"
	"fmt"
	"io"
	"path/filepath"
	"reflect"
	"regexp"
	"slices"
	"strings"

	"github.com/mowglinext/mowglinext/pkg/updates"
)

const inputLimit = 2 << 20

var (
	idPattern       = regexp.MustCompile(`^[a-z0-9][a-z0-9-]{0,62}$`)
	projectPattern  = regexp.MustCompile(`^[a-z0-9][a-z0-9_-]{0,127}$`)
	versionPattern  = regexp.MustCompile(`^(0|[1-9][0-9]*)\.(0|[1-9][0-9]*)\.(0|[1-9][0-9]*)$`)
	revisionPattern = regexp.MustCompile(`^[a-f0-9]{40}$`)
	unitPattern     = regexp.MustCompile(`^[A-Za-z0-9][A-Za-z0-9_.@-]{0,127}\.service$`)
	contractPattern = regexp.MustCompile(`^[A-Za-z0-9][A-Za-z0-9:._/-]{0,255}$`)
	knownGrants     = []string{"network:egress", "network:gui-api", "network:relay-egress", "sandbox:connect-nft-bootstrap", "storage:extension-data", "serial:lora-modem", "network:loopback-rtcm"}
)

// strictJSON rejects ambiguous JSON before decoding the closed Go schema. The
// token walk also covers opaque compose objects, where duplicate keys could
// otherwise change the policy a different JSON consumer sees.
func strictJSON(data []byte, dst any, limit int) error {
	if len(data) > limit {
		return errors.New("JSON exceeds size limit")
	}
	d := json.NewDecoder(bytes.NewReader(data))
	d.UseNumber()
	if err := jsonValue(d, 0); err != nil {
		return err
	}
	if _, err := d.Token(); err != io.EOF {
		return errors.New("trailing JSON input")
	}
	if err := exactJSONFields(data, reflect.TypeOf(dst)); err != nil {
		return err
	}
	d = json.NewDecoder(bytes.NewReader(data))
	d.DisallowUnknownFields()
	if err := d.Decode(dst); err != nil {
		return err
	}
	return nil
}

// encoding/json accepts case-insensitive field aliases. A closed on-disk schema
// accepts only its documented JSON names, including inside maps and pointers.
func exactJSONFields(data []byte, typ reflect.Type) error {
	for typ.Kind() == reflect.Pointer {
		typ = typ.Elem()
	}
	if typ == reflect.TypeOf(json.RawMessage{}) {
		return nil
	}
	switch typ.Kind() {
	case reflect.Struct:
		var fields map[string]json.RawMessage
		if err := json.Unmarshal(data, &fields); err != nil {
			return err
		}
		known := map[string]reflect.Type{}
		for i := 0; i < typ.NumField(); i++ {
			field := typ.Field(i)
			name := strings.Split(field.Tag.Get("json"), ",")[0]
			if name != "" && name != "-" {
				known[name] = field.Type
			}
		}
		for name, raw := range fields {
			fieldType, ok := known[name]
			if !ok {
				return fmt.Errorf("unknown JSON field %q", name)
			}
			if err := exactJSONFields(raw, fieldType); err != nil {
				return err
			}
		}
	case reflect.Map:
		var fields map[string]json.RawMessage
		if err := json.Unmarshal(data, &fields); err != nil {
			return err
		}
		for _, raw := range fields {
			if err := exactJSONFields(raw, typ.Elem()); err != nil {
				return err
			}
		}
	case reflect.Slice, reflect.Array:
		var items []json.RawMessage
		if err := json.Unmarshal(data, &items); err != nil {
			return err
		}
		for _, raw := range items {
			if err := exactJSONFields(raw, typ.Elem()); err != nil {
				return err
			}
		}
	}
	return nil
}

func jsonValue(d *json.Decoder, depth int) error {
	if depth > 64 {
		return errors.New("JSON nesting too deep")
	}
	t, err := d.Token()
	if err != nil {
		return err
	}
	if t == nil {
		return errors.New("null is not a valid schema value")
	}
	delim, ok := t.(json.Delim)
	if !ok {
		return nil
	}
	switch delim {
	case '{':
		keys := map[string]bool{}
		for d.More() {
			k, err := d.Token()
			if err != nil {
				return err
			}
			key, ok := k.(string)
			if !ok {
				return errors.New("invalid object key")
			}
			if keys[key] {
				return fmt.Errorf("duplicate JSON key %q", key)
			}
			keys[key] = true
			if err := jsonValue(d, depth+1); err != nil {
				return err
			}
		}
	case '[':
		for d.More() {
			if err := jsonValue(d, depth+1); err != nil {
				return err
			}
		}
	default:
		return errors.New("unexpected JSON delimiter")
	}
	_, err = d.Token()
	return err
}

func ParseBundle(data []byte) (Bundle, error) {
	var b Bundle
	if err := strictJSON(data, &b, inputLimit); err != nil {
		return b, fmt.Errorf("bundle: %w", err)
	}
	if err := validateBundle(b); err != nil {
		return b, err
	}
	return b, nil
}

func validateBundle(b Bundle) error {
	if err := ValidateManifest(b.Manifest); err != nil {
		return err
	}
	if b.Manifest.Runtime.Kind == "compose" {
		if len(b.Compose) == 0 || len(b.Compose) > inputLimit {
			return errors.New("compose object is required")
		}
		var value map[string]json.RawMessage
		if err := strictJSON(b.Compose, &value, inputLimit); err != nil {
			return fmt.Errorf("compose: %w", err)
		}
		if len(value) == 0 {
			return errors.New("compose object is empty")
		}
	} else if len(b.Compose) != 0 {
		return errors.New("external services cannot contain compose")
	}
	return nil
}

func ValidateManifest(m Manifest) error {
	if m.Schema != Schema {
		return errors.New("unsupported manifest schema")
	}
	if !idPattern.MatchString(m.ID) {
		return errors.New("invalid extension id")
	}
	if !versionPattern.MatchString(m.Version) {
		return errors.New("version must be numeric major.minor.patch")
	}
	if !updates.RepositoryPattern.MatchString(m.Source.Repository) || !revisionPattern.MatchString(m.Source.Revision) {
		return errors.New("source requires owner/repository and full lowercase commit")
	}
	if m.Requires.ManagerAPI < 1 {
		return errors.New("invalid manager API requirement")
	}
	if len(m.Requires.Platforms) == 0 {
		return errors.New("platforms are required")
	}
	if err := validateList(m.Requires.Platforms, func(s string) bool { return s == "linux/arm64" || s == "linux/amd64" }); err != nil {
		return fmt.Errorf("platforms: %w", err)
	}
	if m.Requires.Contracts == nil {
		return errors.New("contracts must be an array")
	}
	if err := validateList(m.Requires.Contracts, contractPattern.MatchString); err != nil {
		return fmt.Errorf("contracts: %w", err)
	}
	if m.Grants == nil {
		return errors.New("grants must be an array")
	}
	if err := validateList(m.Grants, func(s string) bool { return slices.Contains(knownGrants, s) }); err != nil {
		return fmt.Errorf("grants: %w", err)
	}
	if m.DataSchema < 1 {
		return errors.New("data schema must be positive")
	}
	if m.Health != "process-only" && m.Health != "application" {
		return errors.New("unsupported health policy")
	}
	switch m.Runtime.Kind {
	case "compose":
		if len(m.Runtime.Units) != 0 {
			return errors.New("compose runtime cannot declare systemd units")
		}
		if m.Profile != "isolated" && m.Profile != "connect" {
			return errors.New("unsupported compose profile")
		}
	case "external-systemd":
		if m.Profile != "external" || len(m.Runtime.Units) == 0 {
			return errors.New("external runtime requires external profile and units")
		}
		if err := validateList(m.Runtime.Units, unitPattern.MatchString); err != nil {
			return fmt.Errorf("units: %w", err)
		}
	default:
		return errors.New("unsupported runtime")
	}
	return nil
}

func validateList(values []string, valid func(string) bool) error {
	seen := map[string]bool{}
	for _, s := range values {
		if !valid(s) {
			return errors.New("invalid list value")
		}
		if seen[s] {
			return errors.New("duplicate list value")
		}
		seen[s] = true
	}
	return nil
}

func ParseHost(data []byte) (HostConfig, error) {
	var h HostConfig
	if err := strictJSON(data, &h, inputLimit); err != nil {
		return h, fmt.Errorf("host: %w", err)
	}
	return h, validateHost(h)
}

func validateHost(h HostConfig) error {
	if h.Schema != Schema {
		return errors.New("unsupported host schema")
	}
	if !projectPattern.MatchString(h.Installation) {
		return errors.New("invalid installation identity")
	}
	if h.Platform != "linux/amd64" && h.Platform != "linux/arm64" {
		return errors.New("unsupported host platform")
	}
	if h.Contracts == nil || h.TrustedRepositories == nil || h.ApprovedGrants == nil || h.ExternalUnits == nil {
		return errors.New("host allowlists must be arrays")
	}
	for _, check := range []struct {
		values []string
		valid  func(string) bool
	}{
		{h.Contracts, contractPattern.MatchString},
		{h.TrustedRepositories, updates.RepositoryPattern.MatchString},
		{h.ApprovedGrants, func(s string) bool { return slices.Contains(knownGrants, s) }},
		{h.ExternalUnits, unitPattern.MatchString},
	} {
		if err := validateList(check.values, check.valid); err != nil {
			return fmt.Errorf("host allowlist: %w", err)
		}
	}
	return nil
}

// Compatible is read-only and reports independent compatibility blockers.
func Compatible(m Manifest, h HostConfig) []string {
	blockers := []string{}
	if err := ValidateManifest(m); err != nil {
		blockers = append(blockers, "invalid manifest: "+err.Error())
	}
	if err := validateHost(h); err != nil {
		blockers = append(blockers, "invalid host: "+err.Error())
		return blockers
	}
	if m.Requires.ManagerAPI != Schema {
		blockers = append(blockers, "manager API requirement is unsupported")
	}
	if !slices.Contains(h.TrustedRepositories, m.Source.Repository) {
		blockers = append(blockers, "source repository is not trusted")
	}
	if !slices.Contains(m.Requires.Platforms, h.Platform) {
		blockers = append(blockers, "host platform is unsupported")
	}
	for _, c := range m.Requires.Contracts {
		if !slices.Contains(h.Contracts, c) {
			blockers = append(blockers, "missing contract: "+c)
		}
	}
	for _, g := range m.Grants {
		if !slices.Contains(h.ApprovedGrants, g) {
			blockers = append(blockers, "unapproved grant: "+g)
		}
	}
	for _, u := range m.Runtime.Units {
		if !slices.Contains(h.ExternalUnits, u) {
			blockers = append(blockers, "external unit is not approved: "+u)
		}
	}
	return blockers
}

// BundleDigest hashes canonical JSON so object ordering and whitespace cannot
// create different identities for the same locked bundle.
func BundleDigest(b Bundle) string {
	data, err := json.Marshal(b)
	if err != nil {
		return ""
	}
	var value any
	d := json.NewDecoder(bytes.NewReader(data))
	d.UseNumber()
	if d.Decode(&value) != nil {
		return ""
	}
	data, err = json.Marshal(value)
	if err != nil {
		return ""
	}
	return updates.Hash(data)
}

func ValidateBinding(b Binding) error {
	if !projectPattern.MatchString(b.Project) {
		return errors.New("invalid project identity")
	}
	for _, path := range []string{b.ConfigDir, b.DataDir} {
		if !filepath.IsAbs(path) || filepath.Clean(path) != path || path == "/" || strings.ContainsAny(path, "\x00\n\r") {
			return errors.New("binding requires clean absolute config/data paths")
		}
	}
	if b.ConfigDir == b.DataDir || strings.HasPrefix(b.ConfigDir, b.DataDir+"/") || strings.HasPrefix(b.DataDir, b.ConfigDir+"/") {
		return errors.New("config and data paths must be separate")
	}
	return nil
}

func NewState() State {
	return State{Schema: Schema, Entries: map[string]Entry{}, Retained: map[string]Retained{}}
}
