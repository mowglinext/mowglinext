// Package updater owns host-side deployment policy and durable update jobs.
// It deliberately does not import the GUI server or its database.
package updater

import (
	"encoding/json"
	"errors"
	"fmt"
	"os"
	"path/filepath"
	"regexp"
	"strings"
	"time"

	"github.com/mowglinext/mowglinext/pkg/updates"
)

const APIVersion = 1
const LayoutVersion = 1
const DataSchemaVersion = 1
const StateSchema = 5
const DefaultCheckIntervalHours = 24

var Version = "development"
var Revision = ""
var idPattern = regexp.MustCompile(`^[a-zA-Z0-9][a-zA-Z0-9_.-]{0,127}$`)
var revisionPattern = regexp.MustCompile(`^[a-f0-9]{40}$`)

type Source struct {
	Repository string `json:"repository"`
	Track      string `json:"track"`
	Branch     string `json:"branch"`
}

func (s Source) Validate(trusted []string) error {
	allowed := false
	for _, repo := range trusted {
		if s.Repository == repo {
			allowed = true
		}
	}
	if !allowed || !updates.RepositoryPattern.MatchString(s.Repository) {
		return errors.New("repository is not configured as a trusted update source")
	}
	if s.Track != "stable" && s.Track != "dev" && s.Track != "custom" {
		return errors.New("invalid track")
	}
	if s.Track == "stable" && s.Branch != "main" || s.Track == "dev" && s.Branch != "dev" {
		return errors.New("track and branch disagree")
	}
	if s.Branch == "" || len(s.Branch) > 200 || strings.ContainsAny(s.Branch, "\x00\r\n\\ ?*[:~^") || strings.Contains(s.Branch, "..") || strings.HasPrefix(s.Branch, "-") {
		return errors.New("invalid branch")
	}
	return nil
}

type Binary struct {
	Asset   string `json:"asset"`
	SHA256  string `json:"sha256"`
	Version string `json:"version"`
}
type Deployment struct {
	ImageContract          int                      `json:"image_contract,omitempty"`
	Bundle                 *BundleAsset             `json:"compose_bundle,omitempty"`
	Schema                 int                      `json:"schema"`
	ID                     string                   `json:"id"`
	Source                 Source                   `json:"source"`
	Revision               string                   `json:"revision"`
	PublishedAt            time.Time                `json:"published_at"`
	ReleaseTag             string                   `json:"release_tag"`
	Layout                 int                      `json:"layout"`
	DataSchema             int                      `json:"data_schema"`
	UpdaterAPI             int                      `json:"updater_api"`
	FirmwareProtocol       int                      `json:"firmware_protocol"`
	MaintenanceAPI         int                      `json:"maintenance_api"`
	ComponentCompatibility map[string]string        `json:"component_compatibility,omitempty"`
	ServiceChoices         []ServiceChoice          `json:"service_choices,omitempty"`
	GUICompatibility       string                   `json:"gui_compatibility,omitempty"`
	Images                 map[string]updates.Image `json:"images"`
	Updater                map[string]Binary        `json:"updater"`
}

var Services = map[string]string{"mowgli": "mowgli-ros2", "gui": "mowglinext-gui", "gps": "gps", "lidar": ""}

func (d Deployment) Validate(trusted []string) error {
	if err := d.Source.Validate(trusted); err != nil {
		return err
	}
	if (d.Schema != 1 && d.Schema != 2 && d.Schema != 3) || !idPattern.MatchString(d.ID) || !revisionPattern.MatchString(d.Revision) || d.PublishedAt.IsZero() || !idPattern.MatchString(d.ReleaseTag) {
		return errors.New("invalid deployment identity or schema")
	}
	if d.Schema >= 2 && (d.Bundle == nil || d.Bundle.Asset != "mowgli-compose.json" || !updates.DigestPattern.MatchString("sha256:"+d.Bundle.SHA256)) {
		return errors.New("deployment requires a valid Compose bundle")
	}
	if d.Schema == 1 && d.Bundle != nil {
		return errors.New("Compose bundles require deployment schema 2")
	}
	if d.Layout != LayoutVersion || d.DataSchema != DataSchemaVersion || d.UpdaterAPI > APIVersion || d.MaintenanceAPI != 1 || d.FirmwareProtocol < 1 {
		return errors.New("deployment requires unsupported layout, data schema or updater")
	}
	if d.GUICompatibility != "" && !idPattern.MatchString(d.GUICompatibility) {
		return errors.New("invalid GUI compatibility contract")
	}
	for family, contract := range d.ComponentCompatibility {
		if _, ok := d.Images[family]; !ok || !idPattern.MatchString(contract) {
			return fmt.Errorf("invalid component compatibility contract for %s", family)
		}
	}
	for _, choice := range d.ServiceChoices {
		if !idPattern.MatchString(choice.Service) {
			return errors.New("invalid release service choice")
		}
		if _, ok := d.Images[choice.Image]; !ok {
			return errors.New("release service choice has no image")
		}
		for key, value := range choice.When {
			if !idPattern.MatchString(key) || !idPattern.MatchString(value) {
				return errors.New("invalid hardware selection")
			}
		}
	}
	for name, image := range d.Images {
		if !imageNamePattern.MatchString(name) || !updates.DigestPattern.MatchString(image.Digest) {
			return fmt.Errorf("invalid image %s", name)
		}
		external := image.Type == "external"
		if image.Type != "" && image.Type != "built" && !external {
			return fmt.Errorf("unknown image type for %s", name)
		}
		if external {
			if d.Schema < 3 || name == "mowgli-ros2" || name == "mowglinext-gui" || !idPattern.MatchString(image.Version) {
				return fmt.Errorf("invalid external component %s", name)
			}
			if _, _, _, err := updates.RegistryLocation(image.Repository); err != nil {
				return err
			}
			for _, arch := range []string{"linux/amd64", "linux/arm64"} {
				if _, ok := image.Platforms[arch]; !ok {
					return fmt.Errorf("missing external platform %s", arch)
				}
			}
		} else if image.Repository != "ghcr.io/"+strings.ToLower(d.Source.Repository)+"/"+name {
			return fmt.Errorf("invalid image %s", name)
		}
		for platform, p := range image.Platforms {
			if platform != "linux/arm64" && platform != "linux/amd64" || !updates.DigestPattern.MatchString(p.Manifest) || !updates.DigestPattern.MatchString(p.Config) || (!external && p.Revision != d.Revision) {
				return fmt.Errorf("invalid platform or source for %s", name)
			}
		}
	}
	for _, name := range []string{"mowgli-ros2", "mowglinext-gui"} {
		if _, ok := d.Images[name]; !ok {
			return fmt.Errorf("missing image %s", name)
		}
	}
	for platform, b := range d.Updater {
		if platform != "linux/arm64" && platform != "linux/amd64" || !idPattern.MatchString(b.Asset) || !updates.DigestPattern.MatchString("sha256:"+b.SHA256) || !idPattern.MatchString(b.Version) {
			return errors.New("invalid updater asset")
		}
	}
	return nil
}

type Policy struct {
	Source        Source `json:"source"`
	IntervalHours int    `json:"interval_hours"`
	Pinned        bool   `json:"pinned"`
}
type Notice struct {
	ID         string    `json:"id"`
	Kind       string    `json:"kind"`
	Deployment string    `json:"deployment"`
	CreatedAt  time.Time `json:"created_at"`
	Read       bool      `json:"read"`
	Dismissed  bool      `json:"dismissed"`
}
type Plan struct {
	CustomImages map[string]CustomImage `json:"custom_images,omitempty"`
	Stack        *StackPlan             `json:"stack,omitempty"`
	ID           string                 `json:"id"`
	Target       Deployment             `json:"target"`
	Policy       Policy                 `json:"policy"`
	Fingerprint  string                 `json:"fingerprint"`
	ExpiresAt    time.Time              `json:"expires_at"`
	Images       map[string]string      `json:"images"`
	Previous     map[string]string      `json:"previous"`
	Overrides    map[string]Deployment  `json:"overrides,omitempty"`
}
type Job struct {
	PreviousCustomImages map[string]CustomImage `json:"previous_custom_images,omitempty"`
	ID                   string                 `json:"id"`
	Kind                 string                 `json:"kind"`
	Phase                string                 `json:"phase"`
	Committed            string                 `json:"committed,omitempty"`
	Error                string                 `json:"error,omitempty"`
	StartedAt            time.Time              `json:"started_at"`
	Plan                 Plan                   `json:"plan"`
	Backup               string                 `json:"backup,omitempty"`
	PreviousPolicy       Policy                 `json:"previous_policy"`
	PreviousActive       *Deployment            `json:"previous_active,omitempty"`
	PreviousOverrides    map[string]Deployment  `json:"previous_overrides,omitempty"`
	PreviousImages       map[string]string      `json:"previous_images,omitempty"`
	PreviousJobID        string                 `json:"previous_job_id,omitempty"`
}

func (j *Job) Pending() bool {
	return j != nil && j.Phase != "succeeded" && j.Phase != "rolled_back" && j.Phase != "failed"
}

type State struct {
	CustomImages    map[string]CustomImage `json:"custom_images,omitempty"`
	Schema          int                    `json:"schema"`
	Policy          Policy                 `json:"policy"`
	Active          *Deployment            `json:"active,omitempty"`
	InstalledPolicy *Policy                `json:"installed_policy,omitempty"`
	Overrides       map[string]Deployment  `json:"overrides,omitempty"`
	InstalledImages map[string]string      `json:"installed_images,omitempty"`
	ActiveJobID     string                 `json:"active_job_id,omitempty"`
	LastCheck       time.Time              `json:"last_check"`
	LastSuccess     time.Time              `json:"last_success"`
	NextCheck       time.Time              `json:"next_check"`
	CheckError      string                 `json:"check_error,omitempty"`
	Releases        []Deployment           `json:"releases"`
	Notices         []Notice               `json:"notices"`
	Plans           []Plan                 `json:"plans"`
	Job             *Job                   `json:"job,omitempty"`
	History         []Job                  `json:"history"`
}

// AtomicJSON commits a complete state before any destructive step. Syncing the
// directory makes the rename durable across a power failure on Linux.
func AtomicJSON(path string, value any) error {
	data, err := json.MarshalIndent(value, "", "  ")
	if err != nil {
		return err
	}
	return AtomicWrite(path, data, 0600)
}
func AtomicWrite(path string, data []byte, mode os.FileMode) error {
	if err := os.MkdirAll(filepath.Dir(path), 0750); err != nil {
		return err
	}
	f, err := os.CreateTemp(filepath.Dir(path), ".pending-*")
	if err != nil {
		return err
	}
	defer os.Remove(f.Name())
	if err = inheritFileOwner(f, path); err == nil {
		err = f.Chmod(mode)
	}
	if err == nil {
		_, err = f.Write(data)
	}
	if err == nil {
		err = f.Sync()
	}
	closeErr := f.Close()
	if err == nil {
		err = closeErr
	}
	if err != nil {
		return err
	}
	if err = os.Rename(f.Name(), path); err != nil {
		return err
	}
	dir, err := os.Open(filepath.Dir(path))
	if err != nil {
		return err
	}
	defer dir.Close()
	// Windows is supported for development tests, not host installation.
	if err = dir.Sync(); err != nil && os.PathSeparator != '\\' {
		return err
	}
	return nil
}
