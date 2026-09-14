package updater

import (
	"context"
	"encoding/json"
	"errors"
	"fmt"
	"os"
	"path/filepath"
	"reflect"
	"sort"
	"strings"

	"github.com/mowglinext/mowglinext/pkg/updates"
)

var stackFiles = []string{"stack-selection.json", "stack-applied-selection.json", "stack-bundle.json", "stack-release.json", "stack-definition.sha256", "stack-overrides.yaml"}

type ServiceChange struct {
	Service string `json:"service"`
	Action  string `json:"action"`
}
type StackPlan struct {
	Changes   []ServiceChange `json:"changes"`
	Selection StackSelection  `json:"selection"`
	// Private recovery payload. PublicPlan strips this from HTTP responses.
	Compose []byte            `json:"compose,omitempty"`
	Bundle  ComposeBundle     `json:"bundle,omitempty"`
	Retired map[string]string `json:"retired,omitempty"` // container name -> original ID
	Created map[string]string `json:"created,omitempty"` // container name -> Compose service
}

func PublicPlan(p Plan) Plan {
	if p.Stack != nil {
		p.Stack = &StackPlan{Changes: p.Stack.Changes, Selection: p.Stack.Selection}
	}
	return p
}
func PublicState(s State) State {
	for i := range s.Plans {
		s.Plans[i] = PublicPlan(s.Plans[i])
	}
	if s.Job != nil {
		s.Job.Plan = PublicPlan(s.Job.Plan)
	}
	for i := range s.History {
		s.History[i].Plan = PublicPlan(s.History[i].Plan)
	}
	return s
}

func (b DockerBackend) renderFiles(ctx context.Context, files []string, interpolate bool) ([]byte, error) {
	args := []string{"compose", "--project-name", b.Config.Project, "--project-directory", filepath.Dir(b.Config.Directory), "--env-file", filepath.Join(b.Config.Directory, ".env")}
	for _, file := range files {
		args = append(args, "-f", file)
	}
	args = append(args, "config", "--format", "json")
	if !interpolate {
		args = append(args, "--no-interpolate")
	}
	data, err := command(ctx, "docker", args...)
	if err != nil {
		return nil, errors.New("Compose validation failed; check the release definition and local overrides")
	}
	var doc map[string]any
	var config composeConfig
	if err = json.Unmarshal(data, &doc); err != nil {
		return nil, err
	}
	if err = json.Unmarshal(data, &config); err != nil {
		return nil, err
	}
	for name, service := range config.Services {
		sc := object(object(doc["services"])[name])
		if _, exists := sc["labels"]; exists {
			sc["labels"] = service.Labels
		}
		if _, exists := sc["environment"]; exists {
			sc["environment"] = service.Environment
		}
	}
	return json.Marshal(doc)
}

func (b DockerBackend) renderBundle(ctx context.Context, bundle ComposeBundle, selection StackSelection) ([]byte, error) {
	if err := bundle.Validate(); err != nil {
		return nil, err
	}
	names, err := bundle.Select(selection.Options)
	if err != nil {
		return nil, err
	}
	dir, err := os.MkdirTemp(b.Config.Directory, ".compose-bundle-")
	if err != nil {
		return nil, err
	}
	defer os.RemoveAll(dir)
	files := []string{}
	for _, name := range names {
		path := filepath.Join(dir, name)
		if err = os.WriteFile(path, bundle.Fragments[name], 0600); err != nil {
			return nil, err
		}
		files = append(files, path)
	}
	return b.renderFiles(ctx, files, false)
}

func object(value any) map[string]any {
	if m, ok := value.(map[string]any); ok {
		return m
	}
	return map[string]any{}
}

// Preserve non-managed local services/resources, including MQTT. A release may
// retire its own services; it cannot claim an unrelated service with the same key.
func retainLocalServices(before, target []byte, managed map[string]managedService) ([]byte, error) {
	var old, next map[string]any
	if err := json.Unmarshal(before, &old); err != nil {
		return nil, err
	}
	if err := json.Unmarshal(target, &next); err != nil {
		return nil, err
	}
	nextServices := object(next["services"])
	for name, service := range object(old["services"]) {
		if _, owned := managed[name]; owned {
			continue
		}
		if _, exists := nextServices[name]; exists {
			return nil, fmt.Errorf("release conflicts with local service %s", name)
		}
		nextServices[name] = service
	}
	next["services"] = nextServices
	for _, key := range []string{"volumes", "networks", "configs", "secrets"} {
		resources := object(next[key])
		for name, value := range object(old[key]) {
			if other, exists := resources[name]; exists && !reflect.DeepEqual(value, other) {
				// Compose materializes project-derived resource names. Keep the
				// installed physical identity when the logical key/driver agree;
				// adopting a generated project name must never create empty maps.
				left, right := map[string]any{}, map[string]any{}
				for k, v := range object(value) {
					if k != "name" {
						left[k] = v
					}
				}
				for k, v := range object(other) {
					if k != "name" {
						right[k] = v
					}
				}
				if (key != "volumes" && key != "networks") || !reflect.DeepEqual(left, right) {
					return nil, fmt.Errorf("resource %s changed; explicit layout migration required", name)
				}
			}
			resources[name] = value
		}
		if len(resources) > 0 {
			next[key] = resources
		}
	}
	return json.Marshal(next)
}

func (b DockerBackend) checkStackBaseline() error {
	data, err := os.ReadFile(filepath.Join(b.Config.Directory, "docker-compose.yaml"))
	if err != nil {
		return err
	}
	expected, err := os.ReadFile(filepath.Join(b.Config.Directory, "stack-definition.sha256"))
	if err != nil {
		return errors.New("run the current installer once to register hardware selections and the Compose baseline")
	}
	if strings.TrimSpace(string(expected)) != updates.Hash(data) {
		return errors.New("generated Compose was edited manually; move intentional changes into stack-overrides.yaml and regenerate before updating")
	}
	return nil
}

func (b DockerBackend) PlanStack(ctx context.Context, d Deployment, overrides map[string]Deployment) (map[string]string, *StackPlan, error) {
	if err := d.Validate(b.Config.Trusted); err != nil {
		return nil, nil, err
	}
	if err := b.checkStackBaseline(); err != nil {
		return nil, nil, err
	}
	var selection StackSelection
	data, err := os.ReadFile(filepath.Join(b.Config.Directory, "stack-selection.json"))
	if err != nil {
		return nil, nil, err
	}
	if err = json.Unmarshal(data, &selection); err != nil {
		return nil, nil, err
	}
	identity, err := installerConfigIdentity(b.Config.Directory)
	if err != nil {
		return nil, nil, err
	}
	if identity != selection.InstallerConfig {
		return nil, nil, errors.New("installer hardware choices changed; rerun the installer to reconcile container selection first")
	}
	bundle, err := b.loadBundle(ctx, d)
	if err != nil {
		return nil, nil, err
	}
	return b.planBundle(ctx, d, overrides, bundle, selection)
}

func (b DockerBackend) planBundle(ctx context.Context, d Deployment, overrides map[string]Deployment, bundle ComposeBundle, selection StackSelection) (map[string]string, *StackPlan, error) {
	if d.ServiceChoices != nil {
		choices, err := bundle.ServiceChoices()
		if err != nil {
			return nil, nil, err
		}
		if !reflect.DeepEqual(choices, d.ServiceChoices) {
			return nil, nil, errors.New("release service choices disagree with verified Compose bundle")
		}
	}
	ready, err := b.readiness(ctx)
	if err != nil {
		return nil, nil, err
	}
	if ready.FirmwareProtocol != d.FirmwareProtocol {
		return nil, nil, errors.New("target requires a different mainboard firmware protocol")
	}
	current, _, err := b.model(ctx)
	if err != nil {
		return nil, nil, err
	}
	managed, err := managedServices(current)
	if err != nil {
		return nil, nil, err
	}
	old, err := b.composeWithOverride(ctx, false, "config", "--no-interpolate", "--format", "json")
	if err != nil {
		return nil, nil, err
	}
	target, err := b.renderBundle(ctx, bundle, selection)
	if err != nil {
		return nil, nil, err
	}
	// All release services must opt into managed updates. Local services remain
	// outside the transaction and cannot be declared through a release fragment.
	var release composeConfig
	if err = json.Unmarshal(target, &release); err != nil {
		return nil, nil, err
	}
	releaseManaged, err := managedServices(release)
	if err != nil {
		return nil, nil, err
	}
	if len(releaseManaged) != len(release.Services) {
		return nil, nil, errors.New("every release service must declare its update image family")
	}
	target, err = retainLocalServices(old, target, managed)
	if err != nil {
		return nil, nil, err
	}
	dir, err := os.MkdirTemp(b.Config.Directory, ".compose-plan-")
	if err != nil {
		return nil, nil, err
	}
	defer os.RemoveAll(dir)
	file := filepath.Join(dir, "target.json")
	if err = os.WriteFile(file, target, 0600); err != nil {
		return nil, nil, err
	}
	files := []string{file}
	if _, e := os.Stat(filepath.Join(b.Config.Directory, "stack-overrides.yaml")); e == nil {
		files = append(files, filepath.Join(b.Config.Directory, "stack-overrides.yaml"))
	}
	target, err = b.renderFiles(ctx, files, false)
	if err != nil {
		return nil, nil, err
	}
	var document map[string]any
	if err = json.Unmarshal(target, &document); err != nil {
		return nil, nil, err
	}
	var c composeConfig
	if err = json.Unmarshal(target, &c); err != nil {
		return nil, nil, err
	}
	nextManaged, err := managedServices(c)
	if err != nil {
		return nil, nil, err
	}
	for name, contract := range releaseManaged {
		if nextManaged[name].Image != contract.Image {
			return nil, nil, fmt.Errorf("override changes release image ownership for %s", name)
		}
	}
	if len(nextManaged) != len(releaseManaged) {
		return nil, nil, errors.New("local overrides cannot add undeclared managed services")
	}
	if err := validateOverrides(d, overrides, nextManaged); err != nil {
		return nil, nil, err
	}
	images := map[string]string{}
	for name, contract := range nextManaged {
		deployment := d
		if override, ok := overrides[name]; ok {
			deployment = override
		}
		img, ok := deployment.Images[contract.Image]
		platform, supported := img.Platforms[b.Config.Platform]
		if !ok || !supported {
			return nil, nil, fmt.Errorf("release has no %s image for %s", b.Config.Platform, name)
		}
		images[name] = img.Repository + "@" + platform.Manifest
		sc := object(object(document["services"])[name])
		sc["image"] = images[name]
		labels := object(sc["labels"])
		labels[updateLabel+"owner"] = b.Config.Project
		sc["labels"] = labels
	}
	target, err = json.Marshal(document)
	if err != nil {
		return nil, nil, err
	}
	if err = os.WriteFile(file, target, 0600); err != nil {
		return nil, nil, err
	}
	resolved, err := b.renderFiles(ctx, []string{file}, true)
	if err != nil {
		return nil, nil, err
	}
	if err = json.Unmarshal(resolved, &c); err != nil {
		return nil, nil, err
	}
	for name, sc := range current.Services {
		if _, owned := managed[name]; owned {
			continue
		}
		if next, exists := c.Services[name]; !exists || next.Image != sc.Image || !sameServiceDefinition(sc, next) {
			return nil, nil, fmt.Errorf("update overrides change local service %s; manage it separately", name)
		}
	}
	p := &StackPlan{Compose: target, Bundle: bundle, Selection: selection, Retired: map[string]string{}, Created: map[string]string{}}
	oldContainers := map[string]containerInfo{}
	for name := range managed {
		ci, e := b.inspect(ctx, current.Services[name].ContainerName)
		if e != nil {
			return nil, nil, e
		}
		if ci.Config.Labels["com.docker.compose.project"] != b.Config.Project || ci.Config.Labels["com.docker.compose.service"] != name {
			return nil, nil, errors.New("current container ownership mismatch")
		}
		if e = validateManagedMounts(name, ci); e != nil {
			return nil, nil, e
		}
		if (name == "gui" || name == "mowgli") && ci.Config.Labels["garden.mowgli.maintenance-api"] != "1" {
			return nil, nil, errors.New("installed GUI/ROS2 lacks maintenance support; upgrade through the installer first")
		}
		oldContainers[name] = ci
		if _, keep := nextManaged[name]; !keep || c.Services[name].ContainerName != current.Services[name].ContainerName {
			p.Retired[current.Services[name].ContainerName] = ci.ID
		}
	}
	if err = validateStackMounts(current, c, managed, nextManaged); err != nil {
		return nil, nil, err
	}
	for name := range nextManaged {
		sc := c.Services[name]
		if _, core := Services[name]; core && current.Services[name].ContainerName != "" && sc.ContainerName != current.Services[name].ContainerName {
			return nil, nil, fmt.Errorf("renaming the standard %s container requires an explicit layout migration", name)
		}
		if _, existed := managed[name]; !existed || sc.ContainerName != current.Services[name].ContainerName {
			ids, e := command(ctx, "docker", "ps", "-aq", "--filter", "name=^/"+sc.ContainerName+"$")
			if e != nil {
				return nil, nil, e
			}
			if strings.TrimSpace(string(ids)) != "" {
				return nil, nil, fmt.Errorf("container name %s is already occupied; retire/rename in separate releases", sc.ContainerName)
			}
			p.Created[sc.ContainerName] = name
			p.Changes = append(p.Changes, ServiceChange{name, "add"})
		} else {
			action := "update"
			family := nextManaged[name].Image
			deployment := d
			if override, ok := overrides[name]; ok {
				deployment = override
			}
			if oldContainers[name].Image == deployment.Images[family].Platforms[b.Config.Platform].Config && sameServiceDefinition(current.Services[name], sc) {
				action = "keep"
			}
			p.Changes = append(p.Changes, ServiceChange{name, action})
		}
	}
	for name := range managed {
		if _, ok := nextManaged[name]; !ok {
			p.Changes = append(p.Changes, ServiceChange{name, "remove"})
		}
	}
	for name := range current.Services {
		if _, ok := managed[name]; !ok {
			p.Changes = append(p.Changes, ServiceChange{name, "unmanaged"})
		}
	}
	sort.Slice(p.Changes, func(i, j int) bool { return p.Changes[i].Service < p.Changes[j].Service })
	return images, p, nil
}

func sameServiceDefinition(a, b serviceConfig) bool {
	if len(a.Raw) == 0 || len(b.Raw) == 0 {
		return false
	}
	var old, next map[string]any
	if json.Unmarshal(a.Raw, &old) != nil || json.Unmarshal(b.Raw, &next) != nil {
		return false
	}
	for _, doc := range []map[string]any{old, next} {
		delete(doc, "image")
		delete(object(doc["labels"]), updateLabel+"owner")
	}
	return reflect.DeepEqual(old, next)
}

// Layout 1 can reuse its existing persistent mounts, but cannot introduce new
// writable storage without a reviewed backup/data migration contract.
func validateStackMounts(current, target composeConfig, old, next map[string]managedService) error {
	known := map[string]bool{}
	for name := range old {
		for _, v := range current.Services[name].Volumes {
			known[v.Type+":"+v.Source+":"+v.Target] = true
		}
	}
	names := map[string]bool{}
	for name := range next {
		s := target.Services[name]
		if !idPattern.MatchString(s.ContainerName) || names[s.ContainerName] {
			return errors.New("invalid or duplicate target container name")
		}
		names[s.ContainerName] = true
		for _, v := range s.Volumes {
			if !v.ReadOnly && !known[v.Type+":"+v.Source+":"+v.Target] {
				return fmt.Errorf("%s adds writable storage at %s; explicit data/layout migration required", name, v.Target)
			}
			if _, core := Services[name]; !core && !v.ReadOnly && v.Target != "/db" && v.Target != "/mowgli_config" && v.Target != "/ros2_ws/maps" && v.Target != "/ros2_ws/config" {
				return fmt.Errorf("%s requires a backup contract for writable mount %s", name, v.Target)
			}
		}
		if name == "gui" || name == "mowgli" {
			if s.Environment["MOWGLI_UPDATE_MAINTENANCE"] != "/var/lib/mowgli-updater/maintenance" {
				return errors.New("release weakens the maintenance gate")
			}
			gate := false
			gateSource := ""
			for _, v := range current.Services[name].Volumes {
				if v.Target == "/var/lib/mowgli-updater" && v.ReadOnly {
					gateSource = v.Source
				}
			}
			for _, v := range s.Volumes {
				if v.Target == "/var/lib/mowgli-updater" && gateSource != "" && v.Source == gateSource && v.ReadOnly {
					gate = true
				}
			}
			if !gate {
				return errors.New("release removes the shared maintenance mount")
			}
		}
	}
	return nil
}

func (b DockerBackend) BackupStack(ctx context.Context, id string, p *StackPlan) (string, error) {
	path, err := b.Backup(ctx, id)
	if err != nil {
		return "", err
	}
	data, err := os.ReadFile(filepath.Join(path, "backup.json"))
	if err != nil {
		return "", err
	}
	var r backupRecord
	if err = json.Unmarshal(data, &r); err != nil {
		return "", err
	}
	r.Created = p.Created
	if err = AtomicJSON(filepath.Join(path, "backup.json"), r); err != nil {
		return "", err
	}
	return path, nil
}

func (b DockerBackend) ApplyStack(ctx context.Context, p Plan) error {
	if p.Stack == nil {
		return errors.New("missing stack plan")
	}
	if _, err := b.stopManaged(ctx); err != nil {
		return err
	}
	for name, id := range p.Stack.Retired {
		ci, err := b.inspect(ctx, name)
		if err != nil {
			return err
		}
		if ci.ID != id || ci.State.Running || ci.Config.Labels["com.docker.compose.project"] != b.Config.Project {
			return errors.New("retired container ownership changed")
		}
		if _, err = command(ctx, "docker", "rm", id); err != nil {
			return err
		}
	}
	// Clear image-only overrides first. The single atomic Compose replacement
	// then changes membership and image references together across power loss.
	if err := removeFile(b.override()); err != nil {
		return err
	}
	if err := AtomicWrite(filepath.Join(b.Config.Directory, "docker-compose.yaml"), p.Stack.Compose, 0600); err != nil {
		return err
	}
	if err := b.writeStackMetadata(p.Stack.Bundle, p.Stack.Selection, &p.Target, p.Stack.Compose); err != nil {
		return err
	}
	return b.Apply(ctx, p.Images)
}

func removeFile(path string) error {
	if err := os.Remove(path); err != nil && !os.IsNotExist(err) {
		return err
	}
	return nil
}

func (b DockerBackend) writeStackMetadata(bundle ComposeBundle, selection StackSelection, release *Deployment, compose []byte) error {
	for name, value := range map[string]any{"stack-bundle.json": bundle, "stack-selection.json": selection, "stack-applied-selection.json": selection, "stack-release.json": release} {
		if err := AtomicJSON(filepath.Join(b.Config.Directory, name), value); err != nil {
			return err
		}
	}
	return AtomicWrite(filepath.Join(b.Config.Directory, "stack-definition.sha256"), []byte(updates.Hash(compose)), 0600)
}
