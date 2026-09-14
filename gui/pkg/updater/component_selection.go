package updater

import (
	"encoding/json"
	"fmt"
	"sort"
)

// A contract identifies a maintainer-reviewed drop-in interface, including
// configuration, persisted data and ROS topics/services. Matching a build date
// or firmware version alone never makes component mixing compatible.
func compatibleComponent(base, candidate Deployment, family string) bool {
	contract := base.ComponentCompatibility[family]
	other := candidate.ComponentCompatibility[family]
	if family == "mowglinext-gui" && contract == "" && other == "" {
		return compatibleGUI(base, candidate)
	}
	return contract != "" && contract == other && base.Source == candidate.Source &&
		base.Layout == candidate.Layout && base.DataSchema == candidate.DataSchema &&
		base.UpdaterAPI == candidate.UpdaterAPI && base.MaintenanceAPI == candidate.MaintenanceAPI &&
		base.FirmwareProtocol == candidate.FirmwareProtocol
}

func validateOverrides(base Deployment, overrides map[string]Deployment, services map[string]managedService) error {
	for name, candidate := range overrides {
		service, exists := services[name]
		if !exists {
			return fmt.Errorf("%s is not a managed service in the selected stack", name)
		}
		if candidate.Source != base.Source {
			return fmt.Errorf("%s: component source differs from selected release", name)
		}
		if candidate.ID != base.ID && !compatibleComponent(base, candidate, service.Image) {
			return fmt.Errorf("%s: no matching published compatibility contract for %s", name, service.Image)
		}
		if _, exists := candidate.Images[service.Image]; !exists {
			return fmt.Errorf("%s: selected release does not publish %s", name, service.Image)
		}
	}
	return nil
}

// UI projection of the release bundle. The installer selection chooses the
// variant; the verified bundle remains authoritative when making a plan.
type ServiceChoice struct {
	Service string            `json:"service"`
	Image   string            `json:"image"`
	When    map[string]string `json:"when,omitempty"`
}

func (b ComposeBundle) ServiceChoices() ([]ServiceChoice, error) {
	choices := []ServiceChoice{}
	add := func(names []string, when map[string]string) error {
		for _, name := range names {
			var doc map[string]any
			if err := json.Unmarshal(b.Fragments[name], &doc); err != nil {
				return err
			}
			for service, value := range object(doc["services"]) {
				family, _ := object(object(value)["labels"])[updateLabel+"image"].(string)
				if family != "" {
					choices = append(choices, ServiceChoice{Service: service, Image: family, When: when})
				}
			}
		}
		return nil
	}
	if err := add(b.Required, nil); err != nil {
		return nil, err
	}
	for group, values := range b.Options {
		for choice, names := range values {
			if err := add(names, map[string]string{group: choice}); err != nil {
				return nil, err
			}
		}
	}
	sort.Slice(choices, func(i, j int) bool {
		a, _ := json.Marshal(choices[i])
		z, _ := json.Marshal(choices[j])
		return string(a) < string(z)
	})
	return choices, nil
}
