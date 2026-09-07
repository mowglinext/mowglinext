package updater

import (
	"errors"
	"fmt"
	"regexp"
	"sort"
	"strings"
)

var imageNamePattern = regexp.MustCompile(`^[a-z0-9]+(?:[._-][a-z0-9]+)*$`)

const updateLabel = "garden.mowgli.update."

type managedService struct {
	Image  string
	After  []string
	Health string
}

// The installed Compose file owns membership, dependency order and data mounts.
// Release descriptors can supply images, never new containers or host commands.
func managedServices(c composeConfig) (map[string]managedService, error) {
	result := map[string]managedService{}
	for name, sc := range c.Services {
		image, legacy := Services[name]
		declared := sc.Labels[updateLabel+"image"]
		if !legacy && declared == "" {
			continue
		}
		if !idPattern.MatchString(name) || sc.ContainerName == "" {
			return nil, fmt.Errorf("invalid managed service %s", name)
		}
		if declared != "" {
			image = declared
		}
		if name == "lidar" && image == "" {
			for _, candidate := range []string{"lidar-ldlidar", "lidar-rplidar", "lidar-stl27l"} {
				if strings.Contains(sc.Image, "/"+candidate+":") || strings.Contains(sc.Image, "/"+candidate+"@") {
					image = candidate
				}
			}
		}
		if !imageNamePattern.MatchString(image) {
			return nil, fmt.Errorf("unknown image family for %s; declare %simage", name, updateLabel)
		}
		if (name == "gui" && image != "mowglinext-gui") || (name == "mowgli" && image != "mowgli-ros2") {
			return nil, errors.New("core image roles cannot be reassigned")
		}
		s := managedService{Image: image, Health: "container"}
		if name == "gps" || name == "lidar" {
			s.Health = name
		}
		if health := sc.Labels[updateLabel+"health"]; health != "" {
			if health != "container" && health != "gps" && health != "lidar" {
				return nil, fmt.Errorf("unsupported health contract for %s", name)
			}
			if (name == "gps" || name == "lidar") && health != name {
				return nil, errors.New("sensor freshness gates cannot be disabled")
			}
			s.Health = health
		}
		if after := sc.Labels[updateLabel+"after"]; after != "" {
			for _, dependency := range strings.Split(after, ",") {
				s.After = append(s.After, strings.TrimSpace(dependency))
			}
		}
		result[name] = s
	}
	if _, ok := result["mowgli"]; !ok {
		return nil, errors.New("missing managed ROS2 service")
	}
	if _, ok := result["gui"]; !ok {
		return nil, errors.New("missing managed GUI service")
	}
	// Optional sensors are ordered only when installed. Core ordering cannot be
	// weakened by a Compose label; extra dependencies may be added explicitly.
	for _, name := range []string{"gps", "lidar"} {
		if _, ok := result[name]; ok {
			s := result["mowgli"]
			s.After = append(s.After, name)
			result["mowgli"] = s
		}
	}
	s := result["gui"]
	s.After = append(s.After, "mowgli")
	result["gui"] = s
	_, err := serviceOrder(result)
	return result, err
}

func serviceOrder(services map[string]managedService) ([]string, error) {
	keys := make([]string, 0, len(services))
	for name := range services {
		keys = append(keys, name)
	}
	sort.Strings(keys)
	state := map[string]int{}
	order := []string{}
	var visit func(string) error
	visit = func(name string) error {
		s, exists := services[name]
		if !exists {
			return fmt.Errorf("dependency %s is not an installed managed service", name)
		}
		if state[name] == 1 {
			return errors.New("managed service dependency cycle")
		}
		if state[name] == 2 {
			return nil
		}
		state[name] = 1
		for _, dep := range s.After {
			if err := visit(dep); err != nil {
				return err
			}
		}
		state[name] = 2
		order = append(order, name)
		return nil
	}
	for _, name := range keys {
		if err := visit(name); err != nil {
			return nil, err
		}
	}
	return order, nil
}

func compatibleGUI(base, gui Deployment) bool {
	return base.GUICompatibility != "" && base.GUICompatibility == gui.GUICompatibility &&
		base.Source.Repository == gui.Source.Repository && base.Layout == gui.Layout &&
		base.DataSchema == gui.DataSchema && base.UpdaterAPI == gui.UpdaterAPI &&
		base.MaintenanceAPI == gui.MaintenanceAPI && base.FirmwareProtocol == gui.FirmwareProtocol
}
