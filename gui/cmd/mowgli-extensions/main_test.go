package main

import (
	"bytes"
	"context"
	"encoding/json"
	"os"
	"path/filepath"
	"runtime"
	"strings"
	"testing"

	"github.com/mowglinext/mowglinext/pkg/extensions"
)

func TestCLIHelpAndEmptyStatusWithoutRuntimeAccess(t *testing.T) {
	var out bytes.Buffer
	if err := run(context.Background(), []string{"--help"}, &out); err != nil {
		t.Fatal(err)
	}
	root := t.TempDir()
	host := extensions.HostConfig{Schema: 1, Installation: "test", Platform: runtime.GOOS + "/" + runtime.GOARCH,
		Contracts: []string{}, TrustedRepositories: []string{}, ApprovedGrants: []string{}, ExternalUnits: []string{}}
	data, _ := json.Marshal(host)
	config := filepath.Join(root, "host.json")
	_ = os.WriteFile(config, data, 0600)
	out.Reset()
	if err := run(context.Background(), []string{"--config", config, "--state-dir", filepath.Join(root, "state"), "status"}, &out); err != nil {
		t.Fatal(err)
	}
	if strings.TrimSpace(out.String()) != "[]" {
		t.Fatal("unexpected status", out.String())
	}
}

func TestCLIObserveAdoptAndForgetNeverInvokesSystemdMutations(t *testing.T) {
	root := t.TempDir()
	log := filepath.Join(root, "calls")
	bin := filepath.Join(root, "bin")
	_ = os.Mkdir(bin, 0700)
	// A real CLI subprocess boundary, using a private fake systemctl executable.
	script := "#!/bin/sh\nprintf '%s\\n' \"$*\" >> \"$EXTENSION_TEST_LOG\"\nprintf 'ActiveState=inactive\\nSubState=dead\\nInvocationID=\\n'\n"
	_ = os.WriteFile(filepath.Join(bin, "systemctl"), []byte(script), 0700)
	t.Setenv("PATH", bin)
	t.Setenv("EXTENSION_TEST_LOG", log)
	host := extensions.HostConfig{Schema: 1, Installation: "test", Platform: runtime.GOOS + "/" + runtime.GOARCH,
		Contracts: []string{}, TrustedRepositories: []string{"liltaket/mowgli-lora-link"}, ApprovedGrants: []string{},
		ExternalUnits: []string{"mowgli-lora-robot.service"}}
	data, _ := json.Marshal(host)
	config := filepath.Join(root, "host.json")
	_ = os.WriteFile(config, data, 0600)
	bundle := extensions.Bundle{Manifest: extensions.Manifest{Schema: 1, ID: "mowgli-lora-link", Version: "0.2.0",
		Source:   extensions.Source{Repository: "liltaket/mowgli-lora-link", Revision: strings.Repeat("a", 40)},
		Runtime:  extensions.Runtime{Kind: "external-systemd", Units: []string{"mowgli-lora-robot.service"}},
		Requires: extensions.Requirements{ManagerAPI: 1, Platforms: []string{host.Platform}, Contracts: []string{}},
		Grants:   []string{}, DataSchema: 1, Health: "application", Profile: "external"}}
	bundlePath := filepath.Join(root, "bundle.json")
	data, _ = json.Marshal(bundle)
	_ = os.WriteFile(bundlePath, data, 0600)
	global := []string{"--config", config, "--state-dir", filepath.Join(root, "state")}
	for _, action := range [][]string{
		{"adopt", "--bundle", bundlePath, "--project", "test-ext-lora", "--config-dir", filepath.Join(root, "config"), "--data-dir", filepath.Join(root, "data"), "mowgli-lora-link"},
		{"status"}, {"forget", "mowgli-lora-link"},
	} {
		var output bytes.Buffer
		if err := run(context.Background(), append(append([]string{}, global...), action...), &output); err != nil {
			t.Fatal(action, err)
		}
	}
	calls, err := os.ReadFile(log)
	if err != nil {
		t.Fatal(err)
	}
	for _, call := range strings.Split(strings.TrimSpace(string(calls)), "\n") {
		if !strings.HasPrefix(call, "show --no-pager ") {
			t.Fatal("mutating service command", call)
		}
	}
}
