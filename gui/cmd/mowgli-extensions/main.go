// mowgli-extensions is a local administrator CLI. No HTTP routes or daemon.
package main

import (
	"context"
	"encoding/json"
	"errors"
	"flag"
	"fmt"
	"io"
	"os"
	"os/signal"
	"path/filepath"
	"runtime"
	"syscall"
	"time"

	"github.com/mowglinext/mowglinext/pkg/extensions"
)

func readBounded(path string) ([]byte, error) {
	f, err := os.Open(path)
	if err != nil {
		return nil, err
	}
	defer f.Close()
	data, err := io.ReadAll(io.LimitReader(f, 2*1024*1024+1))
	if len(data) > 2*1024*1024 {
		return nil, errors.New("input exceeds 2 MiB limit")
	}
	return data, err
}

func run(ctx context.Context, args []string, out io.Writer) error {
	global := flag.NewFlagSet("mowgli-extensions", flag.ContinueOnError)
	global.SetOutput(out)
	configPath := global.String("config", "/etc/mowgli-extensions/host.json", "local trust/compatibility configuration")
	stateDir := global.String("state-dir", "/var/lib/mowgli-extensions", "private persistent state directory")
	timeout := global.Duration("timeout", 2*time.Minute, "total operation deadline")
	if err := global.Parse(args); err != nil {
		if errors.Is(err, flag.ErrHelp) {
			return nil
		}
		return err
	}
	if len(global.Args()) == 0 {
		return errors.New("usage: mowgli-extensions [global flags] status|preflight|pack|install|adopt|enable|disable|upgrade|rollback|remove|forget|recover [action flags] [id]")
	}
	if *timeout <= 0 || *timeout > 30*time.Minute {
		return errors.New("timeout must be positive and at most 30m")
	}
	configData, err := readBounded(*configPath)
	if err != nil {
		return err
	}
	host, err := extensions.ParseHost(configData)
	if err != nil {
		return err
	}
	if host.Platform != runtime.GOOS+"/"+runtime.GOARCH {
		return errors.New("configured platform differs from this executable's runtime platform")
	}
	root, err := filepath.Abs(*stateDir)
	if err != nil {
		return err
	}
	manager := extensions.Manager{Store: extensions.Store{Root: root}, Host: host, Backend: extensions.NewBackend(root)}
	ctx, cancel := context.WithTimeout(ctx, *timeout)
	defer cancel()
	action := global.Args()[0]
	options := flag.NewFlagSet(action, flag.ContinueOnError)
	options.SetOutput(out)
	bundlePath := options.String("bundle", "", "local approved bundle JSON with pinned images")
	project := options.String("project", "", "separate Compose project (required explicitly for adoption)")
	configDir := options.String("config-dir", "", "extension-only config directory")
	dataDir := options.String("data-dir", "", "extension-only data directory")
	targetHost := options.String("target-host", "", "candidate compatibility configuration for preflight")
	manifestPath := options.String("manifest", "", "manifest JSON for pack")
	composePath := options.String("compose", "", "existing standalone Compose file for pack")
	envPath := options.String("env-file", "", "Compose dotenv file for pack")
	outputPath := options.String("output", "", "new private bundle file for pack")
	if err := options.Parse(global.Args()[1:]); err != nil {
		return err
	}
	write := func(value any) error {
		encoder := json.NewEncoder(out)
		encoder.SetIndent("", "  ")
		return encoder.Encode(value)
	}
	switch action {
	case "status":
		if options.NArg() != 0 {
			return errors.New("status takes no id")
		}
		status, err := manager.Status(ctx)
		if err != nil {
			return err
		}
		return write(status)
	case "recover":
		if options.NArg() != 0 {
			return errors.New("recover takes no id")
		}
		return manager.Recover(ctx)
	case "preflight":
		if options.NArg() != 0 || *targetHost == "" {
			return errors.New("preflight requires --target-host file and no id")
		}
		data, err := readBounded(*targetHost)
		if err != nil {
			return err
		}
		target, err := extensions.ParseHost(data)
		if err != nil {
			return err
		}
		conflicts, err := manager.Preflight(ctx, target)
		if err != nil {
			return err
		}
		if err = write(conflicts); err != nil {
			return err
		}
		if len(conflicts) != 0 {
			return errors.New("active or unresolved extensions conflict with the candidate host")
		}
		return nil
	}
	if options.NArg() != 1 {
		return errors.New("action requires exactly one extension id after its flags")
	}
	id := options.Arg(0)
	var bundle *extensions.Bundle
	if action == "install" || action == "adopt" || action == "upgrade" {
		if *bundlePath == "" {
			return errors.New("action requires --bundle")
		}
		data, err := readBounded(*bundlePath)
		if err != nil {
			return err
		}
		parsed, err := extensions.ParseBundle(data)
		if err != nil {
			return err
		}
		bundle = &parsed
	}
	if action == "adopt" && *project == "" {
		return errors.New("adoption requires an explicit existing --project")
	}
	if *project == "" {
		*project = host.Installation + "-ext-" + id
	}
	if *configDir == "" {
		*configDir = filepath.Join("/etc/mowgli-extensions", id)
	}
	if *dataDir == "" {
		*dataDir = filepath.Join(root, id, "data")
	}
	binding := extensions.Binding{Project: *project, ConfigDir: *configDir, DataDir: *dataDir}
	if action == "pack" {
		if *manifestPath == "" || *composePath == "" || *outputPath == "" {
			return errors.New("pack requires --manifest, --compose and --output")
		}
		data, err := readBounded(*manifestPath)
		if err != nil {
			return err
		}
		packed, err := extensions.Pack(ctx, extensions.ExecRunner{}, data, *composePath, *envPath, binding)
		if err != nil {
			return err
		}
		if packed.Manifest.ID != id {
			return errors.New("packed manifest id differs from requested id")
		}
		if reasons := extensions.Compatible(packed.Manifest, host); len(reasons) > 0 {
			return errors.New("packed manifest is incompatible with configured host")
		}
		if err := extensions.WriteBundle(*outputPath, packed); err != nil {
			return err
		}
		return write(map[string]string{"id": id, "bundle": *outputPath, "digest": extensions.BundleDigest(packed)})
	}
	if err := manager.Apply(ctx, action, id, bundle, binding); err != nil {
		return err
	}
	return write(map[string]string{"id": id, "action": action, "result": "complete"})
}

func main() {
	ctx, stop := signal.NotifyContext(context.Background(), os.Interrupt, syscall.SIGTERM)
	defer stop()
	if err := run(ctx, os.Args[1:], os.Stdout); err != nil {
		fmt.Fprintln(os.Stderr, "mowgli-extensions:", err)
		os.Exit(1)
	}
}
