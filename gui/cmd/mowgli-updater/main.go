package main

import (
	"context"
	"encoding/json"
	"flag"
	"fmt"
	"github.com/mowglinext/mowglinext/pkg/updater"
	"io"
	"net/http"
	"os"
	"path/filepath"
	"strings"
	"time"
)

func main() {
	if len(os.Args) == 3 && os.Args[1] == "installer-health" {
		data, err := os.ReadFile(os.Args[2])
		var config updater.HostConfig
		if err == nil {
			err = json.Unmarshal(data, &config)
		}
		if err == nil {
			ctx, cancel := context.WithTimeout(context.Background(), 30*time.Second)
			defer cancel()
			err = updater.WaitInstallerWorker(ctx, config)
		}
		if err != nil {
			fmt.Fprintln(os.Stderr, err)
			os.Exit(1)
		}
		return
	}
	if len(os.Args) == 4 && os.Args[1] == "installer-select" {
		data, err := os.ReadFile(os.Args[2])
		var config updater.HostConfig
		if err == nil {
			err = json.Unmarshal(data, &config)
		}
		if err == nil {
			var executable string
			executable, err = os.Executable()
			if err == nil {
				var candidate []byte
				candidate, err = os.ReadFile(executable)
				if err == nil {
					err = updater.SelectInstallerWorker(config, os.Args[3], candidate)
				}
			}
		}
		if err != nil {
			fmt.Fprintln(os.Stderr, err)
			os.Exit(1)
		}
		return
	}
	if len(os.Args) == 7 && os.Args[1] == "installer-stack" {
		b := updater.DockerBackend{Config: updater.HostConfig{Directory: os.Args[2], Project: os.Args[3]}}
		err := b.InstallStack(context.Background(), os.Args[4], map[string]string{"gnss": os.Args[5], "lidar": os.Args[6]})
		if err != nil {
			fmt.Fprintln(os.Stderr, err)
			os.Exit(1)
		}
		return
	}
	if len(os.Args) == 8 && os.Args[1] == "installer-config" {
		source := updater.Source{Repository: os.Args[4], Track: "custom", Branch: os.Args[7]}
		if os.Args[6] == "main" {
			source.Track = "stable"
			source.Branch = "main"
		}
		if os.Args[6] == "dev" {
			source.Track = "dev"
			source.Branch = "dev"
		}
		_ = json.NewEncoder(os.Stdout).Encode(updater.HostConfig{Directory: os.Args[2], Project: os.Args[3], StateDir: "/var/lib/mowgli-updater", Trusted: []string{os.Args[4]}, Platform: os.Args[5], InitialSource: source})
		return
	}
	if len(os.Args) == 6 && os.Args[1] == "installer-check" {
		data, err := os.ReadFile(os.Args[2])
		var config updater.HostConfig
		if err == nil {
			err = json.Unmarshal(data, &config)
		}
		if err != nil || config.Directory != os.Args[3] || config.Project != os.Args[4] || config.Platform != os.Args[5] {
			fmt.Fprintln(os.Stderr, "Existing updater configuration belongs to a different installation; migrate it explicitly")
			os.Exit(1)
		}
		if err = updater.CheckInstallerState(config); err != nil {
			fmt.Fprintln(os.Stderr, err)
			os.Exit(1)
		}
		return
	}
	if len(os.Args) > 1 && os.Args[1] == "version" {
		_ = json.NewEncoder(os.Stdout).Encode(map[string]any{"version": updater.Version, "revision": updater.Revision, "api": updater.APIVersion, "state_schema": updater.StateSchema})
		return
	}
	mode := "supervise"
	if len(os.Args) > 1 {
		mode = os.Args[1]
	}
	flags := flag.NewFlagSet("mowgli-updater", flag.ExitOnError)
	path := flags.String("config", "/etc/mowgli-updater.json", "installer-owned host configuration")
	if len(os.Args) > 2 {
		_ = flags.Parse(os.Args[2:])
	}
	data, err := os.ReadFile(*path)
	var c updater.HostConfig
	if err == nil {
		err = json.Unmarshal(data, &c)
	}
	if err == nil {
		switch mode {
		case "status", "check", "recover", "rollback":
			op := mode
			method := "POST"
			if mode == "status" {
				op = "state"
				method = "GET"
			}
			request, _ := http.NewRequest(method, "http://updater/v1/"+op, strings.NewReader("{}"))
			response, e := updater.Client(filepath.Join(c.StateDir, "run", "updater.sock")).Do(request)
			err = e
			if e == nil {
				_, err = io.Copy(os.Stdout, response.Body)
				response.Body.Close()
				if response.StatusCode >= 400 {
					err = fmt.Errorf("updater returned HTTP %d", response.StatusCode)
				}
			}
		case "serve":
			err = updater.Serve(c)
		case "supervise":
			err = updater.Supervise(*path, c)
		default:
			err = fmt.Errorf("usage: mowgli-updater version | supervise | serve --config PATH")
		}
	}
	if err != nil {
		fmt.Fprintln(os.Stderr, err)
		os.Exit(1)
	}
}
