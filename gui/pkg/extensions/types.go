// Package extensions manages optional services independently from the robot stack.
// It has no ROS, firmware, mower maintenance or actuation dependencies.
package extensions

import (
	"context"
	"encoding/json"
)

const Schema = 1

type Source struct {
	Repository string `json:"repository"`
	Revision   string `json:"revision"`
}

type Runtime struct {
	Kind  string   `json:"kind"`
	Units []string `json:"units,omitempty"`
}

type Requirements struct {
	ManagerAPI int      `json:"manager_api"`
	Platforms  []string `json:"platforms"`
	Contracts  []string `json:"contracts"`
}

type Manifest struct {
	Schema     int          `json:"schema"`
	ID         string       `json:"id"`
	Version    string       `json:"version"`
	Source     Source       `json:"source"`
	Runtime    Runtime      `json:"runtime"`
	Requires   Requirements `json:"requires"`
	Grants     []string     `json:"grants"`
	DataSchema int          `json:"data_schema"`
	Health     string       `json:"health"`
	Profile    string       `json:"profile"`
}

// Bundle is local input, not a remote script or a mutable registry tag. Compose
// must be normalized JSON with immutable images and explicit resource identities.
type Bundle struct {
	Manifest Manifest        `json:"manifest"`
	Compose  json.RawMessage `json:"compose,omitempty"`
}

type HostConfig struct {
	Schema              int      `json:"schema"`
	Installation        string   `json:"installation"`
	Platform            string   `json:"platform"`
	Contracts           []string `json:"contracts"`
	TrustedRepositories []string `json:"trusted_repositories"`
	ApprovedGrants      []string `json:"approved_grants"`
	ExternalUnits       []string `json:"external_units"`
}

type Binding struct {
	Project   string `json:"project"`
	ConfigDir string `json:"config_dir"`
	DataDir   string `json:"data_dir"`
}

type Release struct {
	Bundle  Bundle  `json:"bundle"`
	Binding Binding `json:"binding"`
	Digest  string  `json:"digest"`
}

type Entry struct {
	Active           Release  `json:"active"`
	Previous         *Release `json:"previous,omitempty"`
	Enabled          bool     `json:"enabled"`
	RecoveryRequired bool     `json:"recovery_required"`
	Error            string   `json:"error,omitempty"`
}

// Retained entries keep data ownership/schema after unregistering a service.
type Retained struct {
	Manifest        Manifest `json:"manifest"`
	Binding         Binding  `json:"binding"`
	StorageIdentity string   `json:"storage_identity,omitempty"`
	VolumeNames     []string `json:"volume_names,omitempty"`
}

type Journal struct {
	Action        string `json:"action"`
	ID            string `json:"id"`
	Phase         string `json:"phase"`
	Before        *Entry `json:"before,omitempty"`
	Target        *Entry `json:"target,omitempty"`
	Error         string `json:"error,omitempty"`
	RecoveryError string `json:"recovery_error,omitempty"`
}

type State struct {
	Schema   int                 `json:"schema"`
	Entries  map[string]Entry    `json:"entries"`
	Retained map[string]Retained `json:"retained"`
	Journal  *Journal            `json:"journal,omitempty"`
}

type Observation struct {
	Process     string            `json:"process"`
	Application string            `json:"application"`
	Data        string            `json:"data"`
	Identity    string            `json:"identity,omitempty"`
	Services    map[string]string `json:"services,omitempty"`
}

// Backend methods must never print runtime configuration or secret-bearing
// command output. Prepare may pull, Check/Observe must remain read-only.
type Backend interface {
	Check(context.Context, Release, bool) error // bool permits existing project adoption
	Prepare(context.Context, Release) error
	Start(context.Context, Release) error
	Stop(context.Context, Release) error
	Remove(context.Context, Release) error // containers/networks only, never volumes
	Observe(context.Context, Release) (Observation, error)
}

type Status struct {
	ID               string      `json:"id"`
	Version          string      `json:"version"`
	Managed          bool        `json:"managed"`
	Enabled          bool        `json:"enabled"`
	Compatible       bool        `json:"compatible"`
	Blockers         []string    `json:"blockers"`
	RecoveryRequired bool        `json:"recovery_required"`
	Error            string      `json:"error,omitempty"`
	Observation      Observation `json:"observation"`
	Manifest         Manifest    `json:"manifest"`
	Digest           string      `json:"digest"`
	PreviousVersion  string      `json:"previous_version,omitempty"`
	PreviousDigest   string      `json:"previous_digest,omitempty"`
}
