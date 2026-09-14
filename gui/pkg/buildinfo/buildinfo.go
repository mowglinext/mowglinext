// Package buildinfo identifies the GUI binary independently of Docker tags.
package buildinfo

import "runtime/debug"

// Populated by the image build. Empty values mean an unidentified local build.
var Revision, Version, BuiltAt string

type Info struct {
	Revision string `json:"revision,omitempty"`
	Version  string `json:"version,omitempty"`
	BuiltAt  string `json:"built_at,omitempty"`
	Modified bool   `json:"modified"`
}

func Current() Info {
	i := Info{Revision: Revision, Version: Version, BuiltAt: BuiltAt}
	if b, ok := debug.ReadBuildInfo(); ok {
		for _, s := range b.Settings {
			switch s.Key {
			case "vcs.revision":
				if i.Revision == "" {
					i.Revision = s.Value
				}
			case "vcs.modified":
				i.Modified = s.Value == "true"
			}
		}
	}
	return i
}
