package providers

import (
	"encoding/json"
	"fmt"
	"io"

	"github.com/mowglinext/mowglinext/pkg/types"
)

// Stage keys of the three flash paths. They are the i18n keys the GUI renders
// (`flashBoard.stages.<key>`), so renaming one here means renaming it there.
const (
	flashStageManifest  = "manifest"
	flashStageDownload  = "download"
	flashStageClone     = "clone"
	flashStageConfigure = "configure"
	flashStageBuild     = "build"
	flashStageUnzip     = "unzip"
	flashStageFlash     = "flash"
	flashStageVerify    = "verify"
)

var (
	prebuiltFlashStages = []string{flashStageManifest, flashStageDownload, flashStageFlash, flashStageVerify}
	customFlashStages   = []string{flashStageClone, flashStageConfigure, flashStageBuild, flashStageFlash}
	vermutFlashStages   = []string{flashStageDownload, flashStageUnzip, flashStageFlash}
)

// flashProgress announces stage transitions on the flash log stream. Each
// enter() writes one machine-readable marker line (types.FlashStageMarker +
// JSON) that the SSE handler lifts out of the log as a `stage` event; the
// human-readable "------> ..." lines the flash paths already write stay as they
// are and remain the detailed log.
type flashProgress struct {
	w      io.Writer
	stages []string
}

func newFlashProgress(w io.Writer, stages []string) *flashProgress {
	return &flashProgress{w: w, stages: stages}
}

// enter marks the stage with the given key as the one now running. An unknown
// key is a programming error and is reported on the log rather than dropped,
// so a stale plan is visible in the field instead of silently freezing the bar.
func (p *flashProgress) enter(key string) {
	for i, s := range p.stages {
		if s != key {
			continue
		}
		event := types.FlashStageEvent{Stages: p.stages, Current: i}
		payload, err := json.Marshal(event)
		if err != nil {
			_, _ = fmt.Fprintf(p.w, "------> WARNING: could not encode flash stage %q: %v\n", key, err)
			return
		}
		_, _ = fmt.Fprintf(p.w, "%s%s\n", types.FlashStageMarker, payload)
		return
	}
	_, _ = fmt.Fprintf(p.w, "------> WARNING: flash stage %q is not in the plan %v\n", key, p.stages)
}
