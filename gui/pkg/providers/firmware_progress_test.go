package providers

import (
	"encoding/json"
	"strings"
	"testing"

	"github.com/mowglinext/mowglinext/pkg/types"
	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
)

// The marker line is what the SSE handler and the GUI progress bar key on, so
// its shape is pinned here: one line, the marker prefix, then the full plan and
// the 0-based index of the stage just entered.
func TestFlashProgressWritesOneMarkerLinePerStage(t *testing.T) {
	var out strings.Builder
	p := newFlashProgress(&out, prebuiltFlashStages)

	p.enter(flashStageManifest)
	p.enter(flashStageFlash)

	lines := strings.Split(strings.TrimRight(out.String(), "\n"), "\n")
	require.Len(t, lines, 2)
	for i, want := range []int{0, 2} {
		require.True(t, strings.HasPrefix(lines[i], types.FlashStageMarker), lines[i])
		var ev types.FlashStageEvent
		require.NoError(t, json.Unmarshal([]byte(strings.TrimPrefix(lines[i], types.FlashStageMarker)), &ev))
		assert.Equal(t, prebuiltFlashStages, ev.Stages)
		assert.Equal(t, want, ev.Current)
	}
}

// A stage the plan does not know must not emit a marker (the GUI would index
// past its step list) — it is reported on the log instead.
func TestFlashProgressReportsUnknownStageInsteadOfMarking(t *testing.T) {
	var out strings.Builder
	p := newFlashProgress(&out, vermutFlashStages)

	p.enter(flashStageVerify)

	assert.NotContains(t, out.String(), types.FlashStageMarker)
	assert.Contains(t, out.String(), "WARNING")
	assert.Contains(t, out.String(), flashStageVerify)
}

// Every stage each path enters must be in that path's plan, otherwise the bar
// would stall on the field. The plans are the contract with the GUI's i18n
// stage labels as well.
func TestFlashStagePlansCoverEveryKnownStage(t *testing.T) {
	all := map[string]bool{}
	for _, plan := range [][]string{prebuiltFlashStages, customFlashStages, vermutFlashStages} {
		for _, s := range plan {
			all[s] = true
		}
	}
	for _, s := range []string{
		flashStageManifest, flashStageDownload, flashStageClone, flashStageConfigure,
		flashStageBuild, flashStageUnzip, flashStageFlash, flashStageVerify,
	} {
		assert.True(t, all[s], "stage %q is in no plan", s)
	}
}
