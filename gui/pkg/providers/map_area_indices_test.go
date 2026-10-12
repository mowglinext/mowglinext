package providers

import (
	"github.com/mowglinext/mowglinext/pkg/msgs/mowgli"
	"github.com/stretchr/testify/assert"
	"testing"
)

func TestSplitMapAreasPreservesROSIndices(t *testing.T) {
	working, navigation, indices := splitMapAreas([]mowgli.MapArea{
		{Name: "Front"}, {Name: "Passage", IsNavigationArea: true}, {Name: "Back"},
	})
	assert.Equal(t, []uint32{0, 2}, indices)
	assert.Equal(t, "Front", working[0].Name)
	assert.Equal(t, "Back", working[1].Name)
	assert.Equal(t, "Passage", navigation[0].Name)
}

// mowglinext#637: the stable id must survive the split untouched, alongside
// (not instead of) the ROS array index — the frontend needs both.
func TestSplitMapAreasPreservesStableId(t *testing.T) {
	working, navigation, _ := splitMapAreas([]mowgli.MapArea{
		{Name: "Front", Id: 501}, {Name: "Passage", IsNavigationArea: true, Id: 502}, {Name: "Back", Id: 503},
	})
	assert.Equal(t, uint32(501), working[0].Id)
	assert.Equal(t, uint32(503), working[1].Id)
	assert.Equal(t, uint32(502), navigation[0].Id)
}
