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
