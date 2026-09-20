package api

import (
	"os"
	"regexp"
	"testing"

	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
)

// fieldGroupKeyPattern matches the two ways settingsFieldGroups.ts declares a
// key: num("key", ...) and { key: "key", kind: ... }.
var fieldGroupKeyPattern = regexp.MustCompile(`(?:num\(|key:\s*)"([a-zA-Z0-9_]+)"`)

// TestSettingsFieldGroupKeysHaveSchemaDefaults pins that every key rendered by
// the declarative Settings cards (web/src/components/settings/
// settingsFieldGroups.ts) has a JSON-schema default. GET /settings/yaml merges
// schema defaults under the SPARSE installed yaml, so a key without one renders
// as an empty input with no reset-to-default and is never pruned on save.
// Lives here rather than in vitest because the Docker web build typechecks the
// frontend tests with only web/ in its build context.
func TestSettingsFieldGroupKeysHaveSchemaDefaults(t *testing.T) {
	chdirToGuiRoot(t)
	resetSchemaCache()
	t.Cleanup(resetSchemaCache)

	source, err := os.ReadFile("web/src/components/settings/settingsFieldGroups.ts")
	require.NoError(t, err)

	matches := fieldGroupKeyPattern.FindAllStringSubmatch(string(source), -1)
	require.NotEmpty(t, matches, "no keys parsed; did the declaration style change?")

	defaults := map[string]any{}
	extractDefaults(mustSchema(t), defaults)

	for _, match := range matches {
		_, found := defaults[match[1]]
		assert.True(t, found, "%s is rendered by a Settings field group but has no schema default", match[1])
	}
}
