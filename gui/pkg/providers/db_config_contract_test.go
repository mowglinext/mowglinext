package providers

import (
	"errors"
	"testing"

	"git.mills.io/prologic/bitcask"
	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
)

func TestDBProvider_ConfigReadFallbackCompatibility(t *testing.T) {
	tests := []struct {
		name        string
		stored      *string
		environment string
		expected    string
	}{
		{name: "missing uses environment", environment: "environment-node", expected: "environment-node"},
		{name: "missing uses default", expected: "mowglinext"},
		{name: "stored overrides environment and default", stored: configStoredString("stored-node"), environment: "environment-node", expected: "stored-node"},
		{name: "stored empty does not become environment or default", stored: configStoredString(""), environment: "environment-node", expected: ""},
	}
	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			t.Setenv("DB_PATH", t.TempDir())
			t.Setenv("ROS_NODE_NAME", tt.environment)
			db := NewDBProvider()
			t.Cleanup(func() { closeDB(t, db) })
			if tt.stored != nil {
				require.NoError(t, db.Set("system.ros.nodeName", []byte(*tt.stored)))
				closeDB(t, db)
				db = NewDBProvider()
			}
			value, err := db.Get("system.ros.nodeName")
			require.NoError(t, err)
			assert.Equal(t, tt.expected, string(value))
		})
	}
}

func TestDBProvider_ConfigMissingErrorHasNoPublicClassification(t *testing.T) {
	t.Setenv("DB_PATH", t.TempDir())
	db := NewDBProvider()
	t.Cleanup(func() { closeDB(t, db) })
	const key = "config-contract.absent"
	_, underlying := db.db.Get([]byte(key))
	require.True(t, errors.Is(underlying, bitcask.ErrKeyNotFound))
	_, exposed := db.Get(key)
	require.Error(t, exposed)
	// Get preserves legacy error behavior but loses the missing-key sentinel;
	// callers cannot classify it safely by errors.Is through IDBProvider.
	assert.False(t, errors.Is(exposed, bitcask.ErrKeyNotFound))
}

func configStoredString(value string) *string { return &value }
