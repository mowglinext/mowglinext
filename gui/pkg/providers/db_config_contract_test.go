package providers

import (
	"errors"
	"fmt"
	"os"
	"testing"

	"git.mills.io/prologic/bitcask"
	"github.com/mowglinext/mowglinext/pkg/types"
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

func TestDBProvider_ConfigMissingErrorPreservesClassification(t *testing.T) {
	t.Setenv("DB_PATH", t.TempDir())
	db := NewDBProvider()
	t.Cleanup(func() { closeDB(t, db) })
	const key = "config-contract.absent"
	_, underlying := db.db.Get([]byte(key))
	require.True(t, errors.Is(underlying, bitcask.ErrKeyNotFound))
	_, exposed := db.Get(key)
	require.Error(t, exposed)
	// A missing key must remain classifiable across the provider boundary.
	assert.ErrorIs(t, exposed, types.ErrKeyNotFound)
	assert.ErrorIs(t, fmt.Errorf("outer: %w", exposed), types.ErrKeyNotFound)
	assert.EqualError(t, exposed, "config key "+key+" not found")
}

func configStoredString(value string) *string { return &value }

func TestDBProvider_ClosedStorageDoesNotUseFallbacks(t *testing.T) {
	t.Setenv("DB_PATH", t.TempDir())
	t.Setenv("ROS_NODE_NAME", "environment-node")
	db := NewDBProvider()
	require.NoError(t, db.Set("present", []byte("true")))
	require.NoError(t, db.Set("system.ros.nodeName", []byte("stored-node")))
	closeDB(t, db)
	for _, key := range []string{"present", "system.ros.nodeName"} {
		_, native := db.db.Get([]byte(key))
		require.Error(t, native)
		require.NotErrorIs(t, native, bitcask.ErrKeyNotFound)
		value, err := db.Get(key)
		assert.Nil(t, value)
		assert.ErrorIs(t, err, os.ErrClosed)
		assert.NotErrorIs(t, err, types.ErrKeyNotFound)
	}
}
