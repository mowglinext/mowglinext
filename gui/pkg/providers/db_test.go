package providers

import (
	"os"
	"testing"

	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
)

func TestDBProvider_SetAndGet(t *testing.T) {
	dir := t.TempDir()
	os.Setenv("DB_PATH", dir)
	defer os.Unsetenv("DB_PATH")

	db := NewDBProvider()

	err := db.Set("test.key", []byte("hello"))
	require.NoError(t, err)

	val, err := db.Get("test.key")
	require.NoError(t, err)
	assert.Equal(t, "hello", string(val))
}

func TestDBProvider_GetNotFound(t *testing.T) {
	dir := t.TempDir()
	os.Setenv("DB_PATH", dir)
	defer os.Unsetenv("DB_PATH")

	db := NewDBProvider()

	_, err := db.Get("nonexistent.key")
	assert.Error(t, err)
	assert.Contains(t, err.Error(), "not found")
}

func TestDBProvider_GetWithEnvFallback(t *testing.T) {
	dir := t.TempDir()
	os.Setenv("DB_PATH", dir)
	defer os.Unsetenv("DB_PATH")

	db := NewDBProvider()

	// Test env fallback
	os.Setenv("ROS_MASTER_URI", "http://test:11311")
	defer os.Unsetenv("ROS_MASTER_URI")

	val, err := db.Get("system.ros.masterUri")
	require.NoError(t, err)
	assert.Equal(t, "http://test:11311", string(val))
}

func TestDBProvider_GetWithDefault(t *testing.T) {
	dir := t.TempDir()
	os.Setenv("DB_PATH", dir)
	defer os.Unsetenv("DB_PATH")

	db := NewDBProvider()

	// No env var set, should return default
	os.Unsetenv("ROS_MASTER_URI")

	val, err := db.Get("system.ros.masterUri")
	require.NoError(t, err)
	assert.Equal(t, "http://localhost:11311", string(val))
}

func TestDBProvider_SetOverridesEnvAndDefault(t *testing.T) {
	dir := t.TempDir()
	os.Setenv("DB_PATH", dir)
	defer os.Unsetenv("DB_PATH")

	db := NewDBProvider()

	os.Setenv("ROS_MASTER_URI", "http://env:11311")
	defer os.Unsetenv("ROS_MASTER_URI")

	// Set in DB should override env fallback
	err := db.Set("system.ros.masterUri", []byte("http://db:11311"))
	require.NoError(t, err)

	val, err := db.Get("system.ros.masterUri")
	require.NoError(t, err)
	assert.Equal(t, "http://db:11311", string(val))
}

func TestDBProvider_Delete(t *testing.T) {
	dir := t.TempDir()
	os.Setenv("DB_PATH", dir)
	defer os.Unsetenv("DB_PATH")

	db := NewDBProvider()

	err := db.Set("delete.me", []byte("value"))
	require.NoError(t, err)

	err = db.Delete("delete.me")
	require.NoError(t, err)

	_, err = db.Get("delete.me")
	assert.Error(t, err)
}

func TestDBProvider_GetWithEnvFallbackMethod(t *testing.T) {
	dir := t.TempDir()
	os.Setenv("DB_PATH", dir)
	defer os.Unsetenv("DB_PATH")

	db := NewDBProvider()

	// No env, no db -> returns default
	result := db.GetWithEnvFallback("nonexistent", "NONEXISTENT_ENV", "my-default")
	assert.Equal(t, "my-default", result)

	// Env set -> returns env
	os.Setenv("MY_TEST_ENV", "env-value")
	defer os.Unsetenv("MY_TEST_ENV")

	result = db.GetWithEnvFallback("nonexistent", "MY_TEST_ENV", "my-default")
	assert.Equal(t, "env-value", result)

	// DB set -> returns DB value
	err := db.Set("nonexistent", []byte("db-value"))
	require.NoError(t, err)

	result = db.GetWithEnvFallback("nonexistent", "MY_TEST_ENV", "my-default")
	assert.Equal(t, "db-value", result)
}

func TestDBProvider_DefaultValues(t *testing.T) {
	dir := t.TempDir()
	os.Setenv("DB_PATH", dir)
	defer os.Unsetenv("DB_PATH")

	db := NewDBProvider()

	tests := []struct {
		key      string
		expected string
	}{
		{"system.api.addr", ":4006"},
		{"system.api.webDirectory", "/app/web"},
		{"system.map.enabled", "false"},
		{"system.mower.configFile", "/config/mower_config.sh"},
		{"system.mower.runtimeEnvFile", "/runtime_config/.env"},
		{"system.ros.nodeName", "mowglinext"},
		{"system.ros.nodeHost", "localhost"},
		{"system.mqtt.enabled", "false"},
		{"system.mqtt.host", ":1883"},
		{"system.mqtt.prefix", "/gui"},
		{"system.homekit.enabled", "false"},
		{"system.homekit.pincode", "00102003"},
	}

	for _, tt := range tests {
		t.Run(tt.key, func(t *testing.T) {
			// Clear any env vars that might interfere
			if envVar, ok := EnvFallbacks[tt.key]; ok {
				os.Unsetenv(envVar)
			}

			val, err := db.Get(tt.key)
			require.NoError(t, err)
			assert.Equal(t, tt.expected, string(val))
		})
	}
}
