package api

import (
	"bytes"
	"net/http/httptest"
	"os"
	"path/filepath"
	"sync"
	"testing"

	"github.com/gin-gonic/gin"
	"github.com/mowglinext/mowglinext/pkg/types"
	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
	"gopkg.in/yaml.v3"
)

type settingsReadBarrierDB struct {
	*types.MockDBProvider
	once          sync.Once
	read, release chan struct{}
}

type settingsTransactionProbeDB struct {
	*types.MockDBProvider
	onGet func(string)
}

func (d *settingsTransactionProbeDB) Get(key string) ([]byte, error) {
	d.onGet(key)
	return d.MockDBProvider.Get(key)
}

func TestSettingsYAMLAllGUIWritersHoldTransaction(t *testing.T) {
	chdirToGuiRoot(t)
	for _, writer := range []string{"http", "drive", "gnss", "gnss-device-env"} {
		t.Run(writer, func(t *testing.T) {
			seedSchemaCache(t, "par_a")
			t.Setenv("HARDWARE_BACKEND", "mowgli")
			path := filepath.Join(t.TempDir(), "robot.yaml")
			require.NoError(t, os.WriteFile(path, []byte("mowgli:\n  ros__parameters:\n    par_a: retained\n"), 0o640))
			base := types.NewMockDBProvider()
			require.NoError(t, base.Set("system.mower.yamlConfigFile", []byte(path)))
			require.NoError(t, base.Set("system.mower.runtimeEnvFile", nil))
			probes := 0
			db := &settingsTransactionProbeDB{MockDBProvider: base, onGet: func(key string) {
				if key == "system.mower.yamlConfigFile" {
					probes++
					if settingsYAMLWriteMu.TryLock() {
						settingsYAMLWriteMu.Unlock()
						t.Error("YAML must be read inside the shared transaction")
					}
				}
			}}
			switch writer {
			case "http":
				r := gin.New()
				PostSettingsYAML(r.Group("/api"), db)
				w := httptest.NewRecorder()
				r.ServeHTTP(w, httptest.NewRequest("POST", "/api/settings/yaml", bytes.NewBufferString(`{"par_a":"retained"}`)))
				require.Equal(t, 200, w.Code, w.Body.String())
			case "drive":
				require.NoError(t, persistRobotYamlUpdates(db, map[string]any{"ticks_per_meter": 75.0}))
			case "gnss":
				require.NoError(t, persistGNSSRuntimeBaud(db, "460800"))
			case "gnss-device-env":
				previous := gnssSerialDeviceAccess
				t.Cleanup(func() { gnssSerialDeviceAccess = previous })
				gnssSerialDeviceAccess = func(string) (string, string, error) {
					probes++
					if settingsYAMLWriteMu.TryLock() {
						settingsYAMLWriteMu.Unlock()
						t.Error("runtime env must share the YAML transaction")
					}
					return "123", "/dev/mock-gnss", nil
				}
				require.NoError(t, syncGNSSDeviceRuntimeEnv(gnssSavedConfig{
					SerialDevice: "/dev/mock-gnss", RuntimeEnvPath: filepath.Join(t.TempDir(), ".env"),
				}))
			}
			assert.Positive(t, probes)
			raw, err := os.ReadFile(path)
			require.NoError(t, err)
			assert.Contains(t, string(raw), "par_a: retained")
		})
	}
}

func TestSettingsYAMLWriteFailureDoesNotAcknowledgeAndAllowsRetry(t *testing.T) {
	chdirToGuiRoot(t)
	seedSchemaCache(t, "par_a")
	t.Setenv("HARDWARE_BACKEND", "mowgli")
	dir := t.TempDir()
	path := filepath.Join(dir, "robot.yaml")
	original := []byte("mowgli:\n  ros__parameters:\n    par_a: original\n")
	require.NoError(t, os.WriteFile(path, original, 0o640))
	base := types.NewMockDBProvider()
	require.NoError(t, base.Set("system.mower.yamlConfigFile", []byte(path)))
	require.NoError(t, base.Set("system.mower.runtimeEnvFile", nil))
	var once sync.Once
	db := &settingsTransactionProbeDB{MockDBProvider: base, onGet: func(key string) {
		if key == "system.mower.runtimeEnvFile" {
			once.Do(func() {
				require.NoError(t, os.Rename(path, path+".original"))
				require.NoError(t, os.Mkdir(path, 0o700)) // Rename cannot replace a directory.
			})
		}
	}}
	r := gin.New()
	PostSettingsYAML(r.Group("/api"), db)
	send := func() *httptest.ResponseRecorder {
		w := httptest.NewRecorder()
		r.ServeHTTP(w, httptest.NewRequest("POST", "/api/settings/yaml", bytes.NewBufferString(`{"par_a":"new"}`)))
		return w
	}
	w := send()
	require.Equal(t, 500, w.Code)
	assert.NotContains(t, w.Body.String(), "persisted")
	backup, err := os.ReadFile(path + ".original")
	require.NoError(t, err)
	assert.Equal(t, original, backup)
	entries, err := filepath.Glob(filepath.Join(dir, ".mowgli-settings-*.tmp"))
	require.NoError(t, err)
	assert.Empty(t, entries)
	require.NoError(t, os.Remove(path))
	require.NoError(t, os.Rename(path+".original", path))
	w = send()
	require.Equal(t, 200, w.Code, w.Body.String())
}

func TestSettingsYAMLRuntimeFailureReportsPersistedAndReleasesTransaction(t *testing.T) {
	chdirToGuiRoot(t)
	for _, backend := range []string{"mowgli", "mavros"} {
		t.Run(backend, func(t *testing.T) {
			resetSchemaCache()
			t.Cleanup(resetSchemaCache)
			t.Setenv("HARDWARE_BACKEND", backend)
			dir := t.TempDir()
			path := filepath.Join(dir, "robot.yaml")
			require.NoError(t, os.WriteFile(path, []byte("mowgli:\n  ros__parameters:\n    par_a: original\n"), 0o640))
			base := types.NewMockDBProvider()
			require.NoError(t, base.Set("system.mower.yamlConfigFile", []byte(path)))
			require.NoError(t, base.Set("system.mower.runtimeEnvFile", []byte(filepath.Join(dir, ".env"))))
			if backend == "mavros" {
				require.NoError(t, os.WriteFile(filepath.Join(dir, ".env"), []byte("HARDWARE_BACKEND=mavros\n"), 0o600))
				require.NoError(t, os.WriteFile(filepath.Join(dir, "config"), []byte("blocks runtime directory"), 0o600))
			}
			// Resolve the backend from the absent env, then block only its
			// post-YAML synchronization. No permissions/root-dependent failure.
			reads := 0
			db := &settingsTransactionProbeDB{MockDBProvider: base, onGet: func(key string) {
				if key == "system.mower.runtimeEnvFile" {
					reads++
					if reads == 2 && backend == "mowgli" {
						require.NoError(t, os.Mkdir(filepath.Join(dir, ".env"), 0o700))
					}
				}
			}}
			r := gin.New()
			PostSettingsYAML(r.Group("/api"), db)
			w := httptest.NewRecorder()
			r.ServeHTTP(w, httptest.NewRequest("POST", "/api/settings/yaml", bytes.NewBufferString(`{"par_a":"persisted"}`)))
			require.Equal(t, 500, w.Code, w.Body.String())
			assert.Contains(t, w.Body.String(), `"persisted":true`)
			if backend == "mavros" {
				assert.Contains(t, w.Body.String(), "MAVROS runtime config failed")
			}
			raw, err := os.ReadFile(path)
			require.NoError(t, err)
			assert.Contains(t, string(raw), "par_a: persisted")
			require.True(t, settingsYAMLWriteMu.TryLock(), "failure must release transaction")
			settingsYAMLWriteMu.Unlock()
		})
	}
}

func (d *settingsReadBarrierDB) Get(key string) ([]byte, error) {
	if key == "system.mower.runtimeEnvFile" {
		pause := false
		d.once.Do(func() { pause = true })
		if pause {
			close(d.read)
			<-d.release
		}
	}
	return d.MockDBProvider.Get(key)
}

func TestSettingsYAMLConcurrentAcknowledgedEditsSurvive(t *testing.T) {
	chdirToGuiRoot(t)
	for _, tc := range []struct {
		name, body string
		wantA      any
		wantB      any
	}{
		{"distinct", `{"par_b":"new-b"}`, "new-a", "new-b"},
		{"same-key-last-transaction-wins", `{"par_a":"last"}`, "last", "old-b"},
		{"explicit-delete", `{"par_b":null}`, "new-a", nil},
	} {
		t.Run(tc.name, func(t *testing.T) {
			seedSchemaCache(t, "par_a", "par_b")
			t.Setenv("HARDWARE_BACKEND", "mowgli")
			path := filepath.Join(t.TempDir(), "robot.yaml")
			require.NoError(t, os.WriteFile(path, []byte("mowgli:\n  ros__parameters:\n    par_a: old-a\n    par_b: old-b\n"), 0o640))
			base := types.NewMockDBProvider()
			require.NoError(t, base.Set("system.mower.yamlConfigFile", []byte(path)))
			require.NoError(t, base.Set("system.mower.runtimeEnvFile", nil))
			db := &settingsReadBarrierDB{MockDBProvider: base, read: make(chan struct{}), release: make(chan struct{})}
			gin.SetMode(gin.TestMode)
			r := gin.New()
			PostSettingsYAML(r.Group("/api"), db)
			send := func(body string) *httptest.ResponseRecorder {
				w := httptest.NewRecorder()
				q := httptest.NewRequest("POST", "/api/settings/yaml", bytes.NewBufferString(body))
				q.Header.Set("Content-Type", "application/json")
				r.ServeHTTP(w, q)
				return w
			}
			aDone, bDone := make(chan *httptest.ResponseRecorder, 1), make(chan *httptest.ResponseRecorder, 1)
			go func() { aDone <- send(`{"par_a":"new-a"}`) }()
			<-db.read // A has read the old document, before backend/default resolution.
			if settingsYAMLWriteMu.TryLock() {
				settingsYAMLWriteMu.Unlock()
				bDone <- send(tc.body) // Baseline: B is acknowledged before A writes.
				close(db.release)
			} else {
				go func() { bDone <- send(tc.body) }()
				close(db.release) // Serialized implementation: B follows A's transaction.
			}
			a, b := <-aDone, <-bDone
			require.Equal(t, 200, a.Code, a.Body.String())
			require.Equal(t, 200, b.Code, b.Body.String())
			raw, err := os.ReadFile(path)
			require.NoError(t, err)
			var doc map[string]any
			require.NoError(t, yaml.Unmarshal(raw, &doc))
			flat := flattenROS2YAML(doc)
			require.Equal(t, tc.wantA, flat["par_a"])
			require.Equal(t, tc.wantB, flat["par_b"])
		})
	}
}
