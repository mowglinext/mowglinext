package providers

import (
	"archive/tar"
	"io"
	"testing"

	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
)

type tarEntry struct {
	name string
	typ  byte
	mode int64
	body string
}

func readTar(t *testing.T, r io.Reader) []tarEntry {
	t.Helper()
	tr := tar.NewReader(r)
	var entries []tarEntry
	for {
		hdr, err := tr.Next()
		if err == io.EOF {
			break
		}
		require.NoError(t, err)
		body, err := io.ReadAll(tr)
		require.NoError(t, err)
		entries = append(entries, tarEntry{name: hdr.Name, typ: hdr.Typeflag, mode: hdr.Mode, body: string(body)})
	}
	return entries
}

func TestTarFilesEmitsParentDirectoriesBeforeFilesShallowestFirst(t *testing.T) {
	// Arrange: two files sharing a parent, one of them nested deeper.
	files := map[string][]byte{
		"/config/serve.json":   []byte(`{}`),
		"/config/authkey":      []byte("tskey\n"),
		"/config/deep/x/y.txt": []byte("y"),
	}

	// Act
	archive, err := tarFiles(files)
	require.NoError(t, err)
	entries := readTar(t, archive)

	// Assert: sorted by path, each directory exactly once and before its content.
	names := make([]string, 0, len(entries))
	for _, e := range entries {
		names = append(names, e.name)
	}
	assert.Equal(t, []string{
		"config/", "config/authkey", "config/deep/", "config/deep/x/", "config/deep/x/y.txt", "config/serve.json",
	}, names)
	assert.Equal(t, byte(tar.TypeDir), entries[0].typ)
	assert.Equal(t, int64(0o755), entries[0].mode)
	assert.Equal(t, byte(tar.TypeReg), entries[1].typ)
	assert.Equal(t, int64(serviceFileMode), entries[1].mode)
	assert.Equal(t, "tskey\n", entries[1].body)
	assert.Equal(t, "{}", entries[5].body)
}

func TestTarFilesRejectsRelativePaths(t *testing.T) {
	_, err := tarFiles(map[string][]byte{"config/serve.json": []byte("{}")})
	assert.Error(t, err)
}

func TestParentDirs(t *testing.T) {
	assert.Nil(t, parentDirs("file"))
	assert.Equal(t, []string{"a"}, parentDirs("a/file"))
	assert.Equal(t, []string{"a", "a/b", "a/b/c"}, parentDirs("a/b/c/file"))
}
