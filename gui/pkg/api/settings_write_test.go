package api

import (
	"errors"
	"os"
	"path/filepath"
	"syscall"
	"testing"
)

func TestWritePreservingPerms_ReplacesFileAndPreservesMetadata(t *testing.T) {
	dir := t.TempDir()
	path := filepath.Join(dir, "mowgli_robot.yaml")
	if err := os.WriteFile(path, []byte("old settings\n"), 0o600); err != nil {
		t.Fatal(err)
	}
	if err := os.Chmod(path, 0o640); err != nil {
		t.Fatal(err)
	}
	before, err := os.Stat(path)
	if err != nil {
		t.Fatal(err)
	}
	beforeOwner, ok := before.Sys().(*syscall.Stat_t)
	if !ok {
		t.Fatal("file metadata does not expose uid/gid")
	}

	content := []byte("complete replacement settings\n")
	if err := writePreservingPerms(path, content); err != nil {
		t.Fatal(err)
	}
	got, err := os.ReadFile(path)
	if err != nil {
		t.Fatal(err)
	}
	if string(got) != string(content) {
		t.Fatalf("replacement contents = %q, want %q", got, content)
	}
	after, err := os.Stat(path)
	if err != nil {
		t.Fatal(err)
	}
	if got, want := after.Mode().Perm(), before.Mode().Perm(); got != want {
		t.Fatalf("replacement mode = %04o, want %04o", got, want)
	}
	afterOwner, ok := after.Sys().(*syscall.Stat_t)
	if !ok {
		t.Fatal("replacement metadata does not expose uid/gid")
	}
	if afterOwner.Uid != beforeOwner.Uid || afterOwner.Gid != beforeOwner.Gid {
		t.Fatalf("replacement owner = %d:%d, want %d:%d", afterOwner.Uid, afterOwner.Gid, beforeOwner.Uid, beforeOwner.Gid)
	}
}

func TestWritePreservingPerms_NewFileIsGroupWritable(t *testing.T) {
	path := filepath.Join(t.TempDir(), "mowgli_robot.yaml")
	if err := writePreservingPerms(path, []byte("new settings\n")); err != nil {
		t.Fatal(err)
	}
	info, err := os.Stat(path)
	if err != nil {
		t.Fatal(err)
	}
	if got, want := info.Mode().Perm(), os.FileMode(0o664); got != want {
		t.Fatalf("new settings mode = %04o, want %04o", got, want)
	}
}

func TestWritePreservingPerms_FailedPartialWriteKeepsOriginalAndCleansTemp(t *testing.T) {
	dir := t.TempDir()
	path := filepath.Join(dir, "mowgli_robot.yaml")
	original := []byte("known-good settings that must survive\n")
	if err := os.WriteFile(path, original, 0o660); err != nil {
		t.Fatal(err)
	}

	injectedErr := errors.New("injected write failure")
	partialWriter := func(temp *os.File, content []byte) (int, error) {
		n, err := temp.Write(content[:len(content)/2])
		if err != nil {
			return n, err
		}
		return n, injectedErr
	}
	if err := writePreservingPermsWithWriter(path, []byte("partial replacement data\n"), partialWriter); !errors.Is(err, injectedErr) {
		t.Fatalf("writePreservingPermsWithWriter() error = %v, want injected write failure", err)
	}

	got, err := os.ReadFile(path)
	if err != nil {
		t.Fatal(err)
	}
	if string(got) != string(original) {
		t.Fatalf("original settings after failed write = %q, want unchanged bytes %q", got, original)
	}

	entries, err := os.ReadDir(dir)
	if err != nil {
		t.Fatal(err)
	}
	if len(entries) != 1 || entries[0].Name() != filepath.Base(path) {
		t.Fatalf("directory entries after failed write = %v, want only %s", entryNames(entries), filepath.Base(path))
	}
}

func entryNames(entries []os.DirEntry) []string {
	names := make([]string, 0, len(entries))
	for _, entry := range entries {
		names = append(names, entry.Name())
	}
	return names
}
