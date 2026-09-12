//go:build linux

package updater

import (
	"fmt"
	"os"
	"path/filepath"
	"syscall"
)

func freeBytes(path string) (uint64, error) {
	var s syscall.Statfs_t
	err := syscall.Statfs(path, &s)
	return s.Bavail * uint64(s.Bsize), err
}
func processLock(path string) (func(), error) {
	f, err := os.OpenFile(path, os.O_CREATE|os.O_RDWR, 0600)
	if err != nil {
		return nil, err
	}
	if err = syscall.Flock(int(f.Fd()), syscall.LOCK_EX|syscall.LOCK_NB); err != nil {
		f.Close()
		return nil, fmt.Errorf("another updater owns this installation: %w", err)
	}
	return func() { f.Close() }, nil
}

// The privileged runtime must keep installer-owned files readable/editable by
// their existing owner. Private state inherits its root-owned state directory.
func inheritFileOwner(f *os.File, path string) error {
	if os.Geteuid() != 0 {
		return nil
	}
	info, err := os.Stat(path)
	if os.IsNotExist(err) {
		info, err = os.Stat(filepath.Dir(path))
	}
	if err != nil {
		return err
	}
	stat, ok := info.Sys().(*syscall.Stat_t)
	if !ok {
		return fmt.Errorf("cannot determine configuration owner")
	}
	return f.Chown(int(stat.Uid), int(stat.Gid))
}
