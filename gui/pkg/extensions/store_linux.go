//go:build linux

package extensions

import (
	"crypto/rand"
	"encoding/hex"
	"errors"
	"fmt"
	"os"
	"path/filepath"
	"strings"
	"sync"
	"syscall"
)

// Walk using directory descriptors and O_NOFOLLOW, including all ancestors.
// Keeping the final descriptor for every operation avoids path replacement
// races between symlink checks and privileged writes.
func openStoreDir(path string, create bool) (*os.File, error) {
	if !filepath.IsAbs(path) || filepath.Clean(path) != path || path == "/" {
		return nil, errors.New("state root must be a clean absolute directory")
	}
	fd, err := syscall.Open("/", syscall.O_RDONLY|syscall.O_DIRECTORY|syscall.O_CLOEXEC, 0)
	if err != nil {
		return nil, err
	}
	for _, component := range strings.Split(strings.TrimPrefix(path, "/"), "/") {
		next, e := syscall.Openat(fd, component, syscall.O_RDONLY|syscall.O_DIRECTORY|syscall.O_NOFOLLOW|syscall.O_CLOEXEC, 0)
		if e == syscall.ENOENT && create {
			e = syscall.Mkdirat(fd, component, 0700)
			if e == nil {
				e = syscall.Fsync(fd)
			}
			if e == nil || e == syscall.EEXIST {
				next, e = syscall.Openat(fd, component, syscall.O_RDONLY|syscall.O_DIRECTORY|syscall.O_NOFOLLOW|syscall.O_CLOEXEC, 0)
			}
		}
		syscall.Close(fd)
		if e != nil {
			return nil, fmt.Errorf("open state directory: %w", e)
		}
		fd = next
	}
	dir := os.NewFile(uintptr(fd), path)
	info, err := dir.Stat()
	if err != nil {
		dir.Close()
		return nil, err
	}
	stat, ok := info.Sys().(*syscall.Stat_t)
	if !ok || int(stat.Uid) != os.Geteuid() {
		dir.Close()
		return nil, errors.New("state directory is owned by another user")
	}
	if create {
		if err := dir.Chmod(0700); err != nil {
			dir.Close()
			return nil, err
		}
	} else if info.Mode().Perm() != 0700 {
		dir.Close()
		return nil, errors.New("state directory permissions must be 0700")
	}
	return dir, nil
}

func checkStoreFile(f *os.File) error {
	info, err := f.Stat()
	if err != nil {
		return err
	}
	stat, ok := info.Sys().(*syscall.Stat_t)
	if !info.Mode().IsRegular() || !ok || stat.Nlink != 1 || int(stat.Uid) != os.Geteuid() {
		return errors.New("state file must be a regular, unlinked-to-other-paths file owned by this user")
	}
	return nil
}

func openStoreFile(dir *os.File, name string) (*os.File, error) {
	fd, err := syscall.Openat(int(dir.Fd()), name, syscall.O_RDONLY|syscall.O_NOFOLLOW|syscall.O_CLOEXEC|syscall.O_NONBLOCK, 0)
	if err != nil {
		return nil, err
	}
	f := os.NewFile(uintptr(fd), name)
	if err := checkStoreFile(f); err != nil {
		f.Close()
		return nil, err
	}
	return f, nil
}

func lockStore(dir *os.File) (func(), error) {
	fd, err := syscall.Openat(int(dir.Fd()), ".lock", syscall.O_RDWR|syscall.O_CREAT|syscall.O_NOFOLLOW|syscall.O_CLOEXEC|syscall.O_NONBLOCK, 0600)
	if err != nil {
		return nil, err
	}
	f := os.NewFile(uintptr(fd), ".lock")
	if err := checkStoreFile(f); err != nil {
		f.Close()
		return nil, err
	}
	if err := f.Chmod(0600); err != nil {
		f.Close()
		return nil, err
	}
	if err := syscall.Flock(fd, syscall.LOCK_EX|syscall.LOCK_NB); err != nil {
		f.Close()
		return nil, fmt.Errorf("extension manager is locked: %w", err)
	}
	var once sync.Once
	return func() { once.Do(func() { _ = syscall.Flock(fd, syscall.LOCK_UN); _ = f.Close() }) }, nil
}

func writeStoreState(dir *os.File, data []byte) error {
	// Reject an existing unsafe target rather than silently replacing a symlink,
	// hard link, FIFO or device belonging to a different owner.
	old, err := openStoreFile(dir, "state.json")
	if err == nil {
		old.Close()
	} else if !errors.Is(err, os.ErrNotExist) {
		return err
	}
	var nonce [16]byte
	if _, err := rand.Read(nonce[:]); err != nil {
		return err
	}
	name := ".state-" + hex.EncodeToString(nonce[:]) + ".tmp"
	fd, err := syscall.Openat(int(dir.Fd()), name, syscall.O_WRONLY|syscall.O_CREAT|syscall.O_EXCL|syscall.O_NOFOLLOW|syscall.O_CLOEXEC, 0600)
	if err != nil {
		return err
	}
	f := os.NewFile(uintptr(fd), name)
	defer func() { f.Close(); _ = syscall.Unlinkat(int(dir.Fd()), name) }()
	if err := f.Chmod(0600); err != nil {
		return err
	}
	if _, err := f.Write(data); err != nil {
		return err
	}
	if err := f.Sync(); err != nil {
		return err
	}
	if err := f.Close(); err != nil {
		return err
	}
	if err := syscall.Renameat(int(dir.Fd()), name, int(dir.Fd()), "state.json"); err != nil {
		return err
	}
	return dir.Sync()
}
