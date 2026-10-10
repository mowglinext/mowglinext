package blackbox

import (
	"bufio"
	"encoding/json"
	"errors"
	"fmt"
	"io"
	"math"
	"os"
	"path/filepath"
	"regexp"
	"sort"
	"strings"
)

var snapshotName = regexp.MustCompile(`^blackbox-[0-9]{8}T[0-9]{6}\.[0-9]{9}Z-[0-9a-f]{16}\.jsonl$`)

func prepareDirectory(dir string) (bool, error) {
	if err := os.MkdirAll(dir, 0700); err != nil {
		return false, err
	}
	info, err := os.Lstat(dir)
	if err != nil {
		return false, err
	}
	if !info.IsDir() || info.Mode()&os.ModeSymlink != 0 {
		return false, errors.New("blackbox storage must be a real directory")
	}
	// Interrupted writes are never listed as completed recordings. Cleanup only
	// our exact temporary naming convention, never manual rosbag data.
	root, err := os.OpenRoot(dir)
	if err != nil {
		return false, err
	}
	defer root.Close()
	f, err := root.Open(".")
	if err != nil {
		return false, err
	}
	defer f.Close()
	hasCompleted := false
	for inspected := 0; inspected < 2048; {
		entries, readErr := f.ReadDir(128)
		inspected += len(entries)
		for _, entry := range entries {
			name := entry.Name()
			if snapshotName.MatchString(name) && entry.Type().IsRegular() {
				hasCompleted = true
			}
			if strings.HasSuffix(name, ".partial") && snapshotName.MatchString(strings.TrimSuffix(name, ".partial")) {
				if err := root.Remove(name); err != nil {
					return false, fmt.Errorf("cleanup interrupted blackbox: %w", err)
				}
			}
		}
		if readErr == io.EOF {
			return hasCompleted, nil
		}
		if readErr != nil {
			return false, readErr
		}
	}
	return false, errors.New("too many entries in blackbox directory")
}

func (r *Recorder) Open(name string) (*os.File, error) {
	if !snapshotName.MatchString(name) {
		return nil, errors.New("invalid blackbox recording name")
	}
	root, err := os.OpenRoot(r.dir)
	if err != nil {
		return nil, err
	}
	defer root.Close()
	info, err := root.Lstat(name)
	if err != nil {
		return nil, err
	}
	if !info.Mode().IsRegular() {
		return nil, errors.New("blackbox recording must be a regular file")
	}
	file, err := root.Open(name)
	if err != nil {
		return nil, err
	}
	opened, err := file.Stat()
	if err != nil || !opened.Mode().IsRegular() || !os.SameFile(info, opened) {
		file.Close()
		return nil, errors.New("blackbox recording changed while opening")
	}
	return file, nil
}

func (r *Recorder) Delete(name string) error {
	if !snapshotName.MatchString(name) {
		return errors.New("invalid blackbox recording name")
	}
	root, err := os.OpenRoot(r.dir)
	if err != nil {
		return err
	}
	defer root.Close()
	info, err := root.Lstat(name)
	if err != nil {
		return err
	}
	if !info.Mode().IsRegular() {
		return errors.New("blackbox recording must be a regular file")
	}
	return root.Remove(name)
}

func (r *Recorder) List() ([]Snapshot, error) {
	root, err := os.OpenRoot(r.dir)
	if err != nil {
		return nil, err
	}
	defer root.Close()
	f, err := root.Open(".")
	if err != nil {
		return nil, err
	}
	defer f.Close()
	result := make([]Snapshot, 0)
	for inspected := 0; inspected < 2048; {
		entries, readErr := f.ReadDir(128)
		inspected += len(entries)
		for _, entry := range entries {
			if !snapshotName.MatchString(entry.Name()) || entry.Type()&os.ModeSymlink != 0 {
				continue
			}
			info, err := entry.Info()
			if err != nil || !info.Mode().IsRegular() {
				continue
			}
			file, err := r.Open(entry.Name())
			if err != nil {
				continue
			}
			header, readErr := bufio.NewReaderSize(io.LimitReader(file, 16385), 16384).ReadBytes('\n')
			file.Close()
			if readErr != nil || len(header) > 16384 {
				continue
			}
			var metadata Metadata
			if json.Unmarshal(header, &metadata) != nil || metadata.Kind != "blackbox_metadata" || metadata.CaptureID+".jsonl" != entry.Name() {
				continue
			}
			result = append(result, Snapshot{Name: entry.Name(), Size: info.Size(), Metadata: metadata})
		}
		if readErr == io.EOF {
			sort.Slice(result, func(i, j int) bool { return result[i].Name > result[j].Name })
			return result, nil
		}
		if readErr != nil {
			return nil, readErr
		}
	}
	return nil, errors.New("too many entries in blackbox directory")
}

type storageEntry struct {
	name string
	size int64
}

// Corrupt headers still count toward retention. All scans have a hard entry bound.
func scanCompleted(root *os.Root) ([]storageEntry, int64, error) {
	f, err := root.Open(".")
	if err != nil {
		return nil, 0, err
	}
	defer f.Close()
	var files []storageEntry
	var total int64
	for inspected := 0; inspected < 2048; {
		entries, readErr := f.ReadDir(128)
		inspected += len(entries)
		for _, e := range entries {
			if !snapshotName.MatchString(e.Name()) || e.Type()&os.ModeSymlink != 0 {
				continue
			}
			info, err := e.Info()
			if err != nil {
				return nil, 0, err
			}
			if !info.Mode().IsRegular() {
				continue
			}
			if info.Size() > math.MaxInt64-total {
				return nil, 0, errors.New("blackbox storage size overflow")
			}
			files = append(files, storageEntry{e.Name(), info.Size()})
			total += info.Size()
		}
		if readErr == io.EOF {
			break
		}
		if readErr != nil {
			return nil, 0, readErr
		}
		if inspected >= 2048 {
			return nil, 0, errors.New("too many entries in blackbox directory")
		}
	}
	return files, total, nil
}

// prune runs on the disk worker, never the collector.
func (r *Recorder) prune(cfg Config, keep string) error {
	root, err := os.OpenRoot(r.dir)
	if err != nil {
		return err
	}
	defer root.Close()
	files, total, err := scanCompleted(root)
	if err != nil {
		return err
	}
	sort.Slice(files, func(i, j int) bool { return files[i].name < files[j].name })
	count := len(files)
	for _, file := range files {
		if count <= cfg.MaxSnapshots && total <= cfg.MaxDiskBytes {
			break
		}
		// Keep the newly published capture even after a backwards wall-clock
		// step. Older evidence is removed only after its replacement is complete.
		if file.name == keep {
			continue
		}
		if err := r.removeCompleted(root, file.name); err != nil && !os.IsNotExist(err) {
			return err
		}
		total -= file.size
		count--
	}
	if count > cfg.MaxSnapshots || total > cfg.MaxDiskBytes {
		return errors.New("cannot satisfy blackbox retention limits")
	}
	return nil
}

type limitWriter struct {
	writer    io.Writer
	remaining int64
}

// A snapshot may be complete even when its subsequent retention cleanup fails.
type publishedError struct{ error }

func (w *limitWriter) Write(p []byte) (int, error) {
	if int64(len(p)) > w.remaining {
		return 0, errors.New("blackbox snapshot exceeds disk limit")
	}
	n, err := w.writer.Write(p)
	w.remaining -= int64(n)
	return n, err
}

func (r *Recorder) persist(c *capture) error {
	// Estimate generously; the hard writer limit also handles JSON escaping.
	estimate := c.bytes*2 + 16384
	if estimate > c.metadata.Config.MaxDiskBytes {
		return errors.New("blackbox capture too large for configured storage")
	}
	free, known := r.freeDisk(r.dir)
	if known && (free < c.metadata.Config.MinFreeDiskBytes || free-c.metadata.Config.MinFreeDiskBytes < estimate) {
		return errors.New("insufficient free disk space for blackbox snapshot")
	}
	root, err := os.OpenRoot(r.dir)
	if err != nil {
		return err
	}
	defer root.Close()
	files, total, err := scanCompleted(root)
	if err != nil {
		return err
	}
	// One replacement may exceed retention until cleanup completes. If cleanup
	// failed, refuse another publication until settings retry/delete restores
	// the limit; repeated incidents must never grow storage indefinitely.
	if len(files) > c.metadata.Config.MaxSnapshots || total > c.metadata.Config.MaxDiskBytes {
		return errors.New("blackbox retention exceeded; retry cleanup or delete recordings before saving another snapshot")
	}
	name := c.metadata.CaptureID + ".jsonl"
	temp := name + ".partial"
	f, err := root.OpenFile(temp, os.O_WRONLY|os.O_CREATE|os.O_EXCL, 0600)
	if err != nil {
		return err
	}
	completed := false
	defer func() {
		f.Close()
		if !completed {
			root.Remove(temp)
		}
	}()
	buffer := bufio.NewWriterSize(&limitWriter{writer: f, remaining: c.metadata.Config.MaxDiskBytes}, 32<<10)
	encoder := json.NewEncoder(buffer)
	encoder.SetEscapeHTML(false)
	if err = encoder.Encode(c.metadata); err != nil {
		return err
	}
	for _, o := range c.observations {
		line := struct {
			Kind          string          `json:"kind"`
			ReceivedAt    string          `json:"received_at"`
			OffsetSeconds float64         `json:"offset_seconds"`
			Topic         string          `json:"topic"`
			Data          json.RawMessage `json:"data"`
		}{
			"observation", o.at.UTC().Format("2006-01-02T15:04:05.000000000Z"), o.at.Sub(c.trigger).Seconds(), o.topic, json.RawMessage(o.data)}
		if err = encoder.Encode(line); err != nil {
			return err
		}
	}
	if err = encoder.Encode(struct {
		Kind     string `json:"kind"`
		Records  int    `json:"records"`
		Complete bool   `json:"complete"`
	}{"blackbox_summary", len(c.observations), true}); err != nil {
		return err
	}
	if err = buffer.Flush(); err != nil {
		return err
	}
	if err = f.Sync(); err != nil {
		return err
	}
	if err = f.Close(); err != nil {
		return err
	}
	if err = root.Rename(temp, name); err != nil {
		return err
	}
	completed = true
	// Directory sync strengthens rename durability on Linux; platforms that do
	// not support syncing directories still retain atomic completed-file visibility.
	directory, err := os.Open(filepath.Clean(r.dir))
	if err == nil {
		_ = directory.Sync()
		_ = directory.Close()
	}
	if err := r.prune(c.metadata.Config, name); err != nil {
		return publishedError{err}
	}
	return nil
}
