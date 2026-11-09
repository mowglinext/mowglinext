// Package blackbox implements a passive, bounded, RAM-backed diagnostic recorder.
// It has no ROS dependencies and cannot command the mower.
package blackbox

import (
	"errors"
	"time"
)

const (
	queueCapacity  = 64
	maxPayload     = 16 * 1024
	recordOverhead = 256 // conservative accounting for references and allocator overhead
	maxReasons     = 8
)

type Config struct {
	Enabled          bool  `json:"enabled"`
	PreSeconds       int   `json:"pre_seconds"`
	PostSeconds      int   `json:"post_seconds"`
	MemoryBytes      int64 `json:"memory_bytes"`
	MaxMessageBytes  int   `json:"max_message_bytes"`
	MaxSnapshots     int   `json:"max_snapshots"`
	MaxDiskBytes     int64 `json:"max_disk_bytes"`
	CooldownSeconds  int   `json:"cooldown_seconds"`
	MinFreeDiskBytes int64 `json:"min_free_disk_bytes"`
}

func DefaultConfig() Config {
	return Config{Enabled: true, PreSeconds: 60, PostSeconds: 15, MemoryBytes: 16 << 20,
		MaxMessageBytes: 8192, MaxSnapshots: 20, MaxDiskBytes: 256 << 20,
		CooldownSeconds: 60, MinFreeDiskBytes: 32 << 20}
}

func (c Config) Validate() error {
	if c.PreSeconds < 1 || c.PreSeconds > 300 || c.PostSeconds < 1 || c.PostSeconds > 60 {
		return errors.New("blackbox windows must be pre 1..300 and post 1..60 seconds")
	}
	if c.MemoryBytes < 4<<20 || c.MemoryBytes > 64<<20 {
		return errors.New("blackbox memory must be 4..64 MiB")
	}
	if c.MaxMessageBytes < 256 || c.MaxMessageBytes > maxPayload {
		return errors.New("blackbox maximum message must be 256..16384 bytes")
	}
	if c.MaxSnapshots < 1 || c.MaxSnapshots > 200 || c.MaxDiskBytes < 4<<20 || c.MaxDiskBytes > 4<<30 {
		return errors.New("blackbox disk limits must be 1..200 snapshots and 4 MiB..4 GiB")
	}
	if c.CooldownSeconds < 1 || c.CooldownSeconds > 3600 || c.MinFreeDiskBytes < 0 || c.MinFreeDiskBytes > 1<<30 {
		return errors.New("invalid blackbox cooldown or free disk reserve")
	}
	return nil
}

type Status struct {
	Config                Config  `json:"config"`
	Phase                 string  `json:"phase"`
	CaptureID             string  `json:"capture_id,omitempty"`
	BufferedRecords       int     `json:"buffered_records"`
	BufferedBytes         int64   `json:"buffered_bytes"`
	BufferedSeconds       float64 `json:"buffered_seconds"`
	EffectiveMemoryBytes  int64   `json:"effective_memory_bytes"`
	EffectivePreSeconds   int     `json:"effective_pre_seconds"`
	DroppedMessages       uint64  `json:"dropped_messages"`
	HistoryEvictions      uint64  `json:"history_evictions"`
	SkippedTriggerSources uint64  `json:"skipped_trigger_sources"`
	LastWriteSeconds      float64 `json:"last_write_seconds"`
	CoalescedTriggers     uint64  `json:"coalesced_triggers"`
	CompletedSnapshots    uint64  `json:"completed_snapshots"`
	LastError             string  `json:"last_error,omitempty"`
	MemoryPressure        bool    `json:"memory_pressure"`
}

type Metadata struct {
	Kind                   string            `json:"kind"`
	FormatVersion          int               `json:"format_version"`
	CaptureID              string            `json:"capture_id"`
	TriggeredAt            time.Time         `json:"triggered_at"`
	Reasons                []string          `json:"reasons"`
	Identifiers            map[string]string `json:"identifiers,omitempty"`
	Config                 Config            `json:"config"`
	ActualPreSeconds       float64           `json:"actual_pre_seconds"`
	ActualPostSeconds      float64           `json:"actual_post_seconds"`
	WindowElapsedSeconds   float64           `json:"window_elapsed_seconds"`
	Records                int               `json:"records"`
	DroppedMessages        uint64            `json:"dropped_messages"`
	CaptureDroppedMessages uint64            `json:"capture_dropped_messages"`
	Interrupted            bool              `json:"interrupted"`
}

type Snapshot struct {
	Name string `json:"name"`
	Size int64  `json:"size"`
	Metadata
}
