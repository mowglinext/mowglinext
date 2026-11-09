// blackbox-stress exercises the recorder without ROS, control services or a mower.
package main

import (
	"context"
	"encoding/json"
	"flag"
	"fmt"
	"os"
	"runtime"
	"time"

	"github.com/mowglinext/mowglinext/pkg/blackbox"
)

func main() {
	duration := flag.Duration("duration", 30*time.Minute, "sustained workload duration")
	hz := flag.Int("hz", 200, "total synthetic observations per second")
	enabled := flag.Bool("enabled", true, "false runs identical publisher workload without a recorder")
	memory := flag.Int64("memory-mib", 16, "requested recorder memory limit in MiB")
	dir := flag.String("dir", "", "snapshot directory (default temporary)")
	trigger := flag.Bool("trigger", true, "inject one synthetic emergency near end of run")
	flag.Parse()
	if *hz < 1 || *hz > 100000 || *duration <= 0 {
		fmt.Fprintln(os.Stderr, "invalid duration or hz")
		os.Exit(2)
	}
	var r *blackbox.Recorder
	var err error
	if *enabled {
		if *dir == "" {
			*dir, err = os.MkdirTemp("", "blackbox-stress-")
			if err != nil {
				panic(err)
			}
			defer os.RemoveAll(*dir)
		}
		cfg := blackbox.DefaultConfig()
		cfg.MemoryBytes = *memory << 20
		cfg.PostSeconds = 2
		cfg.MinFreeDiskBytes = 0
		r, err = blackbox.New(*dir, cfg, map[string]string{"source": "synthetic-stress"})
		if err != nil {
			panic(err)
		}
	}
	topics := []string{"fusionRaw", "wheelOdom", "rawGps", "highLevelStatus", "cmdVelApplied", "mowerStatus", "diagnostics", "emergency"}
	start := time.Now()
	cpuStart, _, writeStart, metricsKnown := processMetrics()
	ticker := time.NewTicker(time.Second / time.Duration(*hz))
	defer ticker.Stop()
	samples := time.NewTicker(time.Second)
	defer samples.Stop()
	deadline := time.NewTimer(*duration)
	defer deadline.Stop()
	var emitted, accepted uint64
	var peakHeap, peakSys uint64
	var maxIngress time.Duration
	triggered := false
	var filesBeforeTrigger int
	var writesBeforeTrigger uint64
	loop := true
	for loop {
		select {
		case <-ticker.C:
			topic := topics[emitted%uint64(len(topics))]
			// Serialization is the same in both runs; it represents existing
			// provider telemetry processing before the passive recorder tap.
			message := map[string]any{"sequence": emitted, "x": 1.25, "y": 2.5, "velocity": 0.3, "stamp": time.Now().UnixNano()}
			if topic == "emergency" {
				message["active_emergency"] = false
				message["latched_emergency"] = false
			}
			if topic == "diagnostics" {
				message["status"] = []any{map[string]any{"name": "synthetic", "level": 0}}
			}
			data, _ := json.Marshal(message)
			if r != nil {
				before := time.Now()
				if r.Ingest(topic, data) {
					accepted++
				}
				maxIngress = max(maxIngress, time.Since(before))
			}
			emitted++
			if r != nil && *trigger && !triggered && time.Since(start) >= *duration-3*time.Second {
				entries, _ := os.ReadDir(*dir)
				filesBeforeTrigger = len(entries)
				_, _, writes, _ := processMetrics()
				writesBeforeTrigger = writes - writeStart
				r.Ingest("emergency", []byte(`{"active_emergency":true,"latched_emergency":true,"reason":"synthetic fault"}`))
				triggered = true
			}
		case <-samples.C:
			var stats runtime.MemStats
			runtime.ReadMemStats(&stats)
			peakHeap = max(peakHeap, stats.HeapAlloc)
			peakSys = max(peakSys, stats.Sys)
		case <-deadline.C:
			loop = false
		}
	}
	if r != nil {
		ctx, cancel := context.WithTimeout(context.Background(), 30*time.Second)
		err = r.Close(ctx)
		cancel()
		if err != nil {
			panic(err)
		}
	}
	var stats runtime.MemStats
	runtime.ReadMemStats(&stats)
	cpuEnd, peakRSS, _, _ := processMetrics()
	result := map[string]any{"enabled": *enabled, "elapsed_seconds": time.Since(start).Seconds(), "emitted": emitted, "accepted": accepted,
		"peak_heap_bytes": max(peakHeap, stats.HeapAlloc), "peak_runtime_sys_bytes": max(peakSys, stats.Sys), "heap_at_end_bytes": stats.HeapAlloc,
		"gc_cycles": stats.NumGC, "max_ingress_microseconds": float64(maxIngress.Nanoseconds()) / 1000, "go_version": runtime.Version(), "platform": runtime.GOOS + "/" + runtime.GOARCH}
	result["process_metrics_available"] = metricsKnown
	result["cpu_seconds"] = cpuEnd - cpuStart
	result["cpu_percent_one_core"] = (cpuEnd - cpuStart) / time.Since(start).Seconds() * 100
	result["peak_rss_bytes"] = peakRSS
	result["files_before_trigger"] = filesBeforeTrigger
	result["disk_write_bytes_before_trigger"] = writesBeforeTrigger
	if r != nil {
		result["status"] = r.Status()
		list, listErr := r.List()
		result["snapshots"] = list
		if listErr != nil {
			result["list_error"] = listErr.Error()
		}
		result["snapshot_directory"] = *dir
	}
	encoder := json.NewEncoder(os.Stdout)
	encoder.SetIndent("", "  ")
	if err := encoder.Encode(result); err != nil {
		panic(err)
	}
}
