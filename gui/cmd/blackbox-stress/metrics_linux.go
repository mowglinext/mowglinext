//go:build linux

package main

import (
	"os"
	"strconv"
	"strings"
	"syscall"
)

func processMetrics() (cpuSeconds float64, peakRSS, writeBytes uint64, known bool) {
	var usage syscall.Rusage
	if syscall.Getrusage(syscall.RUSAGE_SELF, &usage) != nil {
		return
	}
	cpuSeconds = float64(usage.Utime.Sec+usage.Stime.Sec) + float64(usage.Utime.Usec+usage.Stime.Usec)/1e6
	peakRSS = uint64(usage.Maxrss) * 1024
	known = true
	data, err := os.ReadFile("/proc/self/io")
	if err != nil {
		return
	}
	for _, line := range strings.Split(string(data), "\n") {
		parts := strings.Fields(line)
		if len(parts) == 2 && parts[0] == "write_bytes:" {
			writeBytes, _ = strconv.ParseUint(parts[1], 10, 64)
		}
	}
	return
}
