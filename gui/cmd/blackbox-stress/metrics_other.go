//go:build !linux && !windows

package main

func processMetrics() (float64, uint64, uint64, bool) { return 0, 0, 0, false }
