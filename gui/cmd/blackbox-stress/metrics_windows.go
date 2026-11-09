//go:build windows

package main

import (
	"syscall"
	"unsafe"
)

var psapi = syscall.NewLazyDLL("psapi.dll")
var getProcessMemoryInfo = psapi.NewProc("GetProcessMemoryInfo")
var getProcessIoCounters = syscall.NewLazyDLL("kernel32.dll").NewProc("GetProcessIoCounters")

func processMetrics() (cpuSeconds float64, peakRSS, writeBytes uint64, known bool) {
	handle, err := syscall.GetCurrentProcess()
	if err != nil {
		return
	}
	var created, exit, kernel, user syscall.Filetime
	if syscall.GetProcessTimes(handle, &created, &exit, &kernel, &user) != nil {
		return
	}
	cpuSeconds = float64(uint64(kernel.HighDateTime)<<32|uint64(kernel.LowDateTime))/1e7 + float64(uint64(user.HighDateTime)<<32|uint64(user.LowDateTime))/1e7
	var counters struct {
		Size, PageFaults                                                                                                 uint32
		PeakWorkingSet, WorkingSet, QuotaPeakPaged, QuotaPaged, QuotaPeakNonPaged, QuotaNonPaged, Pagefile, PeakPagefile uintptr
	}
	counters.Size = uint32(unsafe.Sizeof(counters))
	ok, _, _ := getProcessMemoryInfo.Call(uintptr(handle), uintptr(unsafe.Pointer(&counters)), uintptr(counters.Size))
	if ok == 0 {
		return
	}
	peakRSS = uint64(counters.PeakWorkingSet)
	known = true
	var ioCounters struct{ ReadOperations, WriteOperations, OtherOperations, ReadBytes, WriteBytes, OtherBytes uint64 }
	ok, _, _ = getProcessIoCounters.Call(uintptr(handle), uintptr(unsafe.Pointer(&ioCounters)))
	if ok != 0 {
		writeBytes = ioCounters.WriteBytes
	}
	return
}
