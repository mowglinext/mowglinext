//go:build !linux

package blackbox

// The production Linux implementation reads host/container pressure. Other
// platforms use a conservative 8 MiB cap and still enforce all queue, payload,
// snapshot and storage-size bounds. Disk write errors remain recoverable.
func sampleResources() resourceSample { return resourceSample{} }
func diskFree(string) (int64, bool)   { return 0, false }
