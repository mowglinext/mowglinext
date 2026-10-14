//go:build linux

package blackbox

import (
	"os"
	"strconv"
	"strings"
	"syscall"
)

func sampleResources() resourceSample {
	data, err := os.ReadFile("/proc/meminfo")
	if err != nil {
		return resourceSample{}
	}
	s := parseMemoryInfo(data)
	// cgroup v2 and v1 account for the container's actual budget, not the SBC
	// model or host RAM. Unlimited sentinel values are ignored.
	for _, pair := range [][2]string{{"/sys/fs/cgroup/memory.max", "/sys/fs/cgroup/memory.current"}, {"/sys/fs/cgroup/memory/memory.limit_in_bytes", "/sys/fs/cgroup/memory/memory.usage_in_bytes"}} {
		limitData, e1 := os.ReadFile(pair[0])
		usedData, e2 := os.ReadFile(pair[1])
		if e1 != nil || e2 != nil {
			continue
		}
		limit, e1 := strconv.ParseInt(strings.TrimSpace(string(limitData)), 10, 64)
		used, e2 := strconv.ParseInt(strings.TrimSpace(string(usedData)), 10, 64)
		if e1 != nil || e2 != nil || limit <= 0 || used < 0 || limit > s.total {
			continue
		}
		s.total = min(s.total, limit)
		s.available = min(s.available, max(0, limit-used))
		s.known = true
	}
	return s
}

func diskFree(dir string) (int64, bool) {
	var stats syscall.Statfs_t
	if syscall.Statfs(dir, &stats) != nil {
		return 0, false
	}
	return int64(stats.Bavail) * int64(stats.Bsize), true
}
