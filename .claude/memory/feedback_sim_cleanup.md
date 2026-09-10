---
name: Always clean DDS/Webots state before sim launch
description: Must kill all processes and clean shared memory before launching simulation to avoid stale state
type: feedback
---

ALWAYS run `make sim-stop` from `ros2/` (or the equivalent cleanup) before launching simulation or E2E tests.

**Why:** Stale Cyclone DDS shared memory and zombie Webots processes cause "Failed to find a free participant index", "additional publishers on /clock", "Cannot connect to Webots instance" (a stale `/tmp/webots/<user>/<port>/ipc/.../extern` socket with no listener), and lifecycle manager crashes. These look like real bugs but are just leftover state from previous runs.

**How to apply:**
- Use `make sim` or `make e2e-test` — both auto-run `sim-stop` first
- If running ros2 launch directly: run `make sim-stop` manually first
- Never have two simulation instances running simultaneously
- The cleanup sequence (`ros2/scripts/sim-stop.sh`) is: SIGINT `ros2 launch` → SIGKILL stragglers → kill Webots processes (`webots-bin`, `webots_controller_*`, `ros2_supervisor.py`, `webots_ros2_driver`) → kill leftover ROS2 nodes → `rm -rf /dev/shm/cyclone* /dev/shm/dds* /dev/shm/iox* /tmp/webots/*`
