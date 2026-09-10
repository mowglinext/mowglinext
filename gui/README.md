# MowgliNext GUI

A GUI for the MowgliNext project.

## Demo

[https://youtu.be/x45rdy4czQ0](https://youtu.be/x45rdy4czQ0)

## Installation

The GUI ships as a container and is installed with the rest of the stack: the interactive installer
(`install/mowglinext.sh`, or `curl -sSL https://mowgli.garden/install.sh | bash`) enables the
[`install/compose/docker-compose.gui.yml`](../install/compose/docker-compose.gui.yml) overlay for
you, so there is nothing to wire up by hand.

That overlay runs the `mowgli-gui` container from
`ghcr.io/mowglinext/mowglinext/mowglinext-gui:<main|dev>` with `network_mode: host` (and `pid: host`,
so the power menu can run the host's `systemctl reboot` / `poweroff`), and mounts the docker socket,
`/dev`, the bitcask database directory and the robot config directories.

To build the image yourself:

```bash
cd gui && make build      # docker build -t mowglinext .
```

## Usage

Once the container is running, you can access the GUI by opening a browser and going
to `http://<ip of the machine running the container>:4006`

### HomeKit

The default password to use MowgliNext in iOS home app is 00102003 (override it with HOMEKIT_PINCODE)
Do not forget to set env var HOMEKIT_ENABLED to true

### MQTT

With MQTT_ENABLED set to true the GUI runs its own MQTT broker, listening on MQTT_HOST (default
port 1883). Every topic and command below is prefixed with MQTT_PREFIX (default `/gui`).

Payloads are the JSON form of the ROS2 message — see
[ros.generated.ts](web/src/types/ros.generated.ts) for the message types.

Available topics (published retained):

- /gui/highLevelStatus — behavior tree state (`/behavior_tree_node/high_level_status`)
- /gui/status — hardware bridge status (`/hardware_bridge/status`)
- /gui/pose — fused map pose (`/odometry/filtered_map`)
- /gui/gps — `/gps/fix`
- /gui/imu — `/imu/data`
- /gui/ticks — `/wheel_ticks`
- /gui/wheelOdom — `/wheel_odom`
- /gui/map — areas, obstacles and dock pose (assembled from the map_server services)
- /gui/path — full coverage plan (`/coverage/full_plan`)
- /gui/plan — current Nav2 plan (`/plan`)

Available commands — publish the service request as JSON, see
[services_generated.go](pkg/msgs/mowgli/services_generated.go):

- /gui/call/behavior_tree_node/high_level_control — `{"command": <n>}`
- /gui/call/hardware_bridge/emergency_stop — `{"emergency": <n>}`
- /gui/call/hardware_bridge/mower_control — `{"mow_enabled": <n>, "mow_direction": <n>}`
- /gui/call/behavior_tree_node/start_in_area — `{"area": <n>}`

### IrriSense

Optional soil-moisture gate for scheduled mowing, fed by your own
[IrriSense Cloud](https://irrisense-cloud.fly.dev) irrigation service. Mint a
**read-only API token** in IrriSense (Settings → API tokens — a login session is
rejected), then open Settings → IrriSense in the GUI: paste the token, pick the
garden (and optionally the zones), and leave "Block scheduled mowing when wet"
on. The GUI polls the garden every 10 minutes; a zone counts as wet when its
deficit is ≤ 2 mm or it was watered within the last 3 h (both adjustable), and a
due schedule is skipped while any selected zone is wet — the reason is shown on
the schedule card. The gate is fail-open: if the service is unreachable, the
token is wrong, or the data is older than 90 minutes, the state is "unknown" and
schedules run as usual. The token lives only in the GUI database
(`irrisense.token`), is never written to `mowgli_robot.yaml` and is never sent
back to the browser. Endpoints: `GET/PUT /api/irrisense/settings`,
`GET /api/irrisense/status`, `GET /api/irrisense/gardens`.

### Env variables

Every variable below is only a fallback: the value is read from the GUI database first (Settings
page), then from the environment, then from the built-in default.

- API_ADDR=:4006 : HTTP + WebSocket listening address
- WEB_DIR=/app/web : directory the built frontend is served from
- DB_PATH=/db : bitcask database directory (the image sets `/app/db`; the compose overlay overrides it)
- FOXGLOVE_URL=ws://localhost:8765 : foxglove_bridge WebSocket, the only link to ROS2
- DOCKER_HOST=unix:///var/run/docker.sock : docker socket
- MOWER_CONFIG_FILE=/config/mower_config.sh : legacy shell config file location
- MOWER_YAML_CONFIG_FILE=/config/mowgli_robot.yaml : the robot's ROS2 config the Settings page edits
- MOWER_RUNTIME_ENV_FILE=/runtime_config/.env : runtime env file (GNSS / LiDAR install choices)
- ROSBAG_DIR=/ros2_ws/maps/rosbags : where the diagnostics rosbag recorder writes
- MQTT_ENABLED=true : enable mqtt
- MQTT_HOST=:1883 : listening port
- MQTT_PREFIX=/gui : topic prefix
- HOMEKIT_ENABLED=true : enable homekit
- HOMEKIT_PINCODE=00102003 : homekit pairing code
- MAP_TILE_ENABLED=true : enable map tiles
- MAP_TILE_SERVER=http://localhost:5000 : custom map tile server (see https://github.com/2m/mowglinext-map-tiles for
  usage)
- MAP_TILE_URI=/tiles/vt/lyrs=s,h&x={x}&y={y}&z={z}

# Contributing

PR are welcomed :-)

You can run the gui into VSCode or WebStorm with devcontainer (`gui/.devcontainer`)

Then use make deps to install dependencies, open a terminal run make run-gui for the frontend and make run-backend for
the backend

After changing a `.msg` or `.srv` in `ros2/src/mowgli_interfaces`, regenerate the Go and TypeScript
bindings from this directory (use `LC_ALL=C`, macOS sort order fabricates phantom drift) — the
`msg-codegen-drift.yml` CI job fails otherwise:

```bash
LC_ALL=C ./generate_go_msgs.sh      # pkg/msgs/*/types_generated.go + mowgli/services_generated.go
LC_ALL=C ./generate_ts_types.sh     # web/src/types/ros.generated.ts
```

Working on this with a coding agent? Start at [`CLAUDE.md`](CLAUDE.md) in this directory — it points
at the GUI codemaps ([`gui_backend.md`](../docs/claude/codemaps/gui_backend.md),
[`gui_frontend.md`](../docs/claude/codemaps/gui_frontend.md)) and the repo-wide indexes
([`ros-interfaces.md`](../docs/claude/ros-interfaces.md),
[`parameters.md`](../docs/claude/parameters.md), [`testing-ci.md`](../docs/claude/testing-ci.md),
[`doc-index.md`](../docs/claude/doc-index.md)).
