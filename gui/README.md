# MowgliNext GUI

A GUI for the MowgliNext project.

## Demo

[https://youtu.be/x45rdy4czQ0](https://youtu.be/x45rdy4czQ0)

## Installation

### If you are using Mowgli-Docker

If you are using mowgli-docker, you can skip this part as it's now included in the docker-compose file.

### If your are using MowgliNextOS

MowgliNextOS uses podman and containers are managed by systemd.

First, create the /boot/mowglinext/db directory :

```bash
mkdir /boot/mowglinext/db
```

Create a gui.service file in `/etc/systemd/system/` with the following content:

```bash
[Unit]
Description=Podman container - gui.service
Documentation=man:podman-generate-systemd(1)
Wants=network.target
After=network-online.target NetworkManager.service
StartLimitInterval=120
StartLimitBurst=10

[Service]
Environment=PODMAN_SYSTEMD_UNIT=%n
Type=forking
Restart=always
RestartSec=15s
TimeoutStartSec=1h
TimeoutStopSec=120s

ExecStartPre=/bin/rm -f %t/container-gui.pid %t/container-gui.ctr-id

ExecStart=/usr/bin/podman run --conmon-pidfile %t/container-gui.pid --cidfile %t/container-gui.ctr-id --cgroups=no-conmon \
  --replace --detach --tty --privileged \
  --name mowglinext-gui \
  --network=host \
  --env MOWER_CONFIG_FILE=/config/mower_config.sh \
  --env DOCKER_HOST=unix:///run/podman/podman.sock \
  --env ROS_MASTER_URI=http://localhost:11311 \
  --volume /dev:/dev \
  --volume /run/podman/podman.sock:/run/podman/podman.sock \
  --volume /boot/mowglinext/db:/app/db \
  --volume /boot/mowglinext/mower_config.txt:/config/mower_config.sh \
  --label io.containers.autoupdate=image \
  ghcr.io/mowglinext/mowglinext-gui:master

#ExecStartPost=/usr/bin/podman image prune --all --force

ExecStop=/usr/bin/podman stop --ignore --cidfile %t/container-gui.ctr-id -t 10
ExecStopPost=/usr/bin/podman rm --ignore --force --cidfile %t/container-gui.ctr-id
PIDFile=%t/container-gui.pid

[Install]
WantedBy=multi-user.target default.target
```

Then enable and start the service:

```bash
sudo systemctl enable gui.service
sudo systemctl start gui.service
```

## Usage

Once the container is running, you can access the GUI by opening a browser and going
to `http://<ip of the machine running the container>:4006`

### HomeKit

The password to use MowgliNext in iOS home app is 00102003
Do not forget to set env var HOMEKIT_ENABLED to true

### MQTT

MQTT server is listening on port 1883

See [ros.ts](web%2Fsrc%2Ftypes%2Fros.ts) for topic types

Available topics :

- /gui/mower_logic/current_state
- /gui/mower/status
- /gui/xbot_positioning/xb_pose
- /gui/imu/data_raw
- /gui/mower/wheel_ticks
- /gui/xbot_monitoring/map
- /gui/coverage_planner_node/coverage_path
- /gui/mowing_path

Available commands :

- /gui/call/mower_service/high_level_control [HighLevelControlSrv.go](pkg%2Fmsgs%2Fmower_msgs%2FHighLevelControlSrv.go)
- /gui/call/mower_service/emergency [EmergencyStopSrv.go](pkg%2Fmsgs%2Fmower_msgs%2FEmergencyStopSrv.go)
- /gui/call/mower_logic/set_parameters [Reconfigure.go](pkg%2Fmsgs%2Fdynamic_reconfigure%2FReconfigure.go)
- /gui/call/mower_service/mow_enabled [MowerControlSrv.go](pkg%2Fmsgs%2Fmower_msgs%2FMowerControlSrv.go)
- /gui/call/mower_service/start_in_area [StartInAreaSrv.go](pkg%2Fmsgs%2Fmower_msgs%2FStartInAreaSrv.go)

Do not forget to set env var MQTT_ENABLED to true

### Env variables

- MOWER_CONFIG_FILE=mower_config.sh : config file location
- DOCKER_HOST=unix:///var/run/docker.sock : socker socket
- ROS_MASTER_URI=http://localhost:11311 : ros master uri
- ROS_NODE_NAME=mowglinext-gui : node name
- ROS_NODE_HOST=:4006 : listening port
- MQTT_ENABLED=true : enable mqtt
- MQTT_HOST=:1883 : listening port
- HOMEKIT_ENABLED=true : enable homekit
- MAP_TILE_ENABLED=true : enable map tiles
- MAP_TILE_SERVER=http://localhost:5000 : custom map tile server (see https://github.com/2m/mowglinext-map-tiles for
  usage)
- MAP_TILE_URI=/tiles/vt/lyrs=s,h&x={x}&y={y}&z={z}

# Contributing

PR are welcomed :-)

You can run the gui into VSCode or WebStorm with devcontainer

Then use make deps to install dependencies, open a terminal run make run-gui for the frontend and make run-backend for
the backend

To generate go msgs, just run inside the repository this docker command:

docker run -v $PWD:/app ghcr.io/mowglinext/mowglinext-gui:generate-msg
