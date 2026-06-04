SHELL := /bin/bash

ROS_DISTRO    ?= kilted
BUILD_TYPE    ?= Release
DOCKER_IMAGE  ?= mowgli-ros2
DOCKER_TAG    ?= latest
ROBOT_HOST    ?= mowgli.local
ROBOT_USER    ?= pi


.PHONY: help build build-debug build-pkg test clean \
        sim sim-stop e2e-test e2e-test-no-lidar \
        docker docker-sim \
        lint format format-check \
        deploy backup-maps

# ─── Help ──────────────────────────────────────────────────────────────────────

help:
	@echo "Mowgli ROS2 Build System (devcontainer)"
	@echo ""
	@echo "Build:"
	@echo "  build              Build all packages (Release)"
	@echo "  build-debug        Build with Debug symbols"
	@echo "  build-pkg PKG=x    Build a single package"
	@echo "  test               Run all unit tests"
	@echo "  clean              Remove build/ install/ log/"
	@echo ""
	@echo "Simulation & testing:"
	@echo "  sim                Launch headless simulation (auto-stops previous)"
	@echo "  sim-stop           Kill all ROS2/Webots processes and clean DDS state"
	@echo "  e2e-test           Launch sim + run E2E test + stop (self-contained)"
	@echo ""
	@echo "Docker images (CI / production):"
	@echo "  docker             Build production image (runtime stage)"
	@echo "  docker-sim         Build simulation image"
	@echo ""
	@echo "Code quality:"
	@echo "  lint               Run cppcheck + cpplint"
	@echo "  format             Apply clang-format in-place"
	@echo "  format-check       Verify formatting (dry run)"
	@echo ""
	@echo "Deployment:"
	@echo "  deploy             Sync install tree to robot and restart"
	@echo "  backup-maps        Pull SLAM maps from robot"

# ─── Build ─────────────────────────────────────────────────────────────────────

build:
	BUILD_TYPE=$(BUILD_TYPE) ./scripts/build.sh

build-debug:
	$(MAKE) build BUILD_TYPE=Debug

build-pkg:
	PACKAGES="$(PKG)" BUILD_TYPE=$(BUILD_TYPE) ./scripts/build.sh

test:
	./scripts/test.sh

clean:
	rm -rf build/ install/ log/

# ─── Simulation & Testing ─────────────────────────────────────────────────────

# Kill all ROS2/Gazebo processes and clean DDS shared memory.
# MUST be run before launching a new simulation to avoid stale state
# causing "Failed to find a free participant" or clock conflicts.
sim-stop:
	@scripts/sim-stop.sh

sim: sim-stop
	@echo "Starting headless simulation..."
	@echo "  Foxglove: ws://localhost:8765"
	@echo ""
	source /opt/ros/$(ROS_DISTRO)/setup.bash && \
	source install/setup.bash && \
	export DISPLAY=:99 && \
	ros2 launch mowgli_bringup sim_full_system.launch.py \
	  headless:=true use_rviz:=false

e2e-test: sim-stop build
	@echo "Starting simulation + E2E test..."
	@echo ""
	source /opt/ros/$(ROS_DISTRO)/setup.bash && \
	source install/setup.bash && \
	export DISPLAY=:99 && \
	ros2 launch mowgli_bringup sim_full_system.launch.py \
	  headless:=true use_rviz:=false &
	SIM_PID=$$!; \
	echo "Simulation PID: $$SIM_PID"; \
	echo "Waiting 90s for Nav2 to activate..."; \
	sleep 90; \
	echo "Running E2E test..."; \
	python3 src/e2e_test.py; \
	TEST_EXIT=$$?; \
	echo "Stopping simulation..."; \
	kill $$SIM_PID 2>/dev/null; \
	$(MAKE) sim-stop; \
	exit $$TEST_EXIT

e2e-test-no-lidar: sim-stop build
	@echo "Starting no-LiDAR E2E test (GPS-only simulation)..."
	@echo ""
	source /opt/ros/$(ROS_DISTRO)/setup.bash && \
	source install/setup.bash && \
	ros2 launch mowgli_bringup sim_full_system.launch.py \
	  headless:=true use_rviz:=false use_lidar:=false \
	  simulate_gps_degradation:=false &
	SIM_PID=$$!; \
	echo "Simulation PID: $$SIM_PID"; \
	echo "Waiting 90s for Nav2 to activate..."; \
	sleep 90; \
	echo "Running no-LiDAR E2E test..."; \
	python3 src/e2e_test_no_lidar.py; \
	TEST_EXIT=$$?; \
	echo "Stopping simulation..."; \
	kill $$SIM_PID 2>/dev/null; \
	$(MAKE) sim-stop; \
	exit $$TEST_EXIT

# ─── Docker Images (CI / production) ──────────────────────────────────────────

docker:
	docker build --target runtime \
	  --build-arg BUILD_TYPE=$(BUILD_TYPE) \
	  -t $(DOCKER_IMAGE):$(DOCKER_TAG) .

docker-sim:
	docker build --target simulation \
	  --build-arg BUILD_TYPE=$(BUILD_TYPE) \
	  -t $(DOCKER_IMAGE)-sim:$(DOCKER_TAG) .

# ─── Code Quality ──────────────────────────────────────────────────────────────

lint:
	@echo "--- cppcheck ---"
	find src/ \( -name "*.cpp" -o -name "*.hpp" \) \
	  | xargs cppcheck \
	      --enable=all \
	      --suppress=missingInclude \
	      --suppress=unmatchedSuppression \
	      --error-exitcode=1 \
	      --quiet
	@echo "--- cpplint ---"
	find src/ \( -name "*.cpp" -o -name "*.hpp" \) \
	  | xargs cpplint --quiet

format:
	find src/ \( -name "*.cpp" -o -name "*.hpp" -o -name "*.h" \) -print \
	  | xargs clang-format -i -style=file

format-check:
	find src/ \( -name "*.cpp" -o -name "*.hpp" -o -name "*.h" \) -print \
	  | xargs clang-format --dry-run --Werror -style=file

# ─── Deployment ────────────────────────────────────────────────────────────────

deploy:
	@echo "Deploying to $(ROBOT_USER)@$(ROBOT_HOST)..."
	rsync -avz --delete install/ \
	  $(ROBOT_USER)@$(ROBOT_HOST):/opt/mowgli_ros2/install/
	ssh $(ROBOT_USER)@$(ROBOT_HOST) "sudo systemctl restart mowgli"
	@echo "Deployment complete."

backup-maps:
	mkdir -p maps_backup/$$(date +%Y%m%d_%H%M%S)
	scp $(ROBOT_USER)@$(ROBOT_HOST):/opt/mowgli_ros2/maps/* \
	  maps_backup/$$(date +%Y%m%d_%H%M%S)/
	@echo "Maps backed up to maps_backup/$$(date +%Y%m%d_%H%M%S)/"
