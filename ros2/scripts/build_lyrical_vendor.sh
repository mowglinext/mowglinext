#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-or-later
# Build the unreleased ROS dependencies without modifying vendored submodules.
# Prerequisite apt packages are installed by Docker/CI; this script never uses sudo.
set -eo pipefail
if [ "$(id -u)" -eq 0 ]; then
  echo "Run vendor builds as the project user, not root" >&2
  exit 1
fi
prefix="${1:-/opt/lyrical_vendor}"
# Docker supplies a dedicated cache directory; standalone/CI calls use a temp tree.
work="${VENDOR_WORKSPACE:-}"
if [ -z "$work" ]; then
  work="$(mktemp -d)"
  trap 'rm -rf "$work"' EXIT
fi
source /opt/ros/lyrical/setup.bash
mkdir -p "$prefix" "$work/src"

checkout() {
  git init -q "$work/src/$1"
  git -C "$work/src/$1" fetch --depth 1 "$2" "$3"
  git -C "$work/src/$1" checkout -q --force --detach FETCH_HEAD
}
checkout grid_map https://github.com/ANYbotics/grid_map.git 7ae24725d7effa0c22a23524b91b46bada5b2728
checkout beluga https://github.com/Ekumen-OS/beluga.git 22adc90e08cc7229cde042a756f044b908600fed
checkout sophus https://github.com/strasdat/Sophus.git de0f8d3d92bf776271e16de56d1803940ebccab9
checkout navigation2 https://github.com/ros-navigation/navigation2.git a6354f3f39d12c6e5c3e323f8021c831752edb23

# grid_map_ros still uses the ament macro removed in Lyrical. Patch only the
# pinned build copy, linking exported targets and the legacy grid_map_cv export.
python3 - "$work/src/grid_map/grid_map_ros/CMakeLists.txt" <<'PY'
from pathlib import Path
import sys
p = Path(sys.argv[1])
s = p.read_text()
old = 'ament_target_dependencies(${PROJECT_NAME} SYSTEM\n  ${dependencies}\n)'
new = '''foreach(dep IN LISTS dependencies)
  target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${${dep}_INCLUDE_DIRS})
  if(${dep}_TARGETS)
    target_link_libraries(${PROJECT_NAME} ${${dep}_TARGETS})
  else()
    target_link_libraries(${PROJECT_NAME} ${${dep}_LIBRARIES})
  endif()
endforeach()'''
assert old in s, 'grid_map patch no longer matches the pinned source'
p.write_text(s.replace(old, new))
PY

# Keep the shim's local-costmap collision simulation on the commanded arc.
# This patch is tied to the pinned Nav2 source above and participates in the
# Docker/CI vendor cache key through this script. RPP remains apt-provided.
python3 - "$work/src/navigation2/nav2_rotation_shim_controller" <<'PY_SHIM'
from pathlib import Path
import sys

root = Path(sys.argv[1])
source = root / 'src/nav2_rotation_shim_controller.cpp'
header = root / 'include/nav2_rotation_shim_controller/nav2_rotation_shim_controller.hpp'
s = source.read_text()
h = header.read_text()

def replace_once(text, old, new):
    assert text.count(old) == 1, f'RotationShim patch no longer matches: {old!r}'
    return text.replace(old, new)

# Goal alignment has no rough-heading release allowance. Initial alignment
# retains the existing entry/disengage hysteresis and releases at disengage.
# The two call sites are identical; assert their context before changing only goal alignment.
old = '''        auto cmd_vel = computeRotateToHeadingCommand(angular_distance_to_heading, pose, velocity);
        last_angular_vel_ = cmd_vel.twist.angular.z;
        return cmd_vel;
      }
    } catch'''
new = old.replace('pose, velocity);', 'pose, velocity, true);')
s = replace_once(s, old, new)
for text, path in [(s, source), (h, header)]:
    old = 'const geometry_msgs::msg::Twist & velocity)'
    new = ('const geometry_msgs::msg::Twist & velocity, const bool rotate_to_goal)' if path == source
           else 'const geometry_msgs::msg::Twist & velocity, const bool rotate_to_goal = false)')
    text = replace_once(text, old, new)
    # This suffix also belongs to other methods, so anchor the collision signature.
    signature = '''const double & angular_distance_to_heading,
  const geometry_msgs::msg::PoseStamped & pose)''' if path == source else '''const double & angular_distance_to_heading,
    const geometry_msgs::msg::PoseStamped & pose)'''
    text = replace_once(text, signature, signature[:-1] + ', const bool rotate_to_goal)')
    if path == source:
        s = text
    else:
        h = text
s = replace_once(s, 'isCollisionFree(cmd_vel, angular_distance_to_heading, pose);',
                 'isCollisionFree(cmd_vel, angular_distance_to_heading, pose, rotate_to_goal);')
s = replace_once(s, '''  double remaining_rotation_before_thresh =
    fabs(angular_distance_to_heading) - params_->angular_dist_threshold;''', '''  const double remaining_rotation_before_thresh = std::max(
    0.0, fabs(angular_distance_to_heading) -
    (rotate_to_goal ? 0.0 : params_->angular_disengage_threshold));
  const double heading_sign = angular_distance_to_heading > 0.0 ? 1.0 : -1.0;''')
s = replace_once(s, '''    // Stop simulating past the point it would be passed onto the primary controller
    if (angles::shortest_angular_distance(yaw, initial_yaw) >= remaining_rotation_before_thresh) {
      break;
    }

''', '')
s = replace_once(s, '''    if (footprint_cost >= params_->max_cost_threshold) {
      throw nav2_core::NoValidControl("RotationShimController detected collision ahead!");
    }
''', '''    if (footprint_cost >= params_->max_cost_threshold) {
      throw nav2_core::NoValidControl("RotationShimController detected collision ahead!");
    }

    // Check the first command interval and the sample reaching the release
    // boundary before stopping. Opposite-sign slew is real motion, not progress
    // toward release, and must retain the complete simulation horizon.
    if (heading_sign * cmd_vel.twist.angular.z * simulated_time >=
      remaining_rotation_before_thresh)
    {
      break;
    }
''')
# A collision refusal must reach controller_server. Falling through to RPP
# can replace the refused rotation with an unchecked forward proposal.
old = '    } catch (const std::runtime_error & e) {'
assert s.count(old) == 2, 'RotationShim collision catch sites changed'
s = s.replace(old, '''    } catch (const nav2_core::NoValidControl &) {
      throw;
    } catch (const std::runtime_error & e) {''')
source.write_text(s)
header.write_text(h)
PY_SHIM

cmake -S "$work/src/sophus" -B "$work/sophus-build" \
  -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
  -DCMAKE_INSTALL_PREFIX="$prefix" -DBUILD_SOPHUS_TESTS=OFF \
  -DBUILD_SOPHUS_EXAMPLES=OFF -DSOPHUS_USE_BASIC_LOGGING=ON
cmake --build "$work/sophus-build" --parallel "${BUILD_JOBS:-3}"
cmake --install "$work/sophus-build"
export CMAKE_PREFIX_PATH="$prefix:${CMAKE_PREFIX_PATH:-}"
cd "$work"
# GitHub's `ubuntu-26.04` runner is Ubuntu's amd64v3 variant: GCC 15 targets
# x86-64-v3 (AVX2) by default, which turns on Eigen code paths that GCC 15 +
# Eigen 3.4.0 do not compile warning-free, and grid_map (grid_map_cmake_helpers)
# and Nav2 (nav2_package) hardcode -Werror. Inert on baseline amd64 and arm64.
#  * -Wno-error=array-bounds: middle-end false positive on Eigen's AVX loads of
#    fixed-size vectors (grid_map::Position in GridMap.cpp); NOT suppressed by
#    system-header status. Same flag as the GTSAM recipe in ros2-ci.yml. The
#    `-Wno-error=` form is order-independent w.r.t. the later -Werror.
#  * -isystem /usr/include/eigen3: nav2_smac_planner reaches Eigen through OMPL
#    with a plain -I, so front-end warnings inside Eigen itself become errors
#    (F32ToBf16's unused `r` under EIGEN_VECTORIZE_AVX2, AVX/PacketMath.h:1275).
#    GCC de-duplicates -I against -isystem by directory identity, so this marks
#    Eigen a system header everywhere in the vendor build — exactly what
#    grid_map already gets through its SYSTEM include export.
MAKEFLAGS="-j${BUILD_JOBS:-3}" colcon build --merge-install --install-base "$prefix" \
  --base-paths src/grid_map src/beluga/beluga src/beluga/beluga_ros src/navigation2/nav2_smac_planner \
  src/navigation2/nav2_rotation_shim_controller \
  --packages-up-to grid_map_ros beluga_ros nav2_smac_planner nav2_rotation_shim_controller --parallel-workers 2 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_STANDARD=20 -DBUILD_TESTING=OFF \
  -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
  "-DCMAKE_CXX_FLAGS=-Wno-error=array-bounds -isystem /usr/include/eigen3"
