# Copyright 2026 Mowgli Project
#
# Licensed under the GNU GPL v3.

"""
Kinematic chassis driven by actual Webots wheel-position acquisitions.

ODE wheel-floor coupling on this PROTO pitches the chassis and loses most of
its forward motion. We retain the kinematic body workaround, but derive each
body increment from the actual wheel PositionSensors after the native physics
step. Command-derived body motion previously bypassed ros2_control's limits
and the wheel joints' dynamics, despite sharing /cmd_vel_wheels with them.

sim_actuation_node remains the sole firmware-model owner. ros2_control remains
the sole motor writer and owns command timeout, acceleration and velocity
limits. This plugin observes the resulting wheels; it neither writes motors
nor substitutes command-derived encoder observations. Measured residual wheel
motion during braking therefore remains body motion, even after a command has
expired. This is a kinematic no-slip chassis model, not physical stop evidence.

Ground truth and applied twist describe the same completed native interval.
The first finite wheel pair establishes an offset baseline without replaying
initial wheel angles. Invalid acquisitions hold the body and emit no packet;
a finite baseline followed by a positive native interval is required to resume.
Repeated native epochs cannot integrate or publish another observation.

The teleport, planar setVelocity and init-only resetPhysics safeguards remain
necessary: per-step resetPhysics recursively clears wheel velocities, while
omitting planar setVelocity lets gravity accumulate between teleports and
corrupts sensor sampling. Wheel-floor slip and collision response are not
modeled by this kinematic body; the joints retain their actual motor dynamics.
"""

from __future__ import annotations

import math

from geometry_msgs.msg import PoseStamped, TwistStamped
import rclpy


class KinematicDrive:
    """Integrate native wheel acquisitions into one coherent kinematic body."""

    # Must match the shipped PROTO and config_webots/ros2_control.yaml.
    # Positive PositionSensor increments on axis (0, -1, 0) mean forward.
    WHEEL_SEPARATION = 0.325
    WHEEL_RADIUS = 0.093

    def init(self, webots_node, properties):
        self.__supervisor = webots_node.robot
        self.__timestep_ms = int(self.__supervisor.getBasicTimeStep())
        self.__robot_node = self.__supervisor.getSelf()
        if self.__robot_node is None or self.__robot_node._ref is None:
            raise RuntimeError(
                'KinematicDrive: Supervisor.getSelf() returned a NULL node. '
                'The Robot must be supervisor=TRUE in both the PROTO and the '
                'world instantiation.'
            )

        self.__translation_field = self.__robot_node.getField('translation')
        self.__rotation_field = self.__robot_node.getField('rotation')
        if self.__translation_field is None or self.__rotation_field is None:
            raise RuntimeError(
                'KinematicDrive: failed to resolve translation/rotation '
                'fields on the Robot node.'
            )

        init_xyz = self.__translation_field.getSFVec3f()
        init_axis_angle = self.__rotation_field.getSFRotation()
        sign = 1.0 if init_axis_angle[2] >= 0 else -1.0
        self.__x, self.__y, self.__z = init_xyz
        self.__yaw = sign * init_axis_angle[3]

        # These are the PROTO's actual hinge sensors, also read by
        # webots_ros2_control. Never obtain or write the wheel motors here.
        self.__wheel_sensors = []
        for name in ('left_wheel_sensor', 'right_wheel_sensor'):
            sensor = self.__supervisor.getDevice(name)
            if sensor is None or sensor.getType() != sensor.ROTATIONAL:
                raise RuntimeError(f'KinematicDrive: missing rotational PositionSensor {name!r}')
            sensor.enable(self.__timestep_ms)
            self.__wheel_sensors.append(sensor)
        self.__wheel_baseline = None
        self.__wheel_fault = False

        if not rclpy.ok():
            rclpy.init(args=None)
        node_name = self.__supervisor.getName().replace(' ', '_').replace('-', '_')
        self.__node = rclpy.create_node(f'{node_name}_kinematic_drive')
        self.__gt_pose_pub = self.__node.create_publisher(
            PoseStamped, '/sim/ground_truth_pose', 10
        )
        self.__applied_twist_pub = self.__node.create_publisher(
            TwistStamped, '/sim/applied_twist', 10
        )

        # ONCE only: repeated resetPhysics recursively zeroes the wheel
        # HingeJoint velocities and silently defeats the motor controller.
        self.__robot_node.resetPhysics()
        self.__node.get_logger().info(
            f"KinematicDrive initialised. Robot='{self.__supervisor.getName()}', "
            f'actual wheel feedback, timestep={self.__timestep_ms} ms, '
            f'wheel_separation={self.WHEEL_SEPARATION} m, '
            f'wheel_radius={self.WHEEL_RADIUS} m, '
            f'initial_pose=(x={self.__x:.3f}, y={self.__y:.3f}, '
            f'z={self.__z:.3f}, yaw={self.__yaw:.3f}).'
        )

    def __place_body(self, vx, wz):
        self.__translation_field.setSFVec3f([self.__x, self.__y, self.__z])
        self.__rotation_field.setSFRotation([0.0, 0.0, 1.0, self.__yaw])
        # Zero vertical velocity prevents accumulated gravity from sinking
        # the attached sensors below the floor between teleports. Unlike
        # resetPhysics, setVelocity does not zero child hinge velocities.
        self.__robot_node.setVelocity([
            vx * math.cos(self.__yaw), vx * math.sin(self.__yaw),
            0.0, 0.0, 0.0, wz,
        ])

    def step(self):
        # The driver has already performed robot.step() and ros2_control's
        # read/update/write before this plugin runs. Both read the same native
        # position acquisition; motor writes take effect in the next physics
        # interval. No ROS callback or command receipt defines body motion.
        native_s = self.__supervisor.getTime()
        left, right = (sensor.getValue() for sensor in self.__wheel_sensors)
        if not all(math.isfinite(value) for value in (native_s, left, right)) or native_s < 0:
            self.__wheel_baseline = None
            if not self.__wheel_fault:
                self.__node.get_logger().warning(
                    'KinematicDrive: invalid native wheel acquisition; holding body '
                    'without publishing until wheel feedback is rebaselined.'
                )
            self.__wheel_fault = True
            self.__place_body(0.0, 0.0)
            return

        native_ns = round(native_s * 1_000_000_000)
        previous = self.__wheel_baseline
        if previous is not None and native_ns == previous[2]:
            # One native epoch is one acquisition. Do not replace its payload
            # or change the modeled velocity due to a repeated step call.
            return
        self.__wheel_baseline = (left, right, native_ns)
        self.__wheel_fault = False
        if previous is None or native_ns < previous[2]:
            # Initial offset / clock rewind is not measured traveled distance.
            self.__place_body(0.0, 0.0)
            return

        dt = (native_ns - previous[2]) / 1_000_000_000
        left_distance = (left - previous[0]) * self.WHEEL_RADIUS
        right_distance = (right - previous[1]) * self.WHEEL_RADIUS
        distance = 0.5 * (left_distance + right_distance)
        turn = (right_distance - left_distance) / self.WHEEL_SEPARATION
        # Exact differential-drive SE(2) increment, including the sinc term
        # absent from command-based midpoint integration. The zero-turn limit
        # is continuous and needs no widened comparison tolerance.
        half = 0.5 * turn
        sinc = math.sin(half) / half if half else 1.0
        self.__x += distance * sinc * math.cos(self.__yaw + half)
        self.__y += distance * sinc * math.sin(self.__yaw + half)
        self.__yaw += turn
        if self.__yaw > math.pi or self.__yaw < -math.pi:
            self.__yaw = math.atan2(math.sin(self.__yaw), math.cos(self.__yaw))
        vx, wz = distance / dt, turn / dt
        self.__place_body(vx, wz)

        gt = PoseStamped()
        gt.header.stamp.sec = native_ns // 1_000_000_000
        gt.header.stamp.nanosec = native_ns % 1_000_000_000
        gt.header.frame_id = 'map'
        gt.pose.position.x = self.__x
        gt.pose.position.y = self.__y
        gt.pose.position.z = self.__z
        gt.pose.orientation.z = math.sin(self.__yaw * 0.5)
        gt.pose.orientation.w = math.cos(self.__yaw * 0.5)
        self.__gt_pose_pub.publish(gt)

        # Actual interval twist, not a command or another actuation authority.
        applied = TwistStamped()
        applied.header.stamp = gt.header.stamp
        applied.header.frame_id = 'base_footprint'
        applied.twist.linear.x = vx
        applied.twist.angular.z = wz
        self.__applied_twist_pub.publish(applied)
