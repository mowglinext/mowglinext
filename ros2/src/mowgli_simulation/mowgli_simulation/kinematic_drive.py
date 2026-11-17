# Copyright 2026 Mowgli Project
#
# Licensed under the GNU GPL v3.

"""
Observe completed native body intervals (legacy plugin name retained).

Native motors driven by ros2_control and sim_actuation_node are the sole plant
actuation path. No Supervisor body writes, encoder integration or second motor
writer belongs here. Native slip/contact response can differ from wheel odom.

Truth is the Solid origin and full world-from-body orientation in the garden's
ENU/map frame (body axes forward/left/up). When level, this origin coincides
with base_footprint. Applied twist is the SE(2) logarithm of consecutive XY and
projected body-X yaw observations, divided by their native interval. Its x/y
components are in the moving yaw-only base_footprint frame, including lateral
slip; angular.z is the shortest signed yaw increment per second. This constant
interval twist reconstructs the observed planar displacement. It is neither
an endpoint instantaneous velocity nor a full 3D IMU model. In particular,
getVelocity() reports ODE COM velocity, not Solid-origin velocity, and is unused.

Both packets use the native epoch ending that interval. First acquisition,
rewind and recovery after invalid data need a finite pose baseline then a new
positive interval; duplicates cannot replace an observation. Invalid data emits
no packets and never changes the plant. No physical stop evidence is implied.
"""

from __future__ import annotations

import math

from geometry_msgs.msg import PoseStamped, TwistStamped
import rclpy


def _quaternion(r):
    """Convert a native row-major world-from-body rotation to ROS x/y/z/w."""
    # Reject malformed rotations instead of manufacturing a plausible attitude.
    # Native rotation roundoff is allowed; this is numeric validity, not a
    # motion/tilt limit. Reflections and degenerate matrices are not rotations.
    for i in range(3):
        for j in range(i, 3):
            dot = sum(r[3*i+k] * r[3*j+k] for k in range(3))
            if not math.isclose(dot, float(i == j), rel_tol=0.0, abs_tol=1e-9):
                raise ValueError('non-orthonormal native orientation')
    determinant = (r[0]*(r[4]*r[8]-r[5]*r[7]) - r[1]*(r[3]*r[8]-r[5]*r[6])
                   + r[2]*(r[3]*r[7]-r[4]*r[6]))
    if determinant <= 0.0:
        raise ValueError('reflected native orientation')

    # Choose the largest quaternion component to avoid cancellation near pi.
    trace = r[0] + r[4] + r[8]
    if trace > 0.0:
        s = 2.0 * math.sqrt(1.0 + trace)
        q = ((r[7]-r[5])/s, (r[2]-r[6])/s, (r[3]-r[1])/s, s/4.0)
    else:
        i = max(range(3), key=lambda k: r[4*k])
        j, k = (i+1) % 3, (i+2) % 3
        s = 2.0 * math.sqrt(1.0 + r[4*i] - r[4*j] - r[4*k])
        q = [0.0] * 4
        q[i] = s / 4.0
        q[j] = (r[3*j+i] + r[3*i+j]) / s
        q[k] = (r[3*k+i] + r[3*i+k]) / s
        q[3] = (r[3*k+j] - r[3*j+k]) / s
    norm = math.hypot(*q)
    return tuple(value / norm for value in q)


class KinematicDrive:
    """Publish native body observations without actuating the simulated plant."""

    def init(self, webots_node, properties):
        self.__supervisor = webots_node.robot
        self.__robot_node = self.__supervisor.getSelf()
        if self.__robot_node is None or self.__robot_node._ref is None:
            raise RuntimeError(
                'KinematicDrive: Supervisor.getSelf() returned a NULL node. '
                'The Robot must be supervisor=TRUE in both the PROTO and the '
                'world instantiation.'
            )
        self.__pose_baseline = None
        self.__native_epoch = None
        self.__pose_fault = False

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
        self.__node.get_logger().info(
            f"KinematicDrive observing native body. Robot='{self.__supervisor.getName()}'."
        )

    def __invalid_acquisition(self):
        self.__pose_baseline = None
        if not self.__pose_fault:
            self.__node.get_logger().warning(
                'KinematicDrive: invalid native body acquisition; no observation '
                'until a finite pose baseline and new native interval are available.'
            )
        self.__pose_fault = True

    def __publish(self, publisher, message):
        try:
            publisher.publish(message)
        except RuntimeError:
            # rclpy RCLError and InvalidHandle both derive from RuntimeError.
            # Context shutdown can race either publish after step's entry check.
            if self.__node.context.ok():
                raise
            return False
        return True

    def step(self):
        if not self.__node.context.ok():
            return
        # robot.step() has completed. Controller motor writes affect the next
        # interval; neither their command nor wheel history defines body truth.
        native_s = self.__supervisor.getTime()
        if not math.isfinite(native_s) or native_s < 0 or not math.isfinite(native_s * 1e9):
            self.__invalid_acquisition()
            return
        native_ns = round(native_s * 1_000_000_000)
        if native_ns == self.__native_epoch:
            return
        if self.__native_epoch is not None and native_ns < self.__native_epoch:
            self.__pose_baseline = None
        self.__native_epoch = native_ns

        position = tuple(self.__robot_node.getPosition())
        rotation = tuple(self.__robot_node.getOrientation())
        if (len(position) != 3 or len(rotation) != 9
                or not all(math.isfinite(value) for value in (*position, *rotation))):
            self.__invalid_acquisition()
            return
        try:
            quaternion = _quaternion(rotation)
        except ValueError:
            self.__invalid_acquisition()
            return
        yaw = math.atan2(rotation[3], rotation[0])
        previous = self.__pose_baseline
        self.__pose_baseline = (position, yaw, native_ns)
        self.__pose_fault = False
        if previous is None:
            return

        dt = (native_ns - previous[2]) / 1_000_000_000
        dx, dy = position[0] - previous[0][0], position[1] - previous[0][1]
        c, s = math.cos(previous[1]), math.sin(previous[1])
        local_x, local_y = c*dx + s*dy, -s*dx + c*dy
        turn = math.atan2(math.sin(yaw - previous[1]), math.cos(yaw - previous[1]))
        half = 0.5 * turn
        # Inverse SE(2) left Jacobian, continuous at zero turn. Retain native
        # lateral motion instead of imposing the wheel model's no-slip rule.
        a = half / math.tan(half) if half else 1.0
        vx, vy = (a*local_x + half*local_y) / dt, (-half*local_x + a*local_y) / dt
        wz = turn / dt
        if not all(math.isfinite(value) for value in (vx, vy, wz)):
            self.__invalid_acquisition()
            return

        gt = PoseStamped()
        gt.header.stamp.sec = native_ns // 1_000_000_000
        gt.header.stamp.nanosec = native_ns % 1_000_000_000
        gt.header.frame_id = 'map'
        gt.pose.position.x, gt.pose.position.y, gt.pose.position.z = position
        (gt.pose.orientation.x, gt.pose.orientation.y,
         gt.pose.orientation.z, gt.pose.orientation.w) = quaternion
        if not self.__publish(self.__gt_pose_pub, gt):
            return

        applied = TwistStamped()
        applied.header.stamp = gt.header.stamp
        applied.header.frame_id = 'base_footprint'
        applied.twist.linear.x, applied.twist.linear.y = vx, vy
        applied.twist.angular.z = wz
        self.__publish(self.__applied_twist_pub, applied)
