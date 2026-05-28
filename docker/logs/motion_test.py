#!/usr/bin/env python3
"""Commanded-vs-actual motion diagnostic.

Publishes a constant TwistStamped to /cmd_vel_teleop for `dur` seconds,
while recording the commanded value that actually reaches the motors
(/cmd_vel, post twist-mux) and the real chassis response (gyro_z from
/imu/data, wheel twist from /wheel_odom, fused pose from
/odometry/filtered_map). Reports steady-state means over the middle of
the command window (ignoring the first/last 0.8 s ramp) plus the
integrated yaw/translation, so we can separate a genuine
commanded-vs-actual scaling problem from pub start/stop latency.

Usage: motion_test.py <vx> <wz> <dur>
"""
import sys
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class MotionTest(Node):
    def __init__(self, vx, wz, dur):
        super().__init__("motion_test")
        self.vx, self.wz, self.dur = vx, wz, dur
        self.pub = self.create_publisher(TwistStamped, "/cmd_vel_teleop", 10)
        self.create_subscription(TwistStamped, "/cmd_vel", self.on_cmd, 10)
        self.create_subscription(Imu, "/imu/data", self.on_imu, 50)
        self.create_subscription(Odometry, "/odometry/filtered_map", self.on_odom, 10)
        self.cmd = []      # (t, x, z) reaching motors
        self.gyro = []     # (t, gz)
        self.odom = []     # (t, x, y, yaw)
        self.t0 = self.get_clock().now().nanoseconds * 1e-9
        self.timer = self.create_timer(0.05, self.tick)  # 20 Hz publish

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9 - self.t0

    def on_cmd(self, m):
        self.cmd.append((self.now(), m.twist.linear.x, m.twist.angular.z))

    def on_imu(self, m):
        self.gyro.append((self.now(), m.angular_velocity.z))

    def on_odom(self, m):
        p = m.pose.pose
        self.odom.append((self.now(), p.position.x, p.position.y, yaw_of(p.orientation)))

    def tick(self):
        t = self.now()
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        # 0.5 s warmup zero, then command for dur, then stop.
        if 0.5 <= t < (0.5 + self.dur):
            msg.twist.linear.x = self.vx
            msg.twist.angular.z = self.wz
        self.pub.publish(msg)
        if t >= (0.5 + self.dur + 0.6):
            self.finish()

    def finish(self):
        # Steady-state window: middle of the command interval.
        lo, hi = 0.5 + 0.8, 0.5 + self.dur - 0.8
        cmd_w = [(x, z) for (tt, x, z) in self.cmd if lo <= tt <= hi]
        gyro_w = [g for (tt, g) in self.gyro if lo <= tt <= hi]
        cx = sum(x for x, _ in cmd_w) / len(cmd_w) if cmd_w else float("nan")
        cz = sum(z for _, z in cmd_w) / len(cmd_w) if cmd_w else float("nan")
        gz = sum(gyro_w) / len(gyro_w) if gyro_w else float("nan")
        # Integrated response from odom over the command window.
        od = [(tt, x, y, yw) for (tt, x, y, yw) in self.odom if 0.5 <= tt <= 0.5 + self.dur + 0.5]
        if len(od) >= 2:
            dyaw = od[-1][3] - od[0][3]
            dist = math.hypot(od[-1][1] - od[0][1], od[-1][2] - od[0][2])
            dt = od[-1][0] - od[0][0]
        else:
            dyaw = dist = dt = float("nan")
        print("================ MOTION TEST ================")
        print(f"commanded:        vx={self.vx:.3f}  wz={self.wz:.3f}  dur={self.dur:.1f}s")
        print(f"reaching motors:  vx={cx:.3f}  wz={cz:.3f}  (mean of /cmd_vel, steady window)")
        print(f"actual gyro_z:    {gz:.3f} rad/s   (mean, steady window)")
        if not math.isnan(gz) and abs(cz) > 1e-3:
            print(f"  -> rotation ratio actual/commanded(out) = {gz/cz:.2f}")
        if not math.isnan(dyaw):
            print(f"integrated fused: dyaw={math.degrees(dyaw):.1f} deg  dist={dist:.3f} m  over {dt:.1f}s")
            if abs(self.wz) > 1e-3:
                print(f"  -> effective yaw rate = {dyaw/dt:.3f} rad/s vs commanded {self.wz:.3f}  ({(dyaw/dt)/self.wz*100:.0f}%)")
            if abs(self.vx) > 1e-3:
                print(f"  -> effective speed    = {dist/dt:.3f} m/s vs commanded {abs(self.vx):.3f}  ({(dist/dt)/abs(self.vx)*100:.0f}%)")
        print(f"samples: cmd={len(self.cmd)} gyro={len(self.gyro)} odom={len(self.odom)}")
        print("=============================================")
        rclpy.shutdown()


def main():
    vx, wz, dur = float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3])
    rclpy.init()
    node = MotionTest(vx, wz, dur)
    rclpy.spin(node)


if __name__ == "__main__":
    main()
