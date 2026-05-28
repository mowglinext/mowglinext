#!/usr/bin/env python3
"""Compare yaw sources during a slow in-place rotation.

Publishes a slow constant wz to /cmd_vel_teleop and samples, over the
command window:
  - gyro_z integrated         (true chassis rotation, ground truth)
  - /odometry/filtered_map yaw (fused estimate — what Nav2/dock uses)
  - odom->base_footprint yaw   (dead-reckoning, fusion_graph local)
Reports each source's net yaw change + the implied rate, so we can see
whether the fused yaw tracks the gyro in sign and magnitude.

Usage: yaw_compare.py <wz> <dur>
"""
import sys
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import tf2_ros


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def unwrap(prev, cur):
    d = cur - prev
    while d > math.pi:
        d -= 2 * math.pi
    while d < -math.pi:
        d += 2 * math.pi
    return d


class YawCompare(Node):
    def __init__(self, wz, dur):
        super().__init__("yaw_compare")
        self.wz, self.dur = wz, dur
        self.pub = self.create_publisher(TwistStamped, "/cmd_vel_teleop", 10)
        self.create_subscription(Imu, "/imu/data", self.on_imu, 50)
        self.create_subscription(Odometry, "/odometry/filtered_map", self.on_fused, 10)
        self.tfbuf = tf2_ros.Buffer()
        self.tfl = tf2_ros.TransformListener(self.tfbuf, self)
        self.gyro_int = 0.0          # integrated gyro yaw
        self.last_imu_t = None
        self.fused_yaw0 = None
        self.fused_yaw_last = None
        self.fused_delta = 0.0
        self.odom_yaw0 = None
        self.odom_yaw_last = None
        self.odom_delta = 0.0
        self.t0 = self.now()
        self.timer = self.create_timer(0.05, self.tick)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_imu(self, m):
        t = self.now()
        if self.last_imu_t is not None and self.t0 + 0.5 <= t <= self.t0 + 0.5 + self.dur:
            self.gyro_int += m.angular_velocity.z * (t - self.last_imu_t)
        self.last_imu_t = t

    def on_fused(self, m):
        t = self.now()
        if not (self.t0 + 0.5 <= t <= self.t0 + 0.5 + self.dur):
            return
        y = yaw_of(m.pose.pose.orientation)
        if self.fused_yaw0 is None:
            self.fused_yaw0 = y
        else:
            self.fused_delta += unwrap(self.fused_yaw_last, y)
        self.fused_yaw_last = y

    def sample_odom_tf(self):
        try:
            tr = self.tfbuf.lookup_transform("odom", "base_footprint", rclpy.time.Time())
            y = yaw_of(tr.transform.rotation)
            if self.odom_yaw0 is None:
                self.odom_yaw0 = y
            else:
                self.odom_delta += unwrap(self.odom_yaw_last, y)
            self.odom_yaw_last = y
        except Exception:
            pass

    def tick(self):
        t = self.now() - self.t0
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        if 0.5 <= t < 0.5 + self.dur:
            msg.twist.angular.z = self.wz
            self.sample_odom_tf()
        self.pub.publish(msg)
        if t >= 0.5 + self.dur + 0.4:
            self.finish()

    def finish(self):
        print("=============== YAW COMPARE ===============")
        print(f"commanded wz = {self.wz:+.3f} rad/s for {self.dur:.1f}s")
        print(f"  gyro integrated  : {math.degrees(self.gyro_int):+7.1f} deg  "
              f"(rate {self.gyro_int/self.dur:+.3f} rad/s)  [GROUND TRUTH]")
        print(f"  fused map yaw    : {math.degrees(self.fused_delta):+7.1f} deg  "
              f"(rate {self.fused_delta/self.dur:+.3f} rad/s)")
        print(f"  odom->base yaw   : {math.degrees(self.odom_delta):+7.1f} deg  "
              f"(rate {self.odom_delta/self.dur:+.3f} rad/s)")
        # Diagnosis hints
        if self.gyro_int != 0:
            if self.fused_delta * self.gyro_int < 0:
                print("  >> FUSED YAW HAS WRONG SIGN vs gyro!")
            else:
                r = self.fused_delta / self.gyro_int
                print(f"  >> fused/gyro ratio = {r:.2f} (1.0 = perfect)")
            if self.odom_delta * self.gyro_int < 0:
                print("  >> ODOM (dead-reckoning) YAW HAS WRONG SIGN vs gyro!")
        print("===========================================")
        rclpy.shutdown()


def main():
    wz, dur = float(sys.argv[1]), float(sys.argv[2])
    rclpy.init()
    rclpy.spin(YawCompare(wz, dur))


if __name__ == "__main__":
    main()
