"""ROS 2 node: drive to a goal with gap-following steering and a selectable speed policy.

Subscribes : /scan                                   sensor_msgs/LaserScan
             /diff_drive_controller/odom             nav_msgs/Odometry
Publishes  : /diff_drive_controller/cmd_vel geometry_msgs/TwistStamped
             /adaptive_speed/debug                   std_msgs/Float32MultiArray
                 [v_cmd, d_front, clutter, prox, dens, turn]
Parameters : mode        'adaptive' | 'fixed'
             fixed_speed m/s used when mode == 'fixed'
             goal_x, goal_y   goal in the odom frame (robot starts at the odom origin)
             log_csv     optional path; one row per control step for later plotting
"""
import csv
import math

import numpy as np
import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray

from adaptive_speed_nav.speed_policy import AdaptiveSpeed, FixedSpeed
from adaptive_speed_nav.steering import Steering

RATE_HZ = 20.0


class Navigator(Node):
    def __init__(self):
        super().__init__('navigator')
        self.declare_parameter('mode', 'adaptive')
        self.declare_parameter('fixed_speed', 1.0)
        self.declare_parameter('goal_x', 10.0)
        self.declare_parameter('goal_y', 10.0)
        self.declare_parameter('goal_tolerance', 0.4)
        self.declare_parameter('log_csv', '')

        mode = self.get_parameter('mode').value
        self.policy = (AdaptiveSpeed() if mode == 'adaptive'
                       else FixedSpeed(self.get_parameter('fixed_speed').value))
        self.steering = Steering()
        self.goal = (self.get_parameter('goal_x').value, self.get_parameter('goal_y').value)
        self.tol = self.get_parameter('goal_tolerance').value

        self.scan = None
        self.pose = None          # x, y, yaw in the odom frame
        self.done = False
        self.t0 = None

        # sensor-data QoS (best effort) connects to both best-effort and reliable publishers
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/diff_drive_controller/odom', self.on_odom,
                                 qos_profile_sensor_data)
        # Jazzy's diff_drive_controller listens for TwistStamped on ~/cmd_vel
        self.cmd_pub = self.create_publisher(
            TwistStamped, '/diff_drive_controller/cmd_vel', 10)
        self.dbg_pub = self.create_publisher(Float32MultiArray, '/adaptive_speed/debug', 10)
        self.create_timer(1.0 / RATE_HZ, self.control)

        path = self.get_parameter('log_csv').value
        self.csv = None
        if path:
            self.csv_file = open(path, 'w', newline='')
            self.csv = csv.writer(self.csv_file)
            self.csv.writerow(['t', 'x', 'y', 'v_cmd', 'w_cmd', 'd_front', 'min_range'])
        self.get_logger().info(f'navigator started: mode={mode} goal={self.goal}')

    def on_scan(self, msg: LaserScan):
        r = np.array(msg.ranges, dtype=float)
        r[~np.isfinite(r)] = msg.range_max
        r = np.clip(r, msg.range_min, msg.range_max)
        angles = msg.angle_min + msg.angle_increment * np.arange(len(r))
        self.scan = (r, angles)

    def on_odom(self, msg: Odometry):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)

    def publish_cmd(self, v, w):
        t = TwistStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.twist.linear.x, t.twist.angular.z = float(v), float(w)
        self.cmd_pub.publish(t)

    def control(self):
        if self.done:
            return
        if self.scan is None or self.pose is None:
            self.get_logger().info(
                f'waiting for data: scan={"ok" if self.scan is not None else "MISSING"} '
                f'odom={"ok" if self.pose is not None else "MISSING"}',
                throttle_duration_sec=3.0)
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        self.t0 = self.t0 or now
        x, y, yaw = self.pose
        dist = math.hypot(self.goal[0] - x, self.goal[1] - y)
        self.get_logger().info(
            f'pose=({x:.2f}, {y:.2f}, yaw {yaw:.2f}) goal_dist={dist:.2f}',
            throttle_duration_sec=2.0)
        if dist < self.tol:
            self.publish_cmd(0.0, 0.0)
            self.done = True
            self.get_logger().info(
                f'Goal reached in {now - self.t0:.1f} s (pose=({x:.2f}, {y:.2f}), '
                f'dist={dist:.2f}, tol={self.tol})')
            if self.csv:
                self.csv_file.close()
            return

        ranges, angles = self.scan
        bearing = math.atan2(self.goal[1] - y, self.goal[0] - x) - yaw
        bearing = (bearing + math.pi) % (2 * math.pi) - math.pi
        w = self.steering.compute(ranges, angles, bearing)
        v, info = self.policy.compute(ranges, angles, w, 1.0 / RATE_HZ)
        self.publish_cmd(v, w)

        d_front = info.get('d_front', float(ranges.min()))
        dbg = Float32MultiArray()
        dbg.data = [float(v), float(d_front), float(info.get('clutter', 0.0)),
                    float(info.get('prox', 1.0)), float(info.get('dens', 1.0)),
                    float(info.get('turn', 1.0))]
        self.dbg_pub.publish(dbg)
        if self.csv:
            self.csv.writerow([f'{now - self.t0:.2f}', f'{x:.3f}', f'{y:.3f}', f'{v:.3f}',
                               f'{w:.3f}', f'{d_front:.3f}', f'{ranges.min():.3f}'])


def main(args=None):
    rclpy.init(args=args)
    node = Navigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    try:
        node.publish_cmd(0.0, 0.0)      # best-effort stop; the context may already be gone
    except Exception:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
