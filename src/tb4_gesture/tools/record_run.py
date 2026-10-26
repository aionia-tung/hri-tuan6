#!/usr/bin/env python3
"""Ghi /odom, /cmd_vel, /detected_gesture, /robot_behavior_state ra CSV để vẽ đồ thị báo cáo.

  python3 tools/record_run.py run.csv        (Ctrl+C để dừng)
  python3 tools/plot_run.py run.csv docs/images
"""

import csv
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String


class Recorder(Node):

    def __init__(self, path):
        super().__init__('run_recorder')
        self.file = open(path, 'w', newline='')
        self.csv = csv.writer(self.file)
        self.csv.writerow(['t', 'x', 'y', 'yaw', 'v_cmd', 'w_cmd', 'gesture', 'state'])
        self.t0 = time.monotonic()
        self.cmd, self.gesture, self.state = (0.0, 0.0), 'None', ''
        self.create_subscription(Odometry, '/odom', self.on_odom, qos_profile_sensor_data)
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd, 10)
        self.create_subscription(String, '/detected_gesture', self.on_gesture, 10)
        self.create_subscription(String, '/robot_behavior_state', self.on_state, 10)
        self.last = 0.0

    def on_cmd(self, msg):
        self.cmd = (msg.linear.x, msg.angular.z)

    def on_gesture(self, msg):
        self.gesture = msg.data

    def on_state(self, msg):
        self.state = msg.data

    def on_odom(self, msg):
        t = time.monotonic() - self.t0
        if t - self.last < 0.05:
            return
        self.last = t
        q = msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        p = msg.pose.pose.position
        self.csv.writerow([f'{t:.3f}', f'{p.x:.4f}', f'{p.y:.4f}', f'{yaw:.4f}',
                           f'{self.cmd[0]:.3f}', f'{self.cmd[1]:.3f}', self.gesture, self.state])


def main():
    rclpy.init()
    node = Recorder(sys.argv[1] if len(sys.argv) > 1 else 'run.csv')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.file.close()


if __name__ == '__main__':
    main()
