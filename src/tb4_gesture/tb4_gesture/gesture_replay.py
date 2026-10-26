"""Công cụ kiểm thử: phát lại một kịch bản cử chỉ lên /detected_gesture.

Dùng để kiểm tra robot_behavior_node trong Gazebo khi không có webcam
(KHÔNG dùng trong video demo - demo dùng gesture_recognition_node + webcam).

Kịch bản là chuỗi "Nhan:giay,Nhan:giay,...", ví dụ:
  ros2 run tb4_gesture gesture_replay --ros-args -p script:="Wave:2,None:8"
Nhãn được publish 15 Hz giống gesture_recognition_node; hết kịch bản node
thoát (robot_behavior_node sẽ thấy mất tín hiệu và dừng robot).
"""

import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

DEFAULT_SCRIPT = ('None:2,Wave:2,None:10,Pointing_Up:1,None:1,Thumb_Up:1,None:30,'
                  'Victory:1,None:1,Thumb_Up:1,None:12,Open_Palm:1,None:2,Thumb_Up:1,None:30,'
                  'Pointing_Up:1,None:1,Thumb_Up:1,None:5,Closed_Fist:1,None:3')


class GestureReplay(Node):

    def __init__(self):
        super().__init__('gesture_replay')
        self.declare_parameter('script', DEFAULT_SCRIPT)
        self.declare_parameter('rate', 15.0)
        self.steps = []
        for item in self.get_parameter('script').value.split(','):
            label, seconds = item.strip().split(':')
            self.steps.append((label, float(seconds)))
        self.pub = self.create_publisher(String, '/detected_gesture', 10)
        self.index, self.step_start = 0, time.monotonic()
        self.finished = False
        self.create_timer(1.0 / self.get_parameter('rate').value, self.tick)
        self.get_logger().info(f'Kich ban {len(self.steps)} buoc, '
                               f'{sum(s for _, s in self.steps):.0f} s')
        self.announce()

    def announce(self):
        label, seconds = self.steps[self.index]
        self.get_logger().info(f'[{self.index + 1}/{len(self.steps)}] {label} trong {seconds} s')

    def tick(self):
        if self.index >= len(self.steps):
            return
        if time.monotonic() - self.step_start >= self.steps[self.index][1]:
            self.index += 1
            self.step_start = time.monotonic()
            if self.index >= len(self.steps):
                self.get_logger().info('Het kich ban - ngung publish')
                self.finished = True
                return
            self.announce()
        self.pub.publish(String(data=self.steps[self.index][0]))


def main(args=None):
    rclpy.init(args=args)
    node = GestureReplay()
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
