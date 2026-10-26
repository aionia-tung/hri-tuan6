"""Bài 06 - Phần 3: robot_behavior_node.

/detected_gesture (String) -> bắt cạnh + cooldown -> máy trạng thái -> /cmd_vel (Twist).

Cơ chế an toàn:
- Chỉ hành động khi nhãn ĐỔI (edge trigger) + cooldown cho cùng một cử chỉ,
  nên giữ tay lâu hoặc nhận diện chập chờn không kích hoạt lệnh liên tục.
- Hành vi tiến-lùi / vòng tròn chỉ chạy sau khi xác nhận bằng Thumb_Up.
- Open_Palm dừng ngay (không bị cooldown chặn); Closed_Fist huỷ và về IDLE.
- Watchdog: không nhận được /detected_gesture trong signal_timeout giây
  (node nhận diện tắt, mất webcam...) -> dừng robot, huỷ hành vi.
- Va chạm (hazard BUMP của Create 3) -> tạm dừng.
- Tiến độ đo bằng /odom nên quãng đường / góc đúng kể cả khi Gazebo chạy chậm.
"""

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String

from tb4_gesture.behaviors import PLANS, PlanRunner
from tb4_gesture.gesture_filter import EdgeTrigger
from tb4_gesture.state_machine import IDLE, BehaviorStateMachine

try:
    from irobot_create_msgs.msg import HazardDetection, HazardDetectionVector
except ImportError:  # chạy được cả khi không có irobot_create_msgs
    HazardDetection = HazardDetectionVector = None


class RobotBehaviorNode(Node):

    def __init__(self):
        super().__init__('robot_behavior_node')
        params = {
            'gesture_topic': '/detected_gesture',
            'cmd_vel_topic': '/cmd_vel',
            'odom_topic': '/odom',
            'control_rate': 20.0,          # Hz
            'signal_timeout': 1.0,         # s, mất tín hiệu -> dừng
            'gesture_cooldown': 1.5,       # s, chống kích hoạt lặp cùng cử chỉ
            'wave_requires_confirm': False,
            'set_safety_override': True,   # cho phép Create 3 lùi xa hơn giới hạn mặc định
            'stop_on_bump': True,
            # Tiến - lùi
            'fb_distance': 0.3, 'fb_speed': 0.2, 'fb_repeats': 4,
            # Vòng tròn
            'circle_radius': 0.5, 'circle_speed': 0.2, 'circle_turns': 1.0,
            # Chào lại
            'wave_angle': 0.5, 'wave_speed': 1.0, 'wave_cycles': 2,
        }
        for name, default in params.items():
            self.declare_parameter(name, default)
        p = lambda name: self.get_parameter(name).value  # noqa: E731
        self.p = p

        self.fsm = BehaviorStateMachine(wave_requires_confirm=bool(p('wave_requires_confirm')))
        self.trigger = EdgeTrigger(float(p('gesture_cooldown')),
                                   no_cooldown=('Open_Palm', 'Closed_Fist'))
        self.signal_timeout = float(p('signal_timeout'))

        self.cmd_pub = self.create_publisher(Twist, p('cmd_vel_topic'), 10)
        self.state_pub = self.create_publisher(String, '/robot_behavior_state', 10)
        self.create_subscription(String, p('gesture_topic'), self.on_gesture, 10)
        self.create_subscription(Odometry, p('odom_topic'), self.on_odom, qos_profile_sensor_data)
        if HazardDetectionVector is not None and p('stop_on_bump'):
            self.create_subscription(HazardDetectionVector, '/hazard_detection', self.on_hazard,
                                     qos_profile_sensor_data)

        self.runner = None          # PlanRunner của hành vi hiện tại
        self.moving = False
        self.zero_ticks = 0         # số chu kỳ còn phải gửi vận tốc 0 sau khi dừng
        self.pose, self.pose_time = None, 0.0
        self.last_gesture_time = None
        self.signal_ok = False
        self.last_message = 'Dang cho cu chi...'
        self.last_tick = time.monotonic()

        self.create_timer(1.0 / float(p('control_rate')), self.control_loop)
        self.create_timer(0.2, self.publish_state)

        self.override_client = self.create_client(SetParameters, '/motion_control/set_parameters')
        self.override_done = not p('set_safety_override')
        self.override_future = None
        self.override_timer = self.create_timer(2.0, self.try_safety_override)

        self.get_logger().info(
            'San sang. Wave: chao | Pointing_Up: chon tien-lui | Victory: chon vong tron | '
            'Thumb_Up: xac nhan | Open_Palm: dung | Closed_Fist: huy')

    # ------------------------------------------------------------------ input
    def on_gesture(self, msg):
        now = time.monotonic()
        self.last_gesture_time = now
        if not self.signal_ok:
            self.signal_ok = True
            self.get_logger().info('Da nhan tin hieu tu gesture_recognition_node')
        gesture = self.trigger.update(msg.data, now)
        if gesture is None:
            return
        command, message = self.fsm.handle(gesture)
        if message:
            self.report(f'[{gesture}] {message}')
        self.execute(command)

    def on_odom(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)
        self.pose_time = time.monotonic()

    def on_hazard(self, msg):
        for hazard in msg.detections:
            if hazard.type == HazardDetection.BUMP and self.moving:
                self.fsm.handle('Open_Palm')
                self.execute(('stop',))
                self.report('VA CHAM (bumper) -> tam dung. Closed_Fist de huy')
            elif hazard.type == HazardDetection.BACKUP_LIMIT and self.moving:
                self.get_logger().warn('Create 3 dang chan lui (backup_limit)',
                                       throttle_duration_sec=2.0)

    # ---------------------------------------------------------------- actions
    def execute(self, command):
        if command is None:
            return
        kind = command[0]
        if kind == 'start':
            self.runner = PlanRunner(self.make_plan(command[1]))
            self.moving = True
        elif kind == 'resume' and self.runner is not None:
            self.runner.reset_reference()
            self.moving = True
        elif kind in ('stop', 'cancel'):
            self.moving = False
            self.zero_ticks = 10
            if kind == 'cancel':
                self.runner = None
            self.cmd_pub.publish(Twist())   # dừng ngay, không chờ chu kỳ sau

    def make_plan(self, name):
        p = self.p
        if name == 'forward_back':
            return PLANS[name](p('fb_distance'), p('fb_speed'), int(p('fb_repeats')))
        if name == 'circle':
            return PLANS[name](p('circle_radius'), p('circle_speed'), p('circle_turns'))
        return PLANS[name](p('wave_angle'), p('wave_speed'), int(p('wave_cycles')))

    def control_loop(self):
        now = time.monotonic()
        dt, self.last_tick = now - self.last_tick, now

        # Watchdog mất tín hiệu điều khiển
        if self.signal_ok and now - self.last_gesture_time > self.signal_timeout:
            self.signal_ok = False
            self.trigger.update('None', now)
            if self.fsm.signal_lost() or self.moving:
                self.execute(('cancel',))
                self.report(f'MAT TIN HIEU > {self.signal_timeout:.1f} s -> dung robot, ve IDLE')

        if self.moving and self.runner is not None:
            pose = self.pose if now - self.pose_time < 0.5 else None
            v, w = self.runner.step(pose, dt)
            if self.runner.done:
                self.report(self.fsm.finished())
                self.runner = None
                self.execute(('stop',))
                return
            twist = Twist()
            twist.linear.x, twist.angular.z = float(v), float(w)
            self.cmd_pub.publish(twist)
        elif self.zero_ticks > 0:
            self.zero_ticks -= 1
            self.cmd_pub.publish(Twist())

    # ----------------------------------------------------------------- status
    def report(self, message):
        self.last_message = message
        self.get_logger().info(f'{message}  => {self.fsm.describe()}')

    def publish_state(self):
        text = self.fsm.describe()
        if self.runner is not None and self.runner.current is not None:
            text += f' [{self.runner.current.label}]'
        if not self.signal_ok and self.fsm.state == IDLE:
            text += ' | cho tin hieu cu chi'
        self.state_pub.publish(String(data=text))

    def try_safety_override(self):
        """Create 3 mặc định chặn lùi xa (không có cảm biến cliff phía sau).
        Đặt motion_control.safety_override = backup_only để lùi được 0.3 m."""
        if self.override_done:
            self.override_timer.cancel()
            return
        if self.override_future is not None:
            if not self.override_future.done():
                return
            result = self.override_future.result()
            if result is not None and result.results and result.results[0].successful:
                self.get_logger().info('Da dat motion_control safety_override = backup_only')
                self.override_done = True
                return
            self.override_future = None
        if not self.override_client.service_is_ready():
            self.get_logger().info('Cho /motion_control de dat safety_override...',
                                   throttle_duration_sec=10.0)
            return
        request = SetParameters.Request()
        request.parameters = [Parameter(
            name='safety_override',
            value=ParameterValue(type=ParameterType.PARAMETER_STRING, string_value='backup_only'))]
        self.override_future = self.override_client.call_async(request)

    def stop_robot(self):
        for _ in range(3):
            self.cmd_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = RobotBehaviorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.stop_robot()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
