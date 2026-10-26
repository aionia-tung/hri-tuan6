"""Bài 06 - Phần 2: gesture_recognition_node.

Webcam (OpenCV) -> MediaPipe Gesture Recognizer -> lọc nhiễu -> /detected_gesture.

- Đọc ảnh webcam bằng cv2.VideoCapture, lật gương để người dùng thấy tự nhiên.
- MediaPipe Gesture Recognizer (chế độ VIDEO) trả về 21 landmark bàn tay + nhãn
  cử chỉ (Closed_Fist, Open_Palm, Pointing_Up, Thumb_Down, Thumb_Up, Victory,
  ILoveYou, None) kèm độ tin cậy.
- "Wave" (vẫy tay) được suy ra từ chuyển động ngang qua lại của bàn tay xòe.
- Kết quả đã lọc được publish đều đặn mỗi khung hình (std_msgs/String) để làm
  "nhịp tim": robot_behavior_node dừng robot nếu không nhận được tin nhắn nữa.
- Cửa sổ OpenCV hiển thị landmark, nhãn thô + độ tin cậy, nhãn đã lọc, FPS và
  trạng thái robot (subscribe /robot_behavior_state).
"""

import os
import time

import cv2
import mediapipe as mp
import rclpy
from ament_index_python.packages import get_package_share_directory
from mediapipe.tasks import python as mp_tasks
from mediapipe.tasks.python import vision
from rclpy.node import Node
from std_msgs.msg import String

from tb4_gesture.gesture_filter import NONE, WAVE, GestureStabilizer, WaveDetector

# 21 landmark bàn tay: các cặp nối để vẽ khung xương
HAND_CONNECTIONS = [(0, 1), (1, 2), (2, 3), (3, 4), (0, 5), (5, 6), (6, 7), (7, 8),
                    (5, 9), (9, 10), (10, 11), (11, 12), (9, 13), (13, 14), (14, 15),
                    (15, 16), (13, 17), (17, 18), (18, 19), (19, 20), (0, 17)]

# Màu (BGR) cho từng cử chỉ trên cửa sổ hiển thị
COLORS = {
    'Open_Palm': (0, 0, 255), 'Closed_Fist': (0, 140, 255), 'Thumb_Up': (0, 200, 0),
    'Pointing_Up': (255, 160, 0), 'Victory': (255, 0, 200), 'Wave': (0, 220, 220),
}
ACTIONS = {
    'Wave': 'chao lai', 'Pointing_Up': 'chon tien-lui', 'Victory': 'chon vong tron',
    'Thumb_Up': 'xac nhan', 'Open_Palm': 'DUNG', 'Closed_Fist': 'huy',
}


class GestureRecognitionNode(Node):

    def __init__(self):
        super().__init__('gesture_recognition_node')
        default_model = os.path.join(get_package_share_directory('tb4_gesture'),
                                     'models', 'gesture_recognizer.task')
        self.declare_parameter('camera_index', 0)
        self.declare_parameter('frame_width', 640)
        self.declare_parameter('frame_height', 480)
        self.declare_parameter('mirror', True)
        self.declare_parameter('show_window', True)
        self.declare_parameter('model_path', default_model)
        self.declare_parameter('min_score', 0.6)            # ngưỡng độ tin cậy
        self.declare_parameter('stable_window', 8)          # số khung bỏ phiếu
        self.declare_parameter('stable_ratio', 0.75)        # tỉ lệ phiếu tối thiểu
        self.declare_parameter('wave_amplitude', 0.05)      # biên độ đổi chiều (chuẩn hoá)
        self.declare_parameter('wave_reversals', 3)         # số lần đổi chiều tối thiểu
        self.declare_parameter('wave_window', 1.5)          # s
        self.declare_parameter('gesture_topic', '/detected_gesture')
        p = lambda name: self.get_parameter(name).value  # noqa: E731

        self.min_score = float(p('min_score'))
        self.mirror = bool(p('mirror'))
        self.show_window = bool(p('show_window'))
        self.stabilizer = GestureStabilizer(p('stable_window'), p('stable_ratio'))
        self.wave = WaveDetector(p('wave_amplitude'), p('wave_reversals'), p('wave_window'))

        self.publisher = self.create_publisher(String, p('gesture_topic'), 10)
        self.robot_state = 'chua co ket noi robot_behavior_node'
        self.create_subscription(String, '/robot_behavior_state', self.on_robot_state, 10)

        options = vision.GestureRecognizerOptions(
            base_options=mp_tasks.BaseOptions(model_asset_path=p('model_path')),
            running_mode=vision.RunningMode.VIDEO,
            num_hands=1,
            min_hand_detection_confidence=0.5,
            min_hand_presence_confidence=0.5,
            min_tracking_confidence=0.5)
        self.recognizer = vision.GestureRecognizer.create_from_options(options)

        self.camera_index = int(p('camera_index'))
        self.size = (int(p('frame_width')), int(p('frame_height')))
        self.cap = None
        self.open_camera()
        self.t0 = time.monotonic()
        self.last_ts_ms = -1
        self.fps = 0.0
        self.last_frame_time = None
        self.last_published = None
        self.get_logger().info(
            f'Webcam {self.camera_index}, model {p("model_path")}, min_score={self.min_score}. '
            f'Publish len {p("gesture_topic")}. Nhan q/ESC tren cua so de thoat.')

    def open_camera(self):
        if self.cap is not None:
            self.cap.release()
        self.cap = cv2.VideoCapture(self.camera_index, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.size[0])
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.size[1])
        if not self.cap.isOpened():
            self.get_logger().error(f'Khong mo duoc webcam {self.camera_index}')

    def on_robot_state(self, msg):
        self.robot_state = msg.data

    def process(self):
        """Xử lý 1 khung hình. Trả về False nếu người dùng muốn thoát."""
        ok, frame = self.cap.read()
        if not ok:
            # Mất camera: KHÔNG publish -> robot_behavior_node hết thời gian chờ và dừng robot
            self.get_logger().warn('Mat tin hieu webcam, thu mo lai...', throttle_duration_sec=2.0)
            self.open_camera()
            time.sleep(0.2)
            return True
        if self.mirror:
            frame = cv2.flip(frame, 1)

        now = time.monotonic()
        ts_ms = int((now - self.t0) * 1000)
        if ts_ms <= self.last_ts_ms:          # MediaPipe yêu cầu timestamp tăng dần
            ts_ms = self.last_ts_ms + 1
        self.last_ts_ms = ts_ms
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        result = self.recognizer.recognize_for_video(
            mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb), ts_ms)

        raw_label, raw_score, landmarks = NONE, 0.0, None
        if result.hand_landmarks:
            landmarks = result.hand_landmarks[0]
            if result.gestures:
                top = result.gestures[0][0]
                raw_label, raw_score = top.category_name or NONE, top.score
        accepted = raw_label if raw_score >= self.min_score else NONE

        stable = self.stabilizer.update(accepted)
        # Tâm bàn tay = landmark 9 (gốc ngón giữa); vẫy tay chỉ tính khi bàn tay đang xòe
        palm_x = landmarks[9].x if landmarks is not None else None
        waving = self.wave.update(palm_x, now, palm_open=(raw_label == 'Open_Palm'))
        output = WAVE if waving else stable

        self.publisher.publish(String(data=output))
        if output != self.last_published:
            self.get_logger().info(f'Cu chi: {output}  (tho: {raw_label} {raw_score:.2f})')
            self.last_published = output

        if self.last_frame_time is not None:
            self.fps = 0.9 * self.fps + 0.1 / max(now - self.last_frame_time, 1e-3)
        self.last_frame_time = now

        if not self.show_window:
            return True
        self.draw(frame, landmarks, raw_label, raw_score, output)
        cv2.imshow('TurtleBot 4 - Gesture Recognition', frame)
        key = cv2.waitKey(1) & 0xFF
        return key not in (ord('q'), 27)

    def draw(self, frame, landmarks, raw_label, raw_score, output):
        h, w = frame.shape[:2]
        color = COLORS.get(output, (200, 200, 200))
        if landmarks is not None:
            pts = [(int(lm.x * w), int(lm.y * h)) for lm in landmarks]
            for a, b in HAND_CONNECTIONS:
                cv2.line(frame, pts[a], pts[b], (255, 255, 255), 2)
            for pt in pts:
                cv2.circle(frame, pt, 4, color, -1)
            xs, ys = [pt[0] for pt in pts], [pt[1] for pt in pts]
            cv2.rectangle(frame, (min(xs) - 15, min(ys) - 15), (max(xs) + 15, max(ys) + 15), color, 2)
            cv2.putText(frame, f'{raw_label} {raw_score * 100:.0f}%', (min(xs) - 15, min(ys) - 22),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

        overlay = frame.copy()
        cv2.rectangle(overlay, (0, 0), (w, 92), (0, 0, 0), -1)
        cv2.rectangle(overlay, (0, h - 34), (w, h), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.55, frame, 0.45, 0, frame)
        cv2.putText(frame, f'Gesture: {output}', (10, 32), cv2.FONT_HERSHEY_SIMPLEX, 1.0, color, 2)
        action = ACTIONS.get(output, '')
        if action:
            cv2.putText(frame, f'-> {action}', (int(w * 0.62), 32), cv2.FONT_HERSHEY_SIMPLEX,
                        0.8, color, 2)
        cv2.putText(frame, f'Raw: {raw_label}  conf: {raw_score * 100:.1f}%  (min {self.min_score * 100:.0f}%)',
                    (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
        cv2.putText(frame, f'FPS: {self.fps:.1f}', (10, 84), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                    (255, 255, 255), 1)
        cv2.putText(frame, f'Robot: {self.robot_state}', (10, h - 10), cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, (0, 255, 255), 2)

    def destroy_node(self):
        self.cap.release()
        self.recognizer.close()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GestureRecognitionNode()
    try:
        # cv2.imshow phải chạy ở luồng chính -> vòng lặp tay, spin_once để nhận callback
        while rclpy.ok() and node.process():
            rclpy.spin_once(node, timeout_sec=0.0)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
