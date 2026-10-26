"""Bài 06 - Phần 1: đọc và hiển thị ảnh webcam bằng OpenCV (chưa có AI).

  ros2 run tb4_gesture webcam_test [chi_so_camera]
Nhấn q / ESC để thoát.
"""

import sys
import time

import cv2


def main():
    index = int(sys.argv[1]) if len(sys.argv) > 1 and sys.argv[1].isdigit() else 0
    cap = cv2.VideoCapture(index, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    if not cap.isOpened():
        print(f'Khong mo duoc webcam /dev/video{index}')
        return 1
    print(f'Webcam /dev/video{index}: {int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))}x'
          f'{int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))}. Nhan q/ESC de thoat.')
    last, fps = time.monotonic(), 0.0
    while True:
        ok, frame = cap.read()
        if not ok:
            print('Khong doc duoc khung hinh')
            break
        frame = cv2.flip(frame, 1)
        now = time.monotonic()
        fps = 0.9 * fps + 0.1 / max(now - last, 1e-3)
        last = now
        cv2.putText(frame, f'Webcam {index}  {frame.shape[1]}x{frame.shape[0]}  FPS {fps:.1f}',
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.imshow('Webcam test (OpenCV)', frame)
        if cv2.waitKey(1) & 0xFF in (ord('q'), 27):
            break
    cap.release()
    cv2.destroyAllWindows()
    return 0


if __name__ == '__main__':
    sys.exit(main())
