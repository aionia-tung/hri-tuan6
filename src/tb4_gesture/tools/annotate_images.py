#!/usr/bin/env python3
"""Chạy MediaPipe Gesture Recognizer trên ảnh tĩnh, vẽ landmark + nhãn + độ tin cậy.

  python3 tools/annotate_images.py <model.task> <anh_ra.png> anh1.jpg anh2.jpg ...
Dùng để minh hoạ mô hình trong báo cáo (ảnh webcam thật lấy từ video demo).
"""

import sys

import cv2
import mediapipe as mp
import numpy as np
from mediapipe.tasks import python as mp_tasks
from mediapipe.tasks.python import vision

from tb4_gesture.gesture_recognition_node import HAND_CONNECTIONS


def main():
    model, out, images = sys.argv[1], sys.argv[2], sys.argv[3:]
    recognizer = vision.GestureRecognizer.create_from_options(vision.GestureRecognizerOptions(
        base_options=mp_tasks.BaseOptions(model_asset_path=model),
        running_mode=vision.RunningMode.IMAGE, num_hands=1))
    tiles = []
    for path in images:
        img = cv2.imread(path)
        scale = 360 / img.shape[0]
        img = cv2.resize(img, None, fx=scale, fy=scale)
        res = recognizer.recognize(mp.Image(image_format=mp.ImageFormat.SRGB,
                                            data=cv2.cvtColor(img, cv2.COLOR_BGR2RGB)))
        h, w = img.shape[:2]
        label, score = 'None', 0.0
        if res.hand_landmarks:
            pts = [(int(lm.x * w), int(lm.y * h)) for lm in res.hand_landmarks[0]]
            for a, b in HAND_CONNECTIONS:
                cv2.line(img, pts[a], pts[b], (255, 255, 255), 2)
            for pt in pts:
                cv2.circle(img, pt, 4, (0, 200, 0), -1)
        if res.gestures:
            label, score = res.gestures[0][0].category_name, res.gestures[0][0].score
        cv2.rectangle(img, (0, 0), (w, 36), (0, 0, 0), -1)
        cv2.putText(img, f'{label}  {score * 100:.1f}%', (8, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                    (0, 255, 0), 2)
        print(f'{path}: {label} {score:.3f}')
        tiles.append(img)
    cv2.imwrite(out, np.hstack(tiles))


if __name__ == '__main__':
    main()
