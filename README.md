# Bài 06 – Giao tiếp phi ngôn ngữ: điều khiển TurtleBot 4 bằng cử chỉ tay

**Non-verbal Human–Robot Interaction Using Hand Gesture Recognition**

Webcam laptop → **MediaPipe Gesture Recognizer** → ROS 2 topic `/detected_gesture` → máy trạng thái hành vi →
`/cmd_vel` → TurtleBot 4 trong Ignition Gazebo. Trong lúc demo không dùng bàn phím để điều khiển robot.

- Video demo: `<LINK GOOGLE DRIVE>`

## Bảng ánh xạ cử chỉ → hành vi

| Cử chỉ | Nhãn publish | Hành vi robot |
| :--- | :--- | :--- |
| Vẫy tay | `Wave` | Chào lại: xoay trái–phải ±0.5 rad, 2 chu kỳ, kết thúc đúng hướng ban đầu (thực hiện ngay) |
| Giơ 1 ngón tay | `Pointing_Up` | **Chọn** hành vi tiến 0.3 m – lùi 0.3 m × 4 lần (chờ xác nhận) |
| Giơ 2 ngón tay | `Victory` | **Chọn** hành vi quỹ đạo tròn r = 0.5 m, v = 0.2 m/s, 1 vòng (chờ xác nhận) |
| Giơ ngón cái | `Thumb_Up` | **Xác nhận** và thực hiện hành vi đã chọn / tiếp tục hành vi đang tạm dừng |
| Xòe bàn tay | `Open_Palm` | **Dừng ngay** (tạm dừng, giữ hành vi để có thể Thumb_Up chạy tiếp) |
| Nắm tay | `Closed_Fist` | **Hủy** hành vi hiện tại, xóa lựa chọn, về trạng thái chờ `IDLE` |
| (khác) | `Thumb_Down`, `ILoveYou`, `None` | Nhận diện và hiển thị nhưng không gán hành vi |

Node nhận diện phân biệt được 8 nhãn: 7 nhãn của MediaPipe và `Wave` tự suy ra. Trong đó 6 cử chỉ dùng để điều khiển.

## Cấu trúc workspace

```text
tb4_gesture_ws/
├── README.md
└── src/tb4_gesture/                     # ROS 2 package (ament_python)
    ├── tb4_gesture/
    │   ├── gesture_recognition_node.py  # Phần 2: webcam + MediaPipe -> /detected_gesture
    │   ├── robot_behavior_node.py       # Phần 3: /detected_gesture -> /cmd_vel
    │   ├── gesture_filter.py            # lọc nhiễu: bỏ phiếu, phát hiện vẫy tay, edge trigger
    │   ├── state_machine.py             # máy trạng thái IDLE/SELECTED/EXECUTING/PAUSED/GREETING
    │   ├── behaviors.py                 # quỹ đạo: tiến-lùi, vòng tròn, chào; PlanRunner đo bằng /odom
    │   ├── webcam_test.py               # Phần 1: đọc + hiển thị webcam bằng OpenCV
    │   └── gesture_replay.py            # công cụ kiểm thử: phát lại kịch bản cử chỉ (không dùng khi demo)
    ├── launch/
    │   ├── gesture_sim.launch.py        # Phần 1: Gazebo arena + TurtleBot 4
    │   └── gesture_control.launch.py    # Phần 2+3: 2 node (sim:=true bật luôn Gazebo)
    ├── config/gesture_params.yaml       # tham số 2 node
    ├── worlds/gesture_arena.sdf         # sàn trống 8 x 8 m, tường thấp
    ├── models/gesture_recognizer.task   # mô hình MediaPipe (float16)
    ├── test/test_logic.py               # unit test (pytest)
    └── tools/                           # ghi log, vẽ đồ thị, ảnh minh họa cho báo cáo
```

## 1. Cài đặt

Yêu cầu: Ubuntu 22.04, ROS 2 Humble, TurtleBot 4 Simulator (Ignition Gazebo Fortress), webcam.

```bash
sudo apt update
sudo apt install -y ros-humble-turtlebot4-simulator ros-humble-irobot-create-msgs python3-opencv python3-pytest

# MediaPipe: cài --no-deps rồi thêm đúng các gói cần cho Tasks Vision.
# Lý do: cài mặc định sẽ kéo opencv-contrib-python mới nhất (đòi NumPy 2) và jax,
# làm hỏng cv_bridge/rclpy của ROS Humble (được build với NumPy 1.x).
pip install --user --no-deps "mediapipe==0.10.14"
pip install --user "protobuf>=4.25.3,<5" absl-py attrs "flatbuffers>=2.0" "numpy<2"

# Kiểm tra
python3 -c "import mediapipe as mp; from mediapipe.tasks.python import vision; print(mp.__version__)"
```

Mô hình `gesture_recognizer.task` đã có sẵn trong `models/`. Nếu muốn tải lại:

```bash
curl -L -o src/tb4_gesture/models/gesture_recognizer.task \
  https://storage.googleapis.com/mediapipe-models/gesture_recognizer/gesture_recognizer/float16/latest/gesture_recognizer.task
```

## 2. Build

```bash
cd ~/tb4_gesture_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 3. Chạy chương trình

### Phần 1 – Simulation và webcam

Terminal 1, chạy Gazebo + TurtleBot 4 trong arena 8 × 8 m. Robot xuất phát ở (0, 0); dock được đặt ở góc (−3, −3) nên phía trước và phía sau robot đều trống:

```bash
ros2 launch tb4_gesture gesture_sim.launch.py          # gui:=false để chạy không cửa sổ
```

Terminal 2, kiểm tra robot nhận lệnh vận tốc qua `/cmd_vel`:

```bash
ros2 topic pub -r 10 -t 30 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}}"   # tiến ~3 s
ros2 topic echo /odom --once --field pose.pose.position                           # x tăng
```

Kiểm tra webcam bằng OpenCV (nhấn `q` để thoát):

```bash
ros2 run tb4_gesture webcam_test        # hoặc: ros2 run tb4_gesture webcam_test 1
```

### Phần 2 + 3 – Nhận diện cử chỉ và điều khiển robot

Terminal 2 (Gazebo đang chạy ở terminal 1):

```bash
ros2 launch tb4_gesture gesture_control.launch.py      # camera_index:=1 nếu dùng webcam USB
```

Hoặc chạy tất cả trong một lệnh: `ros2 launch tb4_gesture gesture_control.launch.py sim:=true`.

Chạy riêng từng node:

```bash
ros2 run tb4_gesture gesture_recognition_node --ros-args --params-file src/tb4_gesture/config/gesture_params.yaml
ros2 run tb4_gesture robot_behavior_node     --ros-args --params-file src/tb4_gesture/config/gesture_params.yaml
```

Theo dõi:

```bash
ros2 topic echo /detected_gesture          # nhãn cử chỉ đã lọc (~30 Hz)
ros2 topic echo /robot_behavior_state      # trạng thái máy trạng thái
ros2 topic echo /cmd_vel
rqt_graph
```

Cửa sổ webcam hiển thị: khung xương 21 điểm của bàn tay, nhãn thô và **độ tin cậy** (%), nhãn đã lọc đang publish, hành động tương ứng, FPS và trạng thái robot.

## 4. Node và topic

| Node | Subscribe | Publish |
| :--- | :--- | :--- |
| `gesture_recognition_node` | `/robot_behavior_state` (String, để hiển thị) | `/detected_gesture` (`std_msgs/msg/String`) |
| `robot_behavior_node` | `/detected_gesture`, `/odom` (Odometry), `/hazard_detection` (Create 3) | `/cmd_vel` (`geometry_msgs/msg/Twist`), `/robot_behavior_state` (String) |

Khi khởi động, `robot_behavior_node` gọi service `/motion_control/set_parameters` để đặt `safety_override = backup_only`. Mặc định Create 3 giới hạn quãng lùi vì không có cảm biến cliff phía sau.

## 5. Chống nhiễu và an toàn

1. **Ngưỡng độ tin cậy:** nhãn có score < 0.6 bị coi là `None`.
2. **Bỏ phiếu theo thời gian:** một nhãn chỉ được publish khi chiếm ≥ 75 % trong 8 khung gần nhất (~0.27 s ở 30 FPS), nên nhãn nhảy khi chuyển tay sẽ bị loại.
3. **Phát hiện vẫy tay:** bàn tay xòe phải đổi chiều ngang ≥ 3 lần trong 1.5 s, mỗi lần với biên độ ≥ 5 % bề rộng ảnh. Có hysteresis nên tay run không bị tính là vẫy.
4. **Edge trigger + cooldown:** robot chỉ phản ứng khi nhãn *thay đổi*. Giữ tay lâu không lặp lệnh, và cùng một cử chỉ không kích hoạt lại trong 1.5 s. `Open_Palm` và `Closed_Fist` luôn được xử lý ngay.
5. **Xác nhận:** `Pointing_Up` / `Victory` chỉ *chọn* hành vi; robot chỉ chạy sau `Thumb_Up`. Khi robot đang chạy, cử chỉ chọn bị bỏ qua cho tới khi dừng hoặc hủy.
6. **Mất tín hiệu:** node nhận diện publish mỗi khung hình như một nhịp tim. Nếu `robot_behavior_node` không nhận được tin trong 1 s (node tắt, mất webcam, treo), nó dừng robot và về `IDLE`.
7. **Va chạm:** sự kiện `BUMP` trên `/hazard_detection` khiến robot tạm dừng.
8. **Không va chạm:** arena 8 × 8 m. Quỹ đạo lớn nhất là vòng tròn đường kính 1 m quanh điểm xuất phát, cách tường ≥ 2.5 m.

## 6. Kiểm thử

```bash
cd src/tb4_gesture
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/ -q     # 9 test logic (lọc nhiễu, máy trạng thái, quỹ đạo)
```

Kiểm tra trong Gazebo không cần webcam bằng cách phát lại một kịch bản cử chỉ. Công cụ này chỉ để kiểm thử, **không** dùng trong video demo:

```bash
ros2 run tb4_gesture robot_behavior_node --ros-args --params-file src/tb4_gesture/config/gesture_params.yaml
ros2 run tb4_gesture gesture_replay --ros-args -p script:="Victory:1,None:1,Thumb_Up:1,None:30"
```

Ghi dữ liệu và vẽ đồ thị cho báo cáo: `python3 src/tb4_gesture/tools/record_run.py run_circle.csv` và
`python3 src/tb4_gesture/tools/plot_run.py <thư mục csv> <thư mục ảnh>`.

## 7. Kịch bản quay video demo

Bố cục màn hình: cửa sổ webcam bên trái, Gazebo bên phải.

1. `Wave` → robot xoay trái–phải chào lại.
2. `Thumb_Up` khi chưa chọn → robot không làm gì (log: "Chua chon hanh vi nao").
3. `Pointing_Up` → `SELECTED: forward_back`, robot vẫn đứng yên → `Thumb_Up` → robot tiến–lùi 4 lần.
4. `Victory` → `Thumb_Up` → robot chạy vòng tròn → giữa chừng `Open_Palm` → robot dừng ngay (`PAUSED`) → `Thumb_Up` → chạy tiếp.
5. `Pointing_Up` → `Thumb_Up` → khi đang chạy, `Closed_Fist` → hủy, về `IDLE`.
6. `Victory` → `Thumb_Up`, rồi tắt cửa sổ webcam (nhấn `q`) → sau 1 s robot tự dừng (mất tín hiệu).

## 8. Xử lý sự cố

- **Gazebo chạy chậm** (real time factor ~0.3 khi dùng pin, CPU ở chế độ `powersave`): cắm sạc, hoặc dùng `gui:=false`. Quãng đường và góc quay vẫn đúng vì tiến độ đo bằng `/odom`, chỉ thời gian thực kéo dài hơn.
- **`/scan` toàn 0.164 m trên GPU Intel/Mesa:** launch mặc định `lidar_ogre2:=true` sửa lỗi này (bài này không dùng LiDAR).
- **Log `GPU support is not available` / `eglChooseConfig`** của MediaPipe: chỉ là cảnh báo, MediaPipe tự chạy bằng CPU (XNNPACK).
- **Không mở được webcam:** kiểm tra `ls /dev/video*`, quyền nhóm `video`, và đổi `camera_index`.
- **`colcon build` lỗi `canonicalize_version`:** gỡ `setuptools` trong `~/.local` (`pip uninstall setuptools`).
- **Lỗi import `cv2`/`numpy`** sau khi cài gói pip khác: giữ `numpy<2` và `opencv-python<4.12`.

## Tài liệu tham khảo

- MediaPipe Gesture Recognizer: <https://ai.google.dev/edge/mediapipe/solutions/vision/gesture_recognizer>
- OpenCV: <https://docs.opencv.org/>
- TurtleBot 4 User Manual: <https://turtlebot.github.io/turtlebot4-user-manual/>
