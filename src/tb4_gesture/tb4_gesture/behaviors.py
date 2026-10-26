"""Các hành vi chuyển động của robot dưới dạng chuỗi đoạn (segment).

Mỗi đoạn có vận tốc (v, w) và một mục tiêu: quãng đường (m) hoặc góc quay (rad).
Tiến độ được đo bằng /odom nên đúng cả khi Gazebo chạy chậm (real time factor < 1).
"""

import math
from dataclasses import dataclass


@dataclass
class Segment:
    v: float           # vận tốc thẳng (m/s), dấu = chiều
    w: float           # vận tốc góc (rad/s), dấu = chiều
    target: float      # quãng đường (m) nếu measure='dist', góc (rad) nếu 'angle'
    measure: str       # 'dist' | 'angle'
    label: str = ''


def wave_plan(angle=0.5, speed=1.0, cycles=2):
    """Chào lại: xoay trái - phải quanh hướng ban đầu rồi về đúng hướng cũ."""
    # +a, (-2a, +2a) x cycles, -a  => tổng góc quay = 0
    plan = [Segment(0.0, speed, angle, 'angle', 'xoay trai')]
    for _ in range(cycles):
        plan += [Segment(0.0, -speed, 2 * angle, 'angle', 'xoay phai'),
                 Segment(0.0, speed, 2 * angle, 'angle', 'xoay trai')]
    plan.append(Segment(0.0, -speed, angle, 'angle', 've huong cu'))
    return plan


def forward_back_plan(distance=0.3, speed=0.2, repeats=4):
    """Tiến - lùi `repeats` lần (Pointing_Up)."""
    plan = []
    for i in range(repeats):
        plan += [Segment(speed, 0.0, distance, 'dist', f'tien {i + 1}/{repeats}'),
                 Segment(-speed, 0.0, distance, 'dist', f'lui {i + 1}/{repeats}')]
    return plan


def circle_plan(radius=0.5, speed=0.2, turns=1.0):
    """Quỹ đạo tròn bán kính `radius` (Victory): v = speed, w = v / r."""
    w = speed / radius
    return [Segment(speed, w, 2 * math.pi * turns, 'angle', f'vong tron r={radius:.2f} m')]


PLANS = {
    'forward_back': forward_back_plan,
    'circle': circle_plan,
    'wave': wave_plan,
}


def wrap_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


class PlanRunner:
    """Chạy tuần tự các Segment, trả về (v, w) cho mỗi chu kỳ điều khiển.

    Tiến độ đoạn hiện tại được tích luỹ từ độ thay đổi pose /odom giữa hai chu kỳ;
    nếu không có odom thì tích phân vận tốc lệnh theo thời gian (dự phòng).
    Gần đích robot giảm tốc tuyến tính để không vượt quá nhiều.
    """

    def __init__(self, plan, slow_gain=2.0, min_ratio=0.25):
        self.plan = list(plan)
        self.index = 0
        self.progress = 0.0
        self.slow_gain = slow_gain
        self.min_ratio = min_ratio
        self.last_pose = None

    @property
    def done(self):
        return self.index >= len(self.plan)

    @property
    def current(self):
        return None if self.done else self.plan[self.index]

    def reset_reference(self):
        """Gọi khi tạm dừng / tiếp tục để không tính quãng trượt trong lúc dừng."""
        self.last_pose = None

    def step(self, pose, dt):
        """pose = (x, y, yaw) từ odom hoặc None. Trả về (v, w)."""
        if self.done:
            return 0.0, 0.0
        seg = self.plan[self.index]
        if pose is not None and self.last_pose is not None:
            dx, dy = pose[0] - self.last_pose[0], pose[1] - self.last_pose[1]
            dyaw = wrap_angle(pose[2] - self.last_pose[2])
            self.progress += math.hypot(dx, dy) if seg.measure == 'dist' else abs(dyaw)
        elif pose is None:
            self.progress += (abs(seg.v) if seg.measure == 'dist' else abs(seg.w)) * dt
        self.last_pose = pose

        remaining = seg.target - self.progress
        if remaining <= 0.0:
            self.index += 1
            self.progress = 0.0
            return self.step(pose, 0.0) if not self.done else (0.0, 0.0)

        nominal = abs(seg.v) if seg.measure == 'dist' else abs(seg.w)
        scale = min(1.0, max(self.min_ratio, self.slow_gain * remaining / max(nominal, 1e-6)))
        return seg.v * scale, seg.w * scale
