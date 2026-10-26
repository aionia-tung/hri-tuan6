"""Lọc nhiễu nhận diện cử chỉ (không phụ thuộc ROS để dễ kiểm thử).

MediaPipe Gesture Recognizer trả về 1 nhãn / khung hình. Kết quả thô hay "nhảy"
giữa các nhãn khi tay đang chuyển động, nên trước khi publish ta lọc qua:

1. Ngưỡng độ tin cậy  : score < min_score -> coi như "None".
2. Bỏ phiếu theo thời gian (GestureStabilizer): một nhãn chỉ được chấp nhận khi
   chiếm >= ratio số khung trong cửa sổ window khung gần nhất.
3. Phát hiện vẫy tay (WaveDetector): MediaPipe không có nhãn "Wave", ta suy ra từ
   chuyển động ngang qua lại của bàn tay đang xòe (Open_Palm).
"""

from collections import Counter, deque

NONE = 'None'
WAVE = 'Wave'

# Nhãn của mô hình gesture_recognizer.task (canned gestures)
CANNED_GESTURES = ('None', 'Closed_Fist', 'Open_Palm', 'Pointing_Up',
                   'Thumb_Down', 'Thumb_Up', 'Victory', 'ILoveYou')


class GestureStabilizer:
    """Bỏ phiếu đa số trên cửa sổ trượt các nhãn thô."""

    def __init__(self, window=8, ratio=0.75):
        self.window = max(1, int(window))
        self.ratio = float(ratio)
        self.history = deque(maxlen=self.window)
        self.stable = NONE

    def update(self, label):
        self.history.append(label)
        top, count = Counter(self.history).most_common(1)[0]
        if len(self.history) == self.window and count >= self.ratio * self.window:
            self.stable = top
        elif len(self.history) == self.window and self.history.count(self.stable) == 0:
            # Nhãn cũ đã biến mất hoàn toàn mà chưa có nhãn mới đủ phiếu -> None
            self.stable = NONE
        return self.stable

    def reset(self):
        self.history.clear()
        self.stable = NONE


class WaveDetector:
    """Phát hiện vẫy tay: tâm bàn tay (đang xòe) đổi chiều ngang nhiều lần.

    x là toạ độ ngang chuẩn hoá [0, 1] của tâm bàn tay. Một lần "đổi chiều" chỉ
    được tính khi tay đã đi được >= amplitude theo chiều ngược lại (hysteresis),
    nên run tay nhỏ không bị tính là vẫy.
    Khi tay chuyển động nhanh, ảnh nhoè làm MediaPipe mất nhãn Open_Palm vài khung;
    trong grace_s giây sau khung Open_Palm gần nhất vẫn tiếp tục bám theo x.
    """

    def __init__(self, amplitude=0.05, reversals=3, window_s=1.5, hold_s=1.0, grace_s=0.4):
        self.amplitude = amplitude
        self.reversals = reversals
        self.window_s = window_s
        self.hold_s = hold_s
        self.grace_s = grace_s
        self.reset()

    def reset(self):
        self.turns = deque()      # thời điểm các lần đổi chiều
        self.extreme = None       # cực trị hiện tại (x)
        self.direction = 0        # +1 sang phải, -1 sang trái, 0 chưa biết
        self.active_until = -1.0
        self.last_open = -1e9     # thời điểm gần nhất thấy bàn tay xòe

    def update(self, x, now, palm_open):
        """Trả về True nếu đang trong trạng thái vẫy tay."""
        if palm_open and x is not None:
            self.last_open = now
        elif x is None or now - self.last_open > self.grace_s:
            self.turns.clear()
            self.extreme, self.direction = None, 0
            return now < self.active_until
        if self.extreme is None:
            self.extreme = x                       # điểm neo ban đầu
        elif self.direction == 0:
            if abs(x - self.extreme) >= self.amplitude:
                self.direction, self.extreme = (1 if x > self.extreme else -1), x
        elif (x - self.extreme) * self.direction > 0:
            self.extreme = x                       # vẫn đi tiếp cùng chiều
        elif abs(x - self.extreme) >= self.amplitude:
            # quay đầu đủ xa so với cực trị -> 1 lần đổi chiều
            self.turns.append(now)
            self.direction, self.extreme = -self.direction, x
        while self.turns and now - self.turns[0] > self.window_s:
            self.turns.popleft()
        if len(self.turns) >= self.reversals:
            self.active_until = now + self.hold_s
        return now < self.active_until


class EdgeTrigger:
    """Biến dòng nhãn liên tục thành sự kiện: chỉ phát khi nhãn ĐỔI sang giá trị
    mới, và cùng một nhãn không được phát lại trong cooldown_s giây.
    Nhãn trong `no_cooldown` (lệnh an toàn: dừng / huỷ) luôn được phát khi xuất hiện."""

    def __init__(self, cooldown_s=1.5, ignore=(NONE,), no_cooldown=()):
        self.cooldown_s = cooldown_s
        self.ignore = set(ignore)
        self.no_cooldown = set(no_cooldown)
        self.last_label = NONE
        self.last_fired = {}

    def update(self, label, now):
        fired = None
        if label != self.last_label and label not in self.ignore:
            if label in self.no_cooldown or \
                    now - self.last_fired.get(label, -1e9) >= self.cooldown_s:
                fired = label
                self.last_fired[label] = now
        self.last_label = label
        return fired
