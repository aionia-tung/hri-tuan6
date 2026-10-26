"""Máy trạng thái hành vi robot (không phụ thuộc ROS để dễ kiểm thử).

Trạng thái:
  IDLE       chờ lệnh, robot đứng yên
  SELECTED   đã chọn hành vi (Pointing_Up / Victory), CHỜ xác nhận Thumb_Up
  EXECUTING  đang thực hiện hành vi đã xác nhận
  PAUSED     bị dừng bằng Open_Palm, giữ nguyên hành vi (Thumb_Up để chạy tiếp)
  GREETING   đang chào lại (Wave)

handle() trả về (command, message). command là một trong:
  None | ('start', tên_hành_vi) | ('resume',) | ('stop',) | ('cancel',)
"""

IDLE, SELECTED, EXECUTING, PAUSED, GREETING = 'IDLE', 'SELECTED', 'EXECUTING', 'PAUSED', 'GREETING'

# Bảng ánh xạ cử chỉ chọn -> hành vi
SELECT_GESTURES = {
    'Pointing_Up': 'forward_back',
    'Victory': 'circle',
}


class BehaviorStateMachine:

    def __init__(self, wave_requires_confirm=False):
        self.wave_requires_confirm = wave_requires_confirm
        self.state = IDLE
        self.selected = None      # hành vi đã chọn, chờ xác nhận
        self.active = None        # hành vi đang chạy / tạm dừng
        self._after_greeting = (IDLE, None)

    def describe(self):
        if self.state == SELECTED:
            return f'SELECTED: {self.selected} (gio Thumb_Up de xac nhan)'
        if self.state in (EXECUTING, PAUSED, GREETING):
            return f'{self.state}: {self.active}'
        return self.state

    def handle(self, gesture):
        if gesture == 'Closed_Fist':
            had_motion = self.state != IDLE or self.selected is not None
            self._go_idle()
            return ('cancel',), 'Huy hanh vi, ve trang thai cho' if had_motion else 'Dang cho'

        if gesture == 'Open_Palm':
            if self.state in (EXECUTING, GREETING):
                self.state = PAUSED
                return ('stop',), f'DUNG NGAY - tam dung {self.active} (Thumb_Up: chay tiep, Closed_Fist: huy)'
            return ('stop',), 'DUNG NGAY'

        if gesture == 'Wave':
            if self.wave_requires_confirm:
                return self._select('wave')
            if self.state in (IDLE, SELECTED):
                self._after_greeting = (self.state, self.selected)
                self.state, self.active = GREETING, 'wave'
                return ('start', 'wave'), 'Xin chao! Robot xoay trai-phai de chao lai'
            return None, f'Bo qua Wave khi dang {self.state}'

        if gesture in SELECT_GESTURES:
            return self._select(SELECT_GESTURES[gesture])

        if gesture == 'Thumb_Up':
            if self.state == SELECTED:
                self.state, self.active, self.selected = EXECUTING, self.selected, None
                return ('start', self.active), f'Xac nhan -> thuc hien {self.active}'
            if self.state == PAUSED:
                self.state = GREETING if self.active == 'wave' and not self.wave_requires_confirm \
                    else EXECUTING
                return ('resume',), f'Tiep tuc {self.active}'
            return None, 'Chua chon hanh vi nao de xac nhan'

        return None, None   # None, Thumb_Down, ILoveYou...: không ánh xạ

    def _select(self, behavior):
        if self.state in (EXECUTING, GREETING):
            return None, f'Dang {self.state} {self.active}: dung (Open_Palm) hoac huy (Closed_Fist) truoc'
        command = ('cancel',) if self.state == PAUSED else None
        self.state, self.selected, self.active = SELECTED, behavior, None
        return command, f'Da chon {behavior} - gio Thumb_Up de xac nhan'

    def finished(self):
        """Gọi khi kế hoạch chuyển động chạy xong."""
        done = self.active
        if self.state == GREETING or (self.state == EXECUTING and done == 'wave'
                                      and not self.wave_requires_confirm):
            self.state, self.selected = self._after_greeting
            self.active = None
        else:
            self._go_idle()
        return f'Hoan thanh {done}'

    def signal_lost(self):
        was = self.state
        self._go_idle()
        return was != IDLE

    def _go_idle(self):
        self.state, self.selected, self.active = IDLE, None, None
        self._after_greeting = (IDLE, None)
