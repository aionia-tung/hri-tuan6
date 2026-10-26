"""Kiểm thử logic không cần ROS/Gazebo: python3 -m pytest test/ -q"""

import math

from tb4_gesture.behaviors import PlanRunner, circle_plan, forward_back_plan, wave_plan
from tb4_gesture.gesture_filter import EdgeTrigger, GestureStabilizer, WaveDetector
from tb4_gesture.state_machine import (EXECUTING, GREETING, IDLE, PAUSED, SELECTED,
                                       BehaviorStateMachine)


def test_stabilizer_ignores_flicker():
    s = GestureStabilizer(window=8, ratio=0.75)
    labels = ['Thumb_Up', 'Victory'] * 8          # nhảy liên tục -> không đủ phiếu
    assert all(s.update(label) == 'None' for label in labels)
    for _ in range(8):
        out = s.update('Victory')
    assert out == 'Victory'
    assert s.update('Thumb_Up') == 'Victory'      # 1 khung nhiễu không làm đổi nhãn


def test_wave_detector():
    w = WaveDetector(amplitude=0.05, reversals=3, window_s=1.5)
    t, waving = 0.0, False
    for i in range(60):                            # dao động 0.4 <-> 0.6, chu kỳ ~0.5 s
        t = i / 30
        waving = w.update(0.5 + 0.1 * math.sin(2 * math.pi * 2 * t), t, palm_open=True)
    assert waving
    still = WaveDetector()
    assert not any(still.update(0.5 + 0.01 * (i % 2), i / 30, True) for i in range(60))
    blurry = WaveDetector()                        # cứ 3 khung thì 1 khung mất nhãn Open_Palm
    assert any(blurry.update(0.5 + 0.1 * math.sin(2 * math.pi * 2 * i / 30), i / 30, i % 3 != 0)
               for i in range(60))


def test_edge_trigger_and_cooldown():
    e = EdgeTrigger(cooldown_s=1.5, no_cooldown=('Open_Palm',))
    assert e.update('Thumb_Up', 0.0) == 'Thumb_Up'
    assert e.update('Thumb_Up', 0.1) is None      # giữ tay -> không lặp
    assert e.update('None', 0.2) is None
    assert e.update('Thumb_Up', 0.5) is None      # trong cooldown
    e.update('None', 0.6)
    assert e.update('Thumb_Up', 2.0) == 'Thumb_Up'
    assert e.update('Open_Palm', 2.1) == 'Open_Palm'
    e.update('None', 2.2)
    assert e.update('Open_Palm', 2.3) == 'Open_Palm'   # lệnh dừng không bị cooldown


def test_requires_confirmation():
    m = BehaviorStateMachine()
    assert m.handle('Thumb_Up')[0] is None        # chưa chọn gì
    assert m.handle('Pointing_Up')[0] is None and m.state == SELECTED
    assert m.handle('Victory')[0] is None and m.selected == 'circle'
    assert m.handle('Thumb_Up')[0] == ('start', 'circle') and m.state == EXECUTING
    assert m.handle('Pointing_Up')[0] is None and m.state == EXECUTING   # đang bận


def test_stop_resume_cancel():
    m = BehaviorStateMachine()
    m.handle('Pointing_Up'), m.handle('Thumb_Up')
    assert m.handle('Open_Palm')[0] == ('stop',) and m.state == PAUSED
    assert m.handle('Thumb_Up')[0] == ('resume',) and m.state == EXECUTING
    assert m.handle('Closed_Fist')[0] == ('cancel',) and m.state == IDLE
    m.handle('Victory')
    m.handle('Closed_Fist')
    assert m.state == IDLE and m.selected is None


def test_wave_returns_to_previous_state():
    m = BehaviorStateMachine()
    m.handle('Victory')
    assert m.handle('Wave')[0] == ('start', 'wave') and m.state == GREETING
    m.finished()
    assert m.state == SELECTED and m.selected == 'circle'
    assert m.handle('Thumb_Up')[0] == ('start', 'circle')
    m.finished()
    assert m.state == IDLE


def test_signal_lost():
    m = BehaviorStateMachine()
    m.handle('Pointing_Up'), m.handle('Thumb_Up')
    assert m.signal_lost() and m.state == IDLE


def simulate(plan, dt=0.05):
    """Mô phỏng robot vi sai lý tưởng chạy theo PlanRunner."""
    runner, x, y, yaw, steps = PlanRunner(plan), 0.0, 0.0, 0.0, 0
    while not runner.done and steps < 10000:
        v, w = runner.step((x, y, yaw), dt)
        x += v * math.cos(yaw) * dt
        y += v * math.sin(yaw) * dt
        yaw += w * dt
        steps += 1
    return x, y, yaw


def test_plans_return_near_start():
    for plan, tol in ((forward_back_plan(), 0.03), (circle_plan(), 0.06), (wave_plan(), 0.03)):
        x, y, yaw = simulate(plan)
        assert math.hypot(x, y) < tol
        assert abs(math.atan2(math.sin(yaw), math.cos(yaw))) < 0.1


def test_wave_plan_net_zero():
    total = sum(math.copysign(s.target, s.w) for s in wave_plan())
    assert abs(total) < 1e-9
    assert len(forward_back_plan(repeats=4)) == 8
