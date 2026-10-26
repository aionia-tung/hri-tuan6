#!/usr/bin/env python3
"""Vẽ đồ thị kết quả thực nghiệm từ CSV của record_run.py.

  python3 tools/plot_run.py <thu_muc_csv> <thu_muc_anh>
Đọc run_wave.csv, run_forward_back.csv, run_circle.csv, run_signal_loss.csv.
Toạ độ được đổi về hệ quy chiếu gắn với pose robot lúc bắt đầu mỗi kịch bản.
"""

import csv
import math
import os
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Patch  # noqa: E402

COLORS = {'IDLE': '#9e9e9e', 'SELECTED': '#1e88e5', 'EXECUTING': '#43a047',
          'PAUSED': '#e53935', 'GREETING': '#fb8c00'}


def load(path):
    rows = list(csv.DictReader(open(path)))
    x0, y0, th0 = (float(rows[0][k]) for k in ('x', 'y', 'yaw'))
    c, s = math.cos(-th0), math.sin(-th0)
    out = []
    for r in rows:
        dx, dy = float(r['x']) - x0, float(r['y']) - y0
        yaw = math.atan2(math.sin(float(r['yaw']) - th0), math.cos(float(r['yaw']) - th0))
        out.append(dict(t=float(r['t']), x=c * dx - s * dy, y=s * dx + c * dy, yaw=yaw,
                        v=float(r['v_cmd']), w=float(r['w_cmd']), gesture=r['gesture'],
                        state=r['state'].split(':')[0].split(' |')[0].strip() or 'IDLE'))
    t0, offset = out[0]['t'], 0.0
    for prev, r in zip([None] + out[:-1], out):
        r['t'] -= t0
        if prev is not None:   # unwrap yaw để đồ thị liên tục khi vượt ±pi
            d = r['yaw'] + offset - prev['yaw_u']
            offset -= 2 * math.pi * round(d / (2 * math.pi))
        r['yaw_u'] = r['yaw'] + offset
    return out


def shade_states(ax, rows):
    start, state = rows[0]['t'], rows[0]['state']
    for r in rows[1:] + [dict(t=rows[-1]['t'], state=None)]:
        if r['state'] != state:
            ax.axvspan(start, r['t'], color=COLORS.get(state, '#ffffff'), alpha=0.15, lw=0)
            start, state = r['t'], r['state']


def mark_gestures(ax, rows):
    last = 'None'
    for r in rows:
        if r['gesture'] != last and r['gesture'] != 'None':
            ax.axvline(r['t'], color='k', ls=':', lw=0.8)
            ax.text(r['t'], ax.get_ylim()[1], ' ' + r['gesture'], rotation=90, va='top',
                    fontsize=8)
        last = r['gesture']


def timeline(rows, title, path, show='both'):
    fig, axes = plt.subplots(2 if show == 'both' else 1, 1, figsize=(9, 5 if show == 'both' else 3),
                             sharex=True, squeeze=False)
    axes = axes[:, 0]
    i = 0
    if show in ('both', 'v'):
        ax = axes[i]
        ax.plot([r['t'] for r in rows], [r['v'] for r in rows], label='v lệnh (m/s)', color='#1e88e5')
        ax.plot([r['t'] for r in rows], [r['x'] for r in rows], label='x odom (m)', color='#43a047')
        ax.set_ylabel('m, m/s')
        i += 1
    if show in ('both', 'w'):
        ax = axes[i]
        ax.plot([r['t'] for r in rows], [r['w'] for r in rows], label='w lệnh (rad/s)', color='#fb8c00')
        ax.plot([r['t'] for r in rows], [r['yaw_u'] for r in rows], label='yaw odom (rad)', color='#8e24aa')
        ax.set_ylabel('rad, rad/s')
    for ax in axes:
        ax.grid(alpha=0.3)
        ax.legend(loc='lower right', fontsize=8)
        shade_states(ax, rows)
    mark_gestures(axes[0], rows)
    axes[-1].set_xlabel('thời gian thực (s)')
    fig.legend(handles=[Patch(color=c, alpha=0.3, label=k) for k, c in COLORS.items()],
               loc='lower center', ncol=5, fontsize=8, frameon=False)
    axes[0].set_title(title)
    fig.tight_layout(rect=(0, 0.05, 1, 1))
    fig.savefig(path, dpi=130)
    plt.close(fig)


def trajectories(data, path):
    fig, axes = plt.subplots(1, 3, figsize=(12, 4))
    for ax, (name, title) in zip(axes, [('forward_back', 'Pointing_Up + Thumb_Up: tiến-lùi ×4'),
                                         ('circle', 'Victory + Thumb_Up: quỹ đạo tròn r = 0.5 m'),
                                         ('wave', 'Wave: xoay trái-phải chào lại')]):
        rows = data.get(name)
        if not rows:
            continue
        if name == 'wave':
            ax.plot([r['t'] for r in rows], [math.degrees(r['yaw']) for r in rows], color='#fb8c00')
            ax.set_xlabel('t (s)')
            ax.set_ylabel('yaw (độ)')
        elif name == 'forward_back':   # chuyển động 1 chiều -> vẽ x theo thời gian
            ax.plot([r['t'] for r in rows], [r['x'] for r in rows], color='#43a047')
            ax.axhline(0.3, color='k', ls='--', lw=0.8, label='đích 0.3 m')
            ax.set_xlabel('t (s)')
            ax.set_ylabel('x (m)')
            ax.legend(fontsize=7)
        else:
            for state, color in COLORS.items():
                pts = [(r['x'], r['y']) for r in rows if r['state'] == state]
                if pts:
                    ax.scatter(*zip(*pts), s=3, color=color, label=state)
            if name == 'circle':
                a = [i * 2 * math.pi / 200 for i in range(201)]
                ax.plot([0.5 * math.sin(t) for t in a], [0.5 - 0.5 * math.cos(t) for t in a], 'k--',
                        lw=0.8, label='lý thuyết')
            ax.set_aspect('equal')
            ax.set_xlabel('x (m)')
            ax.set_ylabel('y (m)')
            ax.legend(fontsize=7, markerscale=3)
        ax.set_title(title, fontsize=10)
        ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


def summary(data):
    for name, rows in data.items():
        end = rows[-1]
        moving = [r for r in rows if abs(r['v']) > 1e-3 or abs(r['w']) > 1e-3]
        dur = moving[-1]['t'] - moving[0]['t'] if moving else 0.0
        print(f'{name}: ket thuc x={end["x"]:+.3f} y={end["y"]:+.3f} '
              f'yaw={math.degrees(end["yaw"]):+.1f} do, max|x|={max(abs(r["x"]) for r in rows):.3f}, '
              f'max|y|={max(abs(r["y"]) for r in rows):.3f}, '
              f'max yaw={math.degrees(max(r["yaw"] for r in rows)):+.1f}, '
              f'min yaw={math.degrees(min(r["yaw"] for r in rows)):+.1f}, thoi gian chay {dur:.1f} s')


def main():
    src, dst = sys.argv[1], sys.argv[2]
    os.makedirs(dst, exist_ok=True)
    data = {}
    for name in ('wave', 'forward_back', 'circle', 'signal_loss'):
        path = os.path.join(src, f'run_{name}.csv')
        if os.path.exists(path):
            data[name] = load(path)
    trajectories(data, os.path.join(dst, 'trajectories.png'))
    if 'wave' in data:
        timeline(data['wave'], 'Wave → chào lại (xoay trái-phải, về hướng cũ)',
                 os.path.join(dst, 'timeline_wave.png'), 'w')
    if 'forward_back' in data:
        timeline(data['forward_back'], 'Pointing_Up (chọn) → Thumb_Up (xác nhận) → tiến-lùi 4 lần',
                 os.path.join(dst, 'timeline_forward_back.png'), 'v')
    if 'circle' in data:
        timeline(data['circle'], 'Victory → Thumb_Up → Open_Palm (dừng) → Thumb_Up (tiếp tục)',
                 os.path.join(dst, 'timeline_circle.png'))
    if 'signal_loss' in data:
        timeline(data['signal_loss'], 'Mất tín hiệu /detected_gesture > 1 s → robot tự dừng',
                 os.path.join(dst, 'timeline_signal_loss.png'))
    summary(data)


if __name__ == '__main__':
    main()
