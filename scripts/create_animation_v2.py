"""
场景1 动画 v2 — 离心机状态颜色 + 工作区对齐 + 2fps
"""
from __future__ import annotations
import json, sys, re, math
from pathlib import Path
from collections import defaultdict
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from matplotlib.animation import FuncAnimation, FFMpegWriter
try:
    import imageio_ffmpeg
except ImportError:
    imageio_ffmpeg = None

PROJECT = Path(__file__).resolve().parent.parent
EXP = sys.argv[1] if len(sys.argv) > 1 else "260702_test4"
DATA = PROJECT / "outputs" / "scenario_1" / EXP

# ===== Load data =====
from src.map.fixed_map import FixedMap
fm = FixedMap()
terrain, machines_dict, _ = fm.build()
h, w = terrain.shape

with open(DATA / "trajectories.json") as f: trajectories = json.load(f)
with open(DATA / "event_log.jsonl") as f: event_log = [json.loads(l) for l in f]

# ===== Build time index for robot positions =====
time_index: dict[int, dict] = defaultdict(dict)
for rid, pts in trajectories.items():
    for p in pts:
        time_index[int(p["t"])][rid] = (int(p["x"]), int(p["y"]))

all_t = sorted(time_index.keys())
simulation_end_t = max(int(e["t"]) for e in event_log)

# Forward-fill robot positions to cover gaps during work time
# (safety net in case trajectory data still has gaps after plots.py fix)
for rid in trajectories:
    last_pos = None
    for t in range(min(all_t), simulation_end_t + 1):
        if t in time_index and rid in time_index[t]:
            last_pos = time_index[t][rid]
        elif last_pos is not None:
            time_index[t][rid] = last_pos
display_t = list(range(max(1, min(all_t)), simulation_end_t + 1))

# ===== Build machine state timeline from event_log =====
work_re = re.compile(r'completed (DISASSEMBLE|INSPECT|INSTALL) on (M_y\d+_x\d+)')
machine_states: dict[int, dict[str, int]] = defaultdict(dict)
running: dict[str, int] = {}
for e in event_log:
    m = work_re.search(e.get("message", ""))
    if m:
        op, mid = m.group(1), m.group(2)
        if op == "DISASSEMBLE": running[mid] = 8
        elif op == "INSPECT": running[mid] = 9
        elif op == "INSTALL": running[mid] = 10
        machine_states[e["t"]][mid] = running[mid]

# Fill forward
cum_states: dict[int, dict[str, int]] = {}
last: dict[str, int] = {}
for t in range(simulation_end_t + 1):
    if t in machine_states: last.update(machine_states[t])
    cum_states[t] = dict(last)

state_colors = {7: "#e53935", 8: "#ff9800", 9: "#ffc107", 10: "#4caf50"}

# ===== Identify which robot is working at each time =====
work_times: dict[int, dict] = defaultdict(dict)
for e in event_log:
    if e["type"] == "work_start":
        m = re.match(r'(A_\d+|B_\d+): start (\w+) on (M_y\d+_x\d+)', e["message"])
        if m: work_times[e["t"]][m.group(1)] = {"op": m.group(2), "machine": m.group(3)}
    elif e["type"] == "work_complete":
        m = re.match(r'(A_\d+|B_\d+): completed', e["message"])
        if m: work_times[e["t"]][m.group(1)] = None

# Build continuous work intervals
robot_working: dict[str, list[tuple[int,int,str]]] = defaultdict(list)  # rid -> [(start,end,machine_id)]
current_work: dict[str, tuple[int,str]] = {}
for t in sorted(work_times.keys()):
    for rid, info in work_times[t].items():
        if info is None:
            if rid in current_work:
                start, mid = current_work.pop(rid)
                robot_working[rid].append((start, t, mid))
        else:
            current_work[rid] = (t, info["machine"])

# ===== Sampling: cover the complete simulation with a bounded frame count =====
# argv[2] controls the approximate maximum frames. State changes are always
# inserted as keyframes, so a long livelock interval is visible without trying
# to render tens of thousands of identical frames.
max_frames = int(sys.argv[2]) if len(sys.argv) > 2 else 600
step = max(1, math.ceil(len(display_t) / max_frames))
sampled_t = [display_t[i] for i in range(0, len(display_t), step)]

# Insert keyframes at state change moments (redundant with step=1 but safe)
state_change_times = sorted(machine_states.keys())
keyframe_t = [t for t in state_change_times if t >= 1]
keyframe_t.append(simulation_end_t)
all_frames = sorted(set(sampled_t + keyframe_t))
deduped = []
for t in all_frames:
    if not deduped or t - deduped[-1] >= 1:
        deduped.append(t)
sampled_t = deduped
n_frames = len(sampled_t)

# 固定 10 FPS；帧采样由 step 单独控制，step=1 时仍逐时间步展示。
fps = 10

print(f"Animation: {n_frames} frames, {n_frames/fps:.0f}s @ {fps}fps (step={step})")
print(f"t=[{sampled_t[0]}..{sampled_t[-1]}], keyframes={len(keyframe_t)}")

# ===== Figure =====
fig, ax = plt.subplots(figsize=(18, 14))

# Terrain
terrain_cmap = {0: "#1a1a1a", 1: "#fafaf5", 2: "#c8e6c9"}
for y in range(h):
    for x in range(w):
        ax.add_patch(Rectangle((x, y), 1, 1, facecolor=terrain_cmap.get(int(terrain[y,x]), "#f00"),
                               edgecolor="#e0e0e0", linewidth=0.08, zorder=0))

# Trunk boundary
ax.axhline(y=23, color="#ff9800", linewidth=2, linestyle="--", alpha=0.5)
ax.text(12.5, 23.5, "TRUNK (horizontal moves allowed)", ha="center", fontsize=8, color="#e65100")

# Machine patches (dynamic colors)
machine_rects: dict[str, Rectangle] = {}
for mid, m in machines_dict.items():
    for c in m.cells:
        key = f"{c.x},{c.y}"
        rect = Rectangle((c.x-1, c.y-1), 1, 1, facecolor=state_colors[7],
                          edgecolor="#333", linewidth=0.3, alpha=0.85, zorder=3)
        ax.add_patch(rect)
        machine_rects[key] = rect

# Robot patches (hidden initially)
robot_colors = {
    "A_1": "#1565c0",
    "A_2": "#7b1fa2",
    "B_1": "#2e7d32",
}
robot_rects: dict[str, Rectangle] = {}
robot_wz_rects: dict[str, Rectangle] = {}  # work zone highlight
for rid in trajectories:
    color = robot_colors.get(rid, "#616161")
    robot_rects[rid] = Rectangle((-10, -10), 2, 4, facecolor=color,
                                  edgecolor="#111", linewidth=1.5, alpha=0.9, zorder=10)
    ax.add_patch(robot_rects[rid])
    # Work zone highlight (top 2 cells of footprint)
    robot_wz_rects[rid] = Rectangle((-10, -10), 2, 1, facecolor="none",
                                     edgecolor="#ffeb3b", linewidth=2.5, alpha=0, zorder=11,
                                     linestyle="-")
    ax.add_patch(robot_wz_rects[rid])

# Trail lines
trails: dict[str, tuple[list,list]] = {}
trail_lines: dict[str, object] = {}
for rid in trajectories:
    trails[rid] = ([], [])
    (line,) = ax.plot([], [], color=robot_colors.get(rid, "#616161"), linewidth=1.2, alpha=0.3, zorder=5)
    trail_lines[rid] = line

# State count text
state_text = ax.text(0.5, -1.2, "", transform=ax.transAxes, fontsize=10, fontfamily="monospace",
                     ha="center", va="top")

ax.set_xlim(0, w)
ax.set_ylim(h, 0)
ax.set_xticks(range(w))
ax.set_yticks(range(h))
ax.set_xticklabels([str(x) for x in range(1, w+1)], fontsize=5)
ax.set_yticklabels([str(y) for y in range(1, h+1)], fontsize=5)
ax.grid(True, alpha=0.1, linewidth=0.15)
ax.set_aspect("equal")
title = ax.set_title("", fontsize=13, fontweight="bold")

# Work zone alignment verification text
align_text = ax.text(0.98, 0.02, "", transform=ax.transAxes, fontsize=8,
                     fontfamily="monospace", ha="right", va="bottom",
                     bbox=dict(boxstyle="round", facecolor="white", alpha=0.8))

# ===== Update function =====
def update(i):
    t = sampled_t[i]
    mins, secs = t // 60, t % 60
    title.set_text(f"Scenario 1 (1A1B) — t={t} ({mins}m{secs}s) — 48 centrifuges D+I+R")

    # Update machine colors
    st = cum_states.get(t, {})
    counts = {7: 0, 8: 0, 9: 0, 10: 0}
    for mid in machines_dict:
        s = st.get(mid, 7)
        counts[s] = counts.get(s, 0) + 1
        for c in machines_dict[mid].cells:
            key = f"{c.x},{c.y}"
            if key in machine_rects:
                machine_rects[key].set_facecolor(state_colors.get(s, "#999"))

    state_text.set_text(
        f"State7(PendDisasm):{counts[7]}  "
        f"State8(PendInsp):{counts[8]}  "
        f"State9(PendInst):{counts[9]}  "
        f"State10(Done):{counts[10]}"
    )

    # Update robots
    align_info = []
    for rid in trajectories:
        if t in time_index and rid in time_index[t]:
            x, y = time_index[t][rid]
            robot_rects[rid].set_xy((x-1, y-1))
            robot_rects[rid].set_visible(True)

            # Work zone: top 2 cells of footprint — cells (x,y) and (x+1,y)
            wz_rect = robot_wz_rects[rid]
            wz_rect.set_xy((x-1, y-1))
            wz_rect.set_width(2)
            wz_rect.set_height(1)

            # Is this robot WORKING now?
            is_working = False
            for start_t, end_t, mid in robot_working.get(rid, []):
                if start_t <= t <= end_t:
                    is_working = True
                    m = machines_dict.get(mid)
                    if m:
                        m_cells = {(c.x, c.y) for c in m.cells}
                        wz_cells = {(x, y), (x+1, y)}
                        overlap = wz_cells == m_cells
                        align_info.append(
                            f"{rid}: work_zone{wz_cells} vs machine{m_cells} "
                            f"{'MATCH' if overlap else 'MISMATCH!'}"
                        )
                    break

            if is_working:
                wz_rect.set_alpha(0.7)
                wz_rect.set_facecolor("#ffeb3b")
            else:
                wz_rect.set_alpha(0)
                wz_rect.set_facecolor("none")

            # Trail
            tx, ty = trails[rid]
            tx.append(x - 0.5)
            ty.append(y - 0.5)
            trail_lines[rid].set_data(tx, ty)

    align_text.set_text("\n".join(align_info[:5]) if align_info else "")

    return []

# ===== Render =====
interval_ms = int(1000 / fps)
anim = FuncAnimation(fig, update, frames=n_frames, interval=interval_ms, blit=False)

ffmpeg_path = (
    imageio_ffmpeg.get_ffmpeg_exe()
    if imageio_ffmpeg is not None
    else "/opt/anaconda3/bin/ffmpeg"
)
plt.rcParams["animation.ffmpeg_path"] = ffmpeg_path
writer = FFMpegWriter(fps=fps, codec="h264", bitrate=2000, extra_args=["-pix_fmt", "yuv420p"])
out_path = DATA / "animation_low.mp4"
anim.save(str(out_path), writer=writer, dpi=100)
plt.close(fig)

size_mb = out_path.stat().st_size / 1024 / 1024
print(f"\nDone: {out_path}")
print(f"  Frames: {n_frames}, Duration: {n_frames/fps:.0f}s, {fps}fps")
print(f"  t range: [{sampled_t[0]}..{sampled_t[-1]}]")
print(f"  Size: {size_mb:.1f} MB")
