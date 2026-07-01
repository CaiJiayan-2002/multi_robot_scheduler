"""
场景1 机器人运行动画 v2
- 修复: 机器人初始位置 + 离心机状态变化 + t0=1
- 10 fps, GIF 输出
"""
from __future__ import annotations

import json
import sys
from collections import defaultdict
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from matplotlib.animation import FuncAnimation, PillowWriter


PROJECT = Path(__file__).resolve().parent.parent
OUTPUT_DIR = PROJECT / "outputs" / "scenario_1"


def load_all():
    from src.map.fixed_map import FixedMap
    fm = FixedMap()
    terrain, machines_dict, _ = fm.build()

    with open(OUTPUT_DIR / "trajectories.json") as f:
        trajectories = json.load(f)

    event_log = []
    with open(OUTPUT_DIR / "event_log.jsonl") as f:
        for line in f:
            event_log.append(json.loads(line))

    return terrain, machines_dict, trajectories, event_log


def build_time_index(trajectories: dict) -> dict:
    """每帧每台机器人位置: {t: {rid: (x, y)}}"""
    idx: dict[int, dict] = defaultdict(dict)
    for rid, pts in trajectories.items():
        for p in pts:
            idx[int(p["t"])][rid] = (int(p["x"]), int(p["y"]))
    return dict(idx)


def build_machine_states(event_log: list[dict]) -> dict:
    """从 event_log 追踪每台离心机在每个时刻的状态。
    work_complete 事件: "A_1: completed DISASSEMBLE on M_y3_x2, machine state=PENDING_INSPECTION"
    初始所有机器 state=7 (PENDING_DISASSEMBLY)
    DISASSEMBLE完成后→8, INSPECT完成后→9, INSTALL完成后→10
    """
    import re
    state_map: dict[int, dict[str, int]] = defaultdict(dict)  # t -> {mid: state}
    current_states: dict[str, int] = {}  # 所有机器初始=7

    work_re = re.compile(r'completed (DISASSEMBLE|INSPECT|INSTALL) on (M_y\d+_x\d+).*state=(PENDING_\w+|COMPLETED)')

    for e in event_log:
        t = e["t"]
        m = work_re.search(e.get("message", ""))
        if m:
            op_type = m.group(1)
            mid = m.group(2)

            if op_type == "DISASSEMBLE":
                current_states[mid] = 8
            elif op_type == "INSPECT":
                current_states[mid] = 9
            elif op_type == "INSTALL":
                current_states[mid] = 10

            state_map[t][mid] = current_states[mid]

    return dict(state_map)


def build_cumulative_states(state_changes: dict, max_t: int) -> dict:
    """构建累积状态快照: {t: {mid: state}}"""
    cumulative: dict[int, dict[str, int]] = {}
    running: dict[str, int] = {}

    for t in sorted(state_changes.keys()):
        running.update(state_changes[t])
        cumulative[t] = dict(running)

    # 向前填充
    result: dict[int, dict[str, int]] = {}
    last_states: dict[str, int] = {}
    for t in range(max_t + 1):
        if t in cumulative:
            last_states.update(cumulative[t])
        result[t] = dict(last_states)

    return result


def main():
    terrain, machines_dict, trajectories, event_log = load_all()
    h, w = terrain.shape

    # 数据索引
    time_index = build_time_index(trajectories)
    all_times_with_moves = sorted(time_index.keys())
    t_start = max(1, all_times_with_moves[0])  # t0=1
    t_end = all_times_with_moves[-1]

    # 过滤 t>=1 的时间
    filtered_times = [t for t in all_times_with_moves if t >= t_start]

    # 机器状态追踪
    state_changes = build_machine_states(event_log)
    cumulative_states = build_cumulative_states(state_changes, t_end)

    n_total = len(filtered_times)
    sample_step = max(1, n_total // 600)
    sampled_indices = list(range(0, n_total, sample_step))
    sampled_times = [filtered_times[i] for i in sampled_indices]
    n_frames = len(sampled_times)

    print(f"Animation: {n_frames} frames from {n_total} timesteps "
          f"(step={sample_step}), t=[{t_start}..{t_end}], 10fps")
    print(f"Robots: {list(trajectories.keys())}")
    for rid in trajectories:
        print(f"  {rid}: {len(trajectories[rid])} pts, "
              f"t=[{trajectories[rid][0]['t']}..{trajectories[rid][-1]['t']}]")

    # ===== 设置画布 =====
    fig, (ax_map, ax_info) = plt.subplots(
        1, 2, figsize=(22, 15),
        gridspec_kw={"width_ratios": [3, 1]},
    )

    # --- 地图底色 ---
    color_map = {0: "#1a1a1a", 1: "#fafaf5", 2: "#c8e6c9"}
    for y_idx in range(h):
        for x_idx in range(w):
            ax_map.add_patch(Rectangle(
                (x_idx, y_idx), 1, 1,
                facecolor=color_map.get(int(terrain[y_idx, x_idx]), "#f00"),
                edgecolor="#e0e0e0", linewidth=0.1, zorder=0,
            ))

    # 主干道分界线
    ax_map.axhline(y=23, color="#ff9800", linewidth=2, linestyle="--", alpha=0.6)

    # --- 离心机 (动态颜色) ---
    machine_patches: dict[str, Rectangle] = {}
    state_colors = {
        7: "#e53935",   # PENDING_DISASSEMBLY 红
        8: "#ff9800",   # PENDING_INSPECTION  橙
        9: "#ffc107",   # PENDING_INSTALL     黄
        10: "#4caf50",  # COMPLETED           绿
    }
    for mid, m in machines_dict.items():
        for c in m.cells:
            patch = Rectangle(
                (c.x - 1, c.y - 1), 1, 1,
                facecolor=state_colors[7],  # 初始=待拆(红)
                edgecolor="#333", linewidth=0.3, alpha=0.8, zorder=3,
            )
            ax_map.add_patch(patch)
            machine_patches[f"{c.x},{c.y}"] = patch

    # --- 机器人 (动态，初始隐藏) ---
    robot_colors = {"A_1": "#1565c0", "B_1": "#2e7d32"}
    robot_patches: dict[str, Rectangle] = {}
    robot_labels: dict[str, object] = {}

    # 先用 -10,-10 把机器人放到视野外
    for rid in trajectories:
        robot_patches[rid] = Rectangle(
            (-10, -10), 2, 4,
            facecolor=robot_colors.get(rid, "#999"),
            edgecolor="#111", linewidth=2, alpha=0.9, zorder=20,
        )
        ax_map.add_patch(robot_patches[rid])
        robot_labels[rid] = ax_map.text(
            -10, -10, rid, ha="center", va="center",
            fontsize=7, fontweight="bold", color="white", zorder=21,
        )

    # --- 轨迹线 ---
    trail_lines: dict[str, object] = {}
    trail_data: dict[str, tuple[list, list]] = {}
    for rid in trajectories:
        trail_data[rid] = ([], [])
        (line,) = ax_map.plot(
            [], [], color=robot_colors.get(rid, "#999"),
            linewidth=1.8, alpha=0.4, linestyle="-", zorder=10,
        )
        trail_lines[rid] = line

    # --- 轴设置 ---
    ax_map.set_xlim(0, w)
    ax_map.set_ylim(h, 0)
    ax_map.set_xticks(range(w))
    ax_map.set_yticks(range(h))
    ax_map.set_xticklabels([str(x) for x in range(1, w+1)], fontsize=5)
    ax_map.set_yticklabels([str(y) for y in range(1, h+1)], fontsize=5)
    ax_map.grid(True, alpha=0.12, linewidth=0.2)
    ax_map.set_aspect("equal")

    title_txt = ax_map.set_title("", fontsize=13, fontweight="bold")

    # 图例
    from matplotlib.patches import Patch
    ax_map.legend(handles=[
        Patch(facecolor="#1a1a1a", label="Obstacle"),
        Patch(facecolor="#fafaf5", label="Internal"),
        Patch(facecolor="#c8e6c9", label="Trunk"),
        Patch(facecolor="#e53935", label="State7:PendDisasm"),
        Patch(facecolor="#ff9800", label="State8:PendInsp"),
        Patch(facecolor="#ffc107", label="State9:PendInst"),
        Patch(facecolor="#4caf50", label="State10:Done"),
        Patch(facecolor="#1565c0", label="A_1"),
        Patch(facecolor="#2e7d32", label="B_1"),
    ], loc="lower right", fontsize=6, ncol=5, framealpha=0.85)

    # --- 右信息面板 ---
    ax_info.set_xlim(0, 10)
    ax_info.set_ylim(0, 10)
    ax_info.axis("off")
    ax_info.text(5, 9.7, "SIMULATION STATUS", ha="center", fontsize=13, fontweight="bold")
    info_time = ax_info.text(5, 9.1, "", ha="center", fontsize=18, color="#c62828", fontweight="bold")
    info_progress = ax_info.text(5, 8.5, "", ha="center", fontsize=9)

    # 进度条
    progress_bg = Rectangle((1, 8.0), 8, 0.25, facecolor="#ddd", edgecolor="#999", linewidth=0.3)
    ax_info.add_patch(progress_bg)
    progress_bar = Rectangle((1, 8.0), 0, 0.25, facecolor="#4caf50", edgecolor="#2e7d32", linewidth=0.3)
    ax_info.add_patch(progress_bar)

    # 状态计数
    state_count_texts: dict[int, object] = {}
    for i, s in enumerate([7, 8, 9, 10]):
        y = 7.3 - i * 0.5
        ax_info.add_patch(Rectangle((1, y - 0.15), 0.3, 0.3,
                                    facecolor=state_colors[s], edgecolor="#333", linewidth=0.3))
        state_count_texts[s] = ax_info.text(1.5, y, "", fontsize=8, fontfamily="monospace", va="center")

    # 机器人状态
    robot_info_a = ax_info.text(1, 4.5, "", fontsize=8, fontfamily="monospace", va="top",
                                 color="#1565c0")
    robot_info_b = ax_info.text(1, 3.0, "", fontsize=8, fontfamily="monospace", va="top",
                                 color="#2e7d32")

    # 元信息
    ax_info.text(1, 1.5, "10 fps  |  t0=1  |  GIF",
                 fontsize=7, fontfamily="monospace", va="top", color="#888")
    ax_info.text(1, 0.8, "D=Disasm(A,6t) I=Insp(B,10t) R=Inst(A,6t)",
                 fontsize=6, fontfamily="monospace", va="top", color="#aaa")

    # ===== 动画帧更新 =====
    done_count = 0

    def update(frame_no):
        """frame_no: sampled_indices 中的索引"""
        orig_idx = sampled_indices[frame_no]
        t = filtered_times[orig_idx]

        # 标题
        title_txt.set_text(
            f"Scenario 1 (1A1B) -- 48 Centrifuges -- "
            f"DISMANTLE + INSPECT + INSTALL -- t={t}"
        )

        # 时间
        minutes = t // 60
        seconds = t % 60
        info_time.set_text(f"t = {t}  ({minutes}m {seconds}s)")

        # --- 更新离心机颜色 ---
        states_now = cumulative_states.get(t, {})
        counts = {7: 48, 8: 0, 9: 0, 10: 0}
        for mid, s in states_now.items():
            counts[7] -= 1
            counts[s] += 1
            m = machines_dict.get(mid)
            if m:
                for c in m.cells:
                    key = f"{c.x},{c.y}"
                    if key in machine_patches:
                        machine_patches[key].set_facecolor(state_colors.get(s, "#999"))

        # 更新状态计数文本
        info_progress.set_text(
            f"Completed: {counts[10]}/48 machines  |  "
            f"Ops done: {counts[8]+counts[9]+counts[10]*2}/144"
        )
        progress_bar.set_width(8 * counts[10] / 48)

        for s in [7, 8, 9, 10]:
            state_count_texts[s].set_text(
                f"State{s}: {counts[s]}"
            )

        # --- 更新机器人位置 ---
        robots_now = time_index.get(t, {})
        a_info = ""
        b_info = ""

        for rid in trajectories:
            if rid in robots_now:
                x, y = robots_now[rid]
                # 设置 footprint
                robot_patches[rid].set_xy((x - 1, y - 1))
                robot_patches[rid].set_visible(True)
                robot_labels[rid].set_position((x - 1 + 1, y - 1 + 2))
                robot_labels[rid].set_visible(True)

                # 轨迹
                tx, ty = trail_data[rid]
                tx.append(x - 1 + 1)
                ty.append(y - 1 + 2)
                trail_lines[rid].set_data(tx, ty)

                info_line = (
                    f"{rid}: ({x},{y})  "
                    f"trail={len(tx)}pts"
                )
                if rid.startswith("A_"):
                    a_info = info_line
                else:
                    b_info = info_line

        robot_info_a.set_text(a_info)
        robot_info_b.set_text(b_info)

        return []

    # ===== 渲染 =====
    anim = FuncAnimation(
        fig, update, frames=n_frames,
        interval=100, blit=False,
    )

    output_path = OUTPUT_DIR / "animation.gif"
    writer = PillowWriter(fps=10)
    anim.save(str(output_path), writer=writer, dpi=90)
    plt.close(fig)

    size_mb = output_path.stat().st_size / 1024 / 1024
    print(f"\n[DONE] {output_path}")
    print(f"  Frames: {n_frames}, Duration: {n_frames/10:.0f}s")
    print(f"  t range: [{t_start}..{t_end}], FPS: 10")
    print(f"  Size: {size_mb:.1f} MB")
    print(f"  Sim speed: ~{sample_step*10} timesteps/sec")


if __name__ == "__main__":
    main()
