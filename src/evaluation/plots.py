"""可视化模块 -- 甘特图 + 轨迹图 v4.0

事件日志格式:
  move:    "A_1: -> (2,24) t=1 UP"
  work_start: "A_1: start DISASSEMBLE on M_y3_x2, duration=6"
  work_complete: "A_1: completed DISASSEMBLE on M_y3_x2, machine state=PENDING_INSPECTION"
"""

from __future__ import annotations

from pathlib import Path
import json
import re

import numpy as np


class GanttChart:
    """从事件日志生成甘特图。"""

    RID_RE = re.compile(r'^([AB]_\d+):')
    WORK_START_RE = re.compile(
        r'start (DISASSEMBLE|INSPECT|INSTALL) on (M_y\d+_x\d+)'
    )

    @staticmethod
    def build_gantt_data(event_log: list[dict]) -> dict:
        """从事件日志提取每个机器人的作业区间。

        Returns:
            {"robots": {rid: [{task, machine, op_type, start, end, duration}, ...]}, "makespan": int}
        """
        robot_tasks: dict[str, list[dict]] = {}
        pending: dict[str, dict] = {}  # rid -> {task, machine, op_type, start}
        makespan = 0

        for e in event_log:
            msg = e.get("message", "")
            etype = e.get("type", "")
            t = e.get("t", 0)

            rid_m = GanttChart.RID_RE.match(msg)
            if not rid_m:
                continue
            rid = rid_m.group(1)

            if etype == "work_start":
                wm = GanttChart.WORK_START_RE.search(msg)
                if wm:
                    pending[rid] = {
                        "task": f"{wm.group(2)}_{wm.group(1)[0]}",
                        "machine": wm.group(2),
                        "op_type": wm.group(1),
                        "start": t,
                    }

            elif etype == "work_complete":
                if rid in pending:
                    task = pending.pop(rid)
                    task["end"] = t
                    task["duration"] = t - task["start"]
                    robot_tasks.setdefault(rid, []).append(task)
                    makespan = max(makespan, t)

        return {"robots": robot_tasks, "makespan": makespan}

    @staticmethod
    def save_gantt_png(
        gantt_data: dict, filepath: str,
        title: str = "Robot Schedule Gantt Chart",
    ) -> None:
        try:
            import matplotlib.pyplot as plt
            from matplotlib.patches import Rectangle, Patch
        except ImportError:
            print("[WARN] matplotlib not installed")
            return

        robots = list(gantt_data["robots"].keys())
        if not robots:
            print("[WARN] No gantt data")
            return

        n = len(robots)
        fig, ax = plt.subplots(figsize=(22, 3 + n * 1.2))

        color_map = {"DISASSEMBLE": "#e53935", "INSPECT": "#ff9800",
                     "INSTALL": "#42a5f5"}

        for i, rid in enumerate(robots):
            for task in gantt_data["robots"][rid]:
                c = color_map.get(task["op_type"], "#999")
                ax.add_patch(Rectangle(
                    (task["start"], i - 0.38), task["duration"], 0.76,
                    facecolor=c, edgecolor="#333", linewidth=0.2, alpha=0.85,
                ))

        ax.set_yticks(range(n))
        ax.set_yticklabels(robots, fontsize=10)
        ax.set_xlabel("Time Step", fontsize=11)
        ax.set_title(title, fontsize=13, fontweight="bold")
        ax.set_xlim(0, gantt_data["makespan"] * 1.02)
        ax.grid(True, axis="x", alpha=0.3)

        legend_elements = [
            Patch(facecolor="#e53935", label="DISASSEMBLE (A, 6t)"),
            Patch(facecolor="#ff9800", label="INSPECT (B, 10t)"),
            Patch(facecolor="#42a5f5", label="INSTALL (A, 6t)"),
        ]
        ax.legend(handles=legend_elements, loc="upper right",
                  fontsize=8, ncol=3, framealpha=0.9)

        fig.tight_layout()
        fig.savefig(filepath, dpi=150, bbox_inches="tight")
        plt.close(fig)
        print(f"[IMG] Gantt PNG: {filepath}")


class TrajectoryPlot:
    """机器人运行轨迹可视化。"""

    MOVE_RE = re.compile(
        r'^([AB]_\d+):\s*->\s*\((\d+),(\d+)\)\s*t=(\d+)\s*(\w+)'
    )

    @staticmethod
    def build_trajectory_data(event_log: list[dict]) -> dict:
        """从事件日志提取轨迹。"""
        trajectories: dict[str, list[dict]] = {}

        for e in event_log:
            if e.get("type") != "move":
                continue
            m = TrajectoryPlot.MOVE_RE.match(e.get("message", ""))
            if not m:
                continue
            rid, x, y, t, action = m.group(1), int(m.group(2)), int(m.group(3)), int(m.group(4)), m.group(5)
            trajectories.setdefault(rid, []).append({
                "t": t, "x": x, "y": y, "action": action,
            })

        return trajectories

    @staticmethod
    def save_trajectory_json(trajectories: dict, filepath: str) -> None:
        Path(filepath).parent.mkdir(parents=True, exist_ok=True)
        with open(filepath, "w") as f:
            json.dump(trajectories, f, indent=2, ensure_ascii=False)
        print(f"[DATA] Trajectories: {filepath}")

    @staticmethod
    def save_trajectory_png(
        trajectories: dict,
        terrain: np.ndarray,
        machines: dict,
        filepath: str,
        title: str = "Robot Trajectories",
    ) -> None:
        """绘制轨迹静态图：在地图上画出每个机器人的完整路径。"""
        try:
            import matplotlib.pyplot as plt
        except ImportError:
            return

        h, w = terrain.shape
        fig, ax = plt.subplots(figsize=(16, 20))

        # 地图底色
        cmap = {0: "#1a1a1a", 1: "#f5f5f0", 2: "#c8e6c9"}
        for y in range(h):
            for x in range(w):
                ax.add_patch(plt.Rectangle(
                    (x, y), 1, 1,
                    facecolor=cmap.get(int(terrain[y, x]), "#f00"),
                    edgecolor="#ddd", linewidth=0.2,
                ))

        # 离心机
        for mid, m in machines.items():
            for c in m.cells:
                ax.add_patch(plt.Rectangle(
                    (c.x - 1, c.y - 1), 1, 1,
                    facecolor="#e53935", edgecolor="#b71c1c",
                    linewidth=0.5, alpha=0.7,
                ))

        # 轨迹
        colors = {"A": "#42a5f5", "B": "#66bb6a"}
        for rid, traj in trajectories.items():
            if not traj:
                continue
            rtype = "A" if rid.startswith("A_") else "B"
            xs = [p["x"] - 1 + 0.5 for p in traj]  # center of cell
            ys = [p["y"] - 1 + 0.5 for p in traj]
            ax.plot(xs, ys, color=colors.get(rtype, "#000"),
                    linewidth=1.5, alpha=0.7, label=f"{rid} path")
            # 起点标记
            ax.scatter(xs[0], ys[0], color=colors.get(rtype, "#000"),
                       s=80, marker="o", zorder=5)
            # 终点标记
            ax.scatter(xs[-1], ys[-1], color=colors.get(rtype, "#000"),
                       s=80, marker="X", zorder=5)

        ax.set_xlim(0, w)
        ax.set_ylim(h, 0)
        ax.set_xticks(range(w))
        ax.set_yticks(range(h))
        ax.set_title(title, fontsize=12, fontweight="bold")
        ax.legend(loc="lower right", fontsize=8)
        ax.grid(True, alpha=0.2)

        fig.tight_layout()
        fig.savefig(filepath, dpi=120, bbox_inches="tight")
        plt.close(fig)
        print(f"[IMG] Trajectory PNG: {filepath}")
