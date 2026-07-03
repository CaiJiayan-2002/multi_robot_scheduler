"""
地图可视化脚本 — 输出数字矩阵 + 彩色图片
"""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import numpy as np
from src.map.fixed_map import FixedMap
from src.domain.enums import TerrainCode


def main():
    fm = FixedMap()
    terrain, machines, operations = fm.build()

    # ===== 1. 带坐标的数字矩阵 (x=1..25 列, y=1..29 行) =====
    print("=" * 80)
    print("  固定地图 — 数字矩阵 (0=障碍物 1=内部道路 2=主干道)")
    print("  列:x(1..25)  行:y(1..29)  原点:左上角")
    print("=" * 80)

    # 列标题
    header = "y\\x " + "".join(f"{x:2d}" for x in range(1, 26))
    print(header)
    print("-" * 80)

    for y_idx in range(29):
        y = y_idx + 1
        row_str = f"{y:3d}  "
        for x_idx in range(25):
            x = x_idx + 1
            val = terrain[y_idx, x_idx]

            # 检查这个位置是否有离心机
            cell_has_machine = False
            for mid, m in machines.items():
                if any(c.x == x and c.y == y for c in m.cells):
                    cell_has_machine = True
                    break

            if cell_has_machine:
                row_str += " M"  # M = Machine
            elif val == TerrainCode.OBSTACLE.value:
                row_str += "##"
            elif val == TerrainCode.TRUNK_ROAD.value:
                row_str += "=="
            else:
                row_str += " ."

        # 行标注
        if y == 1:
            row_str += "  ← 顶部"
        elif y == 23:
            row_str += "  ← 内部/主干道分界 (y=23以上为内部, y=24起为主干道)"
        elif y == 24:
            row_str += "  ← 主干道起点"
        elif y == 29:
            row_str += "  ← 底部"
        elif y in (3, 7, 11, 15, 19):
            row_str += "  ← 离心机行"
        print(row_str)

    print("-" * 80)
    print("  图例: ## = 障碍物(墙)   . = 内部通道   == = 主干道   M = 离心机(2格横向)")
    print("=" * 80)

    # ===== 2. 离心机分布摘要 =====
    print(f"\n离心机分布:")
    for row in FixedMap.MACHINE_ROWS:
        row_machines = [mid for mid, m in machines.items() if m.row == row]
        positions = [(m.cells[0].x, m.cells[1].x) for m in machines.values() if m.row == row]
        print(f"  行 y={row}: {len(row_machines)}台, 列区间: {positions}")

    # ===== 3. 生成 PNG 图片 =====
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle

    fig, ax = plt.subplots(figsize=(18, 14))

    # 颜色映射
    color_map = {
        0: "#1a1a1a",  # 障碍物 深黑
        1: "#f5f5f0",  # 内部道路 米白
        2: "#c8e6c9",  # 主干道 浅绿
    }

    # 画地图底色
    for y_idx in range(29):
        for x_idx in range(25):
            val = terrain[y_idx, x_idx]
            ax.add_patch(Rectangle(
                (x_idx, y_idx), 1, 1,
                facecolor=color_map.get(int(val), "#ff0000"),
                edgecolor="#cccccc", linewidth=0.3,
            ))

    # 画离心机 (红色)
    for mid, m in machines.items():
        for c in m.cells:
            ax.add_patch(Rectangle(
                (c.x - 1, c.y - 1), 1, 1,
                facecolor="#e53935", edgecolor="#b71c1c", linewidth=0.8,
                alpha=0.85,
            ))

    # 标注
    ax.set_xlim(0, 25)
    ax.set_ylim(29, 0)  # 翻转 Y 轴，使 y=1 在顶部
    ax.set_xticks([i + 0.5 for i in range(25)])
    ax.set_xticklabels([str(x) for x in range(1, 26)], fontsize=6)
    ax.set_yticks([i + 0.5 for i in range(29)])
    ax.set_yticklabels([str(y) for y in range(1, 30)], fontsize=6)
    ax.grid(True, color="#cccccc", linewidth=0.3, alpha=0.5)

    # 分界线
    ax.axhline(y=23, color="#ff9800", linewidth=2, linestyle="--", alpha=0.8)
    ax.text(12.5, 23.5, "内部区域/主干道分界线 (y=24起可水平移动)",
            ha="center", fontsize=9, color="#e65100", fontweight="bold",
            bbox=dict(boxstyle="round,pad=0.3", facecolor="white", alpha=0.85))

    # 障碍柱标注
    for col_1 in (4, 10, 16, 22):
        ax.axvline(x=col_1 - 0.5, color="#666666", linewidth=0.5, alpha=0.3, ymax=23/29)
        ax.axvline(x=col_1 + 0.5, color="#666666", linewidth=0.5, alpha=0.3, ymax=23/29)
    ax.text(12.5, -0.8, "障碍柱: x=4,10,16,22 (在内部区域不可通行)",
            ha="center", fontsize=8, color="#666666")

    ax.set_title("Multi-Robot Scheduling — Fixed Map (25x29)\n"
                 "Red=Machines(48)  Dark=Obstacles  Beige=Internal Road  Green=Trunk Road",
                 fontsize=13, fontweight="bold")

    # 图例
    from matplotlib.patches import Patch
    legend_elements = [
        Patch(facecolor="#1a1a1a", label="Obstacle (0)"),
        Patch(facecolor="#f5f5f0", label="Internal Road (1)"),
        Patch(facecolor="#c8e6c9", label="Trunk Road (2)"),
        Patch(facecolor="#e53935", label=f"Machine ({len(machines)} total)"),
    ]
    ax.legend(handles=legend_elements, loc="lower right",
              fontsize=9, ncol=4, framealpha=0.9)

    output_path = Path(__file__).resolve().parent.parent / "outputs" / "fixed_map.png"
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.tight_layout(pad=1.5)
    fig.savefig(str(output_path), dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"\n[IMG] 地图图片已保存: {output_path}")


if __name__ == "__main__":
    main()
