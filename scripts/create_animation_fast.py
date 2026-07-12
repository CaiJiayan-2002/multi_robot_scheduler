"""Fast PIL-based MP4 renderer for simulation trajectories.

This renderer is intentionally simpler than the Matplotlib version:

- renders every simulation time step by default, so robot motion does not skip
  intermediate grid cells;
- draws trails from consecutive trajectory points, avoiding straight lines
  across skipped samples;
- uses PIL + imageio/ffmpeg for much faster encoding.
"""
from __future__ import annotations

import json
import re
import sys
from collections import defaultdict
from pathlib import Path

import imageio.v2 as imageio
import numpy as np
from PIL import Image, ImageDraw, ImageFont

PROJECT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT))

from src.map.fixed_map import FixedMap


def load_font(size: int) -> ImageFont.ImageFont:
    for path in (
        "/System/Library/Fonts/Supplemental/Arial.ttf",
        "/System/Library/Fonts/Helvetica.ttc",
    ):
        try:
            return ImageFont.truetype(path, size)
        except Exception:
            pass
    return ImageFont.load_default()


def parse_args() -> tuple[str, str, int, int]:
    exp = sys.argv[1] if len(sys.argv) > 1 else "test1"
    scenario_dir = sys.argv[2] if len(sys.argv) > 2 else "scenario_2"
    fps = int(sys.argv[3]) if len(sys.argv) > 3 else 10
    cell = int(sys.argv[4]) if len(sys.argv) > 4 else 24
    return exp, scenario_dir, fps, cell


def build_time_index(trajectories: dict, end_t: int) -> dict[int, dict[str, tuple[int, int]]]:
    raw: dict[int, dict[str, tuple[int, int]]] = defaultdict(dict)
    for rid, points in trajectories.items():
        for point in points:
            raw[int(point["t"])][rid] = (int(point["x"]), int(point["y"]))

    first_t = min(raw) if raw else 0
    robots = list(trajectories)
    last: dict[str, tuple[int, int]] = {}
    full: dict[int, dict[str, tuple[int, int]]] = {}
    for t in range(first_t, end_t + 1):
        if t in raw:
            last.update(raw[t])
        full[t] = {rid: last[rid] for rid in robots if rid in last}
    return full


def build_machine_states(event_log: list[dict], end_t: int) -> dict[int, dict[str, int]]:
    work_re = re.compile(r"completed (DISASSEMBLE|INSPECT|INSTALL) on (M_y\d+_x\d+)")
    changes: dict[int, dict[str, int]] = defaultdict(dict)
    running: dict[str, int] = {}
    for event in event_log:
        match = work_re.search(event.get("message", ""))
        if not match:
            continue
        op, mid = match.group(1), match.group(2)
        if op == "DISASSEMBLE":
            running[mid] = 8
        elif op == "INSPECT":
            running[mid] = 9
        elif op == "INSTALL":
            running[mid] = 10
        changes[int(event["t"])][mid] = running[mid]

    full: dict[int, dict[str, int]] = {}
    last: dict[str, int] = {}
    for t in range(end_t + 1):
        if t in changes:
            last.update(changes[t])
        full[t] = dict(last)
    return full


def main() -> None:
    exp, scenario_dir, fps, cell = parse_args()
    data = PROJECT / "outputs" / scenario_dir / exp
    trajectories = json.loads((data / "trajectories.json").read_text())
    event_log = [json.loads(line) for line in (data / "event_log.jsonl").read_text().splitlines()]

    terrain, machines, _ = FixedMap().build()
    h, w = terrain.shape
    end_t = max((int(event["t"]) for event in event_log), default=0)
    time_index = build_time_index(trajectories, end_t)
    machine_states = build_machine_states(event_log, end_t)

    margin_left = 44
    margin_top = 48
    margin_bottom = 28
    width = margin_left + w * cell + 8
    height = margin_top + h * cell + margin_bottom

    terrain_colors = {0: "#1a1a1a", 1: "#fafaf5", 2: "#c8e6c9"}
    state_colors = {7: "#e53935", 8: "#ff9800", 9: "#ffc107", 10: "#4caf50"}
    robot_colors = {
        "A_1": "#1565c0",
        "A_2": "#7b1fa2",
        "A_3": "#00838f",
        "A_4": "#6d4c41",
        "B_1": "#2e7d32",
        "B_2": "#558b2f",
    }
    trail_colors = {
        rid: tuple(int(robot_colors.get(rid, "#616161").lstrip("#")[i:i + 2], 16)
                   for i in (0, 2, 4))
        for rid in trajectories
    }

    font_title = load_font(18)
    font_small = load_font(10)

    # Static background: terrain and grid.
    background = Image.new("RGB", (width, height), "white")
    bg = ImageDraw.Draw(background, "RGBA")
    for y in range(h):
        for x in range(w):
            x0 = margin_left + x * cell
            y0 = margin_top + y * cell
            bg.rectangle(
                [x0, y0, x0 + cell, y0 + cell],
                fill=terrain_colors.get(int(terrain[y, x]), "#ff0000"),
                outline=(220, 220, 220, 70),
            )
    # Trunk boundary y=24 in 1-based coordinates.
    trunk_y = margin_top + 23 * cell
    bg.line([margin_left, trunk_y, margin_left + w * cell, trunk_y],
            fill=(255, 152, 0, 160), width=2)

    for x in range(1, w + 1):
        px = margin_left + (x - 1) * cell + cell // 2
        bg.text((px - 4, margin_top - 16), str(x), fill=(80, 80, 80), font=font_small)
    for y in range(1, h + 1):
        py = margin_top + (y - 1) * cell + cell // 2 - 5
        bg.text((margin_left - 24, py), str(y), fill=(80, 80, 80), font=font_small)

    # Precompute trail points from actual consecutive trajectory points.
    trail_history: dict[str, list[tuple[int, int, int]]] = {}
    for rid, points in trajectories.items():
        trail_history[rid] = [
            (
                int(point["t"]),
                margin_left + (int(point["x"]) - 1) * cell + cell // 2,
                margin_top + (int(point["y"]) - 1) * cell + cell // 2,
            )
            for point in points
        ]

    out_path = data / "animation_smooth.mp4"
    writer = imageio.get_writer(
        out_path,
        fps=fps,
        codec="libx264",
        quality=7,
        ffmpeg_params=["-pix_fmt", "yuv420p"],
    )

    print(f"Fast animation: frames={end_t}, fps={fps}, t=[1..{end_t}]")
    try:
        for t in range(1, end_t + 1):
            frame = background.copy()
            draw = ImageDraw.Draw(frame, "RGBA")

            title = f"{scenario_dir}/{exp} — t={t} ({t // 60}m{t % 60:02d}s) — 48 centrifuges D+I+R"
            draw.text((margin_left, 14), title, fill=(20, 20, 20), font=font_title)

            states = machine_states.get(t, {})
            for mid, machine in machines.items():
                state = states.get(mid, 7)
                color = state_colors.get(state, "#999999")
                for c in machine.cells:
                    x0 = margin_left + (c.x - 1) * cell
                    y0 = margin_top + (c.y - 1) * cell
                    draw.rectangle(
                        [x0, y0, x0 + cell, y0 + cell],
                        fill=color,
                        outline=(50, 50, 50, 130),
                    )

            # Trails up to current time.
            for rid, points in trail_history.items():
                visible = [(x, y) for tt, x, y in points if tt <= t]
                if len(visible) >= 2:
                    rgba = trail_colors[rid] + (75,)
                    draw.line(visible, fill=rgba, width=2)

            # Robots (2x4 anchor footprint).
            for rid, pos in time_index.get(t, {}).items():
                x, y = pos
                x0 = margin_left + (x - 1) * cell
                y0 = margin_top + (y - 1) * cell
                color = robot_colors.get(rid, "#616161")
                draw.rectangle(
                    [x0, y0, x0 + 2 * cell, y0 + 4 * cell],
                    fill=color + "D8",
                    outline=(0, 0, 0, 220),
                    width=2,
                )
                draw.text((x0 + 3, y0 + 3), rid, fill=(255, 255, 255), font=font_small)

            writer.append_data(np.asarray(frame))
            if t % 500 == 0:
                print(f"  rendered {t}/{end_t}")
    finally:
        writer.close()

    print(f"Done: {out_path}")
    print(f"  Frames: {end_t}, Duration: {end_t / fps:.0f}s, {fps}fps")
    print(f"  Size: {out_path.stat().st_size / 1024 / 1024:.1f} MB")


if __name__ == "__main__":
    main()
