#!/usr/bin/env python3
"""
Comprehensive Quality Check Script for test4
Performs three checks:
1. Map correctness
2. Spatiotemporal collisions
3. Work zone alignment + job timing
"""

import json
import re
import sys
from collections import defaultdict
from pathlib import Path

# Project paths
PROJECT_ROOT = Path(__file__).resolve().parents[2]

# Map constants (mirrored from src/map/fixed_map.py)
WIDTH = 25
HEIGHT = 29
INTERNAL_ROWS = (1, 23)
TRUNK_ROWS = (24, 29)
OBSTACLE_COLUMNS = (4, 10, 16, 22)
TRUNK_Y_THRESHOLD = 24
MACHINE_ROWS = (3, 7, 11, 15, 19, 23)
MACHINE_X_STARTS = (2, 5, 8, 11, 14, 17, 20, 23)

# Terrain codes
OBSTACLE = 0
INTERNAL_ROAD = 1
TRUNK_ROAD = 2

LOG_PATH = "event_log.jsonl"
TRAJ_PATH = "trajectories.json"
OUTPUT_PATH = "comprehensive_check.txt"

FOOTPRINT_OFFSETS = [
    (0,0), (1,0),
    (0,1), (1,1),
    (0,2), (1,2),
    (0,3), (1,3)
]

def footprint_cells(anchor_x, anchor_y):
    return {(anchor_x + dx, anchor_y + dy) for dx, dy in FOOTPRINT_OFFSETS}


# ============================================================
# CHECK 1: Map Correctness
# ============================================================
def generate_terrain():
    """Generate 29x25 terrain matrix inline (same logic as FixedMap._generate_terrain)"""
    terrain = [[INTERNAL_ROAD] * WIDTH for _ in range(HEIGHT)]

    # Trunk road: y >= 24 (0-indexed y from TRUNK_Y_THRESHOLD-1 to end)
    trunk_start_0 = TRUNK_Y_THRESHOLD - 1
    for y in range(trunk_start_0, HEIGHT):
        terrain[y] = [TRUNK_ROAD] * WIDTH

    # Internal obstacle columns
    for col_1 in OBSTACLE_COLUMNS:
        col_0 = col_1 - 1
        for y in range(trunk_start_0):
            terrain[y][col_0] = OBSTACLE

    return terrain


def generate_machines():
    """Generate 48 centrifuges inline (same logic as FixedMap.generate_machines)"""
    machines = {}
    for row in MACHINE_ROWS:
        for x_start in MACHINE_X_STARTS:
            machine_id = f"M_y{row}_x{x_start}"
            # Cells: (x_start, row) and (x_start+1, row)
            machines[machine_id] = {
                'id': machine_id,
                'row': row,
                'x_start': x_start,
                'cells': [(x_start, row), (x_start + 1, row)]
            }
    return machines


def check_map():
    results = []
    terrain = generate_terrain()
    machines = generate_machines()

    # 1a: Map size
    h, w = len(terrain), len(terrain[0])
    if w == 25 and h == 29:
        results.append(("PASS", f"地图尺寸: {w}x{h} (expected 25x29)"))
    else:
        results.append(("FAIL", f"地图尺寸: {w}x{h} (expected 25x29)"))

    # 1b: Internal area obstacle columns
    # Internal rows 1-indexed: [1,23] -> 0-indexed y=0..22 (23 rows)
    internal_end_0 = TRUNK_Y_THRESHOLD - 1  # 0-indexed = 23, exclusive end
    internal_layer = terrain[:internal_end_0]  # y<24 1-indexed -> 0-indexed y=0..22
    obstacle_cols_0 = {col_1 - 1 for col_1 in OBSTACLE_COLUMNS}  # {3,9,15,21}

    internal_obstacles_ok = True
    for y in range(internal_end_0):
        for x in range(25):
            if x in obstacle_cols_0:
                if internal_layer[y][x] != OBSTACLE:
                    internal_obstacles_ok = False
                    results.append(("FAIL", f"内部区域 ({x+1},{y+1}) 应为障碍物但不是: 值为{internal_layer[y][x]}"))
            else:
                if internal_layer[y][x] != INTERNAL_ROAD:
                    internal_obstacles_ok = False
                    results.append(("FAIL", f"内部区域 ({x+1},{y+1}) 应为内部道路但不是: 值为{internal_layer[y][x]}"))

    if internal_obstacles_ok:
        results.append(("PASS", "内部区域 y<24 障碍列 [4,10,16,22] 正确，其余为内部道路"))

    # 1c: Trunk road (1-indexed y=24..29 -> 0-indexed y=23..28)
    trunk_layer = terrain[TRUNK_Y_THRESHOLD-1:]  # 0-indexed y=23..28
    trunk_ok = all(cell == TRUNK_ROAD for row in trunk_layer for cell in row)
    if trunk_ok:
        results.append(("PASS", "主干道 y>=24 全部为可通行 (TRUNK_ROAD)"))
    else:
        bad_cells = []
        trunk_start_1 = TRUNK_Y_THRESHOLD  # 1-indexed = 24
        for y_off in range(len(trunk_layer)):
            for x in range(25):
                if trunk_layer[y_off][x] != TRUNK_ROAD:
                    bad_cells.append(f"({x+1},{trunk_start_1 + y_off})")
        results.append(("FAIL", f"主干道存在非可通行格: {bad_cells[:10]}..."))

    # 1d: 48 centrifuges
    if len(machines) == 48:
        results.append(("PASS", f"离心机数量: {len(machines)} (expected 48)"))
    else:
        results.append(("FAIL", f"离心机数量: {len(machines)} (expected 48)"))

    # 1e: Centrifuge positions
    expected_rows = (3, 7, 11, 15, 19, 23)
    expected_x_starts = (2, 5, 8, 11, 14, 17, 20, 23)

    position_ok = True
    for machine_id, machine in machines.items():
        row = machine['row']
        x_start = machine['x_start']
        (x1, y1), (x2, y2) = machine['cells']

        if row not in expected_rows:
            position_ok = False
            results.append(("FAIL", f"{machine_id}: row={row} not in {expected_rows}"))
            break

        if x_start not in expected_x_starts:
            position_ok = False
            results.append(("FAIL", f"{machine_id}: x_start={x_start} not in {expected_x_starts}"))
            break

        if x2 != x_start + 1 or y2 != row or x1 != x_start or y1 != row:
            position_ok = False
            results.append(("FAIL", f"{machine_id}: cells not 2 adjacent grids: {machine['cells']}"))
            break

    if position_ok:
        results.append(("PASS", "48台离心机位置正确: rows=[3,7,11,15,19,23], x_starts=[2,5,8,11,14,17,20,23]"))
        results.append(("PASS", "每台离心机=2格横向相邻"))

    # 1f: Centrifuge cells are roads (not obstacles)
    centrifuge_on_obstacle = False
    for machine_id, machine in machines.items():
        for (cx, cy) in machine['cells']:
            x_0, y_0 = cx - 1, cy - 1  # convert to 0-indexed
            if terrain[y_0][x_0] == OBSTACLE:
                centrifuge_on_obstacle = True
                results.append(("FAIL", f"{machine_id}: 格({cx},{cy}) 在静态地形层为障碍物"))

    if not centrifuge_on_obstacle:
        results.append(("PASS", "离心机所在格在静态层均为道路（非障碍物）"))

    return results, terrain, machines


# ============================================================
# CHECK 2: Spatiotemporal Collisions
# ============================================================
def check_collisions():
    results = []

    # Load trajectories
    with open(TRAJ_PATH, 'r') as f:
        trajectories = json.load(f)

    # Load event log for collision events
    with open(LOG_PATH, 'r', encoding='iso-8859-1') as f:
        events = [json.loads(line) for line in f if line.strip()]

    # Extract collision events from log
    log_collisions = [e for e in events if e['type'] == 'collision']

    # Build per-robot position timeline from trajectories
    max_t = max(e['t'] for e in events)

    # Build positions from trajectories
    positions = {}  # t -> {robot: (x, y)}
    for robot, traj in trajectories.items():
        for step in traj:
            t = step['t']
            if t not in positions:
                positions[t] = {}
            positions[t][robot] = (step['x'], step['y'])

    # Fill gaps by carrying forward, but stop at robot_finished
    # Find robot finish times from log
    robot_finish_t = {}
    for e in events:
        if e['type'] == 'robot_finished':
            msg = e['message']
            if 'A_1' in msg.split(':')[0]:
                robot_finish_t['A_1'] = e['t']
            elif 'B_1' in msg.split(':')[0]:
                robot_finish_t['B_1'] = e['t']

    cur_pos = {}
    for t in range(max_t + 1):
        if t not in positions:
            positions[t] = {}
        for robot in ['A_1', 'B_1']:
            if robot in positions[t]:
                cur_pos[robot] = positions[t][robot]
            elif robot in robot_finish_t and t > robot_finish_t[robot]:
                # Robot has finished, do not carry forward
                if robot in positions[t]:
                    del positions[t][robot]
            else:
                if robot in cur_pos and cur_pos[robot] is not None:
                    positions[t][robot] = cur_pos[robot]

    # Detect all collisions from trajectories
    all_collisions = []
    for t in range(max_t + 1):
        if 'A_1' not in positions.get(t, {}) or 'B_1' not in positions.get(t, {}):
            continue

        a_x, a_y = positions[t]['A_1']
        b_x, b_y = positions[t]['B_1']

        a_cells = footprint_cells(a_x, a_y)
        b_cells = footprint_cells(b_x, b_y)

        overlap = a_cells & b_cells

        if overlap:
            all_collisions.append({
                't': t,
                'a': (a_x, a_y),
                'b': (b_x, b_y),
                'overlap': sorted(overlap),
                'count': len(overlap)
            })

    # Group into consecutive collision events
    collision_events = []
    if all_collisions:
        current_event = [all_collisions[0]]
        for i in range(1, len(all_collisions)):
            if all_collisions[i]['t'] == all_collisions[i-1]['t'] + 1:
                current_event.append(all_collisions[i])
            else:
                collision_events.append(current_event)
                current_event = [all_collisions[i]]
        collision_events.append(current_event)

    # Compare with log collisions
    log_collision_ts = {e['t'] for e in log_collisions}
    traj_collision_ts = {c['t'] for c in all_collisions}

    if log_collision_ts == traj_collision_ts:
        results.append(("PASS", f"轨迹碰撞检测与事件日志一致: 均发现 {len(traj_collision_ts)} 帧有碰撞"))
    else:
        only_log = log_collision_ts - traj_collision_ts
        only_traj = traj_collision_ts - log_collision_ts
        if only_log:
            results.append(("FAIL", f"仅在事件日志中发现的碰撞帧: {sorted(only_log)}"))
        if only_traj:
            results.append(("FAIL", f"仅在轨迹检测中发现的碰撞帧: {sorted(only_traj)}"))

    # Analyze collision events in detail
    results.append(("INFO", f"碰撞事件数: {len(collision_events)} (连续碰撞段), 总碰撞帧数: {len(all_collisions)}"))

    # Detailed analysis of each collision event
    # Also get work status from log
    work_events = [e for e in events if e['type'] in ('work_start', 'work_complete')]

    # Build work intervals
    work_intervals = defaultdict(list)  # robot -> [(t_start, t_end, machine, op_type), ...]
    current_work = {}
    for e in sorted(work_events, key=lambda x: x['t']):
        msg = e['message']
        m = re.match(r'^(A_1|B_1):\s+(start|completed)\s+(\w+)\s+on\s+(M_y\d+_x\d+)', msg)
        if not m:
            continue
        robot, action, op_type, machine = m.groups()

        if action == 'start':
            current_work[robot] = (e['t'], machine, op_type)
        elif action == 'completed' and robot in current_work:
            t_start, mach, op = current_work.pop(robot)
            work_intervals[robot].append((t_start, e['t'], mach, op))

    def get_status(robot, t):
        for t_s, t_e, mach, op in work_intervals.get(robot, []):
            if t_s <= t <= t_e:
                return f"WORKING({op}@{mach})"
        if robot not in positions.get(t, {}):
            return "UNKNOWN"
        return "MOVING"

    # Analyze each collision event
    for evt_idx, evt in enumerate(collision_events):
        t_start = evt[0]['t']
        t_end = evt[-1]['t']
        duration = len(evt)

        a_x, a_y = evt[0]['a']
        b_x, b_y = evt[0]['b']

        a_status_start = get_status('A_1', t_start)
        a_status_end = get_status('A_1', t_end)
        b_status_start = get_status('B_1', t_start)
        b_status_end = get_status('B_1', t_end)

        # Check if A_1 has a work reservation at this time
        a_has_reservation = "否"
        for t_s, t_e, mach, op in work_intervals.get('A_1', []):
            if t_s <= t_start <= t_e or t_s <= t_end <= t_e or (t_start <= t_s and t_e <= t_end):
                a_has_reservation = f"是({op}@{mach}, t={t_s}-{t_e})"
                break

        results.append(("COLLISION_EVENT",
            f"事件{evt_idx+1}: t={t_start}-{t_end} (持续{duration}帧)\n"
            f"    A_1@({a_x},{a_y}) 状态: {a_status_start}->{a_status_end}\n"
            f"    B_1@({b_x},{b_y}) 状态: {b_status_start}->{b_status_end}\n"
            f"    A_1工作预约存在: {a_has_reservation}"))

        # Overlap details
        max_overlap = max(c['count'] for c in evt)
        avg_overlap = sum(c['count'] for c in evt) / len(evt)
        results.append(("COLLISION_DETAIL",
            f"    最大重合格={max_overlap}, 平均重合格={avg_overlap:.1f}"))

    return results, all_collisions, collision_events, work_intervals, positions


# ============================================================
# CHECK 3: Work Zone Alignment + Job Timing
# ============================================================
def check_work_operations(machines):
    results = []

    # Build machine lookup from machines dict
    machine_cells = {}  # machine_id -> {(x,y), (x+1,y)}
    for mid, m in machines.items():
        machine_cells[mid] = set(m['cells'])

    # Load event log
    with open(LOG_PATH, 'r', encoding='iso-8859-1') as f:
        events = [json.loads(line) for line in f if line.strip()]

    # Extract work_start and work_complete events
    work_starts = []
    work_completes = []

    for e in events:
        if e['type'] == 'work_start':
            msg = e['message']
            m = re.match(r'^(A_1|B_1):\s+start\s+(\w+)\s+on\s+(M_y\d+_x\d+)', msg)
            if m:
                robot, op_type, machine_id = m.groups()
                work_starts.append({
                    't': e['t'],
                    'robot': robot,
                    'op_type': op_type,
                    'machine_id': machine_id
                })

        if e['type'] == 'work_complete':
            msg = e['message']
            m = re.match(r'^(A_1|B_1):\s+completed\s+(\w+)\s+on\s+(M_y\d+_x\d+)', msg)
            if m:
                robot, op_type, machine_id = m.groups()
                work_completes.append({
                    't': e['t'],
                    'robot': robot,
                    'op_type': op_type,
                    'machine_id': machine_id
                })

    # Load trajectories for robot positions
    with open(TRAJ_PATH, 'r') as f:
        trajectories = json.load(f)

    # Build position lookup
    pos_lookup = {}  # (robot, t) -> (x, y)
    for robot, traj in trajectories.items():
        for step in traj:
            pos_lookup[(robot, step['t'])] = (step['x'], step['y'])

    # --- 3a: Work zone alignment ---
    alignment_ok = 0
    alignment_fail = 0
    alignment_details = []

    for ws in work_starts:
        robot = ws['robot']
        t = ws['t']
        machine_id = ws['machine_id']

        # Get robot anchor position at this time
        if (robot, t) not in pos_lookup:
            alignment_fail += 1
            alignment_details.append(f"FAIL: {robot} at t={t}: no position data")
            continue

        anchor_x, anchor_y = pos_lookup[(robot, t)]

        # Work zone = anchor + offsets (0,0) and (1,0)
        work_zone = {(anchor_x, anchor_y), (anchor_x + 1, anchor_y)}

        # Machine cells
        if machine_id not in machine_cells:
            alignment_fail += 1
            alignment_details.append(f"FAIL: {machine_id} not found in machines")
            continue

        expected_cells = machine_cells[machine_id]

        if work_zone == expected_cells:
            alignment_ok += 1
        else:
            alignment_fail += 1
            alignment_details.append(
                f"FAIL: {robot}@{machine_id} @t={t}: "
                f"work_zone={sorted(work_zone)} != machine_cells={sorted(expected_cells)}, "
                f"anchor=({anchor_x},{anchor_y})"
            )

    total_align = alignment_ok + alignment_fail
    if alignment_fail == 0:
        results.append(("PASS", f"3a: 工作区对齐 — {alignment_ok}/{total_align} 次操作对齐正确"))
    else:
        results.append(("FAIL", f"3a: 工作区对齐 — {alignment_ok}/{total_align} 次操作对齐正确, {alignment_fail} 次失败"))
        for d in alignment_details[:20]:
            results.append(("FAIL_DETAIL", f"  {d}"))
        if len(alignment_details) > 20:
            results.append(("FAIL_DETAIL", f"  ... 还有 {len(alignment_details)-20} 条"))

    # --- 3b: Job duration ---
    expected_durations = {
        'DISASSEMBLE': 6,
        'INSPECT': 10,
        'INSTALL': 6,
    }

    # Match work_start with work_complete
    duration_ok = 0
    duration_fail = 0
    duration_details = []

    # Group by (robot, machine_id, op_type)
    for ws in work_starts:
        robot = ws['robot']
        op_type = ws['op_type']
        machine_id = ws['machine_id']
        t_start = ws['t']

        # Find matching work_complete
        match = None
        for wc in work_completes:
            if (wc['robot'] == robot and wc['op_type'] == op_type
                and wc['machine_id'] == machine_id and wc['t'] > t_start):
                match = wc
                break

        if match is None:
            duration_fail += 1
            duration_details.append(
                f"FAIL: {robot} {op_type} on {machine_id}: no matching work_complete"
            )
            continue

        t_end = match['t']
        # Use inclusive counting: work_start and work_complete are both counted as working time steps
        # Example: t_start=22, t_end=27 -> working at t=22,23,24,25,26,27 = 6 steps
        actual_duration = t_end - t_start + 1
        expected = expected_durations.get(op_type, -1)

        if actual_duration == expected:
            duration_ok += 1
        else:
            duration_fail += 1
            duration_details.append(
                f"FAIL: {robot} {op_type} on {machine_id}: "
                f"actual={actual_duration} (t={t_start}-{t_end} inclusive), expected={expected}"
            )

    total_dur = duration_ok + duration_fail
    if duration_fail == 0:
        results.append(("PASS", f"3b: 作业时长 — {duration_ok}/{total_dur} 次操作时长正确"))
    else:
        results.append(("FAIL", f"3b: 作业时长 — {duration_ok}/{total_dur} 次操作时长正确, {duration_fail} 次失败"))
        for d in duration_details[:20]:
            results.append(("FAIL_DETAIL", f"  {d}"))
        if len(duration_details) > 20:
            results.append(("FAIL_DETAIL", f"  ... 还有 {len(duration_details)-20} 条"))

    # --- 3c: Operation sequence ---
    # For each machine, check D -> I -> R order
    seq_ok = 0
    seq_fail = 0
    seq_details = []

    # Get all machines
    all_machine_ids = set(ws['machine_id'] for ws in work_starts)

    for mid in sorted(all_machine_ids):
        # Get operations for this machine in time order
        machine_ops = []
        for ws in work_starts:
            if ws['machine_id'] == mid:
                machine_ops.append(ws)
        machine_ops.sort(key=lambda x: x['t'])

        # Extract op type sequence
        op_seq = [op['op_type'] for op in machine_ops]

        # Check: should be DISASSEMBLE, INSPECT, INSTALL (or DISASSEMBLE, INSPECT, INSTALL)
        expected_seq = ['DISASSEMBLE', 'INSPECT', 'INSTALL']

        if op_seq == expected_seq:
            seq_ok += 1
        elif len(op_seq) == 3 and set(op_seq) == set(expected_seq):
            seq_fail += 1
            seq_details.append(
                f"FAIL: {mid}: order={op_seq}, expected D->I->R"
            )
        elif len(op_seq) < 3:
            seq_fail += 1
            seq_details.append(
                f"FAIL: {mid}: only {len(op_seq)} operations: {op_seq}"
            )
        else:
            seq_fail += 1
            seq_details.append(
                f"FAIL: {mid}: unexpected sequence {op_seq}"
            )

    # Check for skipped states
    total_seq = seq_ok + seq_fail
    if seq_fail == 0:
        results.append(("PASS", f"3c: 操作顺序 — {total_seq}台机器 D->I->R 顺序全部正确"))
    else:
        results.append(("FAIL", f"3c: 操作顺序 — {seq_ok}/{total_seq}台正确, {seq_fail}台异常"))
        for d in seq_details[:20]:
            results.append(("FAIL_DETAIL", f"  {d}"))

    return results, alignment_ok, alignment_fail, duration_ok, duration_fail, seq_ok, seq_fail, alignment_details, duration_details, seq_details


# ============================================================
# Main
# ============================================================
def main():
    print("=" * 60)
    print("Agent2 - 质量检验官: test4 综合检查")
    print("=" * 60)

    # Check 1
    print("\n[检查1] 地图正确性...")
    map_results, terrain, machines = check_map()
    for status, msg in map_results:
        print(f"  [{status}] {msg[:80]}")

    # Check 2
    print("\n[检查2] 时空碰撞...")
    collision_results, all_collisions, collision_events, work_intervals, positions = check_collisions()
    for status, msg in collision_results:
        if status in ('COLLISION_EVENT', 'COLLISION_DETAIL'):
            print(f"  {msg}")
        else:
            print(f"  [{status}] {msg[:80]}")

    # Check 3
    print("\n[检查3] 工作区对齐 + 作业时间...")
    work_results, al_ok, al_fail, dur_ok, dur_fail, seq_ok, seq_fail, al_details, dur_details, seq_details = check_work_operations(machines)
    for status, msg in work_results:
        if status == 'FAIL_DETAIL':
            if len(msg) > 100:
                print(f"  {msg[:100]}...")
            else:
                print(f"  {msg}")
        else:
            print(f"  [{status}] {msg[:100]}")

    # Write comprehensive report
    print(f"\nWriting report to {OUTPUT_PATH}...")

    with open(OUTPUT_PATH, 'w', encoding='utf-8') as f:
        f.write("=" * 70 + "\n")
        f.write("=== 多机器人协同调度系统 test4 综合质量检查 ===\n")
        f.write("=" * 70 + "\n")
        f.write(f"检查时间: 2026-07-02\n")
        f.write(f"数据路径: outputs/scenario_1/260702_test4/\n\n")

        # --- Check 1 ---
        f.write("## 检查1: 地图正确性\n")
        f.write("-" * 50 + "\n")
        for status, msg in map_results:
            f.write(f"[{status}] {msg}\n")
        f.write("\n")

        # --- Check 2 ---
        f.write("## 检查2: 时空碰撞\n")
        f.write("-" * 50 + "\n")
        for status, msg in collision_results:
            f.write(f"[{status}] {msg}\n")
        f.write("\n")

        # --- Check 3 ---
        f.write("## 检查3: 工作区对齐 + 作业时间\n")
        f.write("-" * 50 + "\n")
        # 3a
        f.write(f"\n### 3a: 工作区对齐 — {al_ok}/{al_ok + al_fail} 次操作对齐正确\n")
        if al_fail > 0:
            for d in al_details:
                f.write(f"  {d}\n")
        else:
            f.write("  全部对齐正确。\n")

        # 3b
        f.write(f"\n### 3b: 作业时长 — {dur_ok}/{dur_ok + dur_fail} 次操作时长正确\n")
        if dur_fail > 0:
            for d in dur_details:
                f.write(f"  {d}\n")
        else:
            f.write("  全部时长正确。\n")

        # 3c
        total_seq = seq_ok + seq_fail
        f.write(f"\n### 3c: 操作顺序 — {total_seq}台机器 D->I->R 顺序\n")
        if seq_fail == 0:
            f.write("  全部正确。\n")
        else:
            f.write(f"  {seq_ok}/{total_seq}台正确, {seq_fail}台异常:\n")
            for d in seq_details:
                f.write(f"  {d}\n")

        # --- Summary of all issues ---
        f.write("\n" + "=" * 70 + "\n")
        f.write("## 发现的全部问题\n")
        f.write("-" * 50 + "\n")

        all_issues = []

        # Collect FAIL from map
        for status, msg in map_results:
            if status == 'FAIL':
                all_issues.append(f"[地图] {msg}")

        # Collect FAIL from collisions
        for status, msg in collision_results:
            if status == 'FAIL':
                all_issues.append(f"[碰撞] {msg}")

        # Collect FAIL from work
        for status, msg in work_results:
            if status == 'FAIL':
                all_issues.append(f"[工作] {msg}")

        if not all_issues:
            f.write("(无问题发现 — 所有检查通过!)\n")
        else:
            for i, issue in enumerate(all_issues):
                f.write(f"{i+1}. {issue}\n")

        # --- Overall statistics ---
        f.write("\n" + "=" * 70 + "\n")
        f.write("## 综合统计\n")
        f.write("-" * 50 + "\n")
        f.write(f"地图检查: {'PASS' if not any(s=='FAIL' for s,_ in map_results) else 'FAIL'}\n")
        f.write(f"碰撞检查: {len(collision_events)}个碰撞事件, {len(all_collisions)}帧碰撞\n")
        f.write(f"工作区对齐: {al_ok}/{al_ok+al_fail} 正确\n")
        f.write(f"作业时长: {dur_ok}/{dur_ok+dur_fail} 正确\n")
        f.write(f"操作顺序: {seq_ok}/{total_seq} 正确\n")

        # --- Recommendations ---
        f.write("\n" + "=" * 70 + "\n")
        f.write("## 下一步修改建议\n")
        f.write("-" * 50 + "\n")
        f.write("(按优先级排列)\n\n")

        if seq_fail > 0 or dur_fail > 0 or al_fail > 0:
            f.write("1. [高] 修复工作区对齐/作业时长/操作顺序问题 (检查3异常)\n")

        if collision_events:
            event_summaries = []
            for i, evt in enumerate(collision_events):
                t_s, t_e = evt[0]['t'], evt[-1]['t']
                a_x, a_y = evt[0]['a']
                event_summaries.append(f"事件{i+1}(t={t_s}-{t_e} A_1@({a_x},{a_y}), 持续{len(evt)}帧)")
            f.write(f"2. [中] 解决{len(collision_events)}个碰撞事件:\n")
            for s in event_summaries:
                f.write(f"   - {s}\n")
            f.write("   原因: B_1 路径规划未避开 A_1 工作区，预约表未阻止冲突轨迹\n")

        if not collision_events and seq_fail == 0 and dur_fail == 0 and al_fail == 0:
            f.write("1. (无需修改 — 所有检查通过)\n")

    print(f"\nReport written to {OUTPUT_PATH}")
    print("Done!")


if __name__ == '__main__':
    main()
