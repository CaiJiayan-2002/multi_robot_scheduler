#!/usr/bin/env python3
"""
Collision Analyzer for Multi-Robot Simulation
Analyzes spatiotemporal overlap between A_1 and B_1 robots.
Each robot has a 2x4 footprint.
"""

import json
import re
from collections import defaultdict

# --- Configuration ---
LOG_PATH = "event_log.jsonl"
OUTPUT_PATH = "collision_analysis.txt"
FOOTPRINT_OFFSETS = [
    (0,0), (1,0),
    (0,1), (1,1),
    (0,2), (1,2),
    (0,3), (1,3)
]

def footprint_cells(anchor_x, anchor_y):
    """Return set of (x,y) cells covered by a 2x4 footprint anchored at (anchor_x, anchor_y)."""
    return {(anchor_x + dx, anchor_y + dy) for dx, dy in FOOTPRINT_OFFSETS}

def parse_move_message(message):
    """
    Parse move message formats:
    "A_1: -> (x,y) t=N UP/DOWN/LEFT/RIGHT/WAIT/ALIGN/START/GOAL"
    Returns (robot, x, y) or None.
    """
    m = re.match(r'^(A_1|B_1):\s*->\s*\((\d+),(\d+)\)', message)
    if m:
        return m.group(1), int(m.group(2)), int(m.group(3))
    return None

def parse_work_message(message):
    """
    Parse work-related messages:
    "A_1: start DISASSEMBLE on M_y3_x2, duration=6"
    "B_1: start INSPECT on M_y3_x2, duration=10"
    Returns (robot, action, machine) or None.
    """
    # work_start and work_complete have similar formats
    m = re.match(r'^(A_1|B_1):\s+(start|completed)\s+(\w+)', message)
    if m:
        return m.group(1), m.group(2), m.group(3)
    return None

def get_work_status(work_start_events):
    """
    Determine when each robot is working.
    Returns dict: {robot: set of t values where robot is working}
    """
    working = {r: set() for r in ['A_1', 'B_1']}

    for robot in ['A_1', 'B_1']:
        events = sorted(work_start_events.get(robot, []), key=lambda e: e['t'])
        i = 0
        while i < len(events):
            event = events[i]
            if event['subtype'] == 'work_start':
                t_start = event['t']
                # find corresponding work_complete
                duration = event.get('duration', 0)
                # search for work_complete
                for j in range(i+1, len(events)):
                    if events[j]['subtype'] == 'work_complete' and events[j]['t'] >= t_start:
                        t_end = events[j]['t']
                        for t in range(t_start, t_end + 1):
                            working[robot].add(t)
                        i = j + 1
                        break
                else:
                    i += 1
            else:
                i += 1
    return working

def main():
    print("Reading event log...")
    events = []
    with open(LOG_PATH, 'r', encoding='iso-8859-1') as f:
        for line in f:
            line = line.strip()
            if line:
                events.append(json.loads(line))

    print(f"Loaded {len(events)} events")

    # --- Phase 1: Build per-timestep position table ---
    # Track last known position for each robot
    last_positions = {'A_1': None, 'B_1': None}
    # Also track last known action for each robot (for status determination)
    last_actions = {'A_1': 'IDLE', 'B_1': 'IDLE'}

    # Collect work info
    work_start_events = defaultdict(list)

    # Track robot state at each time step
    max_t = max(e['t'] for e in events)
    print(f"Max t: {max_t}")

    # Positions at each time step
    positions = {}  # t -> {robot: (x, y, action)}

    # First pass: extract all move events and track positions
    for event in events:
        t = event['t']
        etype = event['type']
        msg = event['message']

        if t not in positions:
            positions[t] = {}

        # Parse move events
        if etype == 'move':
            parsed = parse_move_message(msg)
            if parsed:
                robot, x, y = parsed
                # Extract action from the message
                parts = msg.split()
                action = parts[-1] if parts else 'UNKNOWN'
                last_positions[robot] = (x, y)
                last_actions[robot] = action
                positions[t][robot] = (x, y, action)
            # If no robot is in the message but it's a move event, check for alternative format
            elif msg.startswith('A_1') or msg.startswith('B_1'):
                print(f"  WARNING: Unparsed move at t={t}: {msg}")

        # Track work events for status
        if etype == 'work_start':
            parsed_work = parse_work_message(msg)
            if parsed_work:
                robot, _, task = parsed_work
                duration_match = re.search(r'duration=(\d+)', msg)
                duration = int(duration_match.group(1)) if duration_match else 0
                work_start_events[robot].append({
                    't': t, 'subtype': 'work_start', 'task': task, 'duration': duration
                })

        if etype == 'work_complete':
            parsed_work = parse_work_message(msg)
            if parsed_work:
                robot, _, task = parsed_work
                work_start_events[robot].append({
                    't': t, 'subtype': 'work_complete', 'task': task
                })

        if etype == 'robot_finished':
            # Robot finished all tasks
            if 'A_1' in msg:
                last_actions['A_1'] = 'FINISHED'
            elif 'B_1' in msg:
                last_actions['B_1'] = 'FINISHED'

    # --- Phase 2: Fill in gaps ---
    # For every time step, ensure both robots have a position
    # If no move event at time t for a robot, carry forward the last known position

    # Walk chronologically, carrying forward each robot's position
    cur_pos = {'A_1': None, 'B_1': None}  # (x, y, action)
    for t in range(max_t + 1):
        if t not in positions:
            positions[t] = {}
        for robot in ['A_1', 'B_1']:
            if robot in positions[t]:
                cur_pos[robot] = positions[t][robot]
            else:
                if cur_pos[robot] is not None:
                    positions[t][robot] = cur_pos[robot]

    # --- Phase 3: Build working status ---
    # Robot is WORKING if it's at a machine location and has a work_start that hasn't completed
    # Simpler approach: track by work_start/work_complete events
    working = get_work_status(work_start_events)

    # Also mark robot state as WORKING/MOVING/IDLE
    for t in range(max_t + 1):
        for robot in ['A_1', 'B_1']:
            if robot in positions.get(t, {}):
                x, y, action = positions[t][robot]
                # Determine if working: check if t is in working set
                if t in working.get(robot, set()):
                    # Robot is at a work location - override action to indicate working
                    # Keep the original action for reference
                    pass

    print(f"Filled positions for time steps 0 to {max_t}")
    print(f"A_1 has positions in {len([t for t in range(max_t+1) if 'A_1' in positions.get(t,{})])} steps")
    print(f"B_1 has positions in {len([t for t in range(max_t+1) if 'B_1' in positions.get(t,{})])} steps")

    # --- Phase 4: Detect collisions ---
    collisions = []

    for t in range(max_t + 1):
        if 'A_1' not in positions.get(t, {}) or 'B_1' not in positions.get(t, {}):
            continue

        a_info = positions[t]['A_1']
        b_info = positions[t]['B_1']

        a_x, a_y = a_info[0], a_info[1]
        b_x, b_y = b_info[0], b_info[1]

        a_cells = footprint_cells(a_x, a_y)
        b_cells = footprint_cells(b_x, b_y)

        overlap = a_cells & b_cells

        if overlap:
            # Determine robot status
            a_status = 'WORKING' if t in working.get('A_1', set()) else 'MOVING'
            b_status = 'WORKING' if t in working.get('B_1', set()) else 'MOVING'

            # Handle edge cases
            if a_info[2] in ('START', 'GOAL', 'FINISHED'):
                a_status = 'IDLE'
            if b_info[2] in ('START', 'GOAL', 'FINISHED'):
                b_status = 'IDLE'

            collisions.append({
                't': t,
                'a_x': a_x, 'a_y': a_y,
                'b_x': b_x, 'b_y': b_y,
                'a_status': a_status,
                'b_status': b_status,
                'a_action': a_info[2],
                'b_action': b_info[2],
                'overlap_cells': sorted(overlap),
                'overlap_count': len(overlap)
            })

    print(f"Found {len(collisions)} collisions")

    # --- Phase 5: Generate Report ---
    print("Generating report...")

    with open(OUTPUT_PATH, 'w', encoding='utf-8') as f:
        f.write("=== 时空重合分析报告 ===\n")
        f.write(f"仿真时间范围: 0 ~ {max_t}\n")
        f.write(f"总分析步数: {max_t + 1}\n")
        f.write(f"时空重合次数: {len(collisions)}\n\n")

        # Detail section
        f.write("--- 重合详情 (前20次) ---\n")
        for i, c in enumerate(collisions[:20]):
            cells_str = ','.join([f'({cx},{cy})' for cx, cy in c['overlap_cells']])
            f.write(f"t={c['t']}: A_1@({c['a_x']},{c['a_y']})[{c['a_status']}] "
                    f"B_1@({c['b_x']},{c['b_y']})[{c['b_status']}] "
                    f"重合格数={c['overlap_count']} 重合格子=[{cells_str}]\n")

        if len(collisions) > 20:
            f.write(f"\n... (还有 {len(collisions) - 20} 次重合未列出)\n")

        f.write("\n--- 重合统计 ---\n")

        # A_1 WORKING, B_1 passes through
        a_work_b_pass = sum(1 for c in collisions if c['a_status'] == 'WORKING' and c['b_status'] != 'WORKING')
        f.write(f"- A_1 WORKING 时被 B_1 穿过的次数: {a_work_b_pass}\n")

        # B_1 WORKING, A_1 passes through
        b_work_a_pass = sum(1 for c in collisions if c['b_status'] == 'WORKING' and c['a_status'] != 'WORKING')
        f.write(f"- B_1 WORKING 时被 A_1 穿过的次数: {b_work_a_pass}\n")

        # Both MOVING
        both_moving = sum(1 for c in collisions if c['a_status'] == 'MOVING' and c['b_status'] == 'MOVING' and c['a_status'] != 'WORKING' and c['b_status'] != 'WORKING')
        both_moving = sum(1 for c in collisions if c['a_status'] == 'MOVING' and c['b_status'] == 'MOVING')
        f.write(f"- 两机器人都在 MOVING 时的重合次数: {both_moving}\n")

        # Both WORKING (shouldn't happen ideally)
        both_working = sum(1 for c in collisions if c['a_status'] == 'WORKING' and c['b_status'] == 'WORKING')
        f.write(f"- 两机器人都在 WORKING 时的重合次数: {both_working}\n")

        # One IDLE
        one_idle = sum(1 for c in collisions if c['a_status'] == 'IDLE' or c['b_status'] == 'IDLE')
        f.write(f"- 至少一方 IDLE/START/GOAL 时的重合次数: {one_idle}\n")

        # Y coordinate distribution
        f.write("\n- 重合发生的典型 y 坐标分布:\n")
        y_dist = defaultdict(int)
        for c in collisions:
            for cx, cy in c['overlap_cells']:
                y_dist[cy] += 1
        for y in sorted(y_dist.keys()):
            f.write(f"  y={y}: {y_dist[y]} 格次\n")

        # Most severe collisions
        f.write("\n--- 最严重的重合事件 ---\n")

        # By overlap count
        max_overlap = max(c['overlap_count'] for c in collisions) if collisions else 0
        worst_by_count = [c for c in collisions if c['overlap_count'] == max_overlap]
        f.write(f"\n重合格数最多的重合 ({max_overlap} 格):\n")
        for c in worst_by_count[:5]:
            cells_str = ','.join([f'({cx},{cy})' for cx, cy in c['overlap_cells']])
            f.write(f"  t={c['t']}: A_1@({c['a_x']},{c['a_y']})[{c['a_status']}] "
                    f"B_1@({c['b_x']},{c['b_y']})[{c['b_status']}] "
                    f"重合格子=[{cells_str}]\n")

        # By duration (consecutive collisions)
        f.write(f"\n持续时间最长的重合段:\n")
        if collisions:
            # Group consecutive t values
            consecutive_groups = []
            current_group = [collisions[0]]
            for i in range(1, len(collisions)):
                if collisions[i]['t'] == collisions[i-1]['t'] + 1:
                    current_group.append(collisions[i])
                else:
                    consecutive_groups.append(current_group)
                    current_group = [collisions[i]]
            consecutive_groups.append(current_group)

            # Sort by group length (duration)
            consecutive_groups.sort(key=len, reverse=True)

            for group in consecutive_groups[:5]:
                duration = len(group)
                t_start = group[0]['t']
                t_end = group[-1]['t']
                avg_overlap = sum(c['overlap_count'] for c in group) / len(group)
                f.write(f"  t={t_start}~{t_end} (持续{duration}步), "
                        f"平均重合格={avg_overlap:.1f}, "
                        f"A_1状态={group[0]['a_status']}, "
                        f"B_1状态={group[0]['b_status']}\n")

        # Summary statistics
        f.write("\n--- 汇总统计 ---\n")
        if collisions:
            total_overlap_cells = sum(c['overlap_count'] for c in collisions)
            avg_overlap = total_overlap_cells / len(collisions)
            f.write(f"总重合步数: {len(collisions)}\n")
            f.write(f"平均每次重合格数: {avg_overlap:.2f}\n")
            f.write(f"最大单次重合格数: {max_overlap}\n")

            # By status combination
            f.write(f"\n重合状态矩阵:\n")
            status_counts = defaultdict(int)
            for c in collisions:
                key = f"A_1={c['a_status']} | B_1={c['b_status']}"
                status_counts[key] += 1
            for key, count in sorted(status_counts.items(), key=lambda x: -x[1]):
                f.write(f"  {key}: {count}\n")

    print(f"\nReport written to {OUTPUT_PATH}")

    # --- Print summary ---
    print("\n" + "="*60)
    print("Agent2 汇报:")
    print(f"- 总共在 {max_t+1} 步仿真中, 发现 {len(collisions)} 次时空重合")
    if collisions:
        total_overlap_cells = sum(c['overlap_count'] for c in collisions)
        avg_overlap = total_overlap_cells / len(collisions)
        print(f"- 平均每次重合 {avg_overlap:.1f} 个格子")

        worst = max(collisions, key=lambda c: c['overlap_count'])
        print(f"- 最严重的一次: t={worst['t']}, "
              f"A_1@({worst['a_x']},{worst['a_y']}) 与 B_1@({worst['b_x']},{worst['b_y']}) "
              f"重叠 {worst['overlap_count']} 格")

        # Characterize the collisions
        print(f"- 重合主要发生在: ", end="")
        status_desc = []
        if a_work_b_pass > 0:
            status_desc.append(f"A_1作业时B_1穿过({a_work_b_pass}次)")
        if b_work_a_pass > 0:
            status_desc.append(f"B_1作业时A_1穿过({b_work_a_pass}次)")
        if both_moving > 0:
            status_desc.append(f"两机器人交会({both_moving}次)")
        print(", ".join(status_desc))
    else:
        print("- 未发现任何时空重合（非常理想！）")
    print("="*60)

if __name__ == '__main__':
    main()
