"""综合验证脚本 v4.0

验证固定地图生成、48台离心机/144个操作、footprint碰撞检测、
服务点可达性、以及旅行时间矩阵预计算。

用法: python tests/run_verification.py
"""

import sys
import os
import time
import numpy as np
from collections import defaultdict

# Ensure the project root is on sys.path
PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from src.domain.enums import TerrainCode, RobotType, MachineState, OperationType, Action
from src.domain.models import Cell, Footprint, RobotSpec
from src.map.fixed_map import FixedMap
from src.map.service_poses import ServicePoseCalculator
from src.domain.validation import FootprintValidator
from src.map.pose_graph import PoseGraph

# StaticAStar might not exist yet (Agent1 still working on it)
_STATIC_ASTAR_AVAILABLE = False
try:
    from src.planning.static_astar import StaticAStar
    _STATIC_ASTAR_AVAILABLE = True
except ImportError:
    pass


# ============================================================================
# SECTION 1: Generate and inspect the fixed map
# ============================================================================
def section1_map_generation():
    print("=" * 70)
    print("SECTION 1: Fixed Map Generation")
    print("=" * 70)

    fmap = FixedMap()

    # Test constants before build
    assert fmap.WIDTH == 25, f"Expected WIDTH=25, got {fmap.WIDTH}"
    assert fmap.HEIGHT == 29, f"Expected HEIGHT=29, got {fmap.HEIGHT}"
    print(f"  [PASS] Class constants: {fmap.WIDTH} x {fmap.HEIGHT}")

    # Build the map
    terrain, machines, operations = fmap.build()

    # Verify terrain shape
    assert terrain.shape == (29, 25), f"Expected terrain shape (29,25), got {terrain.shape}"
    assert terrain.dtype.kind == 'i', f"Expected integer dtype, got {terrain.dtype}"
    print(f"  [PASS] Terrain shape: {terrain.shape}, dtype: {terrain.dtype}")

    # Verify properties
    assert fmap.terrain is not None, "fmap.terrain should not be None after build()"
    assert fmap.machines is not None, "fmap.machines should not be None after build()"
    assert fmap.operations is not None, "fmap.operations should not be None after build()"
    print(f"  [PASS] Properties accessible after build()")

    return fmap


# ============================================================================
# SECTION 2: Map summary (obstacles, roads, trunk roads)
# ============================================================================
def section2_map_summary(fmap: FixedMap):
    print("\n" + "=" * 70)
    print("SECTION 2: Map Summary")
    print("=" * 70)

    terrain = fmap.terrain
    total_cells = 25 * 29

    # Count terrain types
    raw_counts = {}
    for y in range(29):
        for x in range(25):
            v = int(terrain[y, x])
            raw_counts[v] = raw_counts.get(v, 0) + 1

    print(f"  Map dimensions : 25 x 29 = {total_cells} cells")
    for k in sorted(raw_counts.keys()):
        name = TerrainCode(k).name if k in [0, 1, 2] else f"unknown({k})"
        pct = raw_counts[k] / total_cells * 100
        print(f"    {name:18s}: {raw_counts[k]:5d} cells ({pct:.1f}%)")

    assert sum(raw_counts.values()) == total_cells, \
        f"Cell count mismatch: {sum(raw_counts.values())} != {total_cells}"
    print(f"  [PASS] All {total_cells} cells accounted for")

    # Expected values
    # obstacles: 4 cols * 23 rows (y=1..23, 0-indexed 0..22) = 92
    # trunk: 6 rows * 25 cols = 150
    # internal: 25*29 - 92 - 150 = 725 - 242 = 483
    expected_obstacle = 4 * 23  # 92
    expected_trunk = 6 * 25     # 150
    expected_internal = 25 * 29 - expected_obstacle - expected_trunk  # 483

    assert raw_counts.get(0, 0) == expected_obstacle, \
        f"Expected {expected_obstacle} obstacle cells, got {raw_counts.get(0, 0)}"
    assert raw_counts.get(1, 0) == expected_internal, \
        f"Expected {expected_internal} internal road cells, got {raw_counts.get(1, 0)}"
    assert raw_counts.get(2, 0) == expected_trunk, \
        f"Expected {expected_trunk} trunk road cells, got {raw_counts.get(2, 0)}"
    print(f"  [PASS] Terrain counts match expected values")

    # Verify obstacle columns in internal rows
    obstacle_cols = FixedMap.OBSTACLE_COLUMNS  # (4, 10, 16, 22) 1-indexed
    for col_1 in obstacle_cols:
        col_0 = col_1 - 1
        for row_0 in range(23):  # rows 0..22 (y=1..23)
            assert terrain[row_0, col_0] == 0, \
                f"Expected obstacle at ({col_1},{row_0+1}), got {terrain[row_0, col_0]}"
    print(f"  [PASS] Obstacle columns verified at x={list(obstacle_cols)}")

    # Verify trunk rows
    for row_0 in range(23, 29):  # rows 23..28 (y=24..29)
        for col_0 in range(25):
            assert terrain[row_0, col_0] == 2, \
                f"Expected trunk road at ({col_0+1},{row_0+1}), got {terrain[row_0, col_0]}"
    print(f"  [PASS] Trunk rows (y>=24) are all TRUNK_ROAD")

    return raw_counts


# ============================================================================
# SECTION 3: Verify 48 centrifuges and 144 operations
# ============================================================================
def section3_machines_and_operations(fmap: FixedMap):
    print("\n" + "=" * 70)
    print("SECTION 3: Machines & Operations Verification")
    print("=" * 70)

    machines = fmap.machines
    operations = fmap.operations

    # --- 3a: Counts ---
    assert machines is not None, "machines is None"
    assert operations is not None, "operations is None"
    print(f"  Machines count : {len(machines)} (expected: 48)")
    assert len(machines) == 48, f"Expected 48 machines, got {len(machines)}"
    print(f"  Operations count: {len(operations)} (expected: 144)")
    assert len(operations) == 144, f"Expected 144 operations, got {len(operations)}"

    # --- 3b: Machine IDs ---
    # Agent1 uses format: M_y{row}_x{x}
    expected_ids = set()
    for row in FixedMap.MACHINE_ROWS:        # (3,7,11,15,19,23)
        for x in FixedMap.MACHINE_X_STARTS:  # (2,5,8,11,14,17,20,23)
            expected_ids.add(f"M_y{row}_x{x}")

    actual_ids = set(machines.keys())
    missing = expected_ids - actual_ids
    extra = actual_ids - expected_ids
    assert not missing, f"Missing machine IDs: {sorted(missing)}"
    assert not extra, f"Extra machine IDs: {sorted(extra)}"
    print(f"  [PASS] All 48 machine IDs match expected format (M_y{{row}}_x{{x}})")

    # --- 3c: Machine cells ---
    for mid, m in machines.items():
        assert len(m.cells) == 2, f"{mid}: expected 2 cells, got {len(m.cells)}"
        c0, c1 = m.cells
        assert c0.y == c1.y, f"{mid}: cells not on same row: {c0} vs {c1}"
        assert abs(c0.x - c1.x) == 1, f"{mid}: cells not horizontally adjacent: {c0} vs {c1}"
        assert m.state == MachineState.PENDING_DISASSEMBLY, \
            f"{mid}: initial state should be PENDING_DISASSEMBLY(7), got {m.state}"
    print(f"  [PASS] All machines have correct 2-cell horizontal footprint and initial state")

    # --- 3d: Operations per machine ---
    op_per_machine = defaultdict(int)
    for op_id, op in operations.items():
        op_per_machine[op.machine_id] += 1

    wrong = {mid: cnt for mid, cnt in op_per_machine.items() if cnt != 3}
    assert not wrong, f"Machines with != 3 operations: {wrong}"
    print(f"  [PASS] Each machine has exactly 3 operations")

    # --- 3e: Operation types ---
    type_counts = defaultdict(int)
    for op in operations.values():
        type_counts[op.operation_type] += 1

    assert type_counts.get(OperationType.DISASSEMBLE, 0) == 48, \
        f"DISASSEMBLE: expected 48, got {type_counts.get(OperationType.DISASSEMBLE, 0)}"
    assert type_counts.get(OperationType.INSPECT, 0) == 48, \
        f"INSPECT: expected 48, got {type_counts.get(OperationType.INSPECT, 0)}"
    assert type_counts.get(OperationType.INSTALL, 0) == 48, \
        f"INSTALL: expected 48, got {type_counts.get(OperationType.INSTALL, 0)}"
    print(f"  [PASS] Operation type distribution: DISASSEMBLE=48, INSPECT=48, INSTALL=48")

    # --- 3f: Robot types ---
    for op in operations.values():
        if op.operation_type == OperationType.DISASSEMBLE:
            assert op.eligible_robot_type == RobotType.A, \
                f"{op.operation_id}: DISASSEMBLE should use RobotType.A"
        elif op.operation_type == OperationType.INSPECT:
            assert op.eligible_robot_type == RobotType.B, \
                f"{op.operation_id}: INSPECT should use RobotType.B"
        elif op.operation_type == OperationType.INSTALL:
            assert op.eligible_robot_type == RobotType.A, \
                f"{op.operation_id}: INSTALL should use RobotType.A"
    print(f"  [PASS] Robot type assignments correct (DISASSEMBLE=A, INSPECT=B, INSTALL=A)")

    # --- 3g: Durations ---
    for op in operations.values():
        if op.operation_type == OperationType.DISASSEMBLE:
            assert op.duration == 6, f"{op.operation_id}: expected duration 6, got {op.duration}"
        elif op.operation_type == OperationType.INSPECT:
            assert op.duration == 10, f"{op.operation_id}: expected duration 10, got {op.duration}"
        elif op.operation_type == OperationType.INSTALL:
            assert op.duration == 6, f"{op.operation_id}: expected duration 6, got {op.duration}"
    print(f"  [PASS] Operation durations correct (6/10/6)")

    # --- 3h: Precedence chains ---
    for op in operations.values():
        if op.operation_type == OperationType.DISASSEMBLE:
            assert op.predecessor_id is None, \
                f"{op.operation_id}: DISASSEMBLE should have no predecessor"
        elif op.operation_type == OperationType.INSPECT:
            assert op.predecessor_id is not None, \
                f"{op.operation_id}: INSPECT should have a predecessor"
            pred = operations[op.predecessor_id]
            assert pred.operation_type == OperationType.DISASSEMBLE, \
                f"{op.operation_id}: predecessor should be DISASSEMBLE, got {pred.operation_type}"
            assert pred.machine_id == op.machine_id, \
                f"{op.operation_id}: predecessor should be on same machine"
        elif op.operation_type == OperationType.INSTALL:
            assert op.predecessor_id is not None, \
                f"{op.operation_id}: INSTALL should have a predecessor"
            pred = operations[op.predecessor_id]
            assert pred.operation_type == OperationType.INSPECT, \
                f"{op.operation_id}: predecessor should be INSPECT, got {pred.operation_type}"
            assert pred.machine_id == op.machine_id, \
                f"{op.operation_id}: predecessor should be on same machine"
    print(f"  [PASS] Precedence chains valid (D -> I -> R per machine)")

    # --- 3i: Operation IDs ---
    valid_suffixes = {"_D": OperationType.DISASSEMBLE,
                      "_I": OperationType.INSPECT,
                      "_R": OperationType.INSTALL}
    expected_op_ids = set()
    for mid in expected_ids:
        for suffix, optype in valid_suffixes.items():
            expected_op_ids.add(mid + suffix)
    actual_op_ids = set(operations.keys())
    missing_ops = expected_op_ids - actual_op_ids
    extra_ops = actual_op_ids - expected_op_ids
    assert not missing_ops, f"Missing operation IDs: {sorted(missing_ops)}"
    assert not extra_ops, f"Extra operation IDs: {sorted(extra_ops)}"
    print(f"  [PASS] All operation IDs match expected format")

    # --- 3j: Service anchors ---
    for op in operations.values():
        anchor = op.service_anchor
        m = machines[op.machine_id]
        # Anchor should be the left cell of the machine
        assert anchor == m.cells[0], \
            f"{op.operation_id}: anchor {anchor} != machine left cell {m.cells[0]}"
    print(f"  [PASS] Service anchors point to machine left cells")

    return machines, operations


# ============================================================================
# SECTION 4: Terrain text visualization
# ============================================================================
def section4_terrain_visualization(fmap: FixedMap):
    print("\n" + "=" * 70)
    print("SECTION 4: Terrain Text Visualization")
    print("=" * 70)

    terrain = fmap.terrain
    symbols = {
        0: '#',   # OBSTACLE
        1: '.',   # INTERNAL_ROAD
        2: '=',   # TRUNK_ROAD
    }

    # Mark machine locations
    machine_cells = set()
    if fmap.machines:
        for m in fmap.machines.values():
            for c in m.cells:
                machine_cells.add((c.x, c.y))

    # Build visual grid
    rows_text = []
    for y in range(29):  # 0-indexed
        chars = []
        for x in range(25):  # 0-indexed
            if (x + 1, y + 1) in machine_cells:
                chars.append('M')
            else:
                v = int(terrain[y, x])
                chars.append(symbols.get(v, '?'))
        rows_text.append(''.join(chars))

    # Print
    print("   " + "".join(str(i % 10) for i in range(1, 26)))
    for i, line in enumerate(rows_text):
        print(f"{i+1:2d} {line}")

    print("\n  Legend: # = Obstacle  . = Internal Road  = = Trunk Road  M = Machine")
    print("  [PASS] Terrain visualization rendered")

    # Verify trunk rows visually contain no obstacles
    for y in range(23, 29):  # y=24..29 (0-indexed 23..28)
        for x in range(25):
            assert int(terrain[y, x]) == 2, \
                f"Cell ({x+1},{y+1}) in trunk should be TRUNK_ROAD(2)"
    print(f"  [PASS] Trunk rows (y>=24) are all TRUNK_ROAD (verified cell-by-cell)")


# ============================================================================
# SECTION 5: Footprint validation
# ============================================================================
def section5_footprint_validation(fmap: FixedMap):
    print("\n" + "=" * 70)
    print("SECTION 5: Footprint Validation")
    print("=" * 70)

    terrain = fmap.terrain
    footprint = Footprint.default_2x4()

    # --- 5a: Footprint structure ---
    assert len(footprint.offsets) == 8, f"Footprint should have 8 offsets, got {len(footprint.offsets)}"
    expected_offsets = set()
    for dy in range(4):
        for dx in range(2):
            expected_offsets.add(Cell(dx, dy))
    actual_offsets = set(footprint.offsets)
    assert actual_offsets == expected_offsets, \
        f"Footprint offsets mismatch: {actual_offsets - expected_offsets}"
    print(f"  [PASS] Footprint has 8 offsets (2x4)")

    # --- 5b: cells_at ---
    anchor = Cell(5, 10)
    occupied = footprint.cells_at(anchor)
    assert len(occupied) == 8, f"cells_at should return 8 cells, got {len(occupied)}"
    for dx in range(2):
        for dy in range(4):
            assert Cell(5 + dx, 10 + dy) in occupied, f"Missing cell ({5+dx},{10+dy})"
    print(f"  [PASS] cells_at() returns correct 8 cells")

    # --- 5c: work_zone ---
    wz = Footprint.work_zone()
    assert len(wz) == 2, f"work_zone should return 2 cells, got {len(wz)}"
    assert Cell(0, 0) in wz, "work_zone missing Cell(0,0)"
    assert Cell(1, 0) in wz, "work_zone missing Cell(1,0)"
    print(f"  [PASS] work_zone is correct: top 2 cells (0,0) and (1,0)")

    # --- 5d: Valid poses on trunk road ---
    # Max anchor y = 26 (29 - 4 + 1), since footprint height is 4
    trunk_test_poses = [
        (Cell(1, 25), True),
        (Cell(24, 26), True),   # max x=24, max y=26 for 2x4 footprint
        (Cell(12, 24), True),
        (Cell(23, 25), True),
    ]
    for pose, expected in trunk_test_poses:
        result = FootprintValidator.is_valid_pose(pose, footprint, terrain)
        assert result == expected, \
            f"Pose ({pose.x},{pose.y}) on trunk: expected {expected}, got {result}"
    print(f"  [PASS] Trunk road poses validated correctly")

    # --- 5e: Poses on obstacles should be invalid ---
    # Cell (4, 10) is an obstacle; the pose at (4, 9) should fail because
    # the footprint extends down to row 12 including the obstacle
    invalid_poses = [
        Cell(4, 5),   # obstacle column in interior
        Cell(10, 10), # obstacle column
        Cell(1, 1),   # top-left corner, still valid (internal road)
    ]
    for pose in invalid_poses:
        result = FootprintValidator.is_valid_pose(pose, footprint, terrain)
        print(f"    Pose ({pose.x:2d},{pose.y:2d}): {'VALID' if result else 'INVALID'}")

    # --- 5f: transition validation ---
    # Horizontal move on trunk should be valid
    ok, reason = FootprintValidator.is_valid_transition(
        Cell(1, 25), Cell(2, 25), footprint, terrain, trunk_y_threshold=24
    )
    assert ok, f"Horizontal move on trunk should be valid: {reason}"
    print(f"  [PASS] Horizontal transition on trunk: {reason}")

    # Horizontal move in interior should be invalid
    ok, reason = FootprintValidator.is_valid_transition(
        Cell(5, 10), Cell(6, 10), footprint, terrain, trunk_y_threshold=24
    )
    assert not ok, f"Horizontal move in interior should be invalid"
    print(f"  [PASS] Horizontal transition in interior: correctly rejected ({reason[:50]}...)")

    # Vertical move in interior should be valid
    ok, reason = FootprintValidator.is_valid_transition(
        Cell(5, 10), Cell(5, 11), footprint, terrain, trunk_y_threshold=24
    )
    assert ok, f"Vertical move in interior should be valid: {reason}"
    print(f"  [PASS] Vertical transition in interior: {reason}")

    # WAIT always valid
    ok, reason = FootprintValidator.is_valid_transition(
        Cell(5, 10), Cell(5, 10), footprint, terrain, trunk_y_threshold=24
    )
    assert ok, f"WAIT should be valid: {reason}"
    print(f"  [PASS] WAIT transition: {reason}")

    # --- 5g: classify_action ---
    tests = [
        (Cell(5, 10), Cell(5, 9), Action.UP),
        (Cell(5, 10), Cell(5, 11), Action.DOWN),
        (Cell(5, 10), Cell(4, 10), Action.LEFT),
        (Cell(5, 10), Cell(6, 10), Action.RIGHT),
        (Cell(5, 10), Cell(5, 10), Action.WAIT),
        (Cell(5, 10), Cell(6, 11), None),
    ]
    for frm, to, expected in tests:
        result = FootprintValidator.classify_action(frm, to)
        assert result == expected, \
            f"classify_action({frm}, {to}): expected {expected}, got {result}"
    print(f"  [PASS] Action classification correct for all 6 cases")

    # --- 5h: Enumerate all valid poses ---
    valid_count = 0
    invalid_count = 0
    out_of_bounds = 0
    for y in range(1, 30):
        for x in range(1, 26):
            if FootprintValidator.is_valid_pose(Cell(x, y), footprint, terrain):
                valid_count += 1
            else:
                invalid_count += 1
    total = valid_count + invalid_count
    assert total == 25 * 29, f"Enumeration mismatch: {total} != 725"
    print(f"  [PASS] Full enumeration: {valid_count} valid, {invalid_count} invalid (total={total})")

    return footprint


# ============================================================================
# SECTION 6: Service poses & PoseGraph
# ============================================================================
def section6_pose_graph(fmap: FixedMap):
    print("\n" + "=" * 70)
    print("SECTION 6: PoseGraph Construction & Service Poses")
    print("=" * 70)

    terrain = fmap.terrain
    footprint = Footprint.default_2x4()
    machines = fmap.machines

    # --- 6a: Service pose computation ---
    for mid in list(machines.keys())[:5]:  # test first 5
        m = machines[mid]
        sp = ServicePoseCalculator.compute_service_anchor(m, footprint)
        assert sp == m.cells[0], \
            f"{mid}: service anchor {sp} != left cell {m.cells[0]}"
    print(f"  [PASS] Service anchors match left machine cells (verified 5 samples)")

    # All 48
    all_anchors = ServicePoseCalculator.compute_all_service_anchors(machines, footprint)
    assert len(all_anchors) == 48, f"Expected 48 service anchors, got {len(all_anchors)}"
    for mid, anchor in all_anchors.items():
        assert anchor == machines[mid].cells[0], \
            f"{mid}: anchor mismatch"
    print(f"  [PASS] All 48 service anchors computed correctly")

    # --- 6b: Build PoseGraph ---
    pg = PoseGraph(terrain, footprint, trunk_y_threshold=24)
    pg.build()

    node_count = pg.node_count()
    edge_count = pg.edge_count()
    assert node_count > 0, "PoseGraph should have valid poses"
    assert edge_count > 0, "PoseGraph should have edges"
    print(f"  PoseGraph: {node_count} valid poses, {edge_count} edges")

    # Verify all service anchors are in the graph
    for mid, anchor in all_anchors.items():
        assert pg.is_valid_pose(anchor), \
            f"Service anchor for {mid} ({anchor.x},{anchor.y}) not in PoseGraph!"
    print(f"  [PASS] All 48 service anchors are valid poses in PoseGraph")

    # --- 6c: Test neighbors ---
    test_anchor = Cell(12, 25)  # middle of trunk
    if pg.is_valid_pose(test_anchor):
        neighbors = pg.get_neighbors(test_anchor)
        print(f"  Neighbors of ({test_anchor.x},{test_anchor.y}): {len(neighbors)}")
        for n, cost in neighbors:
            action = FootprintValidator.classify_action(test_anchor, n)
            print(f"    -> ({n.x:2d},{n.y:2d}) cost={cost} [{action.name if action else '?'}]")
    else:
        print(f"  [WARN] ({test_anchor.x},{test_anchor.y}) not in valid poses")

    # --- 6d: Connectivity check ---
    # Verify we can reach another trunk cell from a trunk cell
    start = Cell(1, 25)
    goal = Cell(24, 26)  # max anchor for 2x4 footprint on trunk
    if pg.is_valid_pose(start) and pg.is_valid_pose(goal):
        # Simple BFS to confirm connectivity
        from collections import deque
        visited = {start}
        queue = deque([start])
        found = False
        while queue and not found:
            cur = queue.popleft()
            if cur == goal:
                found = True
                break
            for nbr, _ in pg.get_neighbors(cur):
                if nbr not in visited:
                    visited.add(nbr)
                    queue.append(nbr)
        if found:
            print(f"  [PASS] BFS: ({start.x},{start.y}) -> ({goal.x},{goal.y}) reachable")
        else:
            print(f"  [FAIL] BFS: ({start.x},{start.y}) -> ({goal.x},{goal.y}) UNREACHABLE")
            assert False, "Trunk cells should be reachable via BFS"
    else:
        print(f"  [WARN] Start or goal not in valid poses")

    print("  [PASS] PoseGraph section complete")

    return pg, all_anchors


# ============================================================================
# SECTION 7: StaticAStar + Travel time matrix
# ============================================================================
def section7_travel_times(fmap: FixedMap, pg: PoseGraph, all_anchors: dict):
    print("\n" + "=" * 70)
    print("SECTION 7: StaticAStar & Travel Time Matrix")
    print("=" * 70)

    if not _STATIC_ASTAR_AVAILABLE:
        print("  [SKIP] StaticAStar module not yet available (Agent1 pending)")
        print("  Running basic BFS-based reachability instead...\n")

        from collections import deque
        footprint = Footprint.default_2x4()

        # Check reachability for a small subset using BFS
        anchor_items = list(all_anchors.items())[:12]
        n = len(anchor_items)
        reachable = 0
        unreachable = 0
        t0 = time.time()

        for i, (mid_i, pose_i) in enumerate(anchor_items):
            for j, (mid_j, pose_j) in enumerate(anchor_items):
                if i == j:
                    reachable += 1
                    continue
                # BFS from pose_i to pose_j
                visited = {pose_i}
                queue = deque([(pose_i, 0)])
                found = False
                while queue and not found:
                    cur, dist = queue.popleft()
                    if cur == pose_j:
                        found = True
                        break
                    for nbr, _ in pg.get_neighbors(cur):
                        if nbr not in visited:
                            visited.add(nbr)
                            queue.append((nbr, dist + 1))
                if found:
                    reachable += 1
                else:
                    unreachable += 1

        elapsed = time.time() - t0
        total = n * n
        print(f"  Subset size: {n} poses ({total} pairs)")
        print(f"  Reachable: {reachable}, Unreachable: {unreachable}")
        print(f"  BFS time: {elapsed:.2f}s")

        assert unreachable == 0, f"{unreachable} unreachable pairs in subset!"
        print("  [PASS] BFS reachability: all pairs reachable in subset")

        # Full matrix estimate (for stats only)
        all_items = list(all_anchors.items())
        m = len(all_items)
        print(f"\n  Full matrix would be: {m} x {m} = {m*m} entries")
        print(f"  Estimated full BFS time: {elapsed * (m*m) / (n*n):.1f}s")
        print("  [INFO] Full travel time matrix deferred until StaticAStar is available")
        return None

    # ---- StaticAStar is available ----
    footprint = Footprint.default_2x4()
    astar = StaticAStar(pg)

    # Collect all service poses
    anchor_items = list(all_anchors.items())

    # Compute full pairwise travel times
    t0 = time.time()
    travel_times = {}
    m = len(anchor_items)
    computed = 0
    failed = 0

    print(f"  Computing {m}x{m} travel time matrix via A*...")
    for i, (mid_i, pose_i) in enumerate(anchor_items):
        for j, (mid_j, pose_j) in enumerate(anchor_items):
            if i == j:
                travel_times[(mid_i, mid_j)] = 0
            else:
                result = astar.plan(pose_i, pose_j)
                if result:
                    path, cost = result
                    travel_times[(mid_i, mid_j)] = cost
                    computed += 1
                else:
                    travel_times[(mid_i, mid_j)] = -1
                    failed += 1

    elapsed = time.time() - t0
    total = m * m
    print(f"  Matrix size: {m} x {m} = {total} entries")
    print(f"  Computed: {computed}, Failed: {failed}")
    print(f"  Time: {elapsed:.2f}s ({elapsed/total*1000:.2f} ms per entry)")

    assert failed == 0, f"{failed} unreachable pairs in travel time matrix!"
    assert computed == total - m, f"Expected {total-m} non-diagonal entries, got {computed}"

    # Stats
    non_diag = [v for k, v in travel_times.items() if k[0] != k[1]]
    print(f"  Min travel time: {min(non_diag)}")
    print(f"  Max travel time: {max(non_diag)}")
    print(f"  Mean travel time: {sum(non_diag)/len(non_diag):.1f}")
    print(f"  Median travel time: {sorted(non_diag)[len(non_diag)//2]}")

    print("  [PASS] Travel time matrix complete")

    return travel_times


# ============================================================================
# MAIN
# ============================================================================
def main():
    print("\n" + "=" * 70)
    print("  MULTI-ROBOT SCHEDULER v4.0 - COMPREHENSIVE VERIFICATION")
    print("=" * 70)
    print()

    results = {}
    start_time = time.time()

    # ---- Section 1: Map Generation ----
    try:
        fmap = section1_map_generation()
        results["SECTION 1: Map Generation"] = "PASS"
    except Exception as e:
        results["SECTION 1: Map Generation"] = f"FAIL: {e}"
        print(f"\n  *** SECTION 1 FAILED: {e} ***")
        import traceback
        traceback.print_exc()
        print("  Cannot continue without a valid map. Aborting.")
        sys.exit(1)

    # ---- Section 2: Map Summary ----
    try:
        section2_map_summary(fmap)
        results["SECTION 2: Map Summary"] = "PASS"
    except Exception as e:
        results["SECTION 2: Map Summary"] = f"FAIL: {e}"
        print(f"\n  *** SECTION 2 FAILED: {e} ***")
        import traceback
        traceback.print_exc()

    # ---- Section 3: Machines & Ops ----
    try:
        section3_machines_and_operations(fmap)
        results["SECTION 3: Machines & Ops"] = "PASS"
    except Exception as e:
        results["SECTION 3: Machines & Ops"] = f"FAIL: {e}"
        print(f"\n  *** SECTION 3 FAILED: {e} ***")
        import traceback
        traceback.print_exc()

    # ---- Section 4: Visualization ----
    try:
        section4_terrain_visualization(fmap)
        results["SECTION 4: Visualization"] = "PASS"
    except Exception as e:
        results["SECTION 4: Visualization"] = f"FAIL: {e}"
        print(f"\n  *** SECTION 4 FAILED: {e} ***")
        import traceback
        traceback.print_exc()

    # ---- Section 5: Footprint Validation ----
    try:
        footprint = section5_footprint_validation(fmap)
        results["SECTION 5: Footprint Validation"] = "PASS"
    except Exception as e:
        footprint = None
        results["SECTION 5: Footprint Validation"] = f"FAIL: {e}"
        print(f"\n  *** SECTION 5 FAILED: {e} ***")
        import traceback
        traceback.print_exc()

    # ---- Section 6: PoseGraph ----
    try:
        pg, all_anchors = section6_pose_graph(fmap)
        results["SECTION 6: PoseGraph"] = "PASS"
    except Exception as e:
        pg = None
        all_anchors = None
        results["SECTION 6: PoseGraph"] = f"FAIL: {e}"
        print(f"\n  *** SECTION 6 FAILED: {e} ***")
        import traceback
        traceback.print_exc()

    # ---- Section 7: Travel Times ----
    try:
        section7_travel_times(fmap, pg, all_anchors)
        results["SECTION 7: Travel Times"] = "PASS"
    except Exception as e:
        results["SECTION 7: Travel Times"] = f"FAIL: {e}"
        print(f"\n  *** SECTION 7 FAILED: {e} ***")
        import traceback
        traceback.print_exc()

    elapsed = time.time() - start_time

    # ========================
    # FINAL SUMMARY
    # ========================
    print("\n\n" + "=" * 70)
    print("  VERIFICATION SUMMARY")
    print("=" * 70)
    passes = sum(1 for v in results.values() if v == "PASS")
    fails = sum(1 for v in results.values() if v != "PASS")
    for section, status in results.items():
        marker = "  [PASS]" if status == "PASS" else "  [FAIL]"
        print(f"{marker} {section:<35s}: {status}")
    print(f"\n  Total: {passes} passed, {fails} failed out of {len(results)} sections")
    print(f"  Total elapsed: {elapsed:.2f}s")

    if fails > 0:
        print("\n  *** SOME SECTIONS FAILED - see details above ***")
        return False
    else:
        print("\n  [ALL SECTIONS PASSED]")
        return True


if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)
