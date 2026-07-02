# 多机器人碰撞检测修复方案 v4.0

## 问题根因分析

### 根因1: 仿真引擎缺少碰撞检测（执行层）

`SimulationEngine.step()` 的文档注释声称第8步是"检查碰撞"（见 `engine.py` 第13行），但实际代码从未实现。`_execute_actions()` 执行动作后直接进入 `_update_work_progress()`，没有任何重叠检测。两台机器人的 2x4 footprint 可能完全重叠而仿真继续运行，无任何警告。

### 根因2: 运行时占用与预约表之间存在时间缺口（核心根因）

机器人的物理占用与预约表之间存在**严重的时间维度缺口**。具体机制如下：

**正常运行时路径（以 `plan_with_service` + `_execute_actions` + `_update_work_progress` 为例）：**

| 时间段 | 机器人状态 | 预约表状态 | 缺口？ |
|--------|-----------|-----------|--------|
| 规划后 | IDLE -> MOVING | 整个路径（移动+服务WAIT）已预约 | 无 |
| 移动中 | MOVING | 每个时间步的 `reserve_pose` 覆盖当前位置 | 无 |
| 服务WAIT阶段 | MOVING（执行 WAIT pose） | WAIT pose 的 `reserve_pose` 覆盖 | 无 |
| 路径结束，开始作业 | WORKING，work_remaining > 0 | **WAIT pose 预约已到期，但机器人仍在原地** | **有缺口** |
| 作业中 | WORKING | **无预约** | **有缺口** |
| 作业完成 | IDLE | **无预约** | **有缺口** |
| 全部任务完成 | FINISHED | **无预约** | **有缺口** |

**缺口产生的精确机制：**

1. `plan_with_service()` 生成 `service_duration` 个 WAIT pose（如 duration=5，则在 t=11~15 的 WAIT pose）
2. `reserve_path()` 预约这些 WAIT pose 的占用（t=11~15）
3. WAIT pose 逐个执行完毕后，`path_index >= len(current_path)`，此时设置 `work_remaining = op.duration`（如 5）
4. `_update_work_progress()` 开始倒计时 `work_remaining`，从 5 到 0，耗时 5 个时间步
5. 在 `_update_work_progress()` 消耗 work_remaining 的这 5 个时间步中，预约表中**没有任何该机器人的预约**
6. 总共在目标位置停留 2 * service_duration 步，但只有 service_duration 步有预约

**后果：** 当 B_1 处于 WORKING 状态后段（或刚结束作业处于 IDLE 状态），其他机器人（如 A_1）进行路径规划时，SpaceTimeA* 查询预约表不会发现 B_1 当前位置的占用。规划出的路径可能直接穿过 B_1 的物理位置。

### 根因3: 规划器对静止机器人"失明"

`SpaceTimeAStar._astar_search()` 在扩展节点时只检查：
1. 预约表 `is_pose_free()` — 只查 `pose_cells_by_time`
2. 预约表 `is_swept_free()` — 只查 `swept_cells_by_interval`
3. 姿态图 `is_valid_pose()` — 只查静态障碍物

没有任何机制让规划器感知其他机器人**当前位置**（无论是 IDLE、WORKING 还是 FINISHED）。规划器只看到预约表中显式预约的时空槽位，看不到物理上占据空间但未预约的机器人。

### 根因4（次要）: service_locks 未被路径规划器检查

`ReservationTable.service_locks` 在 `reserve_service()` 中被写入，通过 `is_service_free()` 自我检查避免重复预约同一机器的服务时间。但 `is_service_free()` **从未被 `SpaceTimeAStar._astar_search()` 调用**。这意味着 service_locks 只防止同一台机器被两个操作同时占用，不参与空间冲突检测。这不是碰撞的直接原因（因为 WAIT pose 的 `reserve_pose` 已经占用了空间格），但如果 service_locks 的设计意图是阻止其他机器人进入机器区域，则存在设计缺陷。

---

## 三层修复方案

### 层次1: 执行层碰撞检测（兜底，防御性）

**目的：** 确保最终执行轨迹绝对无碰撞，即使在规划层出现漏洞。

**实现位置：** `engine.py` — `SimulationEngine.step()` 方法，在 `_execute_actions()` 之后、`_update_work_progress()` 之前。

**具体实现：**

```python
# 在 step() 方法中，_execute_actions 之后添加：
self._detect_and_resolve_collisions()
```

新增方法 `_detect_and_resolve_collisions()`:

```python
def _detect_and_resolve_collisions(self) -> None:
    """检查所有机器人对是否碰撞，如有则回退 MOVING 机器人。"""
    robot_ids = list(self.robots.keys())
    
    for i in range(len(robot_ids)):
        for j in range(i + 1, len(robot_ids)):
            rid_a = robot_ids[i]
            rid_b = robot_ids[j]
            robot_a = self.robots[rid_a]
            robot_b = self.robots[rid_b]
            
            # 两个机器人必须都有有效位置
            if robot_a.current_anchor is None or robot_b.current_anchor is None:
                continue
            
            # 计算各自占用的格子
            cells_a = self.footprint.cells_at(robot_a.current_anchor)
            cells_b = self.footprint.cells_at(robot_b.current_anchor)
            
            # 检查是否有重叠
            if cells_a & cells_b:
                # 碰撞！回退逻辑：优先回退 MOVING 的，都不动则回退较晚移动的
                self._log_event(
                    "collision_detected",
                    f"COLLISION: {rid_a}@({robot_a.current_anchor.x},{robot_a.current_anchor.y}) "
                    f"vs {rid_b}@({robot_b.current_anchor.x},{robot_b.current_anchor.y})"
                )
                
                # 回退策略：回退 MOVING 机器人到上一步位置
                # 如果都在移动，回退 path_index 较大的
                rollback_candidate = None
                if robot_a.status == RobotStatus.MOVING and robot_b.status != RobotStatus.MOVING:
                    rollback_candidate = (rid_a, robot_a)
                elif robot_b.status == RobotStatus.MOVING and robot_a.status != RobotStatus.MOVING:
                    rollback_candidate = (rid_b, robot_b)
                elif robot_a.status == RobotStatus.MOVING and robot_b.status == RobotStatus.MOVING:
                    # 两者都在移动：回退 path_index 较大的（移动了更多步的）
                    if robot_a.path_index > robot_b.path_index:
                        rollback_candidate = (rid_a, robot_a)
                    else:
                        rollback_candidate = (rid_b, robot_b)
                
                if rollback_candidate:
                    rid, robot = rollback_candidate
                    self._rollback_robot(rid, robot)
```

新增方法 `_rollback_robot()`:

```python
def _rollback_robot(self, rid: str, robot: RobotRuntime) -> None:
    """回退机器人到上一步位置。"""
    if robot.path_index > 1 and robot.current_path:
        # 回到路径中的前一个位置
        prev_pose = robot.current_path[robot.path_index - 2]
        robot.current_anchor = Cell(prev_pose.x, prev_pose.y)
        robot.path_index -= 1
        self._log_event(
            "rollback",
            f"{rid}: rolled back to ({prev_pose.x},{prev_pose.y})"
        )
    elif robot.path_index == 1 and robot.current_path:
        # 回到路径起点
        prev_pose = robot.current_path[0]
        robot.current_anchor = Cell(prev_pose.x, prev_pose.y)
        robot.path_index = 0
        self._log_event(
            "rollback",
            f"{rid}: rolled back to start ({prev_pose.x},{prev_pose.y})"
        )
```

**影响范围：**
- 新增方法: `_detect_and_resolve_collisions()`, `_rollback_robot()`
- 修改方法: `step()` — 在第3步和第4步之间插入调用
- 新增事件日志类型: `collision_detected`, `rollback`

**优点：**
- 实现简单，零侵入其他模块
- 绝对保证最终轨迹无碰撞（最后防线）
- 碰撞时记录日志便于调试

**缺点：**
- 只能事后检测，不能预防 — 碰撞发生后回退导致仿真抖动
- 回退可能引发连锁回退
- 不解决规划器"失明"问题（碰撞可能频繁发生）
- 仅做 O(n^2) 机器人对检查，大型场景可能有性能问题

**优先级：** 最高（作为安全网最先实现）

---

### 层次2: 规划层障碍物感知（预防，核心）

**目的：** 让 SpaceTimeA* 在搜索路径时"看见"所有静止机器人的当前位置，从源头避免碰撞路径。

**实现位置：** `engine.py` — `_plan_idle_robots()` 方法，以及 `reservation_table.py`。

**具体实现：**

#### 步骤A: 在 `ReservationTable` 中添加临时预约机制

在 `reservation_table.py` 中新增方法：

```python
# reservation_table.py — 新增方法

def reserve_pose_range(
    self,
    t_start: int,
    t_end: int,
    cells: frozenset[Cell],
    robot_id: str,
) -> bool:
    """在时间范围 [t_start, t_end) 内预约本体占用。
    
    用于标记静止机器人的持续占用，使路径规划器可以预见。
    
    Args:
        t_start: 开始时间（含）
        t_end: 结束时间（不含）
        cells: 占用的格子集合
        robot_id: 机器人ID
    
    Returns:
        True 如果预约成功
    """
    for t in range(t_start, t_end):
        if not self.is_pose_free(t, cells, exclude_robot=robot_id):
            # 该时间步不空闲，跳过（不中断整个范围预约）
            # 保守策略：宁可部分预约也要标记占用
            continue
        self.pose_cells_by_time[t][cells] = robot_id
    return True


def release_pose_range(
    self,
    t_start: int,
    t_end: int,
    robot_id: str,
) -> None:
    """释放时间范围 [t_start, t_end) 内的临时预约。
    
    Args:
        t_start: 开始时间（含）
        t_end: 结束时间（不含）
        robot_id: 机器人ID
    """
    for t in range(t_start, t_end):
        if t in self.pose_cells_by_time:
            cells_to_remove = []
            for cells, rid in self.pose_cells_by_time[t].items():
                if rid == robot_id:
                    cells_to_remove.append(cells)
            for cells in cells_to_remove:
                del self.pose_cells_by_time[t][cells]
            if not self.pose_cells_by_time[t]:
                del self.pose_cells_by_time[t]
```

#### 步骤B: 在 `_plan_idle_robots()` 中收集虚拟障碍物并注入预约表

在 `engine.py` 的 `_plan_idle_robots()` 方法中，规划循环之前添加：

```python
def _plan_idle_robots(self) -> None:
    """为所有 IDLE 状态的机器人规划下一个任务的路径。"""
    
    # ===== 层次2修复: 收集静止机器人的虚拟障碍物 =====
    temp_reservations: list[tuple[str, int, int, frozenset[Cell]]] = []
    
    for rid, robot in self.robots.items():
        if robot.current_anchor is None:
            continue
        
        # 只对不在本次规划范围内的机器人添加虚拟预约
        # IDLE 且未 finished: 本次会被规划，自身路径会覆盖预约，无需虚拟预约
        # MOVING: 已有路径预约，无需虚拟预约
        if robot.status in (RobotStatus.IDLE, RobotStatus.MOVING):
            if robot.status == RobotStatus.IDLE and not robot.finished:
                continue  # 本次会被规划
            if robot.status == RobotStatus.MOVING:
                continue  # 已有路径预约
        
        # 需要虚拟预约的状态: WORKING, FINISHED, WAITING_PRECEDENCE
        cells = self.footprint.cells_at(robot.current_anchor)
        
        # 计算合理的占用时间范围
        if robot.status == RobotStatus.WORKING:
            # 工作还会持续 work_remaining 步
            t_end = self.current_time + robot.work_remaining + 1
        elif robot.status == RobotStatus.FINISHED:
            # 永久占用（使用规划时间上限）
            t_end = self._max_steps
        elif robot.status == RobotStatus.WAITING_PRECEDENCE:
            # 不确定何时释放，使用较大范围
            t_end = self.current_time + 200  # 保守估计
        else:
            continue
        
        # 添加临时预约
        self.reservation_table.reserve_pose_range(
            self.current_time, t_end, cells, rid
        )
        temp_reservations.append((rid, self.current_time, t_end, cells))
    
    # ===== 原有的规划循环 =====
    for rid, robot in self.robots.items():
        # ... 原有的规划代码 ...
    
    # ===== 清理临时预约 =====
    for rid, t_start, t_end, cells in temp_reservations:
        self.reservation_table.release_pose_range(t_start, t_end, rid)
```

**影响范围：**
- 修改文件: `reservation_table.py` — 新增 `reserve_pose_range()` 和 `release_pose_range()`
- 修改文件: `engine.py` — `_plan_idle_robots()` 开头和结尾插入代码
- 新增事件日志类型: 建议添加 `virtual_reservation` 日志

**优点：**
- 从源头消除碰撞：SpaceTimeA* 不会生成穿过静止机器人的路径
- 无运行时开销：在执行层不需要回退
- 利用现有的预约表机制，SpaceTimeA* 无需修改
- 临时预约在规划结束后清理，不污染预约表

**缺点：**
- `reserve_pose_range` 是 O(t_end - t_start) 循环，对大规模场景和长时间范围可能有性能影响
- 对 WAITING_PRECEDENCE 的占用时间估计不精确，过于保守可能导致不必要的绕行
- FINISHED 机器人的永久占用可能导致其他机器人无路可走（但这在物理上也是合理的）

**优先级：** 最高（根源级修复）

---

### 层次3: 路径冲突预检（优化）

**目的：** 在新路径规划完成后，立即验证新预约不会与已执行中的路径冲突。提供"乐观规划 + 立即验证"的安全网。

**实现位置：** `engine.py` — `_plan_idle_robots()` 方法，每个机器人规划成功后。

**具体实现：**

在 `_plan_idle_robots()` 中，每个机器人规划并预约成功后，添加冲突检测：

```python
# 在 _plan_idle_robots() 中，每个机器人的规划成功分支内（约第400行后）：
# 设置运行时状态之后，添加：

# ===== 层次3修复: 检查新预约是否与正在执行的路径冲突 =====
conflict_robot = self._check_path_conflict(rid, path)
if conflict_robot is not None:
    # 撤销刚做的新预约
    self.reservation_table.release_future(rid, path[0].t)
    self.state_machine.unlock_machine(operation.machine_id)
    robot.status = RobotStatus.WAITING_CONFLICT
    robot.current_path = None
    self._log_event(
        "path_conflict_detected",
        f"{rid}: new path conflicts with {conflict_robot}'s executing path"
    )
    continue  # 不在本次规划中重试，下次 step 会重试
```

新增方法 `_check_path_conflict()`:

```python
def _check_path_conflict(
    self, new_robot_id: str, new_path: list[TimedPose]
) -> str | None:
    """检查新规划路径是否与任何其他机器人已执行中的路径冲突。
    
    Args:
        new_robot_id: 新规划的机器人ID
        new_path: 新规划的路径
    
    Returns:
        如果冲突，返回冲突机器人ID，否则返回 None
    """
    for rid, robot in self.robots.items():
        if rid == new_robot_id:
            continue
        if robot.current_path is None or robot.path_index >= len(robot.current_path):
            continue
        
        # 提取其他机器人的剩余路径
        remaining = robot.current_path[robot.path_index:]
        
        # 检查每条新路径的 pose 是否与剩余路径中任何 pose 冲突
        for new_pose in new_path:
            new_cells = self.footprint.cells_at(Cell(new_pose.x, new_pose.y))
            for existing_pose in remaining:
                if new_pose.t != existing_pose.t:
                    continue  # 不同时间步不会冲突
                existing_cells = self.footprint.cells_at(
                    Cell(existing_pose.x, existing_pose.y)
                )
                if new_cells & existing_cells:
                    return rid
    
    return None
```

**影响范围：**
- 修改文件: `engine.py` — 新增 `_check_path_conflict()`，修改 `_plan_idle_robots()` 中的成功路径
- 新增事件日志类型: `path_conflict_detected`

**优点：**
- 在预约正式提交前进行验证（乐观规划 + 事后验证）
- 能检测层次2未能覆盖的边界情况（例如两个运动中的机器人路径交叉）
- 冲突时自动触发重规划，不丢失进度

**缺点：**
- 额外的 O(n * path_length^2) 复杂度（可优化为 O(n * path_length) 通过时间索引）
- 如果频繁冲突，可能导致规划抖动（反复规划-检测-失败-重规划）
- 依赖路径时间步精确匹配，不处理扫掠区域重叠

**优先级：** 中（优化安全网）

---

## 各层修复对比总结

| 维度 | 层次1（执行层） | 层次2（规划层） | 层次3（预检层） |
|------|---------------|---------------|---------------|
| 修复层面 | 事后检测 | 源头预防 | 事后验证 |
| 对系统的影响 | 回退已执行动作 | 路径自然绕开 | 拒绝冲突预约 |
| 路径质量 | 可能产生碰撞然后回退 | 规划时绕开（最优） | 检测到冲突则重规划 |
| 性能开销 | 每步 O(n^2) 检查 | 每轮规划 O(n * horizon) | 每轮规划 O(n * path^2) |
| 实现复杂度 | 低 | 中 | 中 |
| 风险 | 回退可能引发级联回退 | 临时预约可能过于保守 | 可能频繁重规划 |
| 解决根因2 | 否 | **是** | 部分 |
| 解决根因3 | 否 | **是** | 否 |

---

## 推荐实施顺序

### 第1步: 层次1（执行层碰撞检测） — 立即实施

**原因：**
- 实现最简单，< 50 行代码
- 作为最终安全网，防止任何碰撞通过规划层
- 碰撞日志帮助诊断规划层的残余问题
- 可以立即验证系统是否存在实际碰撞

**实施：**
1. 在 `engine.py` 添加 `_detect_and_resolve_collisions()` 和 `_rollback_robot()` 方法
2. 在 `step()` 的 `_execute_actions()` 之后、`_update_work_progress()` 之前调用
3. 运行现有场景，观察碰撞日志频率

### 第2步: 层次2（规划层障碍物感知） — 核心修复

**原因：**
- 这是根本性修复，解决根因2和根因3
- 让规划器"看见"静止机器人，从源头避免碰撞
- 显著减少层次1的回退频率

**实施：**
1. 在 `reservation_table.py` 添加 `reserve_pose_range()` 和 `release_pose_range()`
2. 在 `engine.py` 的 `_plan_idle_robots()` 开头收集静止机器人并添加到临时预约，结尾清理
3. 运行验证：确保层次1的碰撞日志显著减少（理想情况下降为零）

### 第3步: 层次3（路径冲突预检） — 按需实施

**原因：**
- 属于优化类修复，解决层次2可能遗漏的边界情况
- 当层次1和层次2实施后碰撞日志仍非零时再实施
- 或者作为额外的安全网保障

**实施：**
1. 在 `engine.py` 添加 `_check_path_conflict()` 方法
2. 在 `_plan_idle_robots()` 的规划成功后插入冲突检测
3. 运行验证

---

## 附加建议

### A. 修正服务时间双重计数（推荐并行修复）

当前 `plan_with_service()` 中的 WAIT pose 时间和 `_update_work_progress()` 中的 `work_remaining` 倒计时存在**双重计数**：
- WAIT pose 占用了 `service_duration` 步
- `work_remaining` 又占用了 `service_duration` 步
- 机器人实际在目标位置停留 2 * service_duration 步

**修复选项：**
- **选项A（推荐）：** 在 `_execute_actions()` 中，当路径结束且即将开始工作时，设置 `work_remaining = 0`（WAIT pose 已经计入了服务时间）。这意味着 `plan_with_service` 的 WAIT pose 就代表了服务时间本身。
- **选项B：** 移除 `plan_with_service` 中的 WAIT pose，让 `work_remaining` 唯一地代表服务时间。在这个方案中，需要在路径到达后预约服务期间的占用。

**修改位置：** `engine.py` 第 551-558 行（`_execute_actions` 中设置 work_remaining 的位置）

### B. 预约表时间索引优化（性能优化建议）

当前 `reserve_pose_range()` 使用 O(horizon) 循环。如果规划时间范围很大（如 `_max_steps = 20000`），对 FINISHED 机器人添加永久预约会产生大量迭代。

**优化方案：** 为预约表添加"永久占用"层：
```python
# reservation_table.py
self.persistent_occupancy: dict[frozenset[Cell], str] = {}  # 不计时间的永久占用

def is_pose_free(self, t, cells, exclude_robot=None):
    # ... 原有检查 ...
    # 新增：检查永久占用
    for occ_cells, robot_id in self.persistent_occupancy.items():
        if exclude_robot and robot_id == exclude_robot:
            continue
        if cells & occ_cells:
            return False
    return True
```

这样 FINISHED 机器人只需一次 O(1) 操作即可标记，不需要 O(horizon) 循环。

### C. 坐标一致性注意

`Cell` 使用 1-indexed 坐标（见 `models.py`），但姿态图 `PoseGraph` 内部可能使用 0-indexed。确保所有 footprint 计算使用一致的坐标系统。

---

## 修订历史

| 日期 | 版本 | 修改 |
|------|------|------|
| 2026-07-02 | 1.0 | 初始版本 |
