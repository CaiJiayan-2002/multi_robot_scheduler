# CP-SAT 完整任务规划

## 重构前真实调用链

正式脚本曾直接调用 `fallback.manual_assign_scenario_1/2`。任务归属和顺序均由
坐标排序及列表拼接产生；`cp_sat_model.py` 的 `assignment_only` 只包含
`x[operation, robot]`，没有开始/结束时间、旅行时间或序列变量。因此仿真器读取
的是手工队列，历史结果中的 `fallback_used=true` 是入口主动选择 baseline，
并非 CP-SAT 求解失败。

## 当前正式调用链

`solver.scheduler.solve_assignment_schedule`
→ footprint-aware `StaticAStar` 旅行矩阵
→ `CpSatScheduler(assignment_schedule)`
→ `schedule_extractor` 沿选中的 arc successor 链提取
→ `SimulationEngine` 按该顺序执行
→ `SpaceTimeAStar` 仅决定逐时间步路径，不重排操作。

正式配置默认 `allow_fallback=false`。求解状态不是 FEASIBLE/OPTIMAL 时抛出
`SchedulingFailure`；只有显式 baseline 或 `allow_fallback=true` 才能调用手工策略。

## 模型

- `assigned[o,r]`：合法机器人唯一分配；
- `start[o]`、`end[o]`；
- `optional_interval[o,r]` + 每机器人 `AddNoOverlap`；
- `arc[r,i,j]`、虚拟 START/END 与 `AddCircuit`；
- arc 蕴含静态 A* 旅行时间；
- 每机 D→I→R；
- 可配置 D/R 同一 A；
- 可配置第二列首任务偏好、列内自上而下偏好/硬约束、换列惩罚；
- 可配置同一列拆机由同一台 A 机器人负责；
- 可配置同一列拆机自下而上连续执行；
- 可配置每台机器人按列块执行任务，列块顺序仍由 CP-SAT 决定；
- 可配置按工序类型拆分列块，用于 scenario2/v2.2 的“先拆机列块、再安装列块”；
- 可配置 A 机拆机优先：所有 DISASSEMBLE 完成后才允许 INSTALL；
- 额外路径反馈 precedence `(before, after, delay)`。

分阶段目标：先 makespan，再固定 makespan 容差优化旅行时间，最后优化换列、
同类型负载差、偏好惩罚和不必要的晚开始。A* 最近邻仅作为 CP-SAT warm-start
hint，不添加固定 arc；最终顺序只来自求解器选中的 arc。

## 职责边界与限制

CP-SAT 负责高层分配、顺序、时间与静态旅行；Space-Time A* 负责 2×4 本体、
扫掠、预约和动态等待。路径失败会产生 `PlanningConflict`，可作为下一轮
`additional_precedence_constraints` 输入。

当前 1A1B、2A1B、4A2B 均已完成端到端零碰撞验证。多机器人场景目前采用
保守的动态协调策略（必要时先清空通道/回停车位），因此路径层实际完工时间
明显大于 CP-SAT 的静态计划 makespan。后续优化重点是把 `PlanningConflict`
自动闭环回灌给 CP-SAT 重求解，并减少过度保守的全局让行。

## 现存场景版本

### scenario1 / v1.3

1A1B 场景使用完整 `assignment_schedule` 模式。CP-SAT 决定 A_1/B_1 的操作
分配、操作顺序、计划开始/结束时间和静态旅行时间衔接。正式结果必须满足：

- `solver_backend == "ortools_cp_sat"`；
- `solver_mode == "assignment_schedule"`；
- `operation_sequence_source == "cp_sat"`；
- `fallback_used == false`。

该版本没有用手工 row-major、column-major 或坐标排序生成任务队列。

### scenario2 / v2.2

2A1B 场景推荐结果为 `outputs/scenario_2/test8/`。除通用 CP-SAT 调度约束外，
v2.2 额外启用：

- `enforce_same_a_robot_for_column_disassembly=True`：同一列拆机由同一台 A 负责；
- `enforce_bottom_up_disassembly_within_column=True`：同一列拆机自下而上连续执行；
- `enforce_robot_column_blocks=True`：每台机器人完成当前列块后才能进入下一列块；
- `column_blocks_by_operation_type=True`：列块按工序类型拆分；
- `enforce_a_disassembly_priority=True`：所有拆机完成后才允许安装。

仿真执行层也读取 `enforce_a_disassembly_priority`，在动态重规划导致实际时间偏移
时阻止 A 机提前启动 INSTALL。路径层允许三台机器人并行移动和工作，由预约表与
safety guard 兜底避碰。

最近一次验证（`scripts/run_cp_sat_validation.py`）：

| 场景 | CP-SAT 状态 | fallback | CP-SAT makespan | 实际完工时间 | 碰撞 | 硬约束违规 |
|---|---:|---:|---:|---:|---:|---:|
| 1A1B | FEASIBLE | false | 1295 | 1408 | 0 | 0 |
| 2A1B | FEASIBLE | false | 1105 | 8890 | 0 | 0 |
| 4A2B | FEASIBLE | false | 533 | 8840 | 0 | 0 |
