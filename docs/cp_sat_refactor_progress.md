# CP-SAT 重构进度 checkpoint

更新时间：2026-07-07

## 当前目标

把高层任务规划从“CP-SAT 只做任务分配 + 后续手工排序”升级为
`solver_mode="assignment_schedule"`：由 OR-Tools CP-SAT 直接决定机器人分配、
完整操作顺序、计划开始/结束时间、相邻操作旅行时间、D→I→R 工序衔接与优化目标。

## 已完成

- 正式入口改为 `src.solver.scheduler.solve_assignment_schedule(...)`。
- 默认 `allow_fallback=false`，正式模式求解失败时抛出 `SchedulingFailure`，
  不再静默切换手工 fallback。
- `ScheduleResult` / `RobotSchedule` 已增加求解器元数据、makespan、旅行时间、
  换列次数、负载差、fallback 信息、序列来源和每台机器人 ordered operations。
- `src/solver/cp_sat_model.py` 已建立完整 CP-SAT 模型：
  - `assigned[o,r]`
  - `start[o]`
  - `end[o]`
  - `optional_interval[o,r]`
  - `arc[r,i,j]`
  - 每机器人 START/END 虚拟节点
  - `AddCircuit` 序列约束
  - `AddNoOverlap`
  - D→I→R 前置约束
  - footprint-aware static A* 旅行时间约束
  - 可配置同一 A 机器人拆/装约束
  - 第二列首任务、列内自上而下、换列惩罚均作为 CP-SAT 偏好/约束，而不是预排序。
- `src/solver/schedule_extractor.py` 沿 CP-SAT 选中的 arc successor 链提取顺序，
  不再按坐标/机器编号重排。
- `src/solver/assignment_only.py` 已隔离为禁用入口。
- `src/solver/baselines.py` 保留 `row_major_baseline` 与 `column_major_baseline`，
  只用于 baseline/对比，不被正式 CP-SAT 流程调用。
- 仿真器按 CP-SAT 顺序执行；Space-Time A* 只做逐时间步路径规划，不重新排序任务。
- 多机器人动态仿真目前采用保守让行/停车策略来优先保证零碰撞。

## 最近验证结果

命令：

```bash
.venv/bin/python -m pytest -q
.venv/bin/python scripts/run_cp_sat_validation.py
.venv/bin/python scripts/compare_strategies.py
```

结果：

- `pytest`：160 passed。
- 1A1B：CP-SAT FEASIBLE，fallback=false，completed_machines=48，collisions=0，
  constraint_violations=0，实际完工时间 1408。
- 2A1B：CP-SAT FEASIBLE，fallback=false，completed_machines=48，collisions=0，
  constraint_violations=0，实际完工时间 8890。
- 4A2B：CP-SAT FEASIBLE，fallback=false，completed_machines=48，collisions=0，
  constraint_violations=0，实际完工时间 8840。

三策略 1A1B 对比：

| 策略 | 序列来源 | fallback | makespan/实际完工 | 预计旅行时间 | 换列次数 | 碰撞 |
|---|---|---:|---:|---:|---:|---:|
| row_major_baseline | 手工 row-major | true | 3222 | 3944 | 142 | 0 |
| column_major_baseline | 手工 column-major | true | 1363 | 1154 | 22 | 0 |
| cp_sat_assignment_schedule | CP-SAT arc | false | 1408 | 1154 | 22 | 0 |

## 当前限制

- CP-SAT 在默认时间限制下通常返回 FEASIBLE，还不是证明 OPTIMAL。
- 1A1B 上 CP-SAT 已经优于 row-major，但当前实际完工时间略慢于手工
  column-major baseline；后续需要调权重、增加搜索时间或改进 warm-start。
- 2A1B/4A2B 虽然零碰撞完成，但实际完工时间很长，主要因为动态路径层使用
  保守的“通道清空/停车后再继续”策略。
- `PlanningConflict` 已能结构化输出，但自动“路径失败 → 回灌 CP-SAT 约束 →
  重求解”的闭环还没有完全产品化。
- 本地全局 Anaconda Python 与 OR-Tools/protobuf 有 ABI 冲突；正式求解和测试请用
  `.venv/bin/python`。绘图脚本通过独立渲染进程规避本地 Matplotlib 问题。

## 下次建议动作

1. 复查 `scripts/run_cp_sat_validation.py` 与输出 JSON，确认报告字段满足最终汇报需要。
2. 如果准备交付，整理最终中文汇报：修改文件、手工排序隔离点、CP-SAT 变量/约束/目标、
   三场景 solver status、fallback 情况、三策略指标和限制。
3. 若要进一步优化性能，优先做：
   - CP-SAT objective/warm-start 调优；
   - 多机器人路径层减少全局保守等待；
   - `PlanningConflict` 自动回灌 CP-SAT 重求解。
4. 用户确认后再提交/打 tag；当前这些 CP-SAT 重构改动尚未提交。
