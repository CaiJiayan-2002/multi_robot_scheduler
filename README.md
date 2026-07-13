# Multi-Robot Scheduler

面向固定工业地图的多机器人协同调度与无碰撞路径规划系统。项目以 25 × 31 固定地图、48 台离心机和 A/B 两类异构机器人为实验场景，打通任务分配、机器状态管理、时空路径规划、预约表、离散事件仿真与结果可视化。

![固定地图](outputs/fixed_map.png)

## v1.1.0 更新

- 主干道扩展至第 31 行，为 2 × 4 机器人提供第 28 行起终停车位
- 场景 1 改为列优先流水线：从 `x=5` 开始，列内自上而下，`x=2` 最后处理
- A 拆卸、B 跟随检查、A 安装采用一致的机器访问顺序
- 完整场景实现 144/144 操作完成、零碰撞、零约束违规
- 新增流水线顺序和 31 行地图回归测试

## 完整 CP-SAT 正式模式

正式入口现使用 `solver_mode="assignment_schedule"`，由 OR-Tools CP-SAT
联合决定操作分配、机器人完整顺序、开始/结束时间和静态 A* 旅行衔接。
默认 `allow_fallback=false`，求解失败会明确终止，不再静默使用手工队列。
手工 row-major/column-major 仅保留为显式基线。

正式验证（建议使用项目虚拟环境）：

```bash
python -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
python scripts/run_cp_sat_validation.py
python scripts/compare_strategies.py
```

模型、调用链和限制见 [CP-SAT 完整任务规划](docs/cp_sat_assignment_schedule.md)。

## 现存版本求解器约束说明

当前正式版本中，高层任务规划由 OR-Tools CP-SAT 求解器完成。这里的
“高层任务规划”包括：操作分配、每台机器人执行顺序、计划开始/结束时间、
相邻操作预计移动时间、拆卸/检测/安装前置关系，以及 makespan 等优化目标。

核心代码：

- [src/solver/scheduler.py](src/solver/scheduler.py)：正式求解入口，
  `solve_assignment_schedule(...)` 会构造旅行时间矩阵并调用 CP-SAT；
- [src/solver/cp_sat_model.py](src/solver/cp_sat_model.py)：CP-SAT 模型主体；
- [src/solver/schedule_extractor.py](src/solver/schedule_extractor.py)：沿 CP-SAT
  选中的 arc successor 链提取操作顺序，不重新按坐标或机器编号排序；
- [src/solver/travel_time.py](src/solver/travel_time.py)：使用 footprint-aware
  static A* 生成操作间移动时间；
- [src/simulation/engine.py](src/simulation/engine.py)：按 CP-SAT 顺序执行动态仿真，
  Space-Time A* 只负责逐时间步路径、等待、避碰和局部重规划。

### scenario1 / v1.3：1A1B 完整 CP-SAT 调度

运行入口：

```bash
python scripts/run_scenario_1_full.py cp_sat_current_video
```

结果目录：

```text
outputs/scenario_1/cp_sat_current_video/
```

主要 CP-SAT 变量和约束：

- `assigned[o,r]`：每个操作必须且只能分配给一台合法机器人；
- `start[o]` / `end[o]`：每个操作由求解器决定计划开始和结束时间；
- `optional_interval[o,r]` + `AddNoOverlap`：同一机器人不能同时执行两个操作；
- `arc[r,i,j]` + `AddCircuit`：每台机器人的完整操作顺序由求解器选择；
- 类型约束：A 机器人只能执行 `DISASSEMBLE` / `INSTALL`，B 机器人只能执行
  `INSPECT`；
- 工序约束：每台离心机必须满足 `DISASSEMBLE → INSPECT → INSTALL`；
- 旅行时间约束：若 CP-SAT 选择 `arc[r,i,j]`，则必须满足
  `start[j] >= end[i] + travel_time[r,i,j]`；
- `travel_time` 来自 footprint-aware static A*，不是手写曼哈顿距离；
- 目标函数分阶段优化：先最小化 makespan，再优化预计移动时间、换列次数、
  负载差和不必要等待。

文字说明：scenario1/v1.3 不再使用手工任务队列。代码不会用 row-major、
column-major、机器编号排序或坐标排序来生成正式任务顺序。最终任务顺序来自
CP-SAT 选中的 arc，并在 `metrics.json` 中体现为：

```json
{
  "solver_backend": "ortools_cp_sat",
  "solver_mode": "assignment_schedule",
  "operation_sequence_source": "cp_sat",
  "fallback_used": false
}
```

### scenario2 / v2.2：2A1B 列块约束 + A 机拆机优先

当前 scenario2 的 v2.2 对应推荐结果为 `test8`。

运行入口：

```bash
python scripts/run_scenario_2_full.py test8
```

结果目录：

```text
outputs/scenario_2/test8/
```

在 scenario1/v1.3 的通用 CP-SAT 约束基础上，scenario2/v2.2 增加了以下
正式约束和执行保护：

1. 三台机器人可以同时开工  
   动态路径层不再使用“单移动令牌”限制多机器人移动；A_1、A_2、B_1 可以同时
   规划、移动和工作。避碰由 Space-Time A* 预约表和 safety guard 负责。

2. 同一列拆机由同一台 A 机器人负责  
   `enforce_same_a_robot_for_column_disassembly=True` 时，同一列所有
   `DISASSEMBLE` 操作必须分配给同一台 A 机器人。这样可以避免一列拆机被
   A_1/A_2 切碎，导致某台 A 机器人沿列移动时跳过中间离心机。

3. 同一列拆机自下而上连续执行  
   `enforce_bottom_up_disassembly_within_column=True` 时，同一台 A 机器人在
   同一列内按主干道进入方向执行拆机：`y=23 → 19 → 15 → 11 → 7 → 3`。

4. 每台机器人按列块执行任务  
   `enforce_robot_column_blocks=True` 时，如果某台机器人承担某一列中的任务，
   它必须完成自己在该列承担的任务后，才能进入下一列。列与列之间的访问顺序
   仍由 CP-SAT 决定，不是手工写死。

5. v2.2 中列块按工序类型分组  
   `column_blocks_by_operation_type=True` 时，列块按工序类型分别约束。
   对 A 机器人来说，拆机列块和安装列块分开；对 B 机器人来说，检测列块独立。
   这样可以满足“拆机优先”，同时保留 B 机检测与 A 机拆机的流水线并行。

6. A 机拆机优先  
   `enforce_a_disassembly_priority=True` 时，CP-SAT 计划层要求所有
   `DISASSEMBLE` 完成后才允许 `INSTALL` 开始。仿真执行层也做同样保护：
   如果动态等待/重规划导致实际时间偏移，只要现场仍有 `PENDING_DISASSEMBLY`，
   A 机就不会启动安装。

文字说明：scenario2/v2.2 的任务分配和列块顺序仍由 CP-SAT 求解器决定。
代码限制的是“机器人不能在自己承担的列块之间来回穿插”，但没有手工指定
A_1/A_2/B_1 必须先做哪一列。三台机器人仍可以并行工作，test8 的验证结果中
最大同时工作机器人数为 3，碰撞和硬约束违规均为 0。

## 核心功能

- 2 × 4 footprint 姿态、转移及扫掠区域碰撞验证
- 固定地图、48 台机器与 144 个前序约束操作
- 静态 A* 距离预计算与 Space-Time A* 动态路径规划
- pose、swept、service 三类时空预约
- CP-SAT 任务分配与无 OR-Tools 时的 fallback 策略
- 1A1B 场景离散事件仿真、指标统计和轨迹/甘特图/动画输出
- 单元测试与端到端集成测试

## 环境安装

建议使用 Python 3.11 或更高版本：

```bash
python -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
```

## 1A1B 场景代码使用说明

1A1B 场景表示 1 台 A 类机器人和 1 台 B 类机器人协同完成 48 台离心机的
拆卸、检测和安装，共 144 个操作。A 类机器人负责 `DISASSEMBLE` 和
`INSTALL`，B 类机器人负责 `INSPECT`。

### 运行 1A1B 完整场景

建议使用项目虚拟环境运行：

```bash
source .venv/bin/activate
python scripts/run_scenario_1_full.py cp_sat_current_video
```

运行后结果会写入：

```text
outputs/scenario_1/cp_sat_current_video/
```

主要输出文件包括：

- `metrics.json`：完整指标，包括 makespan、路径长度、等待时间、碰撞数、
  约束违规数、求解器状态和 fallback 状态；
- `event_log.jsonl`：逐时间步事件日志；
- `gantt.png`：机器人作业甘特图；
- `trajectories.png`：机器人轨迹图；
- `trajectories.json`：动画和轨迹分析使用的原始轨迹数据。

### 生成 1A1B 动画

如果需要生成 MP4 动画，可在场景运行完成后执行：

```bash
python scripts/create_animation_fast.py cp_sat_current_video scenario_1 10 24
```

生成的视频为：

```text
outputs/scenario_1/cp_sat_current_video/animation_smooth.mp4
```

其中 `10` 表示 10 FPS，`24` 表示每个地图格子的渲染像素尺寸。

### 任务规划是否由求解器完成

1A1B 正式运行不会使用手工任务队列。任务规划由 OR-Tools CP-SAT 求解器在
`solver_mode="assignment_schedule"` 下完成，内容包括：

- 每个操作分配给哪台机器人；
- 每台机器人的完整操作顺序；
- 每个操作的计划开始时间和结束时间；
- 相邻操作之间的静态 A* 旅行时间；
- 拆卸 → 检测 → 安装的前置关系；
- makespan、旅行时间、换列次数和负载均衡等优化目标。

可以在 `metrics.json` 中确认正式结果来源：

```json
{
  "solver_backend": "ortools_cp_sat",
  "solver_mode": "assignment_schedule",
  "operation_sequence_source": "cp_sat",
  "fallback_used": false
}
```

如果 `operation_sequence_source` 是 `cp_sat` 且 `fallback_used` 是 `false`，
说明任务顺序来自 CP-SAT 求解器，而不是 row-major、column-major 或其他
手工排序代码。

## 2A1B 场景 test8 代码使用说明

`scenario_2/test8` 表示 2 台 A 类机器人和 1 台 B 类机器人协同完成同一批
48 台离心机的拆卸、检测和安装。该结果用于验证多机器人并行执行：A_1、
A_2 和 B_1 可以同时处于工作状态，且每台机器人按列块完成自己承担的任务。

### 运行 2A1B test8

```bash
source .venv/bin/activate
python scripts/run_scenario_2_full.py test8
```

运行后结果会写入：

```text
outputs/scenario_2/test8/
```

主要输出文件包括：

- `metrics.json`：2A1B 指标，包括 makespan、路径长度、等待时间、碰撞数、
  约束违规数、重规划次数和求解器状态；
- `event_log.jsonl`：逐时间步事件日志，可用于检查 A_1/A_2/B_1 是否并行工作；
- `gantt.png`：三台机器人作业甘特图；
- `trajectories.png`：三台机器人轨迹图；
- `trajectories.json`：动画和轨迹分析使用的原始轨迹数据。

### 生成 2A1B test8 动画

```bash
python scripts/create_animation_fast.py test8 scenario_2 10 24
```

生成的视频为：

```text
outputs/scenario_2/test8/animation_smooth.mp4
```

### scenario_2/test8 由哪些代码生成

`scenario_2/test8` 的主要调用链如下：

1. [scripts/run_scenario_2_full.py](scripts/run_scenario_2_full.py)  
   创建 2A1B 场景、调用正式求解器、运行仿真、保存指标和事件日志。

2. [src/solver/scheduler.py](src/solver/scheduler.py)  
   正式求解入口，调用 `solve_assignment_schedule(...)`。默认
   `allow_fallback=false`，求解失败不会静默切到手工队列。

3. [src/solver/cp_sat_model.py](src/solver/cp_sat_model.py)  
   OR-Tools CP-SAT 模型主体，负责决定操作分配、机器人完整操作顺序、
   计划开始/结束时间、静态旅行时间衔接、工序前置关系和优化目标。

4. [src/solver/schedule_extractor.py](src/solver/schedule_extractor.py)  
   只沿 CP-SAT 选中的 arc successor 链提取顺序，不按坐标、机器编号或
   列顺序重新排序。

5. [src/solver/travel_time.py](src/solver/travel_time.py)  
   使用 footprint-aware static A* 为 CP-SAT 提供操作之间的预计移动时间。

6. [src/simulation/engine.py](src/simulation/engine.py)  
   按 CP-SAT 给出的操作顺序执行仿真；Space-Time A* 负责逐时间步路径、
   2 × 4 本体避碰、预约表、动态等待、按需让行和局部重规划。仿真器不重新
   决定任务顺序。

7. [scripts/render_scenario_outputs.py](scripts/render_scenario_outputs.py)  
   根据事件日志生成甘特图、轨迹图和轨迹 JSON。

8. [scripts/create_animation_fast.py](scripts/create_animation_fast.py)  
   根据 `trajectories.json` 和 `event_log.jsonl` 生成逐时间步 MP4 动画。

### 任务分配和列顺序策略

2A1B 正式模式下，任务分配和执行顺序由 CP-SAT 求解器决定，不再由手工代码
写死。当前 v2.2 约束强调“列块执行 + 拆机优先 + 并行工作”：

- 同一列的 `DISASSEMBLE` 操作作为列级工作包，由同一台 A 机器人负责；
- 同一列拆机按从主干道进入更自然的自下而上顺序执行；
- 每台机器人必须完成自己在某一列承担的任务后，才能进入下一列；
- A 机在仍有未拆列时优先执行拆机任务，不提前进入安装阶段；
- A_1、A_2、B_1 的作业可在不同列、不同工序之间形成流水线并行；
- 路径层允许多机器人同时移动，由时空预约表和 safety guard 保证零碰撞。

也就是说，v2.2 会限制机器人不要在列块之间来回穿插；但列块分配、列块先后顺序、
检测/安装衔接和并行时机仍由 CP-SAT 与路径规划共同执行。

## 快速开始

运行完整场景：

```bash
python scripts/run_scenario_1_full.py
```

运行测试：

```bash
python -m pytest -q
```

生成地图：

```bash
python scripts/visualize_map.py
```

仿真结果默认写入 `outputs/`。该目录中的运行产物不会纳入版本控制。

## 项目结构

```text
configs/   固定地图配置
docs/      设计与问题修复记录
scripts/   场景运行和可视化脚本
src/       领域模型、地图、求解器、规划、仿真与评估模块
tests/     单元测试、集成测试与验证脚本
```

更完整的设计决策、测试结果和迭代记录见 [PROJECT_LOG.md](PROJECT_LOG.md)。
