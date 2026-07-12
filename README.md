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
