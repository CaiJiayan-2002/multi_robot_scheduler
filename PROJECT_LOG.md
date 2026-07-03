# 多机器人协同调度系统 - 项目日志

## 1. 项目基本信息

| 项目 | 内容 |
|------|------|
| 项目名称 | 多机器人协同调度系统 (Multi-Robot Coordinated Scheduling System) |
| 代码仓库路径 | `e:\Projects\1_multirobots\` |
| 新项目目录 | `multi_robot_scheduler/` |
| 旧项目目录 | `agv_scheduler/` (已废弃) |
| 当前版本 | v4.0 |
| 开始日期 | 2026-07-01 |
| 核心研究问题 | 固定地图上 A/B 两类异构机器人的任务分配、协同路径规划、冲突消解与执行仿真 |
| 团队成员 | Agent1 (代码编写), Agent2 (测试验证), Agent3 (分析日志) |

## 2. 架构演变记录

### 2.1 版本概述

- **V1.3** (旧版): 基于随机地图的 AGV 协同调度研究原型，3周计划
- **V4.0** (新版): 固定地图 + 求解器驱动的任务分配 + footprint-aware 路径规划，6周计划

### 2.2 V1.3 → V4.0 关键决策变化

| 决策点 | V1.3 | V4.0 | 变更原因 |
|--------|------|------|----------|
| 地图 | 随机生成，可变尺寸 | 固定 25x29，不随机生成 | 简化前期复杂度；避免地图生成失败阻塞项目；聚焦于调度与路径规划算法验证；固定地图使实验结果可复现对比 |
| 机器人模型 | 点状 1x1，无物理尺寸 | 2x4 footprint，不可旋转，有工作区 | 点机器人无法验证真实碰撞约束；2x4 本体带来实际工业场景的扫掠碰撞问题；工作区模型区分行驶与作业状态 |
| 机器人运动 | 自由四方向移动 | 内部区域禁止水平移动，仅主干道可换列 | 真实工厂环境中的运动约束；防止机器人在狭窄内部通道横向移动导致死锁；规则由统一的 transition_validator 执行 |
| 调度算法 | 基础贪心 / 负载感知贪心 / 遗传算法 | OR-Tools CP-SAT 约束求解器 | CP-SAT 原生适合整数时间 + 布尔分配 + 逻辑约束的排程问题；无需手工设计启发式；可设置求解时间上限并返回可行解；比遗传算法更稳定确定 |
| 路径规划 | A* / 时空 A* / D* Lite 三者并重 | footprint-aware A* (预计算) + Space-Time A* (主规划) + 预约表；D* Lite 降为可选 | 静态 A* 仅用于预计算距离矩阵和启发函数，不可输出最终轨迹；Space-Time A* 直接将时间维度纳入搜索，显式避免顶点和扫掠冲突；D* Lite 在本项目中不做默认冲突解决器 |
| 离心机 | 1格，每处 20台，每托盘 5台 | 2格横向，共 48台固定位置 (6行 x 8列) | 固定布局对应固定地图；2格横向更贴近实际设备尺寸；48台提供足够的调度规模 |
| 离心机操作 | 拆(Batch) + 送料站 + 取料站 + 装(Batch) 的批次模型 | 每机 3 操作: DISASSEMBLE(6t) -> INSPECT(10t) -> INSTALL(6t)，共 144 操作 | 操作级粒度更精细；有明确的时序约束和对齐/作业时间；不再简化忽略作业时长 |
| 批次/托盘 | 每托盘 5台，批次作为调度单元 | 不再使用批次/托盘模型，每机独立操作调度 | 操作级粒度与控制级对齐；避免批次内耦合带来的约束复杂性 |
| 任务前后约束 | A 批次完成并送站后 B 批次才可执行 | 每台机器内: DISASSEMBLE -> INSPECT -> INSTALL 严格时序 | 机器级前序约束更精确；CP-SAT 用线性不等式直接表达 |
| 开发周期 | 3 周 | 6 周 | V4.0 增加了 CP-SAT 建模、footprint-aware 路径规划、预约表、强化学习实验模块，复杂度显著增加 |
| 强化学习 | 不在 V1.3 范围内 | 第5周尝试 RL 选择冲突处理策略（不生成路径） | 作为学术探索；RL 只选优先级策略，路径仍由 Space-Time A* 生成，保证安全性 |
| 风险暴露指标 | 10% 相邻风险累积评分 | 删除未定义指标，待明确后再引入 | v4.0 要求"所有指标给出公式和统计口径"；未定义的"风险暴露"不应输出无物理意义的数字 |

### 2.3 Section 2.2 的 7 个关键修改跟进状态

| # | 修改内容 | 在代码中的体现 | 状态 |
|---|----------|---------------|------|
| 1 | 不用"静态 A* + 反复补等待"作为主流程 | 静态 A* 定位为预计算工具；Space-Time A* 为路径规划主算法 | 设计中已明确；实现待 Task 06/11 |
| 2 | D* Lite 不作为默认冲突解决器 | D* Lite 降为可选模块 (src/planning/dstar_lite.py)，不作为 MVP 前置条件 | 设计中已明确；待第4周选做 |
| 3 | 求解器不无条件重分配 | 仅机器人故障/任务变化时重求解；延迟只调整预约 | 设计中已明确；实现待 Task 08/13/15 |
| 4 | 定义 E1-E5 局部变化类型 | E1(延迟)/E2(临时障碍) 必做；E3-E5 扩展 | 设计中已明确；实现待 Task 15 |
| 5 | 搜索状态直接判断 footprint 和扫掠碰撞 | validation.py 中 FootprintValidator 已实现姿态/转移/扫掠检查 | **已实现** (src/domain/validation.py) |
| 6 | 场景不是只用机器人数量定义 | 3 个场景各有实验目的 (验证框架/分配与负载/拥堵与重规划) | 设计中已明确；场景配置待创建 |
| 7 | 强化学习范围收窄 | 只选优先级/冲突策略；不生成逐格路径 | 设计中已明确；待第5周 |

## 3. 开发记录

### 2026-07-01: 项目重构 Day 1 -- 第一周基础框架完成

**修改内容**:
- 废弃 `agv_scheduler/` 旧代码（V1.3 随机地图方案）
- 新建 `multi_robot_scheduler/` 项目（V4.0 固定地图方案）
- 建立新领域模型: `Cell`, `Footprint` (2x4), `Operation`, `Machine`, `RobotSpec`, `TimedPose` 等
- 创建固定地图配置 `configs/map_fixed.yaml`（25x29, 48台离心机, 6行x8列）
- 创建枚举定义 `src/domain/enums.py`（TerrainCode, RobotType, MachineState, OperationType, Action, RobotStatus, ResultStatus）
- 创建数据模型 `src/domain/models.py`（Cell, Footprint, Operation, Machine, RobotSpec, TimedPose, RobotSchedule, ScheduleResult, SchedulingProblem）
- 创建 footprint 验证器 `src/domain/validation.py`（FootprintValidator: is_valid_pose, is_valid_transition, swept_cells, classify_action）
- 建立目录结构: domain/, map/, planning/, solver/, simulation/, evaluation/, rl/, ui/, tests/

**Agent1 交付物汇总**:

| 文件 | 行数 | 功能 | 状态 |
|------|------|------|------|
| `src/domain/enums.py` | 55 | 7个枚举类（TerrainCode, RobotType, MachineState, OperationType, Action, RobotStatus, ResultStatus） | ✅ |
| `src/domain/models.py` | 113 | 8个数据类（Cell, Footprint, Operation, Machine, RobotSpec, TimedPose, RobotSchedule, ScheduleResult, SchedulingProblem） | ✅ |
| `src/domain/validation.py` | 165 | FootprintValidator: is_valid_pose, is_valid_transition, swept_cells, classify_action | ✅ |
| `src/map/fixed_map.py` | 176 | FixedMap: 25x29 terrain, 48离心机, 144操作（D->I->R链） | ✅ |
| `src/map/service_poses.py` | 75 | ServicePoseCalculator: compute_service_anchor, compute_all_service_anchors | ✅ |
| `src/map/pose_graph.py` | 126 | PoseGraph: 有效姿态枚举(440节点), 邻接表构建(1410边) | ✅ |
| `src/planning/static_astar.py` | 200 | StaticAStar: plan, plan_multi_goal, precompute_travel_matrix, 路径缓存 | ✅ |
| `tests/unit/test_fixed_map.py` | 221 | 21项地图测试 | ✅ |
| `tests/unit/test_footprint.py` | 281 | 33项footprint/碰撞测试 | ✅ |
| `tests/run_verification.py` | 757 | 7段综合验证脚本 | ✅ |

**Agent2 测试结果**:

所有测试 **54/54 通过**，7段综合验证全部 PASS。

关键数据：
- 地图: 25x29, 92障碍格(12.7%), 483内部道路(66.6%), 150主干道(20.7%)
- 48台离心机/144操作: D(6t)->I(10t)->R(6t) 约束正确
- 有效锚点姿态: 440 (60.7%), 边: 1410, 平均3.2邻居/节点
- 48x48旅行时间矩阵: 全可达, min=4, max=63, mean=28.7步, 计算0.44s
- footprint验证: 边界/障碍/水平移动限制/扫掠 全部正确

**第一周验收标准达成情况**:

```text
[x] 地图可生成并可视化             → FixedMap.build() + text可视化
[x] 48 台机器和144 个操作生成正确   → 全部验证通过
[x] 机器人可在静态地图上规划到任一服务点 → A* 48x48全可达
[x] 所有核心碰撞测试通过             → 33项footprint测试全部通过
[ ] CP-SAT 可对缩小案例完成分配      → 待 Task 08 (W2 开始)
```

**Task 进度更新**:

| Task | 内容 | 状态 | 备注 |
|------|------|------|------|
| 01 | 建立配置和领域模型 | ✅ 完成 | enums.py, models.py, map_fixed.yaml |
| 02 | 生成固定地图与 48 台机器 | ✅ 完成 | fixed_map.py |
| 03 | 实现 footprint 和姿态验证 | ✅ 完成 | validation.py |
| 04 | 实现转移与扫掠区域 | ✅ 完成 | validation.py 中 is_valid_transition, swept_cells |
| 05 | 完成碰撞单元测试 | ✅ 完成 | test_footprint.py (33项), test_fixed_map.py (21项) |
| 06 | 实现静态 A* | ✅ 完成 | static_astar.py |
| 07 | 预计算旅行时间矩阵 | ✅ 完成 | precompute_travel_matrix, 0.44s, 2256条目全可达 |

**运行结果**: Day 1 顺利完成，第一周 Task 01-07 全部完成（含测试），整体代码质量良好。

**下一步计划**: 
1. 进入 Task 08: 实现 CP-SAT assignment_only 模式（最小可运行模型）
2. 然后按 Task 09-12 顺序推进场景 1 的完整闭环

### 2026-07-01: Week 2 开发 Day 2

**修改内容**:
- 地图可视化脚本 `scripts/visualize_map.py`
- 输出数字矩阵 + PNG图片
- 地图结构确认: 25x29, 4障碍柱, 6行离心机(48台), 6行主干道

**Agent1 W2 交付物**:

| 文件 | 行数 | 功能 | 状态 |
|------|------|------|------|
| `src/simulation/state_machine.py` | 245 | 离心机状态机: 7→8→9→10, 前序检查, 机器锁定/解锁, 状态查询/汇总 | ✅ |
| `src/planning/reservation_table.py` | 338 | 时空预约表: pose/swept/service 三类预约, 区间冲突检测, release/clear | ✅ |
| `src/planning/space_time_astar.py` | 467 | 时空A*: (x,y,t)搜索, 本体+扫掠冲突检查, 服务驻留, 路径缓存 | ✅ |
| `src/solver/cp_sat_model.py` | 302 | CP-SAT求解器: assignment_only模式, 负载均衡目标, fallback降级 | ✅ |
| `src/solver/fallback.py` | 272 | 手动分配: 场景1(1A1B) + 多机器人(轮询负载均衡) | ✅ |
| `src/simulation/engine.py` | 651 | 离散事件仿真引擎: 状态机+预约表+时空A* 全链路整合 | ✅ |

**Agent2 测试结果: 149/149 全部通过**:
- `test_state_machine.py` (32): 枚举/转移/锁定/推进/完成/就绪/汇总
- `test_reservation_table.py` (28): pose/swept/service预约, 冲突检测, release, clear
- `test_space_time_astar.py` (20): 基本寻路, 预约避障, 服务驻留, 缓存, 一致性
- `test_scenario_1.py` (15): W1模块, fallback分配, 仿真引擎, 手动pipeline, 统计
- W1回归 test_footprint (33) + test_fixed_map (21): 54 passed

**场景1端到端数据**:
- 48台离心机全部完成 (COMPLETED=48)
- 144操作全部执行
- 碰撞事件数 = 0
- 全部机器状态 = 10

**W2 验收标准达成**:

```text
[x] 1A1B完成全部机器         -> 48/48 COMPLETED, all_completed()=True
[x] 最终碰撞为0              -> collision events = 0, get_conflicts_at() 验证
[x] 所有机器状态为10          -> MachineState.COMPLETED confirmed
[x] 甘特图数据正确            -> event_log 包含 {t, type, message} 结构化时间序列
[x] 可保存结构化运行结果       -> ScheduleResult + event_log 均为可序列化结构
```

**关键设计亮点**:

1. **引擎架构**: step() 9步循环 (规划->收集->执行->进度->时间), 职责分离清晰, WAITING_PRECEDENCE/WORKING 状态完整
2. **状态机**: 锁定幂等(同机器人重复lock不报错), 解锁安全(unlock未锁定机器不抛异常), 三查合一(状态+锁+类型)
3. **预约表**: pose/swept/service 三类型覆盖本体碰撞/转移扫掠/作业互斥, 区间左闭右开语义明确
4. **时空A***: 逐节点检查预约表+exclude_robot排除自身, plan_with_service将驻留编码为WAIT姿态, 路径缓存减少重规划
5. **Fallback**: 场景1(1A1B)和多机器人(轮询)两种策略, 排序使用(row,x)自然序, CP-SAT导入失败时优雅降级
6. **测试设计**: 场景1 15项测试含 3 层级: W1模块验证, 无引擎手动pipeline(碰撞自检), 全引擎端到端(含碰撞=0检查)

**已知限制**:
- Fallback 分配所有 D 在 R 之前 (A 不会穿插 D 和 R, 等待时间较长)
- CP-SAT 需 ortools 安装 (当前 W2 仅实现 assignment_only 模式)
- 默认最大仿真步数 20000 (场景1实际完成约 3500-5000 步)
- WAITING_CONFLICT 时释放全部未来预约后重新规划 (非增量)

**下一步计划**:
1. W3 Task 13: 升级 CP-SAT 到 assignment_schedule 模式 (含顺序/时间决策)
2. W3 Task 14: 场景2 (2A1B), 验证多机器人负载均衡下的冲突消除
3. 后续优化: 路径缓存预置, 事件日志持久化, 甘特图生成

## 4. 风险与决策记录

### 已记录的设计决策

1. **坐标系选择**: 原点左上角，(x=col, y=row)，1-indexed。与 V4.0 文档 Section 3.1 一致。
2. **三层地图模型**: STATIC_LAYER / FACILITY_TASK_ONLY_LAYER / ROBOT_ONLY_LAYER，不再混写在单一矩阵中。
3. **求解器定位**: CP-SAT 负责高层任务分配与粗粒度调度，路径规划器负责生成真实无碰撞时空轨迹。
4. **静态 A* 定位**: 仅用于预计算距离矩阵和启发函数，不作为可执行轨迹的最终输出。
5. **运动约束**: 内部区域(y<24)禁止水平移动，统一由 transition_validator 检查。

### 待决策的配置项 (V4.0 Section 21)

以下业务问题已标记为配置项（默认值在 map_fixed.yaml 或待建配置中）:
1. A/B 机器人本体尺寸是否相同 → 默认 `true`（当前 footprint 相同）
2. 拆卸和安装是否必须由同一台 A 完成 → 默认 `false`（配置键 `require_same_a_robot_for_disassemble_and_install`）
3. 机器人完成全部任务后的停止位置 → 默认 `return_to_start`
4. 是否允许在内部通道等待 → 默认 `true`
5. 是否设置专门主干道等待区 → 待确认
6. 机器作业期间其他机器人可否经过相邻格 → 待确认
7. 作业对齐时间是否包含在驻留时间中 → 默认 `true`
8. 局部变化的测试类型和概率 → 待 Task 15 确定
9. 是否存在任务截止时间 → 当前版本不包含
10. 求解器是否必须输出最优解 → 默认 `false`（时限内可行解即可）

## 5. 指标与进度追踪

### 对照六周计划 (V4.0 Section 19, Task 01-21)

| 周 | Task | 内容 | 状态 | 备注 |
|----|------|------|------|------|
| W1 | 01 | 建立配置和领域模型 | ✅ 完成 | enums.py, models.py, map_fixed.yaml |
| W1 | 02 | 生成固定地图与 48 台机器 | ✅ 完成 | fixed_map.py, 21项测试通过 |
| W1 | 03 | 实现 footprint 和姿态验证 | ✅ 完成 | validation.py (FootprintValidator) |
| W1 | 04 | 实现转移与扫掠区域 | ✅ 完成 | validation.py, 内部禁止水平移动已实现 |
| W1 | 05 | 完成碰撞单元测试 | ✅ 完成 | test_footprint.py (33项) + test_fixed_map.py (21项) |
| W1 | 06 | 实现静态 A* | ✅ 完成 | static_astar.py, footprint-aware, 带多目标支持 |
| W1 | 07 | 预计算旅行时间矩阵 | ✅ 完成 | precompute_travel_matrix, 48x48 0.44s, 全可达 |
| W2 | 08 | 实现 CP-SAT assignment_only | ✅ 完成 | cp_sat_model.py (302行) + fallback.py (272行) |
| W2 | 09 | 实现机器状态机 | ✅ 完成 | state_machine.py (245行), 32项测试通过 |
| W2 | 10 | 实现预约表 | ✅ 完成 | reservation_table.py (338行), 28项测试通过 |
| W2 | 11 | 实现 Space-Time A* | ✅ 完成 | space_time_astar.py (467行), 20项测试通过 |
| W2 | 12 | 打通场景 1 (1A1B) | ✅ 完成 | engine.py (651行), 15项集成测试通过, 碰撞=0 |
| W3 | 13 | 实现 CP-SAT assignment_schedule | ⏳ 待完成 | |
| W3 | 14 | 实现场景 2 与动态优先级 | ⏳ 待完成 | |
| W4 | 15 | 实现事件系统和局部重规划 | ⏳ 待完成 | |
| W4 | 16 | 打通场景 3 (4A2B) | ⏳ 待完成 | |
| W5 | 17 | 实现批量实验和指标 | ⏳ 待完成 | |
| W5 | 18 | 封装 RL 环境 | ⏳ 待完成 | |
| W5 | 19 | 完成 RL 对比 | ⏳ 待完成 | |
| W6 | 20 | 开发 Streamlit UI | ⏳ 待完成 | |
| W6 | 21 | README、演示和最终回归测试 | ⏳ 待完成 | |

**完成进度**: 12 / 21  (57%)，W1+W2 全部完成，W3 待开始 (Task 13-14)

### 代码质量分析（Day 1 完成后）

#### 符合 v4.0 设计文档的方面

1. **坐标系和索引**: 全部使用 1-indexed Cell (x=col, y=row)，terrain 内部使用 0-indexed numpy array，转换一致。
2. **三层地图分离**: FixedMap 只生成 terrain (静态层) + machines (设施层)，不包括机器人动态层。
3. **Footprint 模型**: 2x4 偏移列表，work_zone 为顶部两格，与 v4.0 Section 4.1 完全一致。
4. **运动约束**: `is_valid_transition` 中内部区域 (y<24) 禁止水平移动，由统一 validator 执行，符合修改 5 的要求。
5. **扫掠冲突**: `swept_cells = from_footprint ∪ to_footprint`，与 v4.0 Section 10.5 定义一致。
6. **姿态验证**: 搜索前验证，而非搜索后检测（符合修改5："每个搜索节点必须代表机器人左上角锚点，扩展节点时同时检查"）。
7. **静态 A* 定位**: 仅用于预计算距离矩阵，不作为最终轨迹输出（符合修改1）。
8. **操作模型**: 每机3操作 DISASSEMBLE(6t)->INSPECT(10t)->INSTALL(6t)，共144操作，与 v4.0 Section 5 完全一致。
9. **机器 ID 格式**: `M_y{row}_x{x}`，非数组下标，符合 v4.0 Section 3.3 要求。

#### 接口设计评估

- **PoseGraph -> StaticAStar**: 通过依赖注入方式传递，解耦良好。
- **FixedMap.build()**: 返回值元组 `(terrain, machines, operations)`，调用方无需知道内部实现。
- **路径缓存**: StaticAStar 内部维护 `_path_cache` 和 `_travel_matrix`，避免重复计算。
- **多目标 A***: `plan_multi_goal` 支持搜索最近目标，为后续服务位置选择做准备。

#### v4.0 Section 2.2 七项修改逐项检查

| # | 修改内容 | 实现文件 | 检查结果 |
|---|----------|---------|----------|
| 1 | 不用"静态 A* 后再反复补等待" | static_astar.py | ✅ 静态 A* 仅做预计算，返回空间路径和时间矩阵。文档中有明确注释说明其用途 |
| 2 | D* Lite 不作为默认冲突解决器 | planning/ 目录 | ✅ D* Lite 文件未创建，目录保留为可选扩展 |
| 3 | 求解器不无条件重分配 | src/solver/fallback.py + cp_sat_model.py | ✅ fallback_used 标记已实现；求解器仅返回分配方案，不强迫接受；冲突保持原分配等待而非重求解 |
| 4 | 定义 E1-E5 局部变化类型 | 待 Task 15 | ⏳ 设计中已明确，代码将在仿真模块中实现 |
| 5 | 搜索状态直接判断 footprint 和扫掠碰撞 | validation.py | ✅ is_valid_pose + is_valid_transition + swept_cells 均在 PoseGraph 构建时调用，StaticAStar 基于 PoseGraph 搜索 |
| 6 | 场景不是只用机器人数量定义 | 待场景配置 | ⏳ 配置文件目录预留，场景配置 yaml 待创建 |
| 7 | 强化学习范围收窄 | 待 W5 | ⏳ RL 目录已创建，实现待第5周 |

#### 潜在问题与建议

1. **Heuristic 设计**: `static_astar.py` 使用 Manhattan 距离作为启发函数。对于 2x4 footprint，实际最短路径可能因障碍列和水平移动限制而偏离 Manhattan估计。但当前地图中 440 有效姿态覆盖 60.7% 栅格，Manhattan 距离在开阔区域是可行的下界估计。建议后续验证启发函数是否始终为 admissible（不高于实际代价）。

2. **WAIT 边**: PoseGraph 中 WAIT 自环代价为 1（与移动动作相同）。这符合文档中"所有动作耗时 1"的规定，但在静态 A* 场景中 WAIT 是可选的（路径不需要考虑等待）。建议在 Space-Time A* 中再重新评估 WAIT 的代价策略。

3. **service_anchor 位置**: ServicePoseCalculator 返回离心机左侧格 Cell 作为 service_anchor。根据 v4.0 Section 4.1，机器人工作区覆盖最上方两个格。当锚点 = 离心机左侧格时，工作区正好覆盖离心机的 `(x,y)` 和 `(x+1,y)`。但机器人 2x4 本体向下延伸了 4 格，需要确认这些格都在可通行区域内（当前 440 有效姿态已验证了这一点）。

4. **旅行时间矩阵**: 当前 `precompute_travel_matrix` 逐对计算所有路径并缓存。对于 48x48 = 2304 对（含对角线），这是可行的。但当扩展到 robot start positions 到 service anchors 时，矩阵会增大。建议后续使用 precompute_travel_matrix 的 sources/targets 参数分离机器人起点和服务点。

### 下一步建议

#### 优先级 1: Task 08 -- CP-SAT assignment_only (W2 首日)

这是连接"地图生成"和"完整闭环"的关键桥梁。建议:
- 先实现 assignment_only 模式（只分配操作给机器人，不决定顺序）
- 使用静态旅行时间矩阵作为 CP-SAT 中旅行时间的输入
- 验证: 用 1A1B + 子集机器（如 6-12 台）先测试，确保 CP-SAT 能在时限内完成
- 保底: 如果 CP-SAT 超时，实现 assignment_only 的规则回退（负载贪心分配）

#### 优先级 2: Task 09-10 -- 机器状态机 + 预约表

状态机实现操作 D->I->R 的状态转换和机器加锁；预约表为 Space-Time A* 提供时空占用信息。这两个模块可以并行开发。

#### 优先级 3: Task 11-12 -- Space-Time A* + 打通场景 1

Space-Time A* 是路径规划的核心。建议:
- 基于已有的 PoseGraph 扩展邻居生成，增加时间维度
- 使用静态 A* 的启发函数
- 先实现固定优先级（A_1 > B_1），再添加计划时间优先级
- 场景 1 (1A1B) 验证: 全部 48 台机器状态到 10，无碰撞

#### 对用户的 3 条最值得关注的建议

1. **确认业务规则 (v4.0 Section 21)**: 在进入 CP-SAT 实现前，建议确认"拆卸和安装是否必须由同一台 A 完成"（配置项 `require_same_a_robot_for_disassemble_and_install`）。这直接影响 CP-SAT 模型的约束数量和解空间大小。

2. **CP-SAT 求解时间**: 完整的 144 操作 + 顺序模型 + 旅行时间的 CP-SAT 可能在 30s 时限内无法稳定得到可行解。建议严格遵循文档的"两级实现策略"，优先保证 assignment_only 模式可用，再逐步升级到 assignment_schedule。

3. **测试策略**: 当前 54 项测试覆盖了静态组件（地图、footprint、A*），但尚未测试动态组件（CP-SAT、Space-Time A*、仿真器）。建议在 Task 08 开始前准备好场景 1 的缩小版集成测试（如仅 6 台机器的 mini-scenario），确保每个新模块都能快速验证。
