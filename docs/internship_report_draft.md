# 基于 CP-SAT 调度与 Space-Time A* 路径规划的多机器人离心机作业系统

## 摘要

本项目实现了一个面向离心机拆卸、检测与安装任务的多机器人协同调度系统。系统将问题拆分为两个层次：高层由 OR-Tools CP-SAT 求解器完成任务分配、任务顺序和计划时间求解；底层由 Space-Time A* 在离散栅格地图上生成满足机器人 2×4 本体约束的无碰撞运动路径。当前分析最完整的是 scenario_2，即 2 台 A 类机器人和 1 台 B 类机器人完成 48 台离心机的拆卸、检测和安装任务。该场景中，目前一共进行了20次实验，其中test18 是求解成本较低、执行稳定的基线版本，makespan 为 1027；test20 是目前实验结果最优的闭环版本，makespan 为 1014，碰撞次数为 0，硬约束违规为 0，重规划次数为 2。

系统的关键演进是从“手工指定任务顺序”转向“CP-SAT 完整调度”。早期代码中，求解器主要负责任务分配，机器人执行顺序仍依赖按列、按行或人工指定的规则；当前版本中，正式模式下任务顺序来自 CP-SAT 输出的 `sequence_index` 和机器人操作链，路径规划器只负责执行给定顺序，不再重新决定任务队列。为改善实际仿真效果，项目进一步加入了列块连续作业、A 机拆机优先、B 机检测顺序约束、安装顺序约束、运行期 B 队列重排关闭、等待时间统计和动态避让风险近似成本实验。

从实验结果看，v2.4/test18 是当前稳定基线，它将 test12 的 1104 个时间步降低到 1027 个时间步，同时保持零碰撞与零违规。test19 尝试让 CP-SAT 显式建模服务区间级动态避让风险，在不固定完整流水线顺序的情况下得到 1062 个时间步，说明“动态避让成本进入求解器”是可行方向。随后 test20 进一步搭建了“路径规划反馈 → CP-SAT 重新求解”的闭环接口，在 120 秒 CP-SAT 预算下通过 3 轮反馈求解得到 1014 个时间步、0 碰撞、0 违规、2 次重规划，优于 test18。

> 建议插入图 1：系统总体架构图。  
> 放在本摘要之后或第一章开头。图中建议展示：地图与任务建模 → CP-SAT 高层调度 → ScheduleResult → Space-Time A* 路径规划 → 仿真执行 → 指标、甘特图、轨迹图和 MP4 输出。

## 1. 问题定义与项目范围

本项目研究的是固定场地内的多机器人协同作业问题。地图中共有 48 台离心机，每台离心机需要依次完成三个操作：

1. 拆卸，记为 `DISASSEMBLE`，只能由 A 类机器人执行；
2. 检测，记为 `INSPECT`，只能由 B 类机器人执行；
3. 安装，记为 `INSTALL`，只能由 A 类机器人执行。

同一台离心机的三个操作存在严格前置关系：

```text
end(DISASSEMBLE) <= start(INSPECT)
end(INSPECT) <= start(INSTALL)
```

机器人不是点机器人，而是占用 2×4 栅格的矩形本体。因此，系统不仅要避免机器人中心点重合，还必须避免本体重叠、静态障碍物重叠和移动过程中的扫掠碰撞。项目目标可以表述为：在满足工序约束、机器人类型约束、资源约束和路径安全约束的前提下，尽可能缩短全部离心机完成安装的总时间。

本文主要记录 scenario_1、scenario_2 和部分 scenario_3 的开发过程，其中 scenario_2 是当前最稳定、分析最完整的版本。报告中的关键实验数据主要来自：

- `outputs/scenario_2/test12`
- `outputs/scenario_2/test17`
- `outputs/scenario_2/test18`
- `outputs/scenario_2/test19`
- `outputs/scenario_2/test20`

## 2. 系统架构与代码模块

系统采用分层架构。CP-SAT 解决“谁做、先做什么、什么时候做”的高层调度问题；Space-Time A* 解决“机器人每个时间步具体怎么走”的路径问题。两个模块通过 `ScheduleResult` 和仿真引擎连接。

### 2.1 地图与任务建模

相关代码：

```text
src/map/fixed_map.py
src/map/pose_graph.py
src/map/service_poses.py
src/domain/models.py
src/domain/enums.py
```

这一部分负责定义固定地图、离心机坐标、机器人初始姿态、服务姿态和操作对象。地图采用离散栅格表示，包含离心机区域、内部通道、主干道和静态障碍物。`service_poses.py` 为每台离心机生成机器人可执行拆卸、检测或安装的服务姿态；`pose_graph.py` 则在这些姿态之间建立可通行关系。

> 建议插入图 2：固定地图与机器人 footprint 示意图。  
> 推荐使用 `outputs/scenario_2/test18/trajectories.png` 或从 `animation_smooth.mp4` 截图，标注主干道、内部通道、离心机列和 2×4 机器人本体。

### 2.2 高层任务规划

相关代码：

```text
src/solver/config.py
src/solver/scheduler.py
src/solver/cp_sat_model.py
src/solver/schedule_extractor.py
src/solver/travel_time.py
```

高层任务规划由 OR-Tools CP-SAT 完成。当前正式模式为：

```text
solver_backend = "ortools_cp_sat"
solver_mode = "assignment_schedule"
operation_sequence_source = "cp_sat"
```

在该模式下，CP-SAT 不只负责 assignment，还负责输出每台机器人的完整操作顺序和计划时间。`cp_sat_model.py` 建立变量、约束和目标函数；`travel_time.py` 预计算静态旅行时间；`schedule_extractor.py` 从求解结果中提取每个机器人的操作链和每个操作的计划开始、结束时间。

> 建议插入图 3：CP-SAT 模型结构图。  
> 图中建议包括 `operation`、`assigned[o,r]`、`start[o]`、`end[o]`、`optional_interval[o,r]`、`arc[r,i,j]` 和 `sequence_index`。

### 2.3 路径规划与仿真

相关代码：

```text
src/planning/static_astar.py
src/planning/space_time_astar.py
src/planning/reservation_table.py
src/simulation/engine.py
src/simulation/state_machine.py
```

路径规划层负责将 CP-SAT 输出的高层任务计划转换为逐时间步的机器人位置。`Space-Time A*` 的状态为：

```text
state = (pose, time)
```

与普通 A* 相比，它不仅考虑空间位置，还考虑同一位置在不同时间是否被其他机器人占用。`ReservationTable` 记录每个时间步的机器人 footprint 占用区域，用于避免动态碰撞。仿真引擎还会检查扫掠区域，避免两个机器人在运动过程中发生穿越式碰撞。

路径规划层的职责边界非常重要：它可以等待、重规划、避让，但不能改变 CP-SAT 已经决定的任务执行顺序。

> 建议插入图 4：Space-Time A* 与预约表示意图。  
> 可以画三个时间片 `t`、`t+1`、`t+2`，展示机器人 footprint 如何写入预约表。

### 2.4 场景运行与结果输出

相关代码：

```text
scripts/run_scenario_1_full.py
scripts/run_scenario_2_full.py
scripts/render_scenario_outputs.py
scripts/create_animation_fast.py
```

脚本负责构造场景、调用求解器、运行仿真并输出结果。典型输出包括：

- `metrics.json`：总体指标和求解器配置；
- `robot_time_accounting.json`：每台机器人工作、运动、避让和等待时间；
- `event_log.jsonl`：事件日志；
- `gantt.png`：任务甘特图；
- `trajectories.png`：轨迹图；
- `animation_smooth.mp4`：机器人运动动画。

> 建议插入图 5：test18 输出示例。  
> 推荐并排放置 `outputs/scenario_2/test18/gantt.png` 和 `outputs/scenario_2/test18/trajectories.png`。

## 3. 求解器选择与建模依据

本项目选择 OR-Tools CP-SAT 作为高层任务规划求解器，原因是该问题本质上是带资源约束、前置约束和路径成本的组合优化问题。每个操作既有机器人类型要求，又有开始时间、结束时间、执行顺序和相邻移动时间约束。简单贪心算法或人工排序规则很难同时处理这些因素。

CP-SAT 适合本项目的原因主要有三点。

第一，CP-SAT 可以自然表达 0-1 分配变量和整数时间变量。例如 `assigned[o,r]` 表示操作 `o` 是否由机器人 `r` 执行，`start[o]` 和 `end[o]` 表示操作的计划时间。

第二，CP-SAT 支持复杂逻辑约束和资源约束。例如机器人类型限制、同一机器人任务不重叠、拆卸检测安装前置关系、相邻任务移动时间约束、同列连续作业约束等，都可以通过布尔变量、条件约束和 `NoOverlap` 表达。

第三，CP-SAT 可以在满足硬约束的基础上优化多个目标。当前模型以 makespan 为首要优化目标，在此基础上进一步考虑静态移动时间、换列次数、负载均衡、初始等待和动态避让风险近似成本。

需要注意的是，CP-SAT 解决的是高层任务规划，不直接输出每个时间步的机器人坐标。它使用 footprint-aware static A* 预计算出的静态旅行时间作为任务之间的移动时间约束；真正的逐时间步路径仍由 Space-Time A* 负责。因此，本项目采用的是“调度优化 + 路径规划”的分层方法，而不是全时空联合优化。

## 4. CP-SAT 调度模型

当前 CP-SAT 模型的核心目标是：直接生成每台机器人的操作序列和每个操作的计划时间，而不是在求解后再使用手工排序补全任务队列。

### 4.1 决策变量

模型包含以下主要变量。

分配变量：

```text
assigned[o, r] ∈ {0, 1}
```

表示操作 `o` 是否由机器人 `r` 执行。

时间变量：

```text
start[o] ∈ [0, horizon]
end[o] = start[o] + duration[o]
```

当前操作持续时间为：

```text
DISASSEMBLE = 6
INSPECT = 10
INSTALL = 6
```

可选区间变量：

```text
optional_interval[o, r]
```

当 `assigned[o,r] = 1` 时，对应区间存在，并参与该机器人的 `NoOverlap` 约束。

顺序弧变量：

```text
arc[r, i, j] ∈ {0, 1}
```

表示机器人 `r` 执行完操作 `i` 后，下一个执行操作 `j`。模型为每台机器人设置虚拟起点和虚拟终点，使每个机器人的操作形成从 `START_r` 到 `END_r` 的合法链路。

> 建议插入图 6：单机器人 successor 链示意图。  
> 示例：`START_A1 → column_1_D_1 → column_1_D_2 → ... → column_2_D_1 → END_A1`。图的重点是说明任务顺序由 `arc` 变量决定。

### 4.2 硬约束

当前模型包含以下硬约束。

每个操作必须且只能分配给一台合法机器人：

```text
sum_r assigned[o,r] = 1
```

机器人类型约束：

```text
DISASSEMBLE, INSTALL → A 类机器人
INSPECT → B 类机器人
```

工序前置约束：

```text
end(DISASSEMBLE_j) <= start(INSPECT_j)
end(INSPECT_j) <= start(INSTALL_j)
```

同一机器人资源不重叠：

```text
AddNoOverlap(optional_interval[o,r])
```

相邻任务移动时间约束：

```text
arc[r,i,j] = 1 → start[j] >= end[i] + travel_time[r,i,j]
```

其中 `travel_time[r,i,j]` 来自 footprint-aware static A*，不是根据机器编号或坐标差直接估算的曼哈顿距离。

### 4.3 scenario_2/v2.4 的业务约束

scenario_2/test18 在基础 CP-SAT 调度上加入了更强的流水线约束。A 类机器人拆卸和安装按照从左到右的列波次推进：

```text
列波次 = [[2, 5], [8, 11], [14, 17], [20, 23]]
```

这里的数字是地图中的 x 坐标，对应肉眼看到的第 1/2、3/4、5/6、7/8 列。

B 类机器人的检测顺序为：

```text
[5, 2, 8, 11, 14, 17, 20, 23]
```

也就是先检测第 2 列，再检测第 1 列，然后继续向右推进。这样设计的原因是：A 机器人在前两列拆卸时，B 先进入第 2 列可以更早形成流水线，同时给 A 机器人在第 2 列附近掉头和避让留下空间。

此外，test18 还启用了以下约束或配置：

```text
enforce_robot_column_blocks = True
column_blocks_by_operation_type = True
enforce_a_disassembly_priority = True
enforce_b_inspection_follows_disassembly_completion = True
enforce_inspection_after_full_column_disassembly = True
enforce_contiguous_bottom_up_inspection_chain = True
enforce_alternating_disassembly_by_preferred_order = True
enforce_alternating_install_by_preferred_order = True
disable_runtime_b_inspection_reorder = True
allow_early_service_start = True
```

这些约束共同实现了三个目标：一是避免同一列内跳格作业，二是避免机器人频繁跨列来回，三是让仿真器严格执行 CP-SAT 已经规划好的 B 机检测顺序。

> 建议插入表 1：scenario_2/test18 关键约束配置表。  
> 表格建议包含“约束名称、取值、作用、对应代码文件”四列。

### 4.4 目标函数

当前模型采用分阶段优化思想。第一目标是最小化全部安装操作完成时间的最大值，即 makespan：

```text
makespan = max(end(INSTALL_j))
```

第二类目标在 makespan 不恶化或固定 makespan 的前提下优化静态移动时间、换列次数和负载均衡。后续版本还加入了初始等待惩罚和动态避让风险近似成本。

test19 中增加的动态避让风险项为：

```text
dynamic_avoidance_penalty * dynamic_avoidance_weight
```

其含义是：如果 A/B 操作位于同列或邻近列，并且服务时间区间过近，则认为这组操作在真实路径执行中更可能触发等待、让行或重规划。该项目前仍是近似成本，不等价于真实路径层碰撞检测。

## 5. Space-Time A* 路径规划与避障策略

CP-SAT 输出的是高层调度结果，Space-Time A* 输出的是逐时间步运动路径。二者的职责边界如下。

CP-SAT 负责：

- 任务分配；
- 操作顺序；
- 计划开始和结束时间；
- 静态旅行时间；
- 拆卸、检测、安装前置关系；
- 粗粒度等待和流水线节奏。

Space-Time A* 负责：

- 每个时间步的具体位置；
- 2×4 机器人本体约束；
- 静态障碍物避让；
- 多机器人时空预约；
- 本体碰撞和扫掠碰撞检测；
- 必要的局部等待；
- 动态障碍下的局部重规划。

当前路径规划器会根据预约表避免机器人在同一时间占用重叠 footprint。对于移动动作，系统还检查扫掠区域，防止两个机器人虽然起点和终点合法，但移动过程中发生交叉穿越。

局部避让策略主要由仿真引擎触发。当机器人即将发生冲突时，低优先级或非紧急机器人可以等待、让行或重规划到临时避让姿态。后续优化中，系统更倾向于让机器人在内部相邻通道进行短距离避让，而不是总是退回地图角落或主干道末端。

> 建议插入图 7：局部避让示意图。  
> 从 `outputs/scenario_2/test18/animation_smooth.mp4` 截取 A/B 接近同一列通道附近的画面，标注原路径、避让路径和等待位置。

## 6. 版本演进与技术决策

### 6.1 从手工队列到 CP-SAT 完整调度

项目早期版本中，求解器主要用于任务分配，任务顺序依赖人工规则，例如按列排序、按行排序或指定机器人负责固定列。这种方式能够快速得到可运行结果，但泛化能力差：一旦离心机布局或机器人数量变化，原先队列就需要重新手工调整。

后续版本将任务分配、任务顺序和计划时间统一放入 CP-SAT 模型中。正式模式下，仿真器读取的是 CP-SAT 输出的操作顺序，而不是手工排序结果。这一变化使系统从“脚本驱动任务队列”转向“求解器驱动任务规划”。

### 6.2 从单纯优化 makespan 到加入执行可读性约束

单纯最小化 makespan 时，求解器可能给出数学上可行但现场不自然的任务顺序。例如机器人可能路过待处理离心机却不处理，或者在同一列尚未完成时切换到新列。为解决这一问题，后续版本加入了列块连续作业、同列自下而上执行和 A 机拆机优先等约束。

这些约束不是为了替代求解器，而是把现场执行偏好显式写进 CP-SAT。也就是说，规则仍然由求解器统一处理，而不是在求解后用脚本重新排序。

### 6.3 v2.4/test18：快速稳定基线

v2.4/test18 的核心变化是将人类观察到的高效流水线节奏转化为 CP-SAT 约束：

- A_1 和 A_2 按列波次推进拆卸任务；
- B_1 先检测第 2 列，再检测第 1 列，然后向右推进；
- A_1 和 A_2 在检测完成后按相同列顺序交替执行安装；
- 仿真器关闭 B 机运行期队列重排，避免破坏 CP-SAT 计划。

该版本最终 makespan 为 1027，优于 test12 的 1104 和 test17 的 1094。由于 test18 的列顺序约束清晰、求解预算较低、执行过程稳定，它适合作为 scenario_2 的快速稳定基线。

### 6.4 test19：动态避让成本进入求解器的实验

用户提出希望减少固定流水线顺序，让 CP-SAT 自己理解动态避让成本。test19 因此引入服务区间级动态风险惩罚。实验结果为：

```text
makespan = 1062
collision = 0
constraint violation = 0
dynamic_avoidance_penalty = 1584
```

这说明动态避让风险项能够改善自动规划结果，但目前还没有超过 test18。主要原因是 CP-SAT 中的风险项只基于列距离和服务时间接近程度，尚未真正建模机器人在内部通道和主干道中的逐时间步位置。因此它可以作为未来方向，但暂时不应替代 test18。

### 6.5 test20：路径规划反馈闭环 MVP

test20 进一步搭建了路径规划向 CP-SAT 反馈的闭环接口。闭环流程为：

```text
CP-SAT 初次求解
→ Space-Time A* 仿真执行
→ 仿真器输出 PlanningConflict 结构化冲突
→ 将可转化冲突提取为 additional_precedence_constraints
→ CP-SAT 读取反馈约束并重新求解
→ 再次仿真并选择零碰撞、零违规且 makespan 更短的结果
```

当前结构化冲突信息包含机器人 ID、冲突机器人 ID、相关 operation id、冲突时间区间、最小建议延迟、冲突类型、来源事件和建议前置约束。test20 的最终运行中，闭环共执行 3 轮：第 0 轮 makespan 为 1074，产生 2 条反馈约束；第 1 轮 makespan 为 1125，继续产生 3 条反馈约束；第 2 轮在累计 5 条反馈约束后得到 makespan 1014，规划冲突数降为 0。说明该闭环不只是记录冲突，而是已经能够通过反馈约束改变 CP-SAT 的高层调度结果。

test20 的结果为：

```text
makespan = 1014
collision = 0
constraint violation = 0
replans = 2
planning_conflict_count = 0
closed_loop_iterations = 3
accepted_feedback_constraints = 5
```

这一结果说明，路径层反馈能够帮助 CP-SAT 避开一部分会在真实执行中造成重规划或通道冲突的高层计划。不过 test20 的代价是求解时间明显增加，因此它更适合作为“闭环优化实验版”；如果需要快速复现实验或调试代码，test18 仍可作为稳定基线。

> 建议插入图 8：版本演进时间线。  
> 推荐顺序：手工队列 → assignment-only → assignment_schedule → 列块连续作业 → v2.4/test18 流水线约束 → test19 动态避让风险近似成本 → test20 路径反馈闭环 MVP。

## 7. 实验结果与定量分析

### 7.1 版本对比

当前 scenario_2 的主要实验结果如下。

| 版本 | 主要策略 | makespan | replans | 碰撞 | 约束违规 |
|---|---|---:|---:|---:|---:|
| test12 | v2.3 等待/避让优化基线 | 1104 | 6 | 0 | 0 |
| test17 | test12 基础上的轻量避让优化 | 1094 | 13 | 0 | 0 |
| test18 | v2.4 固定流水线列顺序，快速稳定基线 | 1027 | 3 | 0 | 0 |
| test19 | CP-SAT 动态避让风险近似成本实验 | 1062 | 17 | 0 | 0 |
| test20 | 路径规划反馈闭环 MVP + 120 秒 CP-SAT 预算 | 1014 | 2 | 0 | 0 |

test20 的 makespan 最短，test18 则是求解成本更低的稳定基线。test19 虽然没有超过 test18，但相比 test12 仍缩短了 42 个时间步，说明动态避让成本建模具有继续研究价值。test20 在此基础上进一步把路径层结构化反馈接口接入 CP-SAT，并将 CP-SAT 预算提高到 120 秒，最终通过 3 轮闭环反馈将 makespan 降到 1014。

> 建议插入图 9：makespan 对比柱状图。  
> 横轴为 test12、test17、test18、test19、test20，纵轴为 makespan。该图用于支撑“test20 当前 makespan 最短，test18 是快速稳定基线，test19 证明动态避让成本方向有潜力”的结论。

### 7.2 test20 机器人时间构成

test20 中每台机器人时间统计如下。

| 机器人 | 服务时间 | 正常运动时间 | 避让时间 | 总运动时间 | 等待时间 | 总时间 |
|---|---:|---:|---:|---:|---:|---:|
| A_1 | 324 | 474 | 0 | 474 | 216 | 1014 |
| A_2 | 252 | 354 | 0 | 354 | 408 | 1014 |
| B_1 | 480 | 438 | 0 | 438 | 96 | 1014 |

其中：

```text
总时间 = 工作时间 + 等待时间
工作时间 = 服务时间 + 运动时间
运动时间 = 正常运动时间 + 避让时间
```

三个机器人的总时间都等于全局 makespan，这是因为等待时间中包含机器人完成最后一个任务后等待全部系统结束的时间。这个统计方式更适合比较机器人利用率，而不是只看机器人自身完成时间。

从结果看，test20 的避让时间为 0，说明最终轨迹没有实际发生避让路径移动；B_1 的等待时间仅为 96，明显低于 test18 的 123。A_1 与 A_2 的服务时间不完全均衡，A_1 承担了更多 A 类操作，但由于 B_1 检测流水线更顺、动态冲突更少，系统总 makespan 仍低于 test18。

> 建议插入图 10：机器人时间构成堆叠柱状图。  
> 数据来自 `outputs/scenario_2/test20/robot_time_accounting.json`，横轴为 A_1、A_2、B_1，堆叠显示服务时间、正常运动时间、避让时间和等待时间。若希望展示稳定基线，也可以补充 `test18` 的同类图作为对照。

### 7.3 全时空联合优化原型对比

为了评估是否可以直接把机器人每个时间步的位置也纳入 CP-SAT，本项目增加了 `scripts/compare_joint_optimization.py` 作为实验脚本。该脚本没有直接替换正式规划器，而是在同一张固定地图和同一 2×4 footprint 下，构造一个缩小版 time-expanded joint path prototype：3 台机器人、固定起点、固定目标点、46 个时间步，由 CP-SAT 同时决定每个机器人每个时间步处于哪个姿态，并加入 footprint 和 swept collision 约束。

该缩小版原型结果如下：

| 模型 | 范围 | horizon | 布尔变量 | 约束数 | 状态 | 求解时间 |
|---|---|---:|---:|---:|---|---:|
| 分层 test20 | 48 台离心机完整 D/I/R 任务 | 1014 | 不直接建模逐时刻位置 | 不直接建模逐时刻位置 | 可执行 | 完成 |
| 全时空原型 | 3 机器人固定目标路径子问题 | 46 | 144,996 | 90,219 | OPTIMAL | 5.79 秒 |

进一步按完整 scenario_2/test20 的 makespan 估算，如果将同样的 time-expanded 表达扩展到完整地图和 1014 左右的 horizon，仅位置变量和转移变量就需要数百万级布尔变量。按 test20 的 1014 horizon 估算，下界为：

```text
有效姿态数 = 488
有效转移边数 = 1646
机器人数量 = 3
位置布尔变量 ≈ 1,485,960
转移布尔变量 ≈ 5,007,132
位置 + 转移布尔变量 ≈ 6,493,092
```

这还没有包括 144 个 D/I/R 操作的任务分配变量、服务状态变量、工序前置约束、操作持续时间约束和目标函数。因此，全时空联合优化在理论上可以表达，但目前不适合作为完整 48 台离心机场景的主方案。更合理的路线是保留分层架构，并继续完善“路径冲突反馈闭环”，让 Space-Time A* 发现的真实冲突逐步反馈给 CP-SAT。

为了更直观地比较计算成本，可以用缩小版全时空原型的变量规模和求解时间做一个数量级估算。该估算不是严格预测，因为 CP-SAT 的求解时间通常不是线性增长，而是会随约束耦合程度出现明显的非线性增长；但它可以说明全时空方案的规模压力。

| 对比项 | 缩小版全时空原型 | 完整 scenario_2 路径-only 外推 | 说明 |
|---|---:|---:|---|
| horizon | 46 | 1014 | 完整场景按 test20 makespan 估算 |
| 布尔变量 | 144,996 | 6,493,092 | 完整外推仅含位置和转移变量 |
| 变量规模倍数 | 1× | 约 44.8× | `6,493,092 / 144,996` |
| 实测/估算求解时间（线性） | 5.80 秒 | 约 260 秒，约 4.3 分钟 | 这是过于乐观的下界估算 |
| 估算求解时间（二次） | 5.80 秒 | 约 11,627 秒，约 3.2 小时 | 更能体现非线性增长风险 |
| 是否包含任务分配与 D/I/R 工序 | 否 | 否 | 真正完整联合优化还会更大 |
| 当前分层闭环 test20 | — | 约 486 秒，约 8.1 分钟 | 已完成完整 48 台离心机 D/I/R 任务 |

因此，如果只看“路径-only 的线性外推”，全时空联合优化似乎还有机会；但这个估计没有包含 144 个操作的任务分配、工序持续时间、工序前置关系、机器人资源约束和服务占用约束。一旦把完整任务规划也放进同一个 time-expanded CP-SAT 模型，变量和约束数量会继续增加，并且约束之间的耦合更强，求解时间很可能从分钟级上升到小时级甚至更高。相比之下，当前分层闭环 test20 已经在约 8.1 分钟内完成完整任务并得到零碰撞结果，因此更适合作为近期工程路线。

这里需要特别区分三类时间的含义。

第一，`求解时间线性估算` 是一个偏乐观的理论外推。它假设全时空联合优化的求解时间只和布尔变量数量成正比。缩小版全时空原型有 144,996 个布尔变量，实测求解时间约 5.80 秒；完整 scenario_2 路径-only 外推约有 6,493,092 个布尔变量，变量规模约为原型的 44.8 倍。因此线性估算为：

```text
5.80 秒 × 44.8 ≈ 260 秒 ≈ 4.3 分钟
```

这个估算通常偏乐观，因为 CP-SAT 的搜索复杂度并不会简单随变量数量线性增长。随着约束耦合变强，求解难度可能增长得更快。

第二，`求解时间二次估算` 是一个更保守的数量级估算。它假设求解时间大致按变量规模的平方增长，即：

```text
5.80 秒 × 44.8² ≈ 11,627 秒 ≈ 3.2 小时
```

这个估算也不是精确预测，而是用于说明全时空联合优化放大后的非线性风险。真实完整模型还要加入任务分配、工序前置、服务持续时间和资源约束，因此实际时间可能继续高于该估算。

第三，`当前分层闭环` 是真实运行测得的 wall time，而不是估算。它指当前 `scenario_2/test20` 使用“CP-SAT 高层调度 + Space-Time A* 路径规划 + 路径冲突反馈闭环”完整跑完 48 台离心机、144 个操作所花费的时间，约 486 秒，即 8.1 分钟。这个时间包括 CP-SAT 求解、多轮反馈、Space-Time A* 仿真、指标统计和部分结果输出。因此它和前两个“全时空联合优化估算时间”不是同一类指标，但可以作为当前可运行工程方案的实际成本基准。

> 建议插入表 3：分层闭环与全时空原型成本对比。  
> 数据来自 `outputs/analysis/joint_vs_closed_loop/joint_vs_closed_loop_report.json`。

## 8. 局限性与鲁棒性分析

当前系统已经能够在 2A1B 场景中稳定生成零碰撞、零违规的结果，但仍有几个限制。

第一，CP-SAT 当前使用的移动时间主要来自 footprint-aware static A* 预计算结果，无法完整预测 Space-Time A* 中的动态避让。因此，高层计划中的最优解不一定等于真实仿真中的最优执行结果。test20 已经搭建了结构化冲突反馈接口，但目前还只是 MVP，尚未形成成熟的多轮冲突学习策略。

第二，test18 对当前地图使用了较强的列顺序约束。这提高了当前场景的效率和可读性，但降低了地图变化后的泛化能力。如果离心机布局、通道宽度或机器人数量变化，固定列顺序可能不再最优。

第三，求解器与路径规划仍是分层架构。路径规划发现冲突后，目前主要通过等待、让行和局部重规划处理。更理想的闭环方式是：路径规划输出结构化冲突信息，再反馈给 CP-SAT 增加约束并重新求解。

第四，机器人数量扩展后，问题规模会迅速增加。4A2B 场景中，任务分配和路径避让都更复杂，容易出现机器人局部通道互相阻塞或任务顺序不符合现场直觉的问题。后续需要继续优化 CP-SAT 模型规模、列分组策略和路径层避让行为。

## 9. 后续改进方向

后续最值得推进的方向是让 CP-SAT 更好地感知动态避让成本，而不是依赖固定流水线顺序。

一种可行方案是服务区间级风险建模，即 test19 当前使用的方法。它通过 A/B 操作的列距离和时间接近程度估计潜在避让风险，优点是模型规模较小，容易接入现有 CP-SAT；缺点是只能近似真实动态冲突。

第二种方案是冲突反馈闭环。系统可以先运行 CP-SAT 和 Space-Time A*，如果路径层发现冲突或大量等待，就输出冲突机器人、冲突操作、冲突时间区间、建议延迟或建议前置关系，再将这些信息反馈给 CP-SAT 重新求解。这种方法比静态风险惩罚更准确，也比全时空联合建模更可控。

第三种方案是全时空联合优化，即把每台机器人每个时间步的位置也作为求解变量。该方法理论上最完整，但初步实验显示，即使只是 3 台机器人固定目标路径子问题，也会产生 144,996 个布尔变量和 90,219 条约束；完整 scenario_2 的位置与转移变量下界已超过 650 万。因此它不适合当前 48 台离心机、144 个操作的场景作为首选实现。

综合考虑工程成本和效果，建议下一阶段继续推进“路径冲突反馈闭环”，并保留 test18 作为快速稳定基线、test20 作为当前闭环实验基线。

## 10. 结论

本项目完成了一个可运行、可验证的多机器人离心机作业调度系统。系统使用 OR-Tools CP-SAT 完成高层任务规划，使用 Space-Time A* 完成底层路径规划，并通过仿真、动画、甘特图和时间统计验证执行效果。

项目的核心成果有三点。第一，正式模式下任务顺序已经从手工代码转移到 CP-SAT 调度模型，求解器直接决定操作分配、执行顺序和计划时间。第二，路径规划层能够基于预约表和扫掠检测生成零碰撞执行路径。第三，scenario_2/test20 在当前地图中实现了 1014 个时间步的总完工时间，优于 test12、test17、test18 和 test19；其中 test18 仍是求解成本较低的快速稳定基线。

项目的主要不足是 CP-SAT 尚未完整建模真实动态避让成本。test19 的结果表明，把动态避让风险纳入求解器是有潜力的；test20 进一步搭建了路径冲突反馈闭环接口，但当前仍是 MVP，还需要在更多冲突场景中验证“发现冲突—添加约束—重新求解”的多轮收益。

## 附录 A：建议插入的图片与数据

| 编号 | 插入位置 | 建议内容 | 推荐来源 | 作用 |
|---|---|---|---|---|
| 图 1 | 技术摘要后或第 2 章开头 | 系统总体架构图 | 手工绘制 | 说明模块调用链 |
| 图 2 | 2.1 | 地图、主干道、内部通道、离心机列、2×4 footprint | `outputs/scenario_2/test18/trajectories.png` 或动画截图 | 说明作业场景 |
| 图 3 | 2.2 或第 4 章 | CP-SAT 模型结构图 | 手工绘制 | 说明变量关系 |
| 图 4 | 2.3 或第 5 章 | Space-Time A* 时间片与预约表示意图 | 手工绘制 | 说明动态避障 |
| 图 5 | 2.4 | test18 甘特图和轨迹图 | `outputs/scenario_2/test18/gantt.png`、`outputs/scenario_2/test18/trajectories.png` | 展示输出结果 |
| 图 6 | 4.1 | 单机器人 successor 链 | `metrics.json` 或 `event_log.jsonl` | 说明顺序来自 CP-SAT |
| 表 1 | 4.3 | test18 关键约束配置 | `outputs/scenario_2/test18/metrics.json` | 说明当前版本约束 |
| 图 7 | 第 5 章 | 局部避让截图 | `outputs/scenario_2/test18/animation_smooth.mp4` | 说明路径层避让 |
| 图 8 | 第 6 章 | 版本演进时间线 | 手工绘制 | 说明开发过程 |
| 图 9 | 7.1 | makespan 对比柱状图 | test12/test17/test18/test19/test20 的 `metrics.json` | 说明优化效果 |
| 图 10 | 7.2 | 机器人时间构成堆叠柱状图 | `outputs/scenario_2/test20/robot_time_accounting.json` | 说明负载与等待 |
| 表 2 | 6.5 或 7.1 | test20 闭环反馈轮次 | `outputs/scenario_2/test20/closed_loop_feedback.json` | 说明闭环是否触发反馈约束 |
| 表 3 | 7.3 | 分层闭环与全时空原型成本对比 | `outputs/analysis/joint_vs_closed_loop/joint_vs_closed_loop_report.json` | 说明全时空联合优化可行性 |

## 附录 B：关键数据摘录

### B.1 版本对比

| 版本 | makespan | replans | 碰撞 | 约束违规 | 说明 |
|---|---:|---:|---:|---:|---|
| test12 | 1104 | 6 | 0 | 0 | v2.3 基线 |
| test17 | 1094 | 13 | 0 | 0 | 轻量避让优化 |
| test18 | 1027 | 3 | 0 | 0 | v2.4 快速稳定基线 |
| test19 | 1062 | 17 | 0 | 0 | 动态避让风险近似成本实验 |
| test20 | 1014 | 2 | 0 | 0 | 路径规划反馈闭环 MVP |

### B.2 test18 时间统计（快速稳定基线）

| 机器人 | 服务时间 | 正常运动时间 | 避让时间 | 总运动时间 | 等待时间 | 总时间 |
|---|---:|---:|---:|---:|---:|---:|
| A_1 | 288 | 416 | 0 | 416 | 323 | 1027 |
| A_2 | 288 | 422 | 0 | 422 | 317 | 1027 |
| B_1 | 480 | 423 | 1 | 424 | 123 | 1027 |

### B.3 test19 动态避让风险实验

```text
makespan = 1062
collision = 0
constraint violation = 0
replans = 17
dynamic_avoidance_penalty = 1584
dynamic_avoidance_time_buffer = 8
dynamic_avoidance_column_distance = 3
dynamic_avoidance_weight = 50000
```

### B.4 test20 闭环实验数据

```text
makespan = 1014
collision = 0
constraint violation = 0
replans = 2
planning_conflict_count = 0
closed_loop_iterations = 3
accepted_feedback_constraints = 5
```

test20 中每台机器人时间统计：

| 机器人 | 服务时间 | 正常运动时间 | 避让时间 | 总运动时间 | 等待时间 | 总时间 |
|---|---:|---:|---:|---:|---:|---:|
| A_1 | 324 | 474 | 0 | 474 | 216 | 1014 |
| A_2 | 252 | 354 | 0 | 354 | 408 | 1014 |
| B_1 | 480 | 438 | 0 | 438 | 96 | 1014 |

### B.5 全时空联合优化原型数据

```text
prototype_horizon = 46
prototype_bool_variables = 144996
prototype_constraints = 90219
prototype_status = OPTIMAL
prototype_solve_time_seconds = 5.79

full_scenario_pose_count = 488
full_scenario_edge_count = 1646
full_scenario_estimated_position_bool_variables = 1485960
full_scenario_estimated_transition_bool_variables = 5007132
full_scenario_estimated_position_plus_transition_bool_variables = 6493092
full_vs_prototype_variable_ratio = 44.8
linear_time_estimate_seconds = 260
linear_time_estimate_minutes = 4.3
quadratic_time_estimate_seconds = 11627
quadratic_time_estimate_hours = 3.2
layered_closed_loop_test20_wall_time_seconds = 486
layered_closed_loop_test20_wall_time_minutes = 8.1
```
