# M3 简版：用户选择的全局 Topo 引导 State2State 导航

> 状态：M3 已实现。M2 的共享 `GlobalMapRuntime`、全局稀疏 topo 图和
> `MapManager::findTopologyPath()` 提供全局连通性；本文件是已实现接口和后续扩展的约束。\
> 范围：仅 `STATE2STATE` 长距离导航。exploration 消费全局 topo、跨进程持久化和 topo
> 图重构不在本阶段。\
> 相关背景：[统一规划运行时、控制权切换与全局地图/拓扑图设计](planner_runtime_global_map_design.md)、
> [探索链路审计](exploration_chain_epic_falcon_refactor.md)。本文件在“用户选择的 topo
> 引导 state2state”问题上优先于前者的 M3 概述。

## 1. 目标与非目标

目标是在不改变 M2 地图所有权的前提下，让使用者为每个运行中的导航任务选择：

```text
普通局部导航
  = local A* / 直线前端 -> local corridor -> trajectory optimization

全局 topo 引导导航
  = sparse global topo A* -> 当前 local prefix -> local corridor
    -> trajectory optimization
```

全局 topo 的作用仅是保存跨滑窗、跨任务的已知 free-space 连通性，并给长距离任务提供
路线骨架。它不是第二张高分辨率地图，也不是可直接发布的飞行轨迹。

本阶段明确不做：

- 不让用户通过话题重配、清空或重建全局 topo；构图配置只允许在
  `GlobalMapRuntime` 启动时从 `global_topology.yaml` 读取。
- 不用 topo 路径绕过当前 ROG、dynamic obstacle、corridor、backup 或 commit validation。
- 不把 exploration 的 Bubble/frontier/TSP 迁移到 global topo；那是后续独立工作。
- 不在每一个 local replan tick 都重跑 global topo A*。
- 不把未经当前地图验证的 RDP、直线 shortcut 或滑窗外历史边送入 corridor。

## 2. 运行时接口：一个选择话题

### 2.1 用户选择话题

```text
/planner/navigation/use_global_topology    std_msgs/Bool    非 latched
```

语义：

| `data` | 行为 |
| --- | --- |
| `false` | 对下一次 state2state 规划使用普通 local-only pipeline；届时废弃缓存的 global route。 |
| `true` | 对长距离 state2state 任务优先查询和复用 global topo route；最终仍必须经过 local pipeline。 |

约束：

1. 节点启动默认值必须为 `false`。该话题非 latched，运行时重启后不会意外恢复旧策略。
2. 它是**规划策略**，不是 mode request、不是 goal、不是命令控制话题；只在
   `active_mode=STATE2STATE` 时生效。
3. `true -> false` 不取消已发布且安全的当前轨迹，也不触发紧急制动；它使下一次规划或重规划
   使用 local-only pipeline，并禁止新的 topo-guided candidate commit。
4. `false -> true` 不直接发布命令。它只使下一个新目标、常规 replan 或明确 route-refresh
   使用 topo 查询。
5. 该最小接口的 `true` 语义是 **TOPO_PREFERRED**：topo 无快照、无路径或暂时不可 attach 时，
   planner 可回退到已知 free 空间内的普通 local replan；绝不能伪造通向远处 goal 的直线。
   如需“没有 topo route 就不移动”的严格语义，后续可升级为枚举策略，但不是本阶段的需求。

导航运行时的同一 `STATE2STATE` task 可在飞行中直接覆盖 goal。该更新不是新任务：它不改变
`task_epoch`、不执行 `PAUSE/CLEAR/ARM`，且 `ready_for_new_task` 仍为 `false`；底层 FSM 以
`new_goal` 触发 rolling replan。仅 mode transition、非 `STATE2STATE` 模式或未完成 boot 时拒绝 goal。

调试示例：

```bash
rostopic pub -1 /planner/navigation/use_global_topology std_msgs/Bool '{data: true}'
rostopic pub -1 /planner/navigation/use_global_topology std_msgs/Bool '{data: false}'
```

建议新增的配置只设置这个话题和算法阈值，不能覆盖 `global_topology.yaml`：

```yaml
general_planner:
  state2state:
    topology:
      query_capability_enable: true  # 启动期能力开关；不是用户策略
      selection_topic: /planner/navigation/use_global_topology
      min_query_distance: 8.0
      local_prefix_length: 8.0
      local_boundary_margin: 0.8
      route_rejoin_max_candidates: 3
      route_deviation_requery_distance: 1.0
      route_query_min_interval: 0.5
```

现有 `state2state/topology/query_enable` 在实施时应改名或等价迁移为
`query_capability_enable`。它只能表达“该 build/profile 允许进行 topo 查询”；真正的每任务
选择只由 `/planner/navigation/use_global_topology` 决定。不得同时保留两个含义相互冲突的用户开关。

## 3. 不变量与所有权

```text
GlobalMapRuntime
  owns: ROGMapROS + MapManager + BoundaryMap + sparse global topology
  updates: cloud/odom once, topology worker continuously

State2State route consumer
  reads: immutable topo SearchSnapshot
  owns: task-local GlobalRouteContext only
  may: requestTopologyUpdateAround(start) as a non-blocking priority hint
  must not: call configureTopology(), setTopologyActive(), clear topology,
            updateMap(), or create a second MapManager
```

无论用户是否选择 topo-guided，地图和稀疏骨架都持续构建。关闭策略只是停止**消费** topo，
不是停止维护世界模型。

### 3.1 维护调度保证

全局 topo 的正确性不能依赖某个 ROS callback queue 恰好空闲。`GlobalMapRuntime` 每次成功融合
cloud/odom 后，必须在 `MapManager` 已记录该帧 revision 与 dirty bounds 后请求一次 topology
maintenance；请求只唤醒异步 worker，并按 `global_topology/update_period` 合并、限频，绝不在
点云回调内执行 region rebuild。worker 同时以 `steady_clock` 周期兜底，并在 worker 内执行
`missionActive()` gate；ROS timer 仅是兼容性的额外 wake-up。

因此，即使 exploration/LIO 占满单一 ROS spinner，世界生命周期 topo 仍持续消费已融合的
known-free evidence。验收状态为 `map_ready=true` 后 `topo_revision` 必须推进，且
`topology_ready=true`；`/planner/navigation/use_global_topology=false` 不得改变这三个维护条件。

全局图必须保持稀疏导航骨架配置：

```yaml
global_topology:
  construction_mode: persistent_bubble_skeleton
  planar_mode: false
  unknown_as_free: false
  min_clearance: 0.40
  sample_spacing: 0.80
  candidate_separation: 1.50
  connection_radius: 6.00
  max_nodes_per_region: 12
  max_neighbors: 8
```

它保留已观测 free-space component 的高净空核心和跨 region portal，而不是每个 free lattice
sample。`ROG + BoundaryMap` 才是地图事实；topo route 只保存连通性记忆。

## 4. 任务内路线状态

实现应引入 task-local、不可跨 `task_epoch` 使用的 `GlobalRouteContext`：

```cpp
struct GlobalRouteContext {
  bool valid{false};
  bool reaches_goal{false};
  std::uint64_t route_id{0};
  std::uint64_t task_epoch{0};
  std::uint64_t world_epoch{0};
  std::uint64_t map_revision_at_query{0};
  std::uint64_t topo_revision{0};

  Vec3f goal;
  vec_Vec3f raw_topology_route;  // global A* 原始输出，绝不被 RDP 覆盖
  std::vector<double> arc_length;
  double committed_route_s{0.0}; // 单调推进，防止投影跳回已走段
  double last_query_time;
  std::string last_result;
};
```

失效规则：

| 事件 | 处理 |
| --- | --- |
| 新 navigation goal / 新 `task_epoch` | 清空 route；若用户已选 topo，则为新目标查询。 |
| `world_epoch` 改变 | 无条件清空 route；任务/轨迹的安全处理仍由既有 runtime 控制。 |
| 用户发布 `false` | 下次规划清空 route 并使用 local-only；当前已提交的安全轨迹不被硬中断。 |
| route 已执行完或无法投影到 suffix | 重新 global topo A*。 |
| 当前 local prefix 被地图变化阻断 | 先 local repair；repair 失败再重新 global topo A*。 |
| topo revision 单纯增长 | 不立即废弃 route；先只重验当前 local prefix。新节点不应导致高频路线抖动。 |

`map_revision` 变化不能被忽略，但也不应让每帧地图变化都触发全球搜索。只有最新 changed box 与
当前 local prefix、候选轨迹或 route rejoin 范围相交，才需要立即重验；不能安全通过时才进入 repair/
requery。

## 5. 什么时候走哪条 pipeline

设：

```text
H = min(local_prefix_length, planning_horizon, local-map-safe-horizon)
```

当前 profile 中 `planning_horizon=8.0 m`、ROG half size 为 `12 x 12 x 3 m`，所以初始
`H=8.0 m`；`local-map-safe-horizon` 必须扣除 `local_boundary_margin`，不能把局部目标压在
ROG 滑窗边缘。

| 条件 | 策略 |
| --- | --- |
| 用户选择 `false` | `LOCAL_ONLY`。直线可用则直线 guide，否则 bounded local A*。 |
| 用户选择 `true` 且目标距离 `< min_query_distance` | `LOCAL_ONLY`。短距离不为 topo 查询付出额外延迟。 |
| 用户选择 `true`、目标距离 `>= min_query_distance`、global route 有效 | `TOPO_GUIDED`，复用 route suffix。 |
| 用户选择 `true`、route 缺失且 topo ready | 查询 global topo A*；成功则 `TOPO_GUIDED`。 |
| 用户选择 `true`、topo 未就绪/不连通/无法 attach | 记录原因，回退到保守 `LOCAL_ONLY`；不得生成跨未知空间的直线。 |
| `TOPO_GUIDED` 的 local prefix 失效 | 先 `LOCAL_REPAIR`，失败后 `TOPO_REQUERY`，再失败才 local-only/hold。 |

对于远距离但目标还未进入已知 free topo 的情形，允许寻找**经过 topo A* 实际可达**、且朝 goal
方向推进的 known-free anchor；该 anchor 仅是当前 horizon 的局部目标。不得把“离 goal 更近的
任意 topo node”当作可达，也不得把 goal 和 anchor 直线相连。

## 6. Topo-guided local replan 算法

### 6.1 全局查询：低频，只给出路线骨架

在以下时刻查询：新长距离 goal、route context 无效、无法重接 suffix、当前 route 耗尽，或安全
验证确认 route 已失效。查询限频为 `route_query_min_interval`。

```text
预测重规划起点 p0
  -> requestTopologyUpdateAround(p0)          # 非阻塞优先级提示
  -> acquire immutable SearchSnapshot
  -> findTopologyPath(snapshot, p0, goal)
  -> 成功：保存 raw_topology_route 与 revisions
  -> 目标不可 attach：搜索经 topo A* 可达的 known-free progress anchor
  -> 仍失败：报告 NO_TOPO_ROUTE，不伪造 global route
```

`findTopologyPath()` 的 endpoint attachment、起点接入边、目标接入边和图边都必须通过当前
`MapManager` 查询。它返回的路线可能有滑窗外历史点，因此不能直接进入 corridor。

具体的 graph query 使用欧氏代价的 A*：

1. `start` 与 `goal` 必须先是可通行的已知 free 点；若当前 MapManager 证明直连安全，直接返回
   两点路线。
2. 否则在 `connection_radius` 内找起点、终点周围的候选 topo node，只对有限个近邻做当前
   line-traversable attachment 检查。
3. 图内边的代价是两 node 的欧氏长度；open set 的优先级为
   `f(n)=g(n)+||node(n)-goal||`。这个启发式与边代价一致，因此保持最短路正确性并避免 Dijkstra
   扩展整张长距离图。
4. 将起点接入边、A* node 序列和终点接入边拼为 `raw_topology_route`。任何 attachment 或图搜索失败
   都返回失败，绝不以“到目标的直线”伪造路线。

### 6.2 每次 local replan：投影、截取、验证

每次 replan 从预计实际执行状态而非旧 odom 开始：

```text
p0 = current committed trajectory at (now + replan_lookahead)
     或实时 state（plan-from-rest 时）
```

随后：

1. 将 `p0` 投影到 raw route 的未执行 suffix，投影弧长不得小于
   `committed_route_s - tolerance`；防止 route 投影在自交/平行走廊中跳回旧段。
2. 从该弧长按**折线弧长**截取 `H` 米；终点 `pH` 可以位于 sparse topo edge 中间，不能只按
   “取第 N 个 topo node”。
3. 从 `p0` 到 `pH` 逐段检查：点在 local ROG 内、膨胀地图为已知 free、线段无遮挡、动态层
   无占据、净空满足 local policy。检查使用 local ROG 的细粒度步长，而不是只相信 global topo
   建边时的 `edge_sample_spacing`。
4. 全部通过则得到 `verified_topology_prefix`；其语义是局部 guide，不是最终轨迹。

```text
raw global route: S --- A --- B --- C --- Goal
                             ^
current local prefix:       S --- A --- P(H)
```

### 6.3 local A* 是修补器，不是无条件重复计算

`verified_topology_prefix` 已经能作为 local corridor 的 seed path；此时不必再对相同问题运行一遍
local A*。局部规划仍然存在，因为 corridor、轨迹优化、backup 和 commit validation 始终执行。

只有以下情况进入 bounded local A*：

- topo prefix 的某一段被新静态/动态障碍阻断；
- 飞行器偏离 route，无法安全投影或直接接入 suffix；
- corridor 对经过安全验证的简化 guide 生成失败，需要替代离散 guide；
- route 起点与预测执行状态的连续性检查失败。

修补算法：

```text
1. 在 global route suffix 中选择最多 N=3 个 rejoin anchor：
   - 位于当前 local ROG 内且距边界大于 margin；
   - endpoint 为 KNOWN_FREE；
   - 按沿 route 的前向弧长从远到近尝试。
2. 对每个 anchor 运行 bounded local A*：p0 -> anchor。
3. 第一个成功的 local A* path 与 anchor 后的 verified topo suffix 拼接。
4. 若没有 anchor 可重接，raw route 失效并触发一次 topo requery。
5. topo requery 仍失败时，按 TOPO_PREFERRED 语义回退 local-only 或安全 hold；
   不得发布任何未验证的 global suffix。
```

### 6.4 简化、加密和 corridor

必须保留三种不同的路径表示：

```text
raw_topology_route       全局、持久、不可修改；仅作路线语义和重接依据
local_guide_path          当前 local ROG 内的验证 prefix 或 A* repair path
corridor_seed_path        对 local guide 简化后再按 corridor 约束加密的路径
```

处理顺序固定为：

```text
local_guide_path
  -> current-map constrained LOS shortcut
  -> （可选）current-map constrained RDP
  -> segment densification
  -> CorridorGenerator::SearchPolytopeOnPath
  -> corridor state2state optimizer
  -> backup / candidate validation / commit
```

现有 `shortcutPathByLineOfSight()` 已能做“最远可见点”简化。M3 简版先复用它；不应为了使用 RDP
而新增纯几何简化分支。

后续启用 RDP 时，删除 `p[i..j]` 前必须同时满足：

```text
perpendicular_error(p[i..j], line(p[i], p[j])) <= epsilon
AND line(p[i], p[j]) 在当前 local ROG 内
AND line 在 inflated KNOWN_FREE 中连续可通行
AND line 满足动态障碍与最小净空约束
```

RDP 只能简化 `local_guide_path`，不能覆盖 raw global route，也不能在滑窗外运行。简化后必须按
当前 `corridor_line_max_length` 加密；当前配置为 2.0 m，前端的实际目标最大 seed segment 为约
1.6 m。这样 `SearchPolytopeOnPath()` 在每段都有足够局部地图证据生成连续 SFC。

## 7. Commit 前验证与回退

每个 topo-guided candidate 必须记录并在 commit 前核验：

```text
task_epoch, world_epoch, map_revision_at_query, topo_revision,
route_id, route_source={topo_prefix|local_repair|local_only},
guide_result={goal|horizon|anchor}
```

提交前规则：

1. `task_epoch` 或 `world_epoch` 不一致：丢弃 candidate。
2. `map_revision` 未变化：执行既有 trajectory/corridor/backup commit validation。
3. `map_revision` 变化：使用当前 ROG 对完整 candidate trajectory 和 backup 重验；不安全则丢弃，
   进入 local repair、topo requery 或 hold。
4. `topo_revision` 变化本身不代表当前轨迹失效；它只触发 prefix revalidation，不允许直接用新
   topo route 覆盖一个安全的已提交轨迹。

回退顺序：

```text
topo prefix -> 未简化 topo prefix -> local A* repair
-> topo requery -> local-only known-free replan -> keep-current / brake / hold
```

任何一步都不得允许 unknown、滑窗外未验证段或 corridor generation failure 绕过安全检查。

## 8. 实施切分与文件边界

### M3-S1：策略话题与可观测性

- 在 `FsmRos1` 订阅 `std_msgs/Bool`，只保存 task-local policy；不让回调直接规划。
- 在 `Config` 新增 selection topic 和 route-policy 参数，将旧 `query_enable` 迁移为 capability gate。
- 在 diagnostics 中输出 policy、route_id、world/map/topo revision、route result 和 fallback reason。

主要文件：

```text
include/ros_interface/ros1/fsm_ros1.hpp
include/general_core/config.hpp
config/task_planner_runtime_state2state.yaml
```

### M3-S2：GlobalRouteContext 与 topo 查询

- 在 state2state task runtime 持有 `GlobalRouteContext`；切换任务、goal、world epoch 或 policy
  关闭时统一失效。
- 将 `MapManager::topologySearchSnapshot()` 与 `findTopologyPath()` 封装为低频 route query；
  保存 raw route，而不是只返回一次性 local path。
- 仅在上述第 6.1 节条件下查询，禁止在 safety callback 中同步重建 topo 或频繁全图搜索。

主要文件：

```text
src/general_core/state2state/state2state_frontend.cpp
include/general_core/state2state/state2state_frontend_services.hpp
src/general_core/state2state/state2state_pipeline.cpp
```

### M3-S3：local prefix、repair 与 corridor 接入

- 从 raw route 进行单调投影、弧长截取、当前地图验证、LOS 简化和 corridor seed 加密。
- prefix 有效时直接进入既有 corridor path；失败时执行 bounded local A* rejoin repair。
- 所有优化/backup candidate 在 commit 前关联并检查 revisions。

主要文件：

```text
src/general_core/state2state/state2state_frontend.cpp
src/general_core/state2state/state2state_exp_generation.cpp
src/general_core/state2state/state2state_pipeline.cpp
```

`GlobalMapRuntime`、`MapManager::configureTopology()` 和
`global_topology.yaml` 不因该功能而获得任务级重配置接口。

## 9. 验收与测试矩阵

| 场景 | 期望结果 |
| --- | --- |
| 用户未发布策略话题 | 默认 local-only；global topo 仍持续构建。 |
| 发布 `true` 后发送 30 m 已知 free goal | 查询一次 topo A* 并缓存 raw route；每次 replan 仅截取 local prefix 并生成 corridor。 |
| 发布 `false` | 后续 replan 不读取 global route，当前安全轨迹不被硬中断。 |
| topo route 在当前 local ROG 中完整可见 | 不重复 local A*；prefix 直接成为 corridor seed。 |
| route 一段被新障碍阻断 | local A* 可重接时生成 repair guide；不能重接时 requery topo。 |
| topo 未就绪或 target 不可 attach | 输出明确 `NO_TOPO_SNAPSHOT` / `NO_TOPO_ROUTE`；不产生跨未知直线。 |
| 单一 spinner 被 LIO/exploration 回调持续占用 | 已融合地图仍触发 topology worker；`topo_revision` 推进且 Marker 非空。 |
| 地图在优化期间改变 | commit 前 revalidation 拒绝不安全 candidate。 |
| exploration 后切到 state2state | 不重建地图；route 的 `topo_revision` 来自 exploration 已构建的同一图。 |
| world reset | route 缓存失效；任务与轨迹由既有 runtime 安全流程处理。 |

至少增加：

1. 策略运行时 `false/true` 代际单测，验证默认 local-only、重复消息幂等；
2. raw route 弧长截取与单调投影单测；
3. 安全 shortcut/RDP 测试：几何可简化但线段撞障时必须保留转角；
4. local repair 成功、失败后 requery、requery 失败后 hold 的回退测试；
5. map revision 在 query 与 commit 之间变化时的 candidate 丢弃测试；
6. ROS 集成测试：探索构图后，state2state 选择 topo-guided 并只对局部 prefix 生成 corridor。

## 10. 完成定义

M3 简版完成必须同时满足：

- 用户能通过一个非 latched `Bool` 选择是否优先消费 global topo；
- global topo A* 的原始路径只保存在 task-local route cache，永不直接成为控制命令；
- 长距离 topo-guided 模式使用“全局 route 缓存 + 高频 local prefix replan”，不是每 tick 重跑
  global A*；
- 每个最终轨迹仍经过当前 local ROG corridor、优化、backup 和 commit validation；
- topo prefix 失效时先局部修补、后全局重查，无法证明安全时 hold；
- route 和 commit 日志可追溯到 task/world/map/topo revisions；
- 当前稀疏 global topo 的构图所有权、内存上界和未知空间 fail-closed 语义不被破坏。
