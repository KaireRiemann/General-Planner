# 探索链路审计：以 FALCON 覆盖引导结合 EPIC 轻量前端

> 审计对象：`General-Planner` commit `2ebcf08`（2026-08-05）。本文描述的是当前实际调用链，而不是配置注释所表达的设计意图。结论是：**全局覆盖层应保留，但只提供离散的候选集合/次序；局部执行层应回归 EPIC 式的“点云—拓扑—少量视点—快速可行边”闭环。不要再把所有偏好叠加到一个大标量 cost 中。**

## 结论摘要

当前实现已经有 FALCON 式的持久覆盖图和长程顺序，也有 EPIC/HighSpeedExp 式的点云 frontier、bubble 拓扑和高速轨迹优化；问题不在于能力缺失，而在于两种策略被重复地写进了候选生成、候选打分、TSP 和恢复状态机中。

- 候选的全局决策目前至少叠加了 travel、转向/刹停、回访机会损失、信息增益、等待时间、pass debt、coverage rank、失败冷却、编队冲突和 goal lock；其中若干项又已在视点评分或安全门内出现。
- 覆盖引导在常规 frontier 存在时只是“注入少量 cluster + 软惩罚”，而在 frontier 为空时又切到“合成 coverage viewpoint + 再跑完整全局代价”。这既没有形成清晰的全局优先级，也让执行路径分叉。
- 应改成分层、词典序决策：**硬可行性门 → 覆盖窗口筛选/排序 → 一个轻量局部效用**。安全不再作为 reward/penalty；全局覆盖不再和局部运动学混合为可调权重；TSP 只服务全局 guide，不再反过来决定当前第一目标。

## 当前完整探索链路

```text
点云 + odom
  │  ROG/LIO 更新、地图版本与历史 odom
  ├─────────────────────────────► CoverageGuidance（1 Hz、latest-wins worker）
  │                                持久体素 → free/unknown zone 图
  │                                → CP 开放路径/active-zone 顺序
  │                                → preferred cluster / rank / finish guard
  ▼
TopoGraph 增量 bubble 骨架 ──► FrontierManager
                                frontier 提取、cluster、候选观察点、可见性 raycast
                                高速视点评分与 hard gate
  ▼
FastExplorationManager::planGlobalPath
  1. 更新 pass debt、请求 coverage preferred clusters
  2. generateTSPViewpoints：近邻/新 cluster + preferred cluster
  3. （仅 frontier 集合为空且 debounce 后）合成 coverage approach viewpoint
  4. 对每个候选做 TopoSearch + EdgeSafetyCost，剔除不可达/高速反向目标
  5. 计算 composite candidate cost，选择第一目标；再建 O(K²) 边矩阵并跑 LKH/TSP
  ▼
FastExplorationFSM
  PLAN_TRAJ → FastPlannerManager::planExploreTraj
  → A*/bubble guide、corridor、MINCO、backup、碰撞/known-free 校验
  → commit / EXEC_TRAJ / replan / reorient / recovery / FINISH
```

| 层 | 当前职责 | 主要位置 |
| --- | --- | --- |
| 感知与局部几何 | 点云/LIO/ROG、frontier 单元、可见性 | `exploration_utils/lidar_map`、`frontier_manager.cpp` |
| 连通性 | 增量 bubble 拓扑、拓扑搜索 | `exploration_utils/pointcloud_topo` |
| 局部候选 | 每个 frontier cluster 的视点、yaw、可见 gain、安全 gate | `FrontierManager::selectBestViewpoint()` |
| 全局 guide | 持久 free/unknown zone 图、覆盖路径、cluster rank | `coverage_guidance_manager.cpp` |
| 当前目标 | frontier/coverage 候选、edge cost、goal lock、TSP | `fast_exploration_manager.cpp` |
| 执行与安全 | 轨迹生成、commit、重规划、停止和 FINISH | `fast_exploration_fsm.cpp`、`general_planner_adapter.cpp` |

### 当前覆盖层实际做了什么

`CoverageGuidanceManager` 不直接在未知空间中飞行：它将 persistent ROG 体素压缩成已知自由区和未知区，以邻接关系建图；未知区选择相邻的已知自由区作为 observation approach。它用确定性贪心加有限 2-opt 得到开放覆盖路径，并把 active-free zone 的顺序转换为每个 frontier cluster 的 `cluster_priority`。

但当前 `full` 模式并非纯 guide：当可执行 frontier 经失败冷却过滤后连续为空时，`FastExplorationManager` 会把 coverage approach 变成 `TopoNode`，与普通候选一同重跑拓扑边、composite cost、goal lock 和 TSP。这是当前最主要的架构分叉。现有 `EXPLORATION.md` 中“coverage 不会生成 flight goal”的表述已与这段实现不一致，应在后续重构时同步修正。

## 候选 cost：当前确实过于复杂

### 已有两轮局部评分

首先，`FrontierManager::selectBestViewpoint()` 已经在一个 cluster 内选择最佳观察点。高速模式中，它使用可见 frontier gain、前向进度、速度对齐、known-free 长度、净空、yaw 变化、转弯角和 backup 可行性评分，并会将 known-free、净空、大转向和大 yaw 直接 hard reject。这个阶段的合理职责是：**把每个 cluster 缩为一个安全、可观测、可执行的代表视点**。

随后，`planGlobalPath()` 又为每个代表点计算完整 `EdgeSafetyCost`：

\[
T_i=t_i+p_i^{turn}+p_i^{known-free}+p_i^{backup}+p_i^{yaw}
\]

`T_i` 已经包含转向、已知自由长度、backup 和 yaw 的运动学/安全代价。之后当前选择器再计算：

\[
J_i=w_T t_i+w_B(p_i^{turn}+p_i^{known-free}+p_i^{backup}+p_i^{yaw})
+w_R R_i-w_G g_i-w_W a_i-w_D d_i
+P_i^{coverage}+P_i^{failed}+P_i^{swarm}
\]

其中 `R` 是通过所有候选两两拓扑边估计的“未来回访损失”，`g/a/d` 分别是饱和后的 gain、等待时间和 pass debt。`P_coverage` 又来自 FALCON 风格路径 rank。之后还要由 goal lock 覆盖一次选择，并用“强制首点”的 LKH/TSP 重新给剩余点排序。

这不是单纯的“权重多”。更严重的是量纲和职责混合：

- `turn/brake` 在视点层评分、`EdgeSafetyCost` 和 `J_i` 中出现三次；其中前两次已经足以保证高速可执行性。
- `gain` 在视点代表选择时决定“能看多少”，又在当前目标选择中给 reward，并在 coverage priority 中再次给 reward。
- wait/pass debt 与 coverage rank 都在表达“不要遗漏远处/被略过的区域”，是同一策略目标的两套状态。
- `future_return` 需要完整的 O(K²) 拓扑搜索，但随后 TSP 也计算同一批两两边；前者和后者都在近似“先去哪个点更不后悔”。
- `failed_goal_penalty=2000` 在候选生成时已经被过滤，保留在 cost 中基本是防御性重复；失败恢复应是状态迁移，不应是软代价。
- `epic_simple_global_cost` 和 `getPathCost()` 是迁移遗留：主链调用的是 `getPathEdgeCost()`，无论该开关如何均使用 `estimateHighSpeedEdgeCost()`；`bm_without_topo` 在函数内固定为 `false`。因此该开关在当前主链上不实现其名称承诺的 EPIC 简化全局 cost。
- `local_viewpoint_num_`、`global_viewpoint_num_`、`w_vdir_`、`w_yawdir_` 被读入，但当前主选择链没有实际使用它们（方向/yaw 的旧计算已注释）。这是应删除或恢复到唯一职责的死配置。

## 应删除、合并或下沉的设计

| 现状 | 判断 | 修改建议 |
| --- | --- | --- |
| 高速视点评分与 `EdgeSafetyCost` 再次惩罚转向/yaw/known-free/backup | 重复，且会把安全偏好误当作全局收益 | 视点层只做 hard gate + cluster 内排序；edge 层只输出执行时间和可行标志。全局选择不要再单独加 `turn_brake`。 |
| `future_return` 与 LKH 的两次两两边计算 | 最贵的重复 | 删除 `future_return`。全局 coverage path 已提供“以后先后”的信息；若保留 tour，只对宏观 guide 运行。 |
| wait age、pass debt、coverage rank | 三个反短视 reward | 用 coverage window/rank 作为唯一的全局反短视机制；pass debt 可作为无 guide 时的最终 tie-break，wait age 可删除。 |
| frontier 与 coverage 的严格 fallback 两套候选池 | 长程规划只在 frontier 耗尽后生效，且代码分叉 | coverage guide 在每轮都产生少量“当前/下一个 macro region”的可执行 frontier 窗口；未知区 approach 仍只在该窗口无 frontier 时补充。 |
| goal lock、failed cooldown、progress watchdog | 三个连续性/活性所有者 | `failed` 与 `no-progress` 合并为 `CandidateState{active,deferred,blocked}`；goal lock 只负责短时轨迹连续性，不能覆盖状态机的禁用决定。 |
| `epic_simple_global_cost`、旧 vdir/yaw weight、固定 `bm_without_topo` | 迁移遗留和误导性配置 | 先写测试确认主链无依赖，再删除；若需要 A/B，改为显式 `selector_mode=legacy|guide_window`。 |
| 160 个 CP 节点上反复 Dijkstra + 局部候选矩阵 + LKH | 高频规划预算不可预测 | 宏观图低频/异步更新；执行层只处理 K 个候选，K 建议 6--8，且不为当前目标建 TSP。 |

**必须保留的不是 cost，而是约束**：碰撞/净空、known-free 制动距离、可达性、低速前的反向航向保护、未知空间不可执行、失败目标退避、终止条件。这些应以 gate 或明确状态表达，不能为追求“简单”而变成可被 reward 抵消的软项。

## EPIC 与 FALCON 的互补方式

EPIC 的公开定位是“直接利用点云、面向大规模场景的轻量 LiDAR AAV 探索”，其 README 还明确说明其拓扑图构建受 FALCON 启发；FALCON 的核心是 online coverage path guidance。两者并不冲突：前者应作为**快速执行前端**，后者应作为**慢速全局意图层**。

| 目标 | 采用 EPIC 的部分 | 采用 FALCON 的部分 | 本工程落点 |
| --- | --- | --- | --- |
| 高速、低内存 | 原始点云/轻量 frontier、拓扑可达性、少量视点和快速局部轨迹 | 不在控制周期维护致密全局 ESDF | 保留 LIO/ROG + `TopoGraph` + cluster 代表点；限制执行候选 K。 |
| 避免局部短视 | 局部候选始终通过真实点云与安全 gate | persistent map、free/unknown 图和 coverage path 决定应探索的区域次序 | coverage worker 只输出 guide window/region rank，不输出带权总 cost。 |
| 不进入未知空间 | 真实可达性和轨迹校验 | 未知 component 只对应相邻 free approach | approach 仍必须经过拓扑、corridor、MINCO 和 backup；失败后进入状态表。 |
| 可解释、可调 | 时间/可达性为一等指标 | 覆盖序为离散优先级 | 用少量阈值和词典序，不再用 10+ 权重相减。 |

参考：[EPIC repository](https://github.com/Robotics-STAR-Lab/EPIC)（轻量点云探索，RAL 2025）和 [FALCON repository](https://github.com/HKUST-Aerial-Robotics/FALCON)（Coverage Path Guidance，T-RO 2024）。

## 推荐目标架构

```text
                 低频异步（约 1 Hz，latest-wins）
持久 ROG 证据 ─────────────────► MacroCoverageGuide
                                  - free/unknown region graph
                                  - 开放 coverage path
                                  - guide window: 当前区 + 后续 1~2 区
                                  - 每个 region 的 frontier ID 集合
                                                │ 只输出离散意图
                                                ▼
点云/TopoGraph ─► EPICLocalFrontend ─► feasible cluster representatives (K≤8)
                    - cluster / raycast / yaw
                    - 安全 hard gates
                    - 单次 edge travel-time
                                                │
                                                ▼
                                      GuideWindowSelector
                                      1. 取 window 内可行点
                                      2. 无则取全体可行点
                                      3. bounded detour 决策
                                      4. 仅在平局时用 gain/age
                                                │
                                                ▼
                              现有 corridor → MINCO → backup → commit
```

`MacroCoverageGuide` 和执行层必须有单向关系：guide 影响“从哪个小集合选”，而局部轨迹结果只更新 region/candidate state，不应把局部 cost 写回为下一轮 guide 的连续权重。这样可避免循环放大和难以复现的调参。

## 新的候选选择规则

### 1. 先做硬 gate（不产生 cost）

对每个候选保留以下布尔结果及失败原因：可见 gain > 0、视点净空、known-free 制动距离、yaw/大转向限制、拓扑可达、非 failed/deferred、编队不冲突。候选不满足任一不可协商条件时直接删除。只有“速度方向冲突但可先刹停”可进入 `requires_reorient` 分支，不能加入普通 cost 排名。

### 2. guide 产生小窗口，而不是全局 penalty

coverage 路由每次只发布：

- `guide_epoch`；
- `current_region`、`next_regions[1..2]`；
- 每个 region 的可执行 frontier IDs；
- 每个未知 region 的最多 1--2 个已知自由 approach；
- `coverage_done` 所需的未完成 region 状态。

普通 frontier 存在时，窗口内的 frontier 始终参与局部候选；若窗口内没有可行 frontier，才允许选所有 frontier。只有当窗口和普通 frontier 都无可行点时，才暴露 unknown 的 free approach。这样全局路径从一开始就消除“只贪最近房间”的短视，而不会让无人机直接追逐远处未知点。

### 3. 用词典序/有界绕行替代加权总和

每个可行候选只计算一次真实执行时间 `t_i`。令 `t_min` 为全部可行点中的最短时间，`Δ` 为允许为了 global guide 付出的最大额外时间（秒，而非无量纲权重）。

```text
guided = { i | i 在 guide window 且 t_i ≤ t_min + Δ }
pool    = guided 非空 ? guided : all_feasible
choose  = min_lexicographic(pool,
                            guide_stage(i),   # current < next-1 < next-2 < fallback
                            t_i,
                            -normalized_gain(i),
                            stable_id(i))
```

建议初值：`window=当前+后续2区`、`K=8`、`Δ=2.0 s`（由任务最大速度和最小安全制动距离决定，需 A/B 标定）。若环境非常开阔，可将 `Δ` 改为 `min(2.0 s, 0.25*t_min+0.5 s)`。这只有一个可物理解释的 trade-off：**为了遵循覆盖顺序，最多接受多少额外飞行时间**。

`gain` 只应在同一 guide stage 且 travel-time 接近平局时使用；pass debt 可以只在 `guide` 不可用或所有候选都处于 fallback 时使用。wait age、future return、coverage rank penalty、turn/brake weight 从当前目标 cost 中移除。

### 4. TSP 的新职责

当前实际只执行第一个目标，因此不应让 LKH 参与第一个目标的选择。保留 TSP/2-opt 的两种任选其一：

- 在 `MacroCoverageGuide` 中对 region 节点低频求开放路径；这就是 FALCON 职责。
- 或只作为 RViz 的“后续参考路线”，绝不能反向改变已由 selector 决定的首点。

局部候选不再建立 `K×K` 的 `EdgeSafetyCost` 矩阵，也不需要强制第一节点的 LKH。对每轮节省的主要是 `O(K²)` topo search 和 LKH 文件 I/O；K=8 时至少避免 56 条有向候选间搜索。

## 分阶段实施计划

### Phase 0：测量并冻结基线（不改变行为）

1. 为每轮记录 `N_raw / N_gate / N_window / N_final`、各阶段耗时、被 gate 原因、guide stage、`t_min` 与最终绕行 `t_selected-t_min`。
2. 将 `coverage_guidance/mode=shadow` 与当前 `full` 在 garage、house、多岔洞穴各重复至少 10 次；记录完成时间、路径长度、时间积分 coverage、重复目标数、平均/95% 全局规划延迟和恢复次数。
3. 将已有 `coverage_guidance_self_test`、`coverage_recovery_identity_self_test`、`yaw_candidate_selector_self_test` 纳入 CI；补一个 guide-window 的确定性单测。

### Phase 1：低风险去重

1. 把 `failed`/`no-progress` 归并为候选状态表；在生成候选时过滤，删除 `failed_goal` cost 项。
2. 保留视点 hard gate 与 `EdgeSafetyCost.time_cost`，将 global `turn_brake` 权重置零并 A/B；确认高速飞行安全性不下降后删除该项。
3. 删除 `future_return`，并把 LKH 结果降级为仅可视化；保留确定性排序以便回归对比。
4. 清理无主配置/死分支：`epic_simple_global_cost`、`getPathCost()`、固定 `bm_without_topo`、未使用的 viewpoint number/vdir/yaw 参数。若需兼容，先发出一次启动期 deprecation warning。

### Phase 2：接入统一 GuideWindowSelector

1. Coverage worker 输出 region 到 frontier IDs 的映射和 window，不再输出 `clusterPenalty`；`preferredClusterIds()` 改为 `guideWindowClusterIds()`。
2. `generateTSPViewpoints()` 接受“window IDs + fallback budget”，先确保 window cluster 被重新验证，再用近邻 cluster 填满 K。
3. 用上一节的词典序规则直接选择第一个目标。coverage approach 作为同一 `Candidate` 类型，而非第二条 planning path。
4. 不再将 synthetic coverage 与 normal frontier 强制互斥；二者可以同属一个 guide window，但未知 approach 永远排在窗口内普通 frontier 之后。

### Phase 3：性能与可靠性收口

1. 宏观图仅在地图 evidence 或 region/frontier 集变化阈值超过时重建；持续 latest-wins，给 worker 明确 CPU/内存预算。
2. 将 coverage plan 的区域 ID 和 candidate state 持久化为稳定 identity；地图大变化时只失效受影响 region。
3. FINISH 只读取三个离散事实：frontier 稳定为空、guide 无 actionable region、覆盖观测量进入 plateau。不要让冷却中的软 cost 阻塞 FINISH。

## 验收标准与风险边界

重构不是只比较“完成时间”。应同时满足：

| 指标 | 目标 |
| --- | --- |
| 局部短视 | guide window 有可行点时，选择窗口外点的比例接近 0；选择窗口内点的额外 travel time 不超过 `Δ`。 |
| 覆盖质量 | 完成覆盖率不低于基线；时间积分 coverage（AUC）提高或不退化。 |
| 实时性 | 全局选择 p95 延迟下降；当前目标选择不再有 K² 候选边搜索和 LKH I/O。 |
| 飞行安全 | hard-gate 拒绝率、碰撞、backup 失败和受控 reorient 次数均不恶化。 |
| 活性 | 同一 stable candidate 的无进展重试有上界；FINISH 不被冷却/陈旧 coverage plan 无限阻塞。 |
| 可复现性 | 固定地图和 odom 输入时 guide epoch、候选池、最终目标和拒绝原因可确定复现。 |

高风险改动是把安全 penalty 删除后却没有把它升级为 gate；这会让全局 guide 的偏好压过制动距离和净空。另一个风险是把 guide window 做成严格的远距离强制目标；因此必须保留 `Δ` 有界绕行与“window 不可行即回退”规则。

## 与现有文件的映射

- 当前全局候选、cost、LKH 与 coverage fallback：[fast_exploration_manager.cpp](../src/Planner/general_planner/src/general_core/exploration/highspeed/fast_exploration_manager.cpp)
- frontier/视点评分和 hard gate：[frontier_manager.cpp](../src/Planner/general_planner/src/general_core/exploration/exploration_utils/frontier_manager/frontier_manager.cpp)
- frontier shortlist 注入逻辑：[global_planning.cpp](../src/Planner/general_planner/src/general_core/exploration/exploration_utils/frontier_manager/global_planning.cpp)
- 持久 coverage 图和开放路径：[coverage_guidance_manager.cpp](../src/Planner/general_planner/src/general_core/exploration/exploration_utils/coverage_guidance/coverage_guidance_manager.cpp)
- 边运动学/安全代价：[general_planner_adapter.cpp](../src/Planner/general_planner/src/general_core/exploration/highspeed/general_planner_adapter.cpp)
- FSM、恢复和 FINISH：[fast_exploration_fsm.cpp](../src/Planner/general_planner/src/general_core/exploration/highspeed/fast_exploration_fsm.cpp)
- 当前调参入口：[exploration_house.yaml](../src/Planner/general_planner/config/exploration_house.yaml)、[exploration.yaml](../src/Planner/general_planner/config/exploration.yaml)

