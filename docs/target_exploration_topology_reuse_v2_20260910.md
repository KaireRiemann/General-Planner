# Target Exploration 已知拓扑复用重构方案 V2

分析日期：2026-09-10。基线为 `deploy` / `c2b213e9271cdb09dc606e8232ce2ff46d4d5f19` **加本次分析时工作区未提交修改**。后者包含 target 稀疏索引、任务域、安全检查、候选预检查等修改，不能只 checkout 该提交复现本基线。分析期间工作区仍有更新，交付前已复核 workspace 的 coverage 兼容处理、target 静止短路径规则及新增合成走廊测试；实施前仍需按 P0 固定完整快照。

本文是当前源码审计和实施方案；新增接口、配置和状态均为建议，尚未实现。本次只新增本文，不改规划代码、参数或发布包，没有进行新方案的编译、bag 回放或闭环飞行验证。旧对话中的测试结果不能作为当前工作区或本方案的运行证明。

## 1. 本版要实现的行为

Target exploration 应当在同一任务内完成三件事：利用已知连通空间前进，选择有助于抵达目标的未知区域进行观测，执行当前局部地图允许的轨迹。

| 场景 | 应有行为 |
| --- | --- |
| 全局 topo 尚未生成、关闭或查询失败，但局部观测有效 | 原 target 局部探索立即可用；建图在后台继续 |
| 全局 topo 存在通往目标的路线 | 优先执行该路线的局部可行前缀，包括拐角和必要绕行 |
| 目标仍未知，但已知图能通往有价值的出口 | 沿已知通道到出口附近的可观测位置，再继续局部探索 |
| 历史路线某段离开当前 ROG 滑窗 | 保留长期路线，仅提交局部观测允许的前缀 |
| 当前观测发现路线被挡住 | 局部修复、重新查询或回到局部探索，保留原有制动与备份保护 |
| 普通 coverage exploration | 保持当前候选、覆盖目标、结束门控和轨迹流程 |

“没有全局 topo 也可以走”以有效局部观测为前提。当前 target 已明确要求局部 ROG 自由证据，不能把该要求解释成可以在没有局部地图时依靠 LIO 的空 KD-tree 距离占位值起飞。

首期成功标准不是“两张图的数据结构变成一个”，而是全局已知连通性确实决定了 target 的执行路线，同时旧局部能力可回退。长期所有权的进一步统一放在这条链路验收之后。

## 2. 相比上一版，当前基线已经发生的变化

### 2.1 传感器与世界地图

当前 `GlobalMapRuntime::cloudOdomCallback()` 对同一有效输入分别调用一次原始点云 LIO 更新与 `MapManager::updateMap()`。当前 `MapManager::updateMap()` 直接更新 ROG，旧版 temporal static filter 调用已经不存在。

因此，上一轮“多视角确认使静止起步时本地 LIO 没有点”的根因，不能直接套用到当前代码。此次拓扑复用也不应重新引入该过滤链。当前 ROG 参数包含 miss-ray 清理，必须维持现有动态障碍更新语义。[E01][E02]

LIO 的实现通过 KD-tree `Build` / `Add_Points` 累积障碍点；本次所查 `sim_lio.cpp` 的更新函数没有相应删除步骤。它可以提供既有障碍净空估计，但不等价于“此处近期被射线观测为空闲”的证据。路线重验证需要区分这两种含义。[E03]

### 2.2 Target 空间域已经改为按观测增长

工作区当前实现包含：

- Target 模式忽略 `auto_workspace`，不再重写 coverage boxes；coverage 显式启用该参数时仍保留原有自动 workspace 功能。
- Frontier 哈希键变成有符号三维索引，不依赖旧有限维度打包。
- Topo region 索引用 `floor` 处理负坐标；新增 region 在串行维护阶段根据 odom、点云和射线经过区域分配。
- `getRegionNode()` 仍然只读，避免 OpenMP 查询隐式写入容器。
- `targetNavigation()` 下，`IsInBox()` 表示有限坐标且不在显式禁入区；coverage 仍检查 box 并集。

所以旧文档“不扩大固定探索 box”的约束必须改写成“遵守当前任务域”。Target 的最终目标可以不在已分配 region 内，也可以在当前局部地图外；**空间允许、region 存在、当前已知自由是三个独立判断**。[E04][E05]

### 2.3 Target 安全语义比旧版严格

当前 `querySafetyState()` 在 target 下、没有可用 ROG 时返回 UNKNOWN，并禁止将 raw UNKNOWN 类状态通过 LIO fallback 提升为已知自由。Target 的路径处理也加强了 unknown 拒绝；coverage 保留原兼容行为。[E06]

交付前的工作区还在 `estimateHighSpeedEdgeCost()` 中加入 target 静止短步规则：速度不超过 0.20 m/s、路径有效且全段已知自由时，不再仅因巡航所需最短自由距离而否决短接近动作。新来源应兼容这一行为，同时保留后端实际轨迹检查。

同时，当前通用查询仍包含 raw / inflated 状态组合和兼容逻辑。新增历史路线执行应明确要求当前 raw ROG 为 KNOWN_FREE，并检查膨胀、净空和任务域，不要把函数名本身当作“严格实时观测证书”。当前接口没有提供逐体素最后观测时间，不能声称已有 TTL 级新鲜度保证。

### 2.4 已有更完整的任务状态和执行隔离

当前已有 `/planner/target_exploration/status`，其 RUNNING、SUCCEEDED、FAILED、READY 用于上层派发；BLOCKED 经过悬停恢复后可以再次 READY。该接口明确不要求全局拓扑预先可达。[E07]

组合运行时还有独立 odometry、navigation、navigation command、replan、supervisor、gateway、gate 队列。新模块不能把图搜索、拓扑维护或地图锁等待塞进这些控制和监督队列。[E08]

### 2.5 源码入口与发布入口必须区分

当前 `unity_planner_sim.launch` 包含 `task_planner/launch/planner_runtime.launch`，默认初始模式为 state2state；任务 launch 默认 target，Unity 会覆盖它。任务 launch 的参数顺序是基础探索配置、gate 配置、target 配置、场景 overlay、全局拓扑配置，再加显式参数覆盖。[E09]

`general_planner_release` 是独立发布树，带 `CATKIN_IGNORE`，其 launch 启动 `general_planner_release` 包中的二进制。修改源码 YAML 或编译源码工作空间不会自动更新这个发布产物。

本次在 `ros1_noetic` 容器显式 source `/root/ws/real_planner/devel/setup.bash` 后，`rospack find general_planner` 与 `task_planner` 均解析到源码工作空间。该结果只确认这次 shell 的解析，不能替代实际启动进程的来源核对。

## 3. 当前使用已知信息的链路为何不完整

### 3.1 查询到了路线，但路线没有成为执行输入

`TargetTopologyGuidance::update()` 已获取持久搜索快照，查询最终目标；失败时查询候选 anchor。`TargetTopologyGuide` 保存完整 `route`，再沿路线 12 m 左右取一个 `local_prefix_point`。[E10]

主要消费者是：

1. `topologyGuidePenalty()`：给候选增加到该点的距离惩罚。
2. `appendTopologyGuideCandidate()`：把该点变成临时 viewpoint，但要求所属旧 region 存在、终点安全，且 **当前位置到该点的直线** 已知自由。[E11]

在 `S → A → B → G` 绕墙路线中，`S → B` 的直线可能穿墙。全局路线正确也无法通过这一检查；后续还要经过旧 Bubble 搜索。因此现在是“全局知识参与候选提示”，尚不是“全局路线参与完整执行”。

### 3.2 四个层次都仍绑定探索自己的图

| 位置 | 当前结构前提 | 新全局来源需要的调整 |
| --- | --- | --- |
| `getPathEdgeCost()` | `topoSearch(n1,n2)` 可达 | 不用该前提筛掉全局路线意图 |
| `targetBridgeLocallyExecutable()` | 对候选运行 Bubble 搜索 | 区分 frontier 候选检查与完整路线检查 |
| FSM `PLAN_TRAJ` / `updateTopoAndGlobalPath()` | odom 根邻接、普通 tour | 根据路径来源检查准备状态 |
| `callExplorationPlanner()` | `next_goal_node_` 与 Bubble 搜索 | 全局来源直接提供已验证局部折线 |

只修改其中一个入口仍会被其他入口挡住。也不能给全局路线伪造 `global_tour_` 或 `TopoNode`：这些对象还承担 frontier 身份、失败记忆、局部到达和 coverage 完成语义。[E12][E13]

### 3.3 欧氏进度会否决正确绕行

当前多处 target 策略按 `当前到目标距离 − 候选到目标距离` 分类。`target_escape_min_progress=-1.0`，且进入 `target_near_goal_remaining=15.0` 范围后，提交门槛变为不允许负进度。[E14]

例如隔墙距离目标 5 m，但门在身后 8 m：正确路线的第一步可能让欧氏距离增加。当前 guide 即使提示门口，仍可能在 shortlist、detour 筛选或最终 `isCommitEligible()` 被否决。

本次工作区已给全局 anchor 排序保留 detour 名额，并将候选数对齐实际查询次数。这改善了 anchor 查询，但下游 frontier 进度门槛仍然存在；两者不能混为一项已完成的修复。

### 3.4 局部等待与命令故障仍可能冲突

当前 `publishTaskStatus()` 对 PLAN_TRAJ 等活动状态统一发布 RUNNING。Supervisor 随即授权 EXPLORATION；启动命令源宽限默认 2 s。与此同时，target 无可执行桥接点的等待默认 12 s。[E15]

这是现有静态控制流中的冲突条件：如果首条轨迹超过命令源宽限仍未产生，可能先触发命令故障。此次没有新 runtime 复现，不能把它写成当前每次运行都必现。

上一轮增加的 `WAITING_MAP` / 首轨迹标志目前不在该链路中。新版需要重新设计并测试首次命令握手，还要处理没有旧图根时的早退，否则 target 的 12 s 计时函数可能根本没有被调用。

### 3.5 当前“const 查询”不全是无副作用读取

`MapManager::findTopologyPath()` 的两个重载都调用 `syncBoundaryMap()`；`getBoundaryGridType()` 也会同步。`TargetTopologyGuidance::update()` 还请求了拓扑维护焦点。[E16]

因此不能把这些调用直接放到新增后台路由线程并称其为只读。不可变 SearchSnapshot 保证的是图快照不变，不代表实时 ROG / Boundary 查询自动获得一致快照或无线程风险。

### 3.6 “有 expansion_mask”不等于找到真实 frontier

当前全局 expansion 语义是 region portal 没有连到另一 active region。原因可以是未观测、维护未完成、图稀疏或局部障碍，不能直接等同于“从这里继续看一定有收益”。

当前 `findPath()` 还要求起终点在地图视图中可通行，通过一定半径内的直线挂接连接快照图；图搜索内部主要使用快照邻接，不逐次重验全部历史边。[E17]

所以：全局查询失败不证明目标不可达；返回成功也不证明整条历史路线当前都能飞。

## 4. 推荐架构：统一路线使用，保留各自证据职责

```text
GlobalMapRuntime / MapManager
  ├─ 当前 ROG：局部占据、自由与膨胀证据
  ├─ BoundaryMap：跨滑窗历史空间信息
  └─ IncrementalTopologyGraph：长期连通骨架
                      │ 受限读取结果
                      ▼
               TargetRouteProvider
                      │ 完整路线、身份、诊断
                      ▼
               TargetRouteRuntime
          ┌───────────┼─────────────┐
          │           │             │
     已知目标路线  已知出口转场   当前局部 target 探索
          │           │             │
          └───────────┼─────────────┘
                      ▼
         来源选择 → 公共局部路径检查
                      ▼
       现有 General adapter / corridor / MINCO
                      ▼
         backup / commit / 轨迹发布 / Gateway
```

首期新增一个路线提供器和一个 task-local 路线状态对象即可。不要同时重写全部 target 策略、候选收益和通用 FSM。覆盖目标选择继续由既有 coverage 逻辑处理。

State2state 已有完整路线缓存、弧长切片、局部边界截断与重接流程，可以参考其几何算法和测试。但它有独立 policy、task epoch、未来规划起点及任务状态，不能直接共享可变 route runtime，也不能让 target 通过切换 Supervisor 模式去调用另一个任务。[E18]

当前 state2state 的单调投影只设置弧长下界，仍可能跳到空间接近的远端路线分支。若提取共享几何工具，应增加有界投影，先维持 state2state 原调用兼容，再分别验收。

## 5. 地图读取与路线结果契约

### 5.1 路线结果必须携带的信息

建议的最小数据结构，名称可随项目风格调整：

```cpp
struct TargetRouteProposal {
  RouteIdentity identity;  // world_epoch / task_epoch / goal_generation
  uint64_t route_id;
  uint64_t topology_revision;
  uint64_t evidence_revision;
  RouteStatus status;
  RouteEndpointRole endpoint_role; // MISSION_GOAL / OBSERVATION_ANCHOR
  std::vector<Eigen::Vector3d> polyline;
  std::vector<double> arc_length;
  std::vector<EdgeIdentity> edges;
  Eigen::Vector3d anchor;
  double query_wall_time;
};
```

这是意图提议，尚无执行许可。`route_id` 应在路线被接纳或几何实质变化时更新，不能每秒查询同一路线就生成新的失败记忆身份。查询序号和路线身份分开。

建议结果至少区分：NO_CAPABILITY、EMPTY_GRAPH、START_NOT_OBSERVED、START_UNATTACHED、GOAL_UNATTACHED、NO_GRAPH_CONNECTION、QUERY_BUDGET、ROUTE_TO_GOAL、ROUTE_TO_ANCHOR。若继续只返回 bool，不应凭猜测生成更细失败原因。

当前 SearchSnapshot 邻接只保存节点 ID 与 cost，`findPath()` 返回节点坐标序列。首期沿用当前直线边构图契约；需要路线边身份时增加兼容的详细查询重载，不改变旧 state2state 返回值。将来若支持曲线或多段边，必须把实际 edge polyline 一并放入搜索结果，不能只连端点。

### 5.2 读取权限与线程

推荐顺序：

1. 地图所有者完成 Boundary 同步；探索接收已完成更新的通知。
2. 提供不 drain、不重配、不更改 active 状态的读取接口。
3. 不可变图快照可跨线程；起点附近 ROG 挂接证据在世界队列有界获取，或由明确加锁/不可变副本的接口提供。
4. 后台只进行快照图搜索与候选排序，不持有 LIO、frontier、TopoNode 或实时 ROG 的裸引用。
5. 结果回到世界队列后检查任务身份，并重新验证当前局部执行前缀。

只有步骤 2–3 的接口和同步机制已实现，才能启用异步路线查询。单纯给现有 MapManager 指针加 `const` 不满足要求。

图与地图证据 revision 分别记录；不能在搜索完成时读取“最新 map_revision”就宣称整条路线在此 revision 被验证。任务替换结果必须丢弃，局部地图变化则触发局部重验证，不必每帧销毁整个长期路线。

### 5.3 查询预算与长期成本

当前全局查询对全图扫描近邻并排序，anchor 查询可能重复多次；限流到 1 Hz 并不等于计算时间有界。首期需设置真实搜索扩展量、挂接数量、墙钟预算和取消检查；超限直接返回 QUERY_BUDGET，由局部探索继续执行。

先复用当前查询周期和有限候选规模，在测试中确定预算。全图较大时，再在不可变快照上附带空间索引和连通分量；多 anchor 可复用一次起点搜索树。不要把全候选到全候选全局距离矩阵加入高频探索循环。

局部活跃数据和缓存应有容量界限。每 region 的节点上限只限制密度，不限制无限移动任务的总历史内存；全局历史保留与后续卸载属于单独设计项。

## 6. 路径来源与选择策略

建议 target 内部明确区分：

- `LOCAL_TARGET`：当前 direct mission / frontier bridge / recovery 路径。
- `KNOWN_GOAL_ROUTE`：已知图通往最终目标。
- `KNOWN_ANCHOR_ROUTE`：已知图通往可观测出口附近。

新功能关闭时完整走当前逻辑，包括已经存在的 topology guide 提示；关闭新增“路线执行”不应偷偷关闭原有引导功能。

建议部署档位为 `legacy`、`shadow`、`prefer_known`。Shadow 只计算和记录，不修改候选、目标锁、失败计数或 commit，也必须受 CPU 预算限制。最终 target 使用 `prefer_known`：有通往目标且局部可执行的全局路线时优先选择，不必等原探索先失败才使用已知信息。

已知出口路线的优先级应更谨慎：结合已知旅行代价、出口关联的未知区域、近期失败和可观测位置评估，与本地目标探索比较；不能把任意“更靠近目标的拓扑节点”永久锁为任务目标。

来源切换发生在下一个安全规划点。一轮最多进行一次会产生 commit 的后端调用；新来源失败后的 fallback 不清空仍安全的已提交轨迹。可在前端候选阶段比较多个输入，但 `planExploreTraj()` 成功已更新 commit，不能作为无副作用试跑函数。

## 7. 完整路线的局部执行

### 7.1 从当前位置提取保持拐角的前缀

1. 在路线当前位置附近的有限弧长窗口内投影真实 odom；同时约束横向距离、分支连续性与实际位移。
2. 从该位置沿折线提取前缀，保留途中拐角，不退化成当前位置到前缀终点的弦。
3. 检查当前位姿到路线的接入段。必要时进行一次有界局部修复；不能仅因距离近就跨墙接入。
4. 按当前 ROG 窗口、观测状态、净空、禁入区、虚拟地板/天花板截断。
5. 把可用折线交给公共路径处理和 General adapter。全局来源不再要求旧 Bubble 根或 `global_tour`；旧局部来源仍使用原 Bubble 搜索。

Region 未创建不构成全局路线执行的否决条件；当前局部观测不满足则构成否决条件。不能为了执行远端路线预分配从机器人到目标之间全部 region。

### 7.2 哪些公共处理需要来源信息

保留现有限速、曲率、转向、净空、制动、走廊和轨迹检查。对“折线拉直”“回头段裁剪”“未来起点对齐”分别处理：

- 拉直或删拐角产生的新弦必须重新通过局部空间检查。
- 旧路径中的无效 out-and-back 与全局路线的必要 U 形绕行不能仅按角度混同。
- 未来重规划 head、切换时刻、旧轨迹拼接仍由 adapter 唯一决定。路线前端不拼接 `[future, current, ...]`。
- 对齐后的实际候选轨迹需要验证，不能仅检查前端给出的那条折线。

### 7.3 端点角色和制动必须明确

局部前缀终点不是最终目标。至少区分 MISSION_GOAL、CONTINUATION、OBSERVATION_STOP。只有到达原始任务目标，并完成现有受控制动和停稳验证，才报告 SUCCEEDED。

遇到未知边界时，保留安全前缀不意味着允许以非零末速驶出已观测空间。根据当前速度、执行延迟、可用制动距离和 backup 决定能否继续；简化预算可参考 `v·latency + v²/(2·a_brake)`，但最终以现有动力学与备份轨迹校验为准。

长度不足时缩短步长并生成可停稳的观测动作，或等待新观测。不能不断提高最小前缀长度使起步永远失败，也不能通过将 UNKNOWN 视为 FREE 强行通过。

### 7.4 提交前新增来源的最后检查

对最终 candidate 的真实 head、执行段和会被发布的 backup 段检查当前 raw KNOWN_FREE、膨胀、净空、任务域与有限值；包含起点、终点和短于一个采样周期的最后区间。

当前 adapter 的整条 candidate 检查存在 `backup_available` 分支。新增来源门禁应放在两个 commit 分支共同的前置位置，并覆盖有/无 backup；不能假设只要前端 preflight 通过，优化后的所有曲线点就安全。[E19]

## 8. 沿路线进度与未知目标选择

对有效且已接入的路线，进度采用受限投影得到的实际弧长变化 `Δs`。提交了更长轨迹不算机器人已经前进；路线查询成功也不重置停滞计时。

候选 v 只有经过合法连接后，才能使用类似下面的已知路线代价：

`连接到路线的代价 + 路线剩余长度 + 与目标关联的未知延伸估计`。

仅按几何最近点投影会把墙两侧、上下楼层或平行走廊错误等同。3D 连通与路径长度仍保留完整 z；当前 2D 点击沿用目标高度策略，`target_vertical_weight=0` 的候选偏置不能使跨楼层路线长度变成零成本。

实施时应统一检查 shortlist、detour 分类、GoalLock 保留条件、最终 commit eligibility 和停滞逻辑，避免只改评分、后续又被欧氏硬门槛筛掉。

范围控制：首个执行阶段只让 `KNOWN_GOAL_ROUTE` 使用路线进度和独立来源，不改变所有 frontier 的排序。后续 `KNOWN_ANCHOR_ROUTE` 阶段才对已连接路线的候选使用路线进度。无有效全局路线的分支继续当前局部策略；未知 U 形环境中更强的探索完备性属于后续候选策略优化，不能由此次接入保证。

对于未知目标，先找当前位置在图中的可达分量，再从可达出口集合选 anchor；保留有限 detour 预算。当前 expansion mask 只是候选提示，需结合附近未知证据和可停稳观察点。到达普通已知死角却没有新观测价值时，记录 anchor 失败并换出口，不把它当作 mission 完成。

## 9. 生命周期、恢复与状态接管

### 9.1 身份和失效

| 事件 | 路线状态处理 | 世界地图处理 |
| --- | --- | --- |
| 替换目标/新任务 | 增加 goal/task generation，丢弃旧异步结果和意图 | 保留 |
| target → coverage / state2state / tracking / gate | 停用 target 路线提议，执行已有交接 | 保留 |
| 世界重置 | 增加 world epoch，清空路线及相关历史身份 | 由世界所有者处理 |
| 相关局部区域被重新观测 | 前缀与失败记录重验证 | 原建图正常更新 |
| 不相关远处 map revision 增加 | 不自动解除当前坏边冷却 | 正常维护 |

路线失败键应绑定任务、稳定边/anchor 身份、失败原因和相关证据版本。重复查询产生的新 request ID 不能逃逸冷却。障碍证据由地图所有者更新；优化失败只冷却本任务路线，不能删除全局边或标记普通 frontier 已访问。

### 9.2 失败分类

| 原因 | 处理 |
| --- | --- |
| EMPTY_GRAPH / NO_CAPABILITY / QUERY_BUDGET | 局部探索继续；不计 frontier 失败 |
| 接入段不通、历史边出现占据 | 有界局部修复或改选路线；不绕过当前障碍 |
| 前缀遇到窗口边界/UNKNOWN | 正常滚动截断、观测或等待，不立即永久封边 |
| 优化器失败/超时 | 路线级重试预算；继续已有安全 commit 或受控停止 |
| 所有来源长期无法产生可执行动作 | 进入现有 BLOCKED/悬停恢复协议 |
| 任务替换/取消 | 丢弃旧结果，不向新任务传播失败 |

### 9.3 首次命令握手

建议新增内部 `WAITING_LOCAL_PLAN`，由 target 状态上报表示“任务已接受，尚无本任务首条轨迹”。等待时 Gateway 持有当前安全 HOLD，Supervisor 不把这一阶段当作活跃探索命令源故障。

等待必须有独立墙钟期限及取消处理，并在 Supervisor 所在线程有最终超时保护，防止世界线程或优化器卡住导致永远等待。该状态不等待全局 topo 构建完成，也不因周期图 revision 更新无限延期。

首条有效轨迹发布后，按同一 task epoch 切换命令源；必须测试轨迹、状态、execution_enabled 的发布顺序，确认 traj_server 首轨迹不会被丢弃，且 Gateway 不会重放上一任务缓存。仅添加一个布尔值或仅延长 2 s 参数不足以证明握手正确。

对外继续现有 TargetExplorationStatus：有有效地图的活动任务等待仍为 RUNNING，且 `ready_for_new_task=false`；没有局部地图仍按当前协议报告等待地图的 FAILED。不能因为内部使用 WAITING_INPUT 就意外变成允许接收下一目标的 READY。

## 10. 实施阶段与修改范围

| 阶段 | 交付 | 放行条件 |
| --- | --- | --- |
| P0 当前基线 | 保存 HEAD、dirty diff、新增文件、实际参数、启动入口、二进制来源及场景 | 当前 target 和 coverage 的基线能复现 |
| P1 只读路线与 shadow | 无同步副作用的读取边界、路线结果、身份、预算、日志 | 查询不影响现有候选/commit；能记录完整绕行路线 |
| P2 已知目标执行 | 独立路径来源、局部折线、沿线进度、终点语义、最后门禁、首次命令握手 | 无旧图根/无 frontier 的已知通道仍能走；空全局图回退通过 |
| P3 已知出口与恢复 | 可达出口选择、anchor 失败记忆、有界重接、进度停滞检测 | U 形绕行、动态堵路、滑窗返回、目标替换通过 |
| P4 可选整理 | 逐步抽取 TargetStrategy，必要时统一纯几何工具 | 独立行为等价回归；不与前面功能提交捆绑 |

P2/P3 是本次目标的核心。目标相关信息增益、coverage 的已选目标转场复用、删除旧长期历史图均放在独立后续阶段。

文件级建议（路径从 `src/Planner/` 起算）：

| 文件 | 修改内容 |
| --- | --- |
| `map_manager/include/map_manager/map_manager.hpp` | 新增明确无 drain 的读取入口；既有 API 兼容 |
| `map_manager/.../incremental_topology_graph.*` | 详细搜索结果、稳定边身份、预算；需要时附快照索引 |
| `general_planner/.../target_route_provider.{h,cpp}`（新增） | 图快照查询、起终点挂接、可达 anchor 与诊断 |
| `general_planner/.../target_route_runtime.{h,cpp}`（新增） | 任务路线缓存、实际进度、来源锁和失败记忆 |
| `.../target_topology_guidance.{h,cpp}` | 保留现有提示分支；新能力复用数据、避免同时注入同一 anchor 竞争目标 |
| `.../fast_exploration_manager.{h,cpp}` | target 意图选择与 source-aware preflight；当前 sparse 和 coverage 修改保留 |
| `.../fast_exploration_fsm.cpp`、`fsm_utils.cpp` | 所有入口按来源检查；公共执行、到达、等待和失败分支解耦旧 tour |
| `.../planner_manager.h`、`general_planner_adapter.cpp` | 兼容的 route context；真实 head 后的验证及 commit 前门禁 |
| `.../planner_supervisor.cpp`、状态解析/测试 | 首命令握手及有限等待，保留对外状态语义 |
| 新 target 路线 overlay / launch 参数 | 显式开启新来源，只作用于 target；输出生效配置 |

已有 `global_topology.yaml` 决定共享世界图构建。`task_planner_runtime_state2state.yaml` 中的 topology capability 和 `/planner/navigation/use_global_topology` 是 state2state 的选择入口，不是 target 的开关。不得靠调整此处来宣称 target 已启用新路线。

源码入口验收后再单独构建发布包，同时同步 launch、配置、消息和二进制。两个入口分别核验版本，避免源码测试新功能而部署仍执行旧产物。

## 11. 验证方案

实现时，所有编译、单测、集成与回放均在现有 `ros1_noetic` 容器进行。沿用 `/root/ws/real_planner` 的 `catkin_make` 工作空间；各阶段编译自身改动包与必要依赖，不能用旧 devel 中残留的测试二进制替代当前源码验证。

已有可复用测试包括 target sparse domain、target directed、target status、state2state topology route、gateway policy/behavior，以及 coverage guidance / recovery。`target_sparse_domain_self_test` 已显式 `-UNDEBUG`；其覆盖了稀疏坐标、模式切换和 UNKNOWN 基本语义。`target_sparse_runtime_smoke_test.py` 使用隔离 master 检查启动和状态心跳，不提供飞行正确性证据。[E20]

交付前工作区新增 `target_sparse_corridor_integration.py`，通过合成射线走廊和理想位置指令跟随器形成软件闭环，可用于远目标、模式切换和覆盖边界回归。它没有真实飞行动力学，也未构造上述绕墙路线执行用例；本次只读取其源码，没有运行或引用其通过结果。

新增测试应验证跨模块行为，而不止结构/枚举：

| 用例 | 必须观察的结果 |
| --- | --- |
| topo 关闭、空图、无快照 | 当前局部 target 能生成并执行轨迹，无全局等待条件 |
| 起步无局部观测 | 有限等待与安全 HOLD；不虚构 KNOWN_FREE |
| 首条轨迹晚于 2 s、早于规划等待期限 | 不误报命令源故障，之后正常接管 |
| 规划线程卡住 | 独立监督期限仍能终止等待并保留安全控制 |
| L 形走廊：前缀点弦穿墙，折线安全 | 执行折线且轨迹不穿墙 |
| 旧 Bubble 根断开、frontier 数为零、全局图可达 | 全局来源可进入后端，不伪造 tour |
| 已知 U 形通道、距离目标小于 15 m | 允许路线要求的暂时欧氏回退，真实弧长前进 |
| 平行走廊/上下楼/自交路线 | 不跳投影、不跨墙接入，不把规划终点当实际进度 |
| 目标处于未观测远处/旧 boxes 外 | 接受合法任务，不提前分配远端 region，不执行 UNKNOWN |
| 有/无 backup、短尾段、未来 head 偏移 | 最终候选所有可发布部分完成检查 |
| 动态障碍堵路又离开 | 及时停止/修复，相关证据更新后可恢复；检查 LIO 残留点影响 |
| 新目标、取消、target→coverage→target | 旧异步结果丢弃，地图身份保留，coverage boxes 和行为不变 |
| 锚点到达与最终到达 | 锚点不报成功；最终目标停车漂移仍走现有修正流程 |
| 图持续增长且机器人不动 | map revision 不伪造进度；停滞机制仍生效 |
| state2state、tracking、gate 同场景回归 | 命令唯一所有权、独立 odom 和控制时延保持基线 |
| 源码与 release 两种入口 | 实际加载参数、消息协议和二进制版本一致 |

启动/恢复测试使用隔离 ROS master，不启动真实控制消费者。Bag 可验证地图更新、路由结果、状态和历史输入下的规划；由于录制的 odom 不会响应新轨迹，闭环到达成功率、耗时和路径长度必须使用 Unity/Marsim 场景测试。

记录 route_id→traj_id 关联、来源选择原因、实际弧长、前缀拒绝原因、首次命令延迟、图查询时间、地图/odom 年龄、BLOCKED 原因、路线切换次数及覆盖指标。只打印 `global topology enabled` 或渲染全局图不算复用成功。

P0 先测得时间、成功率与规划时延分布，再设置非劣容差。不能在没有基线时编造性能提升百分比，也不能把几个 helper self-test 通过等同于其他探索行为完全无回归。

## 12. 对两份旧方案的取舍

原《地图管理与探索全局拓扑复用升级方案》中，完整路线、来源专属前提、一次提交、世界生命周期和路线失败隔离仍然适用。固定有限任务域、旧传感器过滤和旧状态接管必须按当前代码重写。

另一个《Target Exploration 完整框架设计》提出路线势能、目标相关信息增益和独立策略类，适合作为后续算法方向；但其“路线仍必须通过旧 Bubble 挂接”的表达如果被实现成旧根节点硬依赖，会保留当前断点。已验证的全局前缀可以经过公共安全与 General 后端执行，Bubble 用于原路径或可选局部修复。

本版把完整路线执行提前到核心阶段，仅同步修改新来源的进度、状态与恢复契约；全体候选评分/信息增益和大规模搬类后置。这样每个阶段都有明确可复现的行为收益和回退边界。

## 13. 当前源码证据索引

以下行号以本次工作区为准；文件链接相对本文路径。

- E01：[GlobalMapRuntime 输入与更新](../src/Planner/general_planner/src/general_core/planner_runtime/global_map_runtime.cpp)，约 211–270 行。
- E02：[MapManager 原始 ROG 更新](../src/Planner/map_manager/include/map_manager/map_manager.hpp)，约 134–151 行；[当前 ROG 配置](../src/Planner/general_planner/config/exploration_rog_map.yaml)。
- E03：[LIO 点云更新与空树距离](../src/Planner/general_planner/src/general_core/exploration/exploration_utils/lidar_map/sim_lio.cpp)，约 20–116 行。
- E04：[LIO 任务域](../src/Planner/general_planner/include/general_core/exploration/exploration_utils/lidar_map/lidar_map.h)，约 120–165 行；[启动 workspace 兼容处理](../src/Planner/general_planner/Apps/planner_runtime_node_ros1.cpp) 中 `configureAutomaticTargetWorkspace()` 与动态 box 选择分支。
- E05：[Topo 稀疏 region 更新](../src/Planner/general_planner/src/general_core/exploration/exploration_utils/pointcloud_topo/topo_skeleton_graph.cpp)，约 101–125、771–855 行；[Frontier 有符号键](../src/Planner/general_planner/include/general_core/exploration/exploration_utils/frontier_manager/frontier_manager.h)。
- E06：[Target 安全查询](../src/Planner/general_planner/src/general_core/exploration/highspeed/general_planner_adapter.cpp)，约 3474–3554 行。
- E07：[Target 状态协议](../src/Planner/general_planner/docs/target_exploration_status.md)；[状态投影器](../src/Planner/general_planner/include/general_core/planner_runtime/target_exploration_status.hpp)。
- E08：[组合运行时队列与共享注入](../src/Planner/general_planner/Apps/planner_runtime_node_ros1.cpp) 中 `main()`、`initPlanModules()` 与各 `AsyncSpinner`。
- E09：[源码 runtime launch](../src/Planner/task_planner/launch/planner_runtime.launch)，约 45–85、145–183 行；[发布入口](../general_planner_release/src/general_planner_release/launch/planner_runtime.launch)。Unity 入口位于工作空间 `src/unity_planner_bridge/launch/unity_planner_sim.launch`。
- E10：[全局路线查询](../src/Planner/general_planner/src/general_core/exploration/highspeed/target_topology_guidance.cpp)；[路线结果与 anchor 排序](../src/Planner/general_planner/include/general_core/exploration/highspeed/target_topology_guidance.h)。
- E11：[前缀点消费](../src/Planner/general_planner/src/general_core/exploration/highspeed/fast_exploration_manager.cpp)，约 929–1012 行。
- E12：[旧图代价与候选 preflight](../src/Planner/general_planner/src/general_core/exploration/highspeed/fast_exploration_manager.cpp)，约 673–690、1105–1198 行。
- E13：[FSM PLAN_TRAJ](../src/Planner/general_planner/src/general_core/exploration/highspeed/fast_exploration_fsm.cpp)，约 211–245 行；[局部执行入口](../src/Planner/general_planner/src/general_core/exploration/highspeed/fsm_utils.cpp)，约 509–550 行。
- E14：[进度硬门槛](../src/Planner/general_planner/src/general_core/exploration/highspeed/fast_exploration_manager.cpp)，约 1503–1535、3500–3565 行；[target 参数](../src/Planner/general_planner/config/target_exploration.yaml)。
- E15：[探索状态上报](../src/Planner/general_planner/src/general_core/exploration/highspeed/fsm_utils.cpp)，约 1134–1163 行；[Supervisor 状态处理与超时](../src/Planner/general_planner/src/general_core/planner_runtime/planner_supervisor.cpp)，约 1007–1020、1648–1665 行；[宽限默认值](../src/Planner/general_planner/include/general_core/planner_runtime/planner_supervisor.hpp)，约 207–208 行。
- E16：[MapManager 查询同步行为](../src/Planner/map_manager/include/map_manager/map_manager.hpp)，约 281–310、363–398 行。
- E17：[持久图路径搜索](../src/Planner/map_manager/src/map_manager/incremental_topology_graph.cpp)，约 1906–2105 行；[SearchSnapshot 结构](../src/Planner/map_manager/include/map_manager/incremental_topology_graph.hpp)，约 216–226 行。
- E18：[State2state 路线状态与几何工具](../src/Planner/general_planner/include/general_core/state2state/state2state_topology_route.hpp)；[局部路线执行](../src/Planner/general_planner/src/general_core/state2state/state2state_frontend.cpp)，约 314–676 行。
- E19：[轨迹最终检查与 commit](../src/Planner/general_planner/src/general_core/exploration/highspeed/general_planner_adapter.cpp)，约 2603–2655 行。
- E20：[Sparse domain 测试](../src/Planner/general_planner/tests/target_sparse_domain_self_test.cpp)；[独立启动 smoke 测试](../src/Planner/general_planner/tests/target_sparse_runtime_smoke_test.py)；[合成走廊测试](../src/Planner/general_planner/tests/target_sparse_corridor_integration.py)。
