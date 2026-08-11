# 统一规划运行时、控制权切换与全局地图/拓扑图设计

> 状态：M1、M2 已实施；M3 的 exploration 全局路线消费仍为后续项。\
> 范围：ROS1 Noetic 下的 HighSpeedExp exploration、state2state、ROG map、LIO map 和增量 topo 图。\
> 目标：上层只做“执行什么任务”的决策；底层保证模式切换安全，并持续维护所有任务共享的全局地图和 topo 图。\
>\
> **M1 进度（已落地）**：`planner_runtime_node`（Supervisor + Gateway）、`/planner/status`、\
> `/planner/mode_request(_text)`、悬停切换、`task_epoch`、导航 `PAUSE/ARM/CLEAR`、\
> launch：`task_planner/launch/planner_runtime.launch`。\
> **M2 进度（已落地）**：同一进程中的 `GlobalMapRuntime` 现在是唯一 cloud/odom\
> 融合入口和唯一 `ROGMapROS`/`MapManager`/`BoundaryMap`/global topo 所有者；\
> exploration 与 state2state 均注入同一 `MapManager::Ptr`。全局 topo 默认采用\
> 真 3D、`dense_known_free` 的增量路网，并发布 `/planner/global_topology`。

## 1. 需求与结论

系统需要支持如下连续任务，而不重启规划栈、不丢失全局地图：

```text
探索区域 A
  -> 稳定悬停
  -> 点到点导航至 B
  -> 稳定悬停
  -> 探索区域 C
  -> ...
```

“何时探索、何时点到点导航”由上层决策器决定。规划层不做任务编排决策，只接受模式/任务请求并保证：

1. 同一时刻只有一个规划器拥有最终飞行命令控制权；
2. 任意模式切换都必须先受控减速并确认稳定悬停；
3. 任务切换不复用上一任务的轨迹、目标和异步规划结果；
4. ROG map、LIO map 和增量 topo 图是跨任务常驻的世界模型，不因任务切换清空；
5. 所有模式把新观测贡献给同一份全局 topo 图；M2 中 state2state 已读取它做长距引导，
   exploration 仍保留专用 Bubble/frontier 图，后续 M3 再把全局 route 作为其跨区域代价/骨架。

核心结论是：

> 不应让每个模式各自拥有一份“全局地图/拓扑图”。应由一个常驻的全局地图运行时拥有它们，所有模式只作为读者和局部规划消费者。

## 2. 当前实现与问题定位

### 2.1 已具备的基础能力

`map_manager` 已经包含长期 topo 图所需的主要结构：

- `MapManager` 持有 `ROGMapROS`、`BoundaryMap` 和 `IncrementalTopologyGraph`；
- `BoundaryMap` 在 ROG 滑动窗口移出旧区域后，保留全局已知空间的稀疏边界记忆；
- `IncrementalTopologyGraph` 接收 ROG 状态变化；region 首次完整抽样，提交后只按新
  证据增量补点，不再覆盖已经保存的节点；
- 查询侧使用不可变 `SearchSnapshot`，规划线程可以安全读取图快照；
- `MapManager` 有 `mapRevision()`；拓扑图 snapshot/statistics 有独立 `revision`；
- state2state 已可以通过 `findTopologyPath()` 做 topo A* 查询，当前部分配置仅构图/可视化，未默认启用查询。

相关实现：

- `src/Planner/map_manager/include/map_manager/map_manager.hpp`
- `src/Planner/map_manager/include/map_manager/incremental_topology_graph.hpp`
- `src/Planner/map_manager/src/map_manager/topology_graph_ros1.cpp`
- `src/Planner/general_planner/src/general_core/state2state/state2state_frontend.cpp`

HighSpeedExp 也已经有探索结束后的 `PAUSING -> PAUSED` 受控交接路径，并在暂停后继续接收 cloud/odom 用于地图和前沿维护。该能力可作为统一运行时中 exploration adapter 的基础，但它当前只处理探索自身的命令释放。

### 2.2 当前的关键矛盾

当前 state2state 与 HighSpeedExp 分别创建地图对象：

```text
fsm_node / state2state
  -> ROGMapROS #1
  -> MapManager #1
  -> IncrementalTopologyGraph #1

exploration_node / HighSpeedExp
  -> LIO map #1
  -> ROGMapROS #2
  -> MapManager #2
  -> exploration TopoGraph / BubbleGraph #2
```

即使它们订阅相同的 cloud、odom 和使用相同 yaml，也不是同一份地图：地图更新顺序、局部滑窗边界、回调时刻、状态变化 drain 和 topo revision 都可能不同。

更重要的是，`MapManager::setMap()` 会在其 `ROGMapROS` 上注册状态变化和机器人状态回调。多个 `MapManager` 不能分别管理同一份 ROG 对象；否则后注册者会覆盖先前回调，`drainStateChanges()` 也会使消费者互相抢占状态变化流。

因此全局地图的硬约束是：

```text
一个世界坐标系 / 一个 world_epoch
一个 ROGMapROS
一个 MapManager
一个 BoundaryMap
一个 IncrementalTopologyGraph
一个传感器融合入口
任意数量的模式适配器（只读/请求更新优先级）
```

### 2.3 M2 已落地的边界

M2 已把地图所有权和持续构图从任务模式中剥离，但没有把任务专属算法混为一体：

- `GlobalMapRuntime`（`general_core/planner_runtime/global_map_runtime.*`）订阅一次原始
  odom 和一次 cloud+odom 同步对；一帧被接受后依次更新 LIO、ROG、`MapManager`，随后才通知
  exploration adapter。`rog_map/ros_callback/enable` 被强制要求为 `false`，因此不存在第二条
  ROG 融合回调；
- `FsmRos1` 的 state2state 实例使用被注入的 `MapManager`，不再创建 `ROGMapROS` 或
  `TopologyGraphROS1`；`FastPlannerManager` 同样使用该指针，且 exploration FSM 的传感器
  入口改为由 `GlobalMapRuntime` 调用；
- 全局维护器使用 `TopologyGraphROS1(..., "global_topology")`，但不传入
  `state2stateMode()` gate，因此 exploration、state2state、WAIT 和 hold 均持续维护；
- `/planner/status` 从该 runtime 读取实际 `world_epoch`、`map_revision`、`topo_revision`、
  `map_ready` 和 `topology_ready`，不再由模式状态伪造地图就绪；
- `planner_runtime.launch` 只启动一个组合式 `planner_runtime_node`，`serial_handover=false`。
  模式切换不会 kill/restart exploration 或 navigation，也不会重新创建地图。

### 2.3 为什么仅用一个 launch 不够

`roslaunch` 可启动多个节点，但多个可执行程序处于不同进程、不同地址空间，不能共享 `MapManager::Ptr` 或 `ROGMapROS::Ptr`。

因此以下方式不能实现真正共享地图：

```text
planner_runtime.launch
  -> exploration_node        # 进程 A，MapManager A
  -> fsm_node                # 进程 B，MapManager B
```

若继续多进程，只能将地图序列化为 ROS 消息或 map server 接口再传给规划器。这会带来复制、时延、版本不一致及高频查询开销，且现有规划算法本身需要 C++ 内存中的 `MapManager::Ptr`。

推荐方案是：`planner_runtime.launch` 启动一个 **`planner_runtime_node`**；该进程内部持有全局地图并实例化 exploration/state2state 的适配器。保留多个 ROS 节点并非不可能，但必须改为 nodelet 或完整的地图 RPC/快照协议，不是第一阶段应选择的路径。

## 3. 目标架构与所有权

### 3.1 总体数据流

```text
                        上层决策器
                            |
                    /planner/mode_request
                    /planner/task_request
                            |
                            v
 +---------------------------------------------------------------+
 |                    planner_runtime_node                       |
 |                                                               |
 |  +------------------+        +----------------------------+  |
 |  | PlannerSupervisor|------->| PlannerCommandGateway      |  |
 |  | - 模式切换       |        | - 唯一 /planning/pos_cmd   |  |
 |  | - 悬停确认       |        | - hold / source 放行       |  |
 |  | - task_epoch     |        +--------------+-------------+  |
 |  +-----+------------+                       |                |
 |        |                                    v                |
 |        |              /planner/state2state/pos_cmd           |
 |        |              /planner/exploration/pos_cmd           |
 |        v                                                     |
 |  +----------------------+                                    |
 |  | GlobalMapRuntime     |                                    |
 |  | - ROGMapROS          |<---- cloud + odom（唯一入口）      |
 |  | - MapManager         |                                    |
 |  | - BoundaryMap        |                                    |
 |  | - Global topo graph  |                                    |
 |  | - LIO map adapter    |                                    |
 |  +------+---------------+                                    |
 |         |                                                    |
 |     +---+-------------------------------+                    |
 |     |                                   |                    |
 |     v                                   v                    |
 | State2StateAdapter                ExplorationAdapter         |
 | - topo A* + local optimize        - frontier/TSP/Bubble 图    |
 | - fresh plan-from-rest            - global topo 作为长距引导  |
 +---------------------------------------------------------------+
                            |
                            v
                    /planner/status（唯一状态出口）
```

### 3.2 世界模型与任务模型分离

| 数据 | 所有者 | 生命周期 | 任务切换时处理 |
|---|---|---|---|
| ROG 局部地图/ESDF | `GlobalMapRuntime` | world 生命周期 | 保留 |
| BoundaryMap 全局记忆 | `MapManager` | world 生命周期 | 保留 |
| IncrementalTopologyGraph | `MapManager` | world 生命周期 | 保留、继续增量更新 |
| LIO 点云地图 | `GlobalMapRuntime` | world 生命周期 | 保留 |
| exploration Bubble/历史图 | `ExplorationAdapter` | 可跨探索任务保留 | 不作为唯一全局路由依据 |
| frontier、TSP tour、当前 view point | exploration 任务态 | task 生命周期 | 清空 |
| state2state goal、候选路径、重规划状态 | state2state 任务态 | task 生命周期 | 清空 |
| 已提交轨迹、待发布轨迹、异步优化结果 | execution 任务态 | task 生命周期 | 清空/使 epoch 失效 |

地图是“环境事实”，轨迹和目标是“任务意图”。任务切换仅清除后者，不清除前者。

### 3.3 建议的运行时上下文

为避免把任务状态塞入 `MapManager`，新增一个上层容器：

```cpp
struct GlobalMapContext {
  rog_map::ROGMapROS::Ptr rog_map;
  general_planner::MapManager::Ptr map_manager;
  fast_planner::LIOInterface::Ptr lio_map;

  std::atomic<std::uint64_t> world_epoch{0};
  std::atomic<std::uint64_t> sensor_revision{0};
};
```

约束：

- `GlobalMapRuntime` 是唯一创建和更新 `ROGMapROS` 的组件；
- 只有它可以调用 `MapManager::setMap()`、`configureTopology()`、`setTopologyActive()` 和 world reset；
- state2state 与 exploration 通过依赖注入拿到相同的 `MapManager::Ptr`；
- 任意模式只能调用只读查询或 `requestTopologyUpdateAround()`，不能重配或清图；
- 已接受的一帧 cloud + odom 只能被融合一次。

## 4. 全局 ROG map 与增量 topo 图

### 4.1 全局地图更新流水线

地图维护必须与当前 planner mode 解耦。无论在 exploration、state2state、WAIT 还是稳定悬停状态，传感器更新都持续进入如下流水线：

```text
接收并校验 cloud + odom
  -> 检查时间戳、坐标系和 odom 新鲜度
  -> 更新一次 LIO map
  -> 更新一次 ROGMapROS
  -> MapManager.map_revision++
  -> 同步 ROG 离散状态变化到 BoundaryMap
  -> 将变化体素映射为 global topo 脏 region / dense evidence
  -> GlobalTopologyMaintainer 按预算更新 N 个 region
  -> 发布不可变 SearchSnapshot（topo_revision++）
```

`MapManager` 当前的状态变化回调、`BoundaryMap` 和 `IncrementalTopologyGraph::markDirty*()` 已经提供了主要能力。需要将维护定时器从 state2state FSM 中移出，变为 `GlobalTopologyMaintainer` 的常驻 worker。

### 4.2 global topo 图的语义

全局 topo 图是地图驱动的自由空间连通性图，不是历史规划折线集合：

- 节点只能由传感器证实的 `KNOWN_FREE` 空间生成；
- 边必须经当前地图可通行性验证；
- ROG 滑窗外区域通过 `BoundaryMap` 的持久证据参与查询；
- 新障碍、占据变化只增量补充未覆盖自由空间或失效明确占据的节点/边，不重抽样已提交图；
- 规划路径最多用于“优先更新哪些 region”，不能被当作自由空间证据写回；
- topo A* 输出仅是全局引导，最终可执行轨迹仍须由局部 planner 和碰撞检查负责。

对当前单架无人机，建议第一阶段使用一个统一、安全的 topology 配置：

```yaml
global_topology:
  enabled: true
  construction_mode: dense_known_free
  planar_mode: false
  unknown_as_free: false
  sample_spacing: 0.45
  snapshot_every_update: false
  min_clearance: <至少机体半径；边仍使用膨胀地图验证>
  max_regions_per_update: 4
  update_period: 0.05
  publish_period: 0.50
```

M2 的默认 `global_topology.yaml` 是此配置的具体版本。`dense_known_free` 不是把每个 ROG
体素复制一份：它在全局对齐的 0.45 m lattice 上，为每个传感器确认的 `KNOWN_FREE` 样本保留
节点，3D 模式保留 collision-validated 的 26 邻接连接。它因此是**真实 3D 的稠密自由空间
路网**，但不是“完整占据栅格”的替代品：地图真相仍属于 ROG + BoundaryMap，未知空间不会被
提升为 free，最终轨迹仍必须在当前局部 ROG 上重验证。

为控制全局图增长的实时性，脏 region 默认按 10 Hz 更新（避免稠密 3D region 重建超过 Noetic
目标机的 50 ms 周期），而 immutable SearchSnapshot 按
`publish_period`（默认 2 Hz）复制发布。首次传感器更新会优先处理实际 changed box，不会把
整个 72 m 滚动窗口的未观测 region 都排队重建。

不要在切换 exploration/state2state 时改变 `min_clearance`、`planar_mode` 或 construction mode；当前 `configureTopology()` 的语义会重置/重建拓扑，这是任务切换时不可接受的。

若所有任务都在固定飞行高度带，使用 2.5D `planar_mode`。若需要楼层间、竖直穿越或明显不同高度的长距离路线，应在 world 启动时选择真正的 3D 全局图；不能按 planner mode 在 2.5D 和 3D 间切换。

### 4.3 全局图由谁维护

当前 `TopologyGraphROS1` 通过 `missionActive()` 控制拓扑活跃状态，state2state 传入的条件是 `state2stateMode()`。这与“所有模式持续构图”的目标冲突。

实施后应改为：

```text
旧语义：state2state 任务拥有 topo 图，进入 exploration 即停止维护
新语义：GlobalMapRuntime 拥有 topo 图，只要 world 有效就持续维护
```

建议新增 `GlobalTopologyMaintainer`，复用 `TopologyGraphROS1` 的异步 worker 和 Marker 发布代码，但不再接受任务模式 callback。其 active 条件仅由以下因素决定：

```text
global topology enabled
AND ROG map ready
AND odom/world frame valid
AND world 没有 reset/fault
```

## 5. 各模式如何消费同一全局 topo 图

### 5.1 state2state

```text
实时稳定起点 + 目标点
  -> 获取 GlobalMapContext 中 topo SearchSnapshot
  -> global topo A*
  -> 得到全局 guide path
  -> local frontend / corridor / trajectory optimization
  -> 对候选轨迹使用当前 ROG map 再做碰撞检查
  -> 通过 command gateway 提交
```

state2state 每次规划都应记录使用的 `map_revision` 和 `topo_revision`。如果轨迹提交前其穿越区域发生地图变化，必须重新验证；不安全则重新规划或保持悬停。

### 5.2 exploration

探索仍有自己的前沿、视点、TSP 和 Bubble 图，这是合理的，因为它们是任务特有结构。改变点是：

```text
当前位姿 + frontier/viewpoint
  -> global topo A*：判断跨区域可达、提供长距路由代价/骨架
  -> Bubble/local topology：在局部感知范围选择可执行通路
  -> frontier/TSP：选择下一目标
  -> trajectory optimizer：生成安全探索轨迹
```

全局 topo 找不到 route 时不得伪造直线连接；应将候选记为当前不可达、等待新观测或选择其他前沿。探索的 Bubble 图可以继续服务局部连通性和前沿规划，但不能取代长期全局导航图。

### 5.3 其他模式与 WAIT

tracking、perching、takeoff 和 WAIT 状态至少都应：

- 让 `GlobalMapRuntime` 持续融合传感器；
- 对自身附近调用 `requestTopologyUpdateAround()` 提高更新优先级；
- 使用同一 ROG map 做碰撞/净空检查；
- 不清空或重配置全局 topo 图。

它们不一定需要执行 topo A*，但其飞行期间获得的新环境证据必须贡献给同一张图。

## 6. 上层控制接口：仅话题，不使用 service

模式切换是异步过程，因此不用 service 是可行的；但不能只依赖无编号的字符串作为正式协议。推荐“命令话题 + 状态确认话题”：

```text
/planner/mode_request      PlannerModeRequest    非 latched
/planner/task_request      PlannerTaskRequest    非 latched
/planner/status            PlannerStatus         latched，10 Hz
```

### 6.1 模式请求

```text
# PlannerModeRequest.msg
std_msgs/Header header
uint64 request_id
string task_id
uint8 mode              # HOLD / STATE2STATE / EXPLORATION / ...
```

上层可持续重发同一个 `request_id`，直到在 `/planner/status` 中看到该请求已被接受。`/planner/mode_request` **不能 latched**：运行时重启后自动重播旧导航/探索命令是不安全的。

为了命令行调试，可提供兼容话题：

```text
/planner/mode_request_text    std_msgs/String
```

仅接受 `exploration`、`state2state`、`hold`、`emergency_stop` 等文本，并由 `PlannerSupervisor` 转换为带编号的内部请求。该字符串话题不能直接连接现有 `fsm/task_mode_topic`，因为后者只会立即切换 state2state 内部模式，并不知道另一个规划器、命令网关和悬停交接。

### 6.2 任务请求

模式与任务内容分开：

- `STATE2STATE` 激活后进入 `S2S_WAIT_GOAL`；还需要点到点目标才会规划。
- `EXPLORATION` 激活后进入 `EXP_WAIT_TRIGGER`；还需要探索区域、ROI 或 trigger 才会开始。

可以先保留两类任务输入，再由 supervisor gate：

```text
/planner/navigation/goal         geometry_msgs/PoseStamped
/planner/exploration/trigger     geometry_msgs/PoseStamped 或区域定义
```

正式统一接口可使用 `PlannerTaskRequest.msg`，其中含 `request_id`、`task_id`、目标 mode 和 mode-specific payload。无论采用哪种形式，Supervisor 都只在 `ready_for_new_task=true` 时向目标 adapter 下发；过早到达的任务可缓存一份最新请求，模式再次变化时立即废弃。

### 6.3 统一状态

`PlannerStatus.msg` 建议字段：

```text
std_msgs/Header header
uint64 transition_id
uint64 task_epoch
uint64 world_epoch
uint64 map_revision
uint64 topo_revision

string task_id
uint8 active_mode
uint8 requested_mode
uint8 phase
uint16 mode_state
uint8 task_result

bool stable_hover
bool ready_for_new_task
bool odom_valid
bool map_ready
bool topology_ready
uint8 command_owner
float32 speed_mps
string reason
```

建议的统一 `phase`：

```text
BOOT
WAITING_INPUT
PLANNING
EXECUTING
BRAKING
HOLD_VERIFY
STABLE_HOLD
FAILED
EMERGENCY
```

`mode_state` 保留模式原生细节，例如：

```text
S2S_WAIT_GOAL / S2S_GENERATE_TRAJ / S2S_FOLLOW_TRAJ
EXP_WAIT_TRIGGER / EXP_PLAN_TRAJ / EXP_EXEC_TRAJ / EXP_REORIENT
```

上层只能依据 `phase`、`task_result`、`stable_hover`、`ready_for_new_task` 做控制决定；`mode_state` 用于诊断和可视化。

## 7. 飞行控制权与安全切换

### 7.1 唯一最终命令发布者

`PlannerCommandGateway` 必须是唯一向飞控发布 `/planning/pos_cmd` 的节点：

```text
state2state command     -> /planner/state2state/pos_cmd
exploration command     -> /planner/exploration/pos_cmd
gateway output           -> /planning/pos_cmd
controller               <- /planning/pos_cmd
```

目前的 `mission_command_mux.cpp` 可以作为起点，但不能直接作为最终实现：它按模式立即切源，缺少刹停/稳定确认；并且它每帧使用当前 odom 生成 hold 点，会导致 hold 点漂移，不是真正的位置保持。

正确的 gateway 必须在进入 hold 时锁定一次锚点：

```text
hold_anchor = {position, yaw} at controlled-stop completion
```

随后持续发布该固定位置、零速度、零加速度的 hold command，直到新的目标轨迹被安全放行。

### 7.2 模式切换状态机

以上层从 exploration 切到 state2state 为例：

```text
mode_request(STATE2STATE)
  -> Supervisor 记录 transition_id / requested_mode
  -> phase=BRAKING；exploration 停止接受新 frontier 与新规划
  -> exploration adapter 执行受控 stop
  -> gateway 保持当前源，等待其减速结束
  -> gateway 锁定 hold_anchor，command_owner=HOLD
  -> phase=HOLD_VERIFY
  -> 连续检查 odom 有效、速度/偏航角速度均低于阈值
  -> phase=STABLE_HOLD, stable_hover=true
  -> task_epoch++；清理 exploration 任务态
  -> 初始化 state2state adapter
  -> active_mode=STATE2STATE, phase=WAITING_INPUT
  -> ready_for_new_task=true
```

建议初始阈值（应通过实机调参验证）：

```text
odom age               <= 0.10 ~ 0.20 s
|velocity|             <= 0.10 m/s
|yaw rate|             <= 0.10 rad/s
连续满足时间           >= 0.5 s
```

反向的 `state2state -> exploration` 使用相同状态机。任务逻辑完成不等于可切换：只有 `STABLE_HOLD` 才表示上层可以安全下发下一任务。

## 8. 防止旧规划数据重新生效

任务切换仅靠“停止发布”不够。异步优化、重规划或 ROS 回调可能在切换后迟到，因此需要 `task_epoch`。

### 8.1 规则

1. 每次开始切换立即递增或预留新的 `task_epoch`；
2. 任何 plan、replan、轨迹优化、轨迹发布任务启动时捕获 epoch；
3. 计算结束、准备 commit、准备 publish 前再次检查 epoch；
4. epoch 不一致则无条件丢弃，不能修改已提交轨迹，也不能发布命令；
5. gateway 只接收当前获授权 source 的新鲜命令；这是第二道防线，不替代 epoch 检查；
6. 新模式的第一条轨迹必须从稳定悬停的实时 `(p, v=0, a=0, yaw)` 做 `planFromRest`。

### 8.2 必须清除的任务态

切换时，adapter 至少应清理：

```text
state2state:
  goal、候选 guide path、当前/备份轨迹、replan in progress、失败计数、pending commit

exploration:
  active frontier、global tour、path_next_goal、TSP result、coverage 当前目标、finish gate、pending trajectory

shared execution:
  trajectory id、queued poly trajectory、trajectory server 的未生效输入
```

不要清理：ROG、BoundaryMap、全局 topo、LIO map 和已验证的地图版本历史。

## 9. 代码结构与实施项

### 9.1 新增接口

```text
src/Planner/general_planner/include/general_core/planner_status.hpp
  - C++ 内部枚举、PlannerStatusData、原生状态映射
  - 不能替代 ROS msg

src/Planner/general_planner/msg/PlannerStatus.msg
src/Planner/general_planner/msg/PlannerModeRequest.msg
src/Planner/general_planner/msg/PlannerTaskRequest.msg

src/Planner/general_planner/include/general_core/global_map_context.hpp
src/Planner/general_planner/src/general_core/global_map_runtime.cpp
src/Planner/general_planner/src/general_core/planner_supervisor.cpp
src/Planner/general_planner/src/general_core/planner_command_gateway.cpp
src/Planner/general_planner/Apps/planner_runtime_node_ros1.cpp
src/Planner/task_planner/launch/planner_runtime.launch
```

需要在 `general_planner/CMakeLists.txt` 中加入 ROS message generation，并在 `package.xml` 中声明 `message_generation` 与 `message_runtime`。若出于兼容原因暂时不新建生成消息，可使用版本化 `std_msgs` 封装作为过渡，但不应长期使用裸字符串解析正式状态。

### 9.2 MapManager 改造

| 位置 | 当前行为 | 目标改造 |
|---|---|---|
| `MapManager::setMap()` | 单 planner 私有初始化 | 仅 `GlobalMapRuntime` 调用一次 |
| `configureTopology()` | `GeneralPlanner` 构造时配置 | 只在 global runtime 启动时配置一次 |
| `setTopologyActive()` | 随 state2state mission gate 开关 | world 有效期间恒为 true |
| `TopologyGraphROS1` | 挂在 state2state FSM | 移为全局 topo maintainer |
| `GeneralPlanner` 构造 | 内部创建 `MapManager` | 支持注入共享 `MapManager::Ptr` |
| `FastPlannerManager::initPlanModules` | 内部创建 ROG/MapManager | 支持注入 `GlobalMapContext` |
| cloud/odom callback | 多个 planner 分别融合 | GlobalMapRuntime 单次融合 |

### 9.3 mode adapter 边界

两个 adapter 应至少提供一致的控制接口：

```cpp
class PlannerModeAdapter {
public:
  virtual void requestStart(const TaskRequest&, uint64_t task_epoch) = 0;
  virtual void requestControlledStop(uint64_t task_epoch) = 0;
  virtual void resetTaskState(uint64_t next_task_epoch) = 0;
  virtual bool hasReachedSafeStop() const = 0;
  virtual PlannerStatusData status() const = 0;
};
```

该接口是运行时内部 C++ 接口，不是上层 ROS API。exploration adapter 可以复用现有 `PAUSING/PAUSED`；state2state adapter 必须新增同等的 cancel、轨迹失效和静止重启能力。

### 9.4 launch 目标

最终不再要求用户先后启动两个 launch：

```bash
roslaunch task_planner planner_runtime.launch \
  marsim:=false \
  initial_mode:=exploration
```

`initial_mode` 是启动后的期望模式，不得绕过 odom/地图/悬停安全检查。若 exploration 区域来自 RViz 或上层任务，启动后状态应是：

```text
active_mode=EXPLORATION
phase=WAITING_INPUT
mode_state=EXP_WAIT_TRIGGER
```

这不是失败，而是等待探索区域。`STATE2STATE` 同理进入 `S2S_WAIT_GOAL`，收到目标后才开始点到点规划。

## 10. 实施顺序

### M0：保持现有交接兼容

- 保留当前 exploration finish 后 `PAUSING -> PAUSED` 和 trajectory server output gate；
- 修复/验证任何模式都不会在 exploration pause 后继续向最终控制话题写命令；
- 不在此阶段重构地图所有权。

验收：探索完成后 state2state 可启动，控制输出不冲突。

### M1：统一命令与状态运行时

- 添加 `PlannerStatus`、mode request 话题和 `PlannerSupervisor`；
- 将最终 `/planning/pos_cmd` 收敛到 gateway；
- 引入 `BRAKING -> HOLD_VERIFY -> STABLE_HOLD`；
- 两个模式输出改为内部 topic；
- 引入 `task_epoch` 并完成轨迹 stale-result 丢弃。

验收：任意方向切换都必须先稳定悬停；注入迟到轨迹不能穿透 gateway 或重新提交。

### M2：统一 MapManager 所有权

- 抽取 `GlobalMapContext` 和 `GlobalMapRuntime`；
- 将 state2state、HighSpeedExp 改为依赖注入同一 `MapManager::Ptr`；
- 传感器融合只执行一次；
- 移除 topology 对 `state2stateMode()` 的任务 gate；
- 将 topo worker 迁移到 global runtime。

验收：任务切换前后 `world_epoch` 不变，`map_revision/topo_revision` 单调递增；两模式查询到同一 topo snapshot revision。

**实施状态：已完成。** 具体配置位于
`general_planner/config/global_topology.yaml`，组合入口位于
`general_planner/Apps/planner_runtime_node_ros1.cpp`。M2 验收还应确认启动后只存在一个
`planner_runtime_node`（而不是额外的 `exploration_node`/`fsm_node`），并且全局 topo 可视化
话题为 `/planner/global_topology`。

### M3：全局 topo 路由接入

- state2state 将 topo A* 作为长距离 guide path；
- exploration 在 frontier 代价、跨区域连接中读取同一全局图；
- 所有局部 planner 对 guide path 和最终轨迹做当前地图重验证；
- 建立 topo revision 与规划提交的关联日志。

验收：探索建立的拓扑可直接被随后 state2state 长距离导航使用；点到点飞行的新传感器证据也会提高后续探索的连通性判断。

### M4：跨进程持久化（可选）

当前目标是“同一次运行中的跨任务保持”。若需要进程重启后恢复，则应把以下资产作为同一版本原子保存/加载：

```text
BoundaryMap + topology snapshot + topology configuration + world frame metadata
```

只保存 topo 节点/边而不保存对应占据证据不安全；加载后还必须对当前局部窗口和接入边重新验证。

## 11. 测试与验收矩阵

| 场景 | 应观察到的结果 |
|---|---|
| exploration 完成后请求 state2state | 状态依次经过 `BRAKING/HOLD_VERIFY/STABLE_HOLD/S2S_WAIT_GOAL` |
| state2state 到点后请求 exploration | 先悬停，再进入 `EXP_WAIT_TRIGGER` 或执行新 ROI |
| 飞行中立刻请求切换 | 新 mode 不会在速度未降到阈值前开始规划/放行命令 |
| 切换后注入旧 planner 的迟到轨迹 | epoch 检查或 gateway 拒绝，最终 `/planning/pos_cmd` 不受影响 |
| exploration 和 state2state 连续运行 | `world_epoch` 不变；map/topo revision 连续增长 |
| state2state 使用 exploration 后的路线 | 同一 topo revision 可用于 global A*，局部轨迹重新验证成功 |
| ROG 地图发生新占据变化 | topo 脏 region 重建；旧 guide/轨迹在提交前被重新验证 |
| world frame 重定位/地图 reset | `world_epoch` 递增，旧 topo/轨迹/任务全部失效并进入安全 hold |

建议在 ROS1 Noetic Docker 中建立以下自动化测试：

1. command gateway source/hold 路由测试；
2. 稳定悬停判定测试（fresh/stale odom、不同速度序列）；
3. task epoch 延迟回调丢弃测试；
4. MapManager 单一 owner 与单次 cloud 融合测试；
5. exploration/state2state 对同一 topo snapshot 的一致性测试；
6. topo A* 长距离查询与地图变化后的边失效测试；
7. `planner_runtime.launch` XML、参数和节点存活测试。

## 12. 风险与边界

- 全局 topo 增长后，当前 nearest-node 查询仍可能线性扫描全图；大范围长期任务需要在 immutable snapshot 中增加 KD-tree/空间索引。
- `BoundaryMap` 是稀疏全局记忆，不等价于无限分辨率的完整体素地图；全局 route 永远是引导，不可跳过局部碰撞验证。
- 地图坐标系必须稳定。LIO 重定位、原点跳变和手动 reset 是 world 级事件，不是普通任务切换。
- 如果不同模式要求不同净空，不应切换 global topology configuration。第一阶段使用统一保守净空；后续可为节点/边记录更细净空信息，并在查询时按模式过滤。
- exploration 的局部 Bubble 图与 global topo 图可并存，但只有全局 `MapManager` topo 图承担跨模式、长距离、跨任务的导航记忆职责。

## 13. 最终运行语义

完成上述改造后，上层的最小交互可以是：

```text
启动一次 planner_runtime.launch，initial_mode=exploration
  -> 探索任务执行
  -> 上层看到 /planner/status: STABLE_HOLD
  -> 发布 mode_request: STATE2STATE
  -> 上层看到 S2S_WAIT_GOAL / ready_for_new_task
  -> 发布点到点目标
  -> 到点后再次 STABLE_HOLD
  -> 发布 mode_request: EXPLORATION
  -> 发布下一探索区域或 trigger
```

在整个过程中：

- 上层决定任务；
- Supervisor 决定何时安全交权；
- Gateway 决定唯一最终命令来源；
- GlobalMapRuntime 持续维护全局地图；
- 每个模式都读取同一份全局 topo 图；
- 旧任务的轨迹和异步规划结果永远不能跨 `task_epoch` 生效。
