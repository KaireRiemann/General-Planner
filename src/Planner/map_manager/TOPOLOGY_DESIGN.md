# 增量拓扑地图设计

## USS-NAV 中拓扑图的实际职责

USS-NAV 的 `SceneGraph/SkeletonGenerator` 维护的是全局自由空间骨架，而不是
“规划结果折线集合”：

1. `updateSceneGraph()` 随机器人位姿调用 `doDenseCheckAndExpand()`；当前位置附近
   节点不够密时，从局部地图向外生成自由空间多面体。
2. 多面体 frontier 产生下一多面体及 gate；相邻节点通过原始地图路径验证后连边，
   同时补充邻近连接和 loopback。
3. 机器人挂载节点变化时会记录可达关系，因此图包含导航经历带来的连通记忆。
4. frontier、object 挂载到拓扑节点；远距离导航先在骨架上 A*，局部规划器再执行。
5. Scene graph 另有 save/load，可跨进程恢复多面体、边、区域和物体。

所以它是“地图驱动的几何骨架 + 位姿驱动的增量扩展 + 高层语义挂载”，不是由
exploration FSM 独占的数据结构；探索只是它的一个消费者和更新触发者。

## General Planner 中的分层

当前实现把普通规划需要的部分放入 `MapManager`：

- `TopologyMapView`：只定义可通行性和净空距离，不依赖 LIO、ROGMap 或 FSM。
- `IncrementalTopologyGraph`：脏区域内自适应细分，生成 clearance bubble，合并
  重叠 bubble，并为每个局部连通分量保留稳定代表节点。
- 对受窄高度带约束的 state2state 配置，`planar_mode` 将脏区压缩为 XY 区域，在
  ROG 实际高度边界的中心层采样，并只用水平射线计算 bubble 净空；虚拟上下边界
  仍参与节点合法性判断，所有边仍使用三维膨胀地图逐段验证。
- `MapManager` adapter：局部使用 ROGMap 膨胀占据，历史区域使用 BoundaryMap；
  地图体素状态变化只标脏受影响区域。
- ROG odom 更新完成滑窗后，只向 topology 提交机器人位置 focus；机器人移动超过
  半个 region 时标脏新的邻域。odom 回调中不做候选采样、净空射线或连边，实际
  重建仍由异步 worker 按预算完成。首次全窗口播种等待真实 odom 初始化地图原点，
  避免 `unknown_as_free` 在默认原点产生伪历史节点。
- `TopologyGraphROS1`：独立 worker 低频消费脏区域；ROS timer 只唤醒 worker 和
  发布快照，建图不运行在 state2state 重规划线程中。
- state2state 当前配置为 `topology.enable=true`、`query_enable=false`：拓扑只在
  后台构建、记录和可视化，前端不获取拓扑快照、不执行图搜索或可见性挂接，正常
  规划继续使用原有直连与局部 A*。图查询代码保留为显式实验能力，不是默认路径。
- 成功 guide path 调用 `observePlannedTopologyPath()`，只为沿途未观察区域播种增量
  构图任务。规划折线不会未经地图验证直接固化成安全边。

图节点和边在进程生命周期内保持全局累积。ROGMap 滑窗离开某区域不会删除其
BoundaryMap 证据或拓扑；新的传感器占据变化会使相关区域和穿越边重新验证。

ROS 状态 Marker 同时发布最近一次区域重建的 sampled/free/clearance_reject 计数
和累计 empty 区域数。`rev` 仅表示执行过的区域重建次数，不再被误解为成功生成
节点的次数。

## 与 exploration TopoGraph 的隔离

两套图没有共享节点、边、更新入口或规划查询：

- exploration 继续拥有 `exploration_utils/pointcloud_topo/TopoGraph`，由其
  `PlannerManager` 和 `LIOInterface` 更新，服务 frontier、viewpoint 和探索路径。
- `MapManager/IncrementalTopologyGraph` 只服务 state2state 的长期导航记忆。
- ROS adapter 通过 mission gate 管理运行期所有权：进入 exploration 时停止脏区域
  跟踪、后台构图和拓扑规划查询；返回 state2state 后重新标记当前局部窗口并恢复。
- state2state 默认不查询 topology，因此 15 Hz 重规划线程没有拓扑挂接、图搜索或
  路径重复验证开销。

因此，启用 state2state topology 不会改变 exploration TopoGraph 的建立过程，也
不会让 exploration 为这套图支付后台构图开销。

## 与 USS-NAV 的语义对齐情况

| 能力 | 当前实现 | USS-NAV |
|---|---|---|
| 全局自由空间导航骨架 | 已对齐 | 多面体骨架 |
| 增量扩展与运行期记忆 | 已对齐 | 位姿附近扩展 |
| 规划查询参与全局引导 | 能力保留，当前配置关闭 | Skeleton A* |
| 局部规划器最终安全负责 | 已对齐 | EGO/A* |
| 地图变化后边重验证 | 更严格 | 以追加和回环为主 |
| 几何原语 | bubble 代表节点 | 多面体和 gate |
| area/object/frontier 语义挂载 | 未实现，且不属于 state2state 必需能力 | 已实现 |
| 跨进程 save/load | 未实现 | 已实现 |

因此，当前模块已经对齐普通 state2state 所需的“几何拓扑路由”语义，但不是完整的
USS-NAV SceneGraph 克隆。若要求跨任务重启仍保留记忆，需要把拓扑快照和
BoundaryMap 作为同一版本的地图资产原子保存/加载，并在加载后重新验证局部边；
只保存拓扑而不保存对应占据证据是不安全的。
