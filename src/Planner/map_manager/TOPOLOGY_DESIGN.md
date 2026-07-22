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
- `MapManager` adapter：局部使用 ROGMap 膨胀占据，历史区域使用 BoundaryMap；
  地图体素状态变化只标脏受影响区域。
- state2state：直连失败后先请求拓扑全局引导；拓扑不可用、动态障碍阻断或图尚未
  扩展到目标时，回退原有局部 A*。拓扑结果仍经过局部安全检查、horizon 裁剪、
  corridor 和轨迹优化。
- 成功 guide path 调用 `observePlannedTopologyPath()`，只为沿途未观察区域播种增量
  构图任务。规划折线不会未经地图验证直接固化成安全边。

图节点和边在进程生命周期内保持全局累积。ROGMap 滑窗离开某区域不会删除其
BoundaryMap 证据或拓扑；新的传感器占据变化会使相关区域和穿越边重新验证。

## 与 USS-NAV 的语义对齐情况

| 能力 | 当前实现 | USS-NAV |
|---|---|---|
| 全局自由空间导航骨架 | 已对齐 | 多面体骨架 |
| 增量扩展与运行期记忆 | 已对齐 | 位姿附近扩展 |
| 规划查询参与全局引导 | 已对齐 | Skeleton A* |
| 局部规划器最终安全负责 | 已对齐 | EGO/A* |
| 地图变化后边重验证 | 更严格 | 以追加和回环为主 |
| 几何原语 | bubble 代表节点 | 多面体和 gate |
| area/object/frontier 语义挂载 | 未实现，且不属于 state2state 必需能力 | 已实现 |
| 跨进程 save/load | 未实现 | 已实现 |

因此，当前模块已经对齐普通 state2state 所需的“几何拓扑路由”语义，但不是完整的
USS-NAV SceneGraph 克隆。若要求跨任务重启仍保留记忆，需要把拓扑快照和
BoundaryMap 作为同一版本的地图资产原子保存/加载，并在加载后重新验证局部边；
只保存拓扑而不保存对应占据证据是不安全的。

