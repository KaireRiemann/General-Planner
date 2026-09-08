# 全局拓扑起点观测恢复（2026-09-08）

## 故障与边界

`runtime_20260908_070624_0.bag` 中 state2state 导航反复在起点证据检查
失败，并非优化器死锁。该片段缺少此前约 408 秒的地图历史，不能用从空图
回放该片段证明历史体素状态，更不能声称已完成原场景闭环验证。

## 修改

- ROG 对通过最小回波距离筛选的有效回波，从当前射线原点开始融合 miss，
  不再把自由视线截断到 0.5 m 之外。继续保留每帧体素去重、回波端点保护、
  正常 hit/miss 概率融合。只有有效回波才产生自由证据。
- 移除第一次更新时 `missPointUpdate(..., 999)` 强制清球。无回波、只有
  odom 移动以及未观测到的邻近空间均不产生空闲证据。
- 严格拓扑的 PlanFromRest 保留真实连续起点，不再投影到任意非占据体素；
  前端不再先通过 UNKNOWN_AS_FREE 的 escape 改写起点。
- 起点实时证据检查前置于图查询、连接器及候选锚点遍历，避免确定失败时
  重复搜索所有候选。UNKNOWN 等待最多 10 秒（steady clock）；等待不扣除
  FSM 失败次数，超时后恢复现有有限失败机制。新任务/策略/世界版本重置
  等待计时，普通 map revision 增加不会延长等待。
- 日志区分 WAITING_FOR_OBSERVATION、OBSERVATION_TIMEOUT、OCCUPIED，输出
  起点坐标、偏移后坐标、raw/inflated/effective evidence 和 map revision。
- 局部膨胀占据优先于历史全局 FREE，防止 UNKNOWN 分支绕过安全膨胀约束。

等待不使用 sleep、不持锁轮询地图；复用原有 FSM backoff 和任务取消机制。
没有增加局部宽松 fallback，没有把整个盲区标成 FREE，也没有放宽全局未知
空间的通行规则。真实占据下仍应安全阻塞，不自动执行未经验证的“脱困”。

## 传感器契约

上述射线模型延续现有接口：传入 pose 的位置是点云的射线原点，点云已转换
到同一世界坐标系。有效回波意味着原点到回波之间的视线；min range 是
回波有效性门限，而非对所有有效视线制造一个未知球。
如果实际传感器有显著机体外参，必须由输入层提供正确的传感器原点，不能
把机体中心无条件当成相机光心。对真正遮挡/没有回波的区域，不做自由推断。

## 验证

本次在 ROS1 Noetic 容器中完成 `planner_runtime_node` 构建。以下检查通过：

- `observed_ray_fusion_self_test`（使用 exploration_rog_map.yaml）
- `state2state_topology_route_self_test`
- `raycaster_self_test`（60000 组射线）
- `incremental_topology_self_test`（新链接的 map_manager）
- `state2state_planning_control_self_test`
- `runtime_startup_smoke_test.py`（独立 master、真实 PlannerStatus、进程存活）

测试 master 已退出；未向用户当前 ROS master 或仿真发送导航指令。

扩展 observed_ray_fusion_self_test：空扫描启动不清球、移动后原地有效观测
清除原点 UNKNOWN、近距离视线连续、未观测邻域不被清空、端点仍占据，
同时保留动态障碍离开后的清除回归。

扩展 state2state_topology_route_self_test：等待期限有限、不因重试延期、
等待不消耗失败预算而超时会消耗。还应执行 raycaster、incremental topology
以及独立 ROS master 的 runtime_startup_smoke_test。

实机/仿真验收需重新启动新二进制，记录从建图启动到长距离导航结束的完整
bag；检查有效观测后起点转为 KNOWN_FREE、连接器恢复、轨迹发布，并测试
近障碍阻塞、无点云超时、新目标替换和 PAUSE。单元测试和无传感器启动
测试不替代此闭环验收。不要在运行旧进程时认为磁盘重编译已更新其代码。
