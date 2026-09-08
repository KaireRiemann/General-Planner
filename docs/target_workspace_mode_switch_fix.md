# Target exploration 启动模式耦合修复

> 历史记录：固定自动容量及域外目标拒绝方案已被本轮稀疏索引方案替代。
> 当前设计和验证边界见 target_navigation_sparse_domain.md。

## 证据与根因

`runtime_20260908_054033_0.bag` 对应参数：initial_mode=state2state，
exploration/mission_mode=target，auto_workspace/enabled=false。
odom 固定 (21.5,0,1.5)，房屋 box_0 的 x 最大值为 20；box_1 也不包含起点。
地图 revision 178→250，occupied 点约 5500，地图不是空的。
13 条 topology odom 诊断的直连和 fallback 均为 START_FAIL，无连接边。
直连 only_raycast 搜索在碰撞分支之前就检查 IsInBox，因此这次是域拒绝，
不是轨迹优化失败，也不能通过清图、降低碰撞阈值或增加重试解决。

exploration_navigation.launch 原本显式开启自动容量，而基础
planner_runtime.launch 按 initial_mode 开关容量。这解释了为什么之前的
导航探索入口正常，而 state2state 启动后切换 target 会失败。

## 修改

- 基础 launch 按 exploration_mission_mode 决定自动容量和 coverage guidance，
  不再按 initial_mode；target/target_directed 在 hold、state2state 启动时
  也提前初始化目标任务容量。coverage 显式任务仍使用原始区域。
- runtime 节点在未提供 enabled 参数时也按 mission 默认；显式 false 保留，
  但 target 模式会警告。不会偷偷绕过用户设定的边界。
- PLAN_TRAJ 和全局规划更新入口先检查起点和目标，再构建/等待拓扑；
  不可恢复的容量/禁入区错误走受控停止→BLOCKED，停止任务规划计时器，
  保留地图并允许新任务。状态协议仍使用现有 BLOCKED，具体原因写入 rosout。
- 2D 目标在初次初始化前按当前 odom 高度校验，初始化后使用固定任务高度；
  远端目标不要求已经观测为空闲。禁入区、局部碰撞、已知空闲提交检查保留。
- IsInBox 显式拒绝非有限坐标，避免 NaN 比较使其误判为合法。

## 设计边界

这不是无限空间算法改造。旧 frontend 的 frontier key、Bubble 区域索引缓存
有限域尺寸，当前沿用已有的 odom 中心自动容量（默认 XY 半径 120m）。
该容量不是覆盖目标，不要求扫描完整区域；任务仍以目标到达完成。
超出容量会明确阻塞，不运行时直接改尺寸，也不无限重试。
真正无限距离需要单独实现分块索引/滚动索引及跨块历史图迁移。

自动容量沿用既有实现，在初始化前替换 frontend 的 box 参数；因此不要将
这个自动域当作覆盖任务边界。区域覆盖部署应明确使用
exploration_mission_mode:=coverage，保留配置的覆盖 boxes。
如果在覆盖部署中切入目标任务，其有限域仍是启动时选定的域。

## 验证与运行

- target_workspace_launch_self_test.py：只展开 launch，不启动节点；12 个
  initial_mode×mission_mode 组合及显式 false 参数覆盖。
- target_workspace_self_test：bag 起点复现旧域拒绝、自动容量通过、远端目标、
  2D/3D 高度、禁入区、NaN、覆盖域内正常点。
- 需重新启动 runtime 以重建缓存索引；仅 rosparam set 不会修复已经初始化的
  旧域。不要修改历史 bag 的 params 文件，它是证据，不是运行配置。
- 启动日志应出现 automatic capacity；先用仿真验证 state2state→target 的
  模式切换和 EXEC_TRAJ，再检查换目标/取消。编译与单测不等价于飞行闭环验收。
