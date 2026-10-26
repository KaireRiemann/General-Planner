# BDM 边界列用于 Corridor 障碍提取

## 实现边界

点种子和线种子的 CIRI 输入均改由 `MapManager::corridorOccupiedSearch()` 提供。
仍保留种子 AABB 作为几何范围和走廊外边界；取消的是对其中所有三维体素的扫描，
不是取消走廊的空间范围约束。CIRI、机器人半径、障碍采样顺序和轨迹优化保持原算法。

`BoundaryMap::occupiedIndices()` 对 XY 列做哈希查找、对 Z 下界做二分，然后只遍历范围内
的边界记录。它也适用于完整 BDM 的混合 Free/Unknown/Occupied 边界列。
当前 BDM 对每个 Occupied 体素都有记录，因此输出保留全部障碍体素，不做表面抽稀。
复杂度从体积扫描 O(Nx Ny Nz) 变为 O(Nx Ny log L + B)，其中 L 为列长度、B 为命中的边界记录。

## 为什么需要局部 Occupied 投影

持久 `MapManager::boundary_map_` 保存滑窗外的历史证据，且 MapManager 创建之前的
预加载地图并不一定存在完整的状态变化流。直接用它替代 ROG 的局部 boxSearch 会引入
历史障碍或漏掉初始化障碍。另外 `getGridType()` 与 `isOccupied(index)` 的虚拟边界条件
也并不相同，不能用前者生成的持久 BDM 直接代替后者。

因此在同一个 ProbMap 中维护一个 `BoundaryMap` 类型的局部 Occupied 投影：

- 在实际 Occupied 状态转换时增量插入/删除，不重新扫描 Free 体积。
- 不依赖持久拓扑的状态流是否启用，支持 occupancy-only 初始化。
- 环形缓冲区逐格淘汰时删除旧全局索引，远距离滑窗及全图清理时清空。
- 保留持久 BDM 的历史语义，不进行第二次射线融合。
- 使用 ROG 自身的世界坐标转换，并严格复制原 boxSearch 的开区间索引范围。
- 虚拟地面/天花板按原 isOccupied(index) 规则补充相交薄层，并保证输出字典序无重复。

本次接入的是原始 Occupied CIRI 走廊，包括普通点到点主轨迹及备份轨迹使用的点/线种子。
Tracking 的膨胀占据、目标排除规则以及 EPICON 自有点云查询不属于这个原始体素接口，继续
使用各自的查询。Unknown 不会被新接口解释为 Free。

## 配置和验证

```yaml
rog_map:
  corridor_obstacle_backend: bdm  # bdm（默认）、dense、verify
```

`dense` 完全不分配或维护新增索引，可用作包含索引维护成本的基线。
`verify` 每次执行两种查询，要求点集合及顺序完全一致，否则抛出异常，并输出
`[BDM_QUERY] points=... bdm_us=... dense_us=...`。它仅用于验证；双查询和日志开销
不应混入生产模式的端到端速度比较。

新增 `bdm_corridor_self_test` 直接比较真实 ProbMap 稠密查询和 BDM 查询，覆盖随机框、
负坐标、预加载、虚拟边界、双向部分滑窗、远距离清空、重新进入旧区域以及命中/漏检更新。
原 `boundary_map_self_test` 补充边界列范围查询、闭区间端点和删除测试。

实验脚本、固定目标、配置快照与原始日志保存在工作区 `analysis/bdm_corridor_20261026/`。

## 在线一致性与计时修正

预跑的 verify 捕获了在线地图更新与顺序执行两种查询之间的不一致。BDM 查询现在与
`probabilisticMapFromCache()` 的占据提交、滑窗切换和 occupancy-only 加载互斥。
射线遍历不持有这个查询锁；锁粒度是缓存提交，不是逐体素加锁。BDM 端到端数据包含
等待该锁的时间。dense 保留原始查询路径，不维护新增索引，也不等待查询锁，因此
对比包含 BDM 为保证快照一致性付出的成本。新增并发回归同时执行融合/滑窗与 verify。

原 benchmark 有两个统计陷阱：

1. `getLatestCorridorTime()` 实际只返回 CIRI 内核计时，而且读取后会清零；它不包含
   本次优化的 boxSearch。报告使用完整 `SearchPolytopeOnPath`、点/线走廊构造的作用域计时。
2. 点到点任务入口没有填写 `TOTAL_REPLAN`，发布消息中常为 0，benchmark 会退化为
   前端和优化子阶段之和。本次在执行器直接调用的 `state2state_task::planFromRest/replanOnce` 通过作用域计时填写完整调用时长，
   包括提前返回和提交，修正端到端指标。

设置环境变量 `GP_BDM_BENCH_PROFILE=1` 可输出 `BDM_PROFILE_corridor_path`、
`BDM_PROFILE_corridor_point`、`BDM_PROFILE_corridor_line`、`BDM_PROFILE_plan_from_rest`
和 `BDM_PROFILE_replan`。path 内部可能调用 point/line，不能把这些时长再次相加。
生产默认关闭这些日志。正式 A/B 两种配置均使用同一套完整调用计时。

## 真实 click demo 最终结果

结论：**边界列提取稳定减少了障碍查询和走廊构造开销，但这组实验尚不能证明稳定的
全系统加速。** 整次规划平均值下降约 9.3%，收益集中在第一轮，另外两轮基本持平。

环境为 AMD Ryzen 7 9700X、ROS1 Noetic Docker、Release 构建；运行真实
`task_planner/click_demo.launch`，使用 `garage.pcd`（19,277,703 个点）、原
`click_real_highspeed.yaml` 的参数、perfect_drone/LiDAR 模拟、实际 CIRI/MINCO 和闭环控制。
关闭两个 RViz 窗口，地图与规划器原有可视化发布保持原配置。不是独立算法原型或离线
CIRI 替身。目标由 `/goal` 自动发布，和人工 click 使用同一入口。

固定目标依次为 `(2.708503,3.532101,1.5)`、`(17.925463,17.321403,1.5)`、
`(19.419142,1.892098,1.5)`、`(1.383039,18.726159,1.5)`；每轮从初始位置重启场景，
轮内连续完成四次任务。正式顺序为 verify、dense_1、bdm_1、bdm_2、dense_2、dense_3、bdm_3。
每种正式模式 3 轮、12 个任务。早期预跑不纳入最终结果，最终证据目录为
`analysis/bdm_corridor_20261026/final/`。

### 查询等价性与局部收益

- 四项回归通过：`boundary_map_self_test`、`bdm_corridor_self_test`、
  `local_dynamic_fusion_self_test`、`shared_map_view_self_test`。
- 1,500 次随机真实 ProbMap 稠密/稀疏查询对照零差异；500 次并发查询验证零差异。
- 最终真实 click demo verify：828 次在线查询零差异，其中 328 次输出非空障碍。
- 同快照查询平均 dense **65.554 μs**、BDM **25.710 μs**，累计加速 **2.55×**；
  非空查询累计加速 **2.58×**。此计时在取得快照锁后开始，不包含等待锁时间；下表的
  完整 corridor 计时包含等待锁时间。

### 三轮 A/B

| 指标 | Dense | BDM | 解释 |
|---|---:|---:|---|
| 完整路径 Corridor，ms/调用 | 0.710 | 0.556 | 耗时减少 21.7%，约 1.28× |
| 已发布轨迹对应的完整规划，ms/调用 | 4.933 | 4.473 | 平均减少 9.3%，尚不稳定 |
| 地图融合 Total，ms/帧 | 46.859 | 45.954 | 差异小，不能归因为索引加速 |
| 占据缓存更新，ms/帧 | 2.632 | 2.562 | 未观察到明显的索引维护回退 |
| 已发布轨迹数量 | 223 | 260 | 闭环调度不同，不能当成逐样本配对 |
| 66.67 ms deadline 超限 | 0/223 | 0/260 | 两者均满足当前 15 Hz 周期 |
| benchmark 成功任务 | 10/12 | 10/12 | 失败详情见下文 |

表中的时间先计算每轮均值，再对三轮等权平均。完整规划均值包含已发布轨迹的主/备
轨迹生成和提交，未用旧 benchmark 的阶段求和代替。它不包含未发布的失败尝试，
因此另保留所有任务入口作用域计时：dense 共 513 次调用、累计 1319.05 ms；
BDM 共 569 次调用、累计 1268.92 ms。调用集合不同，累计值只作描述，不能当成固定
工作量的速度比。

| 配对轮次 | Dense Corridor ms | BDM Corridor ms | Dense 完整规划 ms | BDM 完整规划 ms |
|---|---:|---:|---:|---:|
| 1 | 0.733 | 0.536 | 5.280 | 3.961 |
| 2（执行顺序 B→D） | 0.727 | 0.572 | 4.728 | 4.736 |
| 3 | 0.671 | 0.561 | 4.790 | 4.721 |

走廊三轮均有收益；完整规划三轮变化分别约 **−25.0%、+0.17%、−1.45%**。
因此，稳定的结论是“消除了体积扫描的局部开销”。搜索、优化、地图融合及调度仍然
决定端到端表现，不能把 2.55× 的障碍查询加速宣称为整套规划器加速 2.55×。
目前两种模式也都满足 15 Hz deadline，本实验没有证明可用规划频率提高。

### 任务结果与限制

Dense：第 2 轮目标 4 的候选轨迹有 4 个净空检查命中，第 3 轮目标 4 首次规划超时。
BDM：第 1/2 轮目标 4 的候选轨迹分别有 4/19 个净空检查命中。所有正式 A/B 运行的
**执行轨迹采样均为 0 个命中**。最终 verify 轮也有一条候选轨迹触发检查，不混入 A/B。

这里的 benchmark `collision_clearance=0.25 m`，原配置 `robot_r=0.20 m`，而且候选轨迹
的未来部分可能被后续重规划替换；这些命中必须保留，但不能直接等同于实际撞障。
同样，执行采样零命中也不是连续轨迹安全证明。两组的候选命中率不同，目前不能声称
安全性完全等价；本次严格验证的是**同地图快照下障碍查询内容和顺序等价**。

只有单个 garage 场景、每模式三轮，且闭环地图/拓扑会随调度变化。平均 9.3% 的完整
规划收益没有充分统计证据证明稳定，也没有验证探索、tracking 或所有真实飞行场景。
索引维护未出现明显回退只是本场景结果，不代表高动态占据变化下没有额外成本。

### 复现与交付

工作区中的 `analysis/bdm_corridor_20261026/run_benchmark.sh` 保存完整运行流程，
会在隔离端口 11426 使用/启动 ROS master。容器内执行：

```bash
bash /root/ws/real_planner/analysis/bdm_corridor_20261026/run_benchmark.sh
```

主机上重新汇总和绘图：

```bash
python3 analysis/bdm_corridor_20261026/analyze.py
python3 analysis/bdm_corridor_20261026/plot.py
```

`final/metrics.json` 是每轮详细统计，`final/comparison.json` 是三轮汇总，
`final/run_summary.csv` 是可直接比较的表格；各子目录保留配置引用、原始 click.csv、
roslaunch 日志，`*_map.csv` 保留完整地图融合记录。图为 `final/benchmark.png`。
源码默认启用 bdm，设置 `rog_map/corridor_obstacle_backend: dense` 可恢复原始提取路径。
本次不替换 `general_planner_release` 内的预编译发布二进制。
