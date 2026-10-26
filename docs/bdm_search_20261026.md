# BDM 区间搜索接入与验证

## 范围与算法

本次在原有 BDM Corridor 基础上增加可选的局部膨胀占据区间搜索。配置：

```yaml
rog_map:
  corridor_obstacle_backend: bdm
  bdm_search_enable: true
```

默认关闭新搜索开关；关闭时不维护新增膨胀障碍列。原来的 A* 完整保留。

不是把持久 BDM 的历史 Free 直接作为实时可飞证据。当前普通 click frontend 在
膨胀地图上搜索，且默认允许 Unknown。为保持同一几何和 Unknown 策略，在 InfMap
增加 BoundaryMap 类型的局部 Occupied 投影：只在膨胀计数跨越 0 时更新有序列，
命中增加、障碍清除、滑窗淘汰和全图重置都沿原地图更新链维护。

单列查询用 Occupied 的补集产生有界可通行 Z 区间；它是规划策略下的可通行区间，
不表示原始地图已经观测到整个区间为 Free。启用 Unknown inflation 且拒绝 Unknown
时，当前加速器返回 unavailable，沿用原来的体素 A* 及 frontier 行为。

`BdmIntervalSearch` 按需查询 XY 列，将每个连续区间压缩为一个图节点。
相邻列区间高度重叠时建立水平通道，在当前列内竖直移动到通道高度后跨列。
支持四邻接和八邻接 XY；四邻接模式对应六邻接体素的连通性。
八邻接额外检查两个侧列，防止对角穿角。
八邻接不加入 Z 方向错位的斜向通道，因此是原 26 邻接的保守子图。
不预先枚举整个地图或全部 XY 列，不生成完整 Free 体素列表。

每个区间保留一个路径标签，不承诺体素最短路径最优性。重建路径包括竖直通道段，
图搜索沿用原 A* 的局部 horizon 终止条件，候选按连续几何长度裁剪；
裁剪后的全部点、线段以及连续起终点都通过原膨胀地图检查后才被采用。下游原有动态障碍、shortcut、Corridor 和优化检查继续执行。

搜索最多展开 6000 个区间，图搜索与候选验证预算为 min(3 ms, 原搜索预算的 15%)。
预算耗尽、起终点无法附着、图不连通、候选验证失败均回退原 A*，并共享其总计时起点。
这些失败不会作为全局不可达证明，不改变 Unknown 探索策略，也不会缓存跨地图版本的
“不连通”结论。本实现减少成功候选中的体素展开以及区间内部重复探索；没有实现
全局动态连通分量缓存或直接拒绝所有不可达任务。

## 计时与测试方法

`GP_BDM_BENCH_PROFILE=1` 输出 `[BDM_SEARCH]`：

- route: voxel / interval / fallback_graph / fallback_validation。
- status: -1 未尝试区间，0 FOUND，1 DISCONNECTED，2 BUDGET，3 UNAVAILABLE。
- voxel / interval: 两种搜索实际弹出/展开节点数；分析计入两者之和。
- columns: 按需查询列数。
- outcome: 原 A* 返回码，8/9 为 horizon/goal 成功，7/10/11 为初始化/无路/超时失败。
- ms: 整次 pointToPointPathSearch 在 setup 之后的耗时，包含候选验证及回退。

原有 direct-line frontend 可以直接绕过 A*；因此搜索均值不是每次完整规划均值。
完整规划另沿用任务入口作用域计时。A/B 均开启已有 BDM Corridor，仅切换新增搜索。
地图融合计时包含新增索引的维护成本，不能只看搜索时间宣称全系统加速。

随机测试将区间图和六邻接体素 BFS 对照 500 张地图，检查可达性和输出路径。
另测试负高度、大块自由空间、节点预算和 unavailable。
真实 ProbMap 回归扩展为检查膨胀列与 InfMap 状态一致，覆盖初始化、随机框、
命中/清除、正反滑窗、远距离重置；原有并发 Corridor 验证仍保留。

独立 microbenchmark 使用相同静态网格、欧氏启发式，对照简单六邻接体素 A* 和
区间图（不是直接调用产品 Astar 类）。区间预处理在计时外，不含在线索引维护，
仅用于解释节点压缩机制。场景包括墙体绕行、完全封堵和上下楼层。
实际收益以真实 click demo 为准。

## 可复现脚本

工作区 `analysis/bdm_search_20261026/` 保存脚本、配置、测试与原始结果。
容器内运行 `bash /root/ws/real_planner/analysis/bdm_search_20261026/run_benchmark.sh`。
主机运行 `python3 analysis/bdm_search_20261026/analyze.py` 和 `plot.py` 汇总展示。

## 预跑修正

`pilot_v1/` 保留第一版六轮记录，不与最终结果混合。第一版寻找完整目标、再截断 horizon，
会比原 A* 多做工作；部分对角候选又在原始射线检查中失败，导致重复回退。最终版加入
horizon 图终止与禁止对角穿角，并重跑相同 A/B。没有依据测速结果修改 CIRI 或优化权重。

## 最终真实 click demo 结果

场景：garage.pcd，19,277,703 点；ROS1 Noetic Docker，Release，AMD Ryzen 7 9700X。
沿用 click_real_highspeed 的 CIRI/MINCO、15 Hz 重规划和原始四个目标，每轮重启场景。
顺序 baseline_1、interval_1、interval_2、baseline_2、baseline_3、interval_3。
两组均使用 BDM Corridor，baseline 仅关闭新增区间搜索及其索引维护。
正式测速关闭 RViz；另开 RViz 的展示轮不进入下表。

表中时间和节点数先计算每轮均值，再对三轮等权平均。搜索调用集合随闭环行为变化，
不能当成相同快照上的逐请求配对。完整规划是已发布轨迹对应的任务入口调用，
不包含异步地图融合；地图成本另列，不能把二者直接相加声称整机吞吐量提升。

| 指标 | Baseline | BDM interval | 变化 |
|---|---:|---:|---:|
| 搜索展开节点/调用 | 12295.188 | 1530.279 | -87.6% |
| 搜索调用 | 12.606 ms | 1.035 ms | -91.8% |
| 已发布轨迹对应完整规划 | 4.453 ms | 3.995 ms | -10.3% |
| 完整路径 Corridor | 0.576 ms | 0.697 ms | +21.1% |
| 地图融合/帧 | 46.020 ms | 46.326 ms | +0.7% |
| 占据缓存提交/帧 | 2.564 ms | 2.724 ms | +6.2% |

| 轮次 | Baseline 完整规划 ms | BDM 完整规划 ms | 变化 |
|---|---:|---:|---:|
| 1 | 4.617 | 3.433 | -25.6% |
| 2 | 4.358 | 3.815 | -12.5% |
| 3 | 4.383 | 4.737 | +8.1% |

搜索三轮均有明显收益；完整规划平均减少 10.3%，但第三轮反而增加约 8.1%。
完整 Corridor 增加约 21.1%，说明不同引导路线也会改变后续几何/优化工作量；
地图融合平均增加约 0.7%，缓存提交增加约 6.2%。因此不能称“所有规划阶段都加速”。

- 搜索调用总数：26 → 15；BDM 15 次全部采用区间候选，0 次回退。
- 搜索返回失败：两组均为 0。本实验不能证明减少了不可达请求失败数；证实的是减少展开。
- 搜索调用合并样本 P95：33.787 → 2.389 ms；样本仅 26/15 次，尾部结论仍需更大样本。
- 任务通过：11/12 → 12/12。Baseline 1 轮第 4 目标的候选轨迹有 4 个净空检查命中。
- 已发布轨迹：255 → 254；66.67 ms deadline 超限均为 0。
- 两组执行轨迹采样检查命中均为 0；候选轨迹未来部分命中不等于实际执行撞障。
- 首次规划响应延迟（三轮等权均值）：见 metrics.json，包含闭环等待，不等同单次搜索耗时。

## 解质量评估

正式六轮同步记录 /lidar_slam/odom、/planning_cmd/poly_traj 和 /goal，保存在
`final/*_quality.bag`。离线导出 JSON；按目标执行窗口约 20 Hz 采样里程计。
**原 benchmark 的 total_length_m 是所有候选轨迹长度之和，不能作为实际飞行距离。**
这里用里程计相邻点长度和计算实际距离。

净空为执行采样点到原 garage 点云最近点的欧氏距离，使用覆盖路线的
x/y=(-10,35)、z=(-2,6) m 点云范围；它不同于 benchmark 的体素净空检查，
不构成连续时间碰撞证明。加速度/jerk 使用在该时刻已发布且已开始执行的多项式解析导数，
只采样已执行时间段，排除被替换的未来候选段；它是命令轨迹指标，不是 IMU 实测。

| 指标（12 个任务） | Baseline | BDM interval |
|---|---:|---:|
| 实际飞行距离均值，m | 17.930 | 18.009 |
| 飞行时间均值，s | 3.421 | 3.349 |
| 执行采样最小点云净空，m | 0.678 | 0.670 |
| 执行命令加速度采样最大值，m/s² | 19.946 | 19.928 |
| 执行命令 jerk 采样最大值，m/s³ | 57.306 | 109.283 |
| 每任务 jerk P95 的均值，m/s³ | 45.353 | 44.962 |

实际距离平均 +0.44%，飞行时间平均 -2.09%；只比较两组均成功的 11 对任务，
距离 +0.37%，时间 -3.50%。目标 2 的平均距离 25.978 → 26.339 m，存在约 1.4% 绕行增长。
没有观察到大幅绕路或执行采样撞障，但不应声称解质量严格等价。

**平滑性存在峰值退化：interval_2 的目标 4 出现 109.283 m/s³ jerk，
Baseline 全部任务最大 57.306 m/s³。** 常态 jerk P95 没有增加，但这个单次峰值不能忽略。
样本峰值仍低于配置 max_jerk=120、max_acc=20；采样结果不是连续约束证明。
CIRI/优化权重没有改变，路线及重规划时序变化仍可能影响最终平滑性。
新开关保留默认关闭，供明确选择该性能/质量权衡时开启。

## 验证结论与限制

- 两个运行节点 fsm_node、general_planner_runtime_node 编译通过，未替换 release 预编译二进制。
- bdm_interval_search_self_test：500 张随机图、输出路径、负高度、大体积压缩、horizon、
  对角穿角、预算耗尽、unavailable 通过；ASan/UBSan 同样通过。
- bdm_corridor_self_test：1500 次真实地图对照（含膨胀区间），并发 Corridor 回归通过。
- boundary_map_self_test、local_dynamic_fusion_self_test、shared_map_view_self_test 通过。
- 地图单元测试使用已有 map.yaml 派生的 test_map.yaml；最初误用 click 配置时，测试夹具
  再次 resetMapSize 缩小虚拟高度导致动态测试失败，改用原测试配置后通过；日志保留。
- 当前为局部策略可通行区间，不是完整历史 BDM Free/Unknown 连通分量缓存。
  禁止 Unknown inflation 的策略组合会回退，不以保守图的无路结论直接拒绝全局请求。
- 只有一个场景、每模式三轮，不能推广为所有地图或真实飞行条件下的确定收益。

图与原始结果：`final/benchmark.png`、`final/executed_paths.png`、`final/metrics.json`、
`final/comparison.json`、`final/quality.csv`。RViz 展示截图独立保存在 `showcase/`。

## 图像与展示轮

- [性能图](../../../analysis/bdm_search_20261026/final/benchmark.png)
- [三轮实际执行路线叠加](../../../analysis/bdm_search_20261026/final/executed_paths.png)
- [原 A* 的 RViz 俯视截图](../../../analysis/bdm_search_20261026/showcase/baseline_rviz.png)
- [BDM 区间搜索的 RViz 俯视截图](../../../analysis/bdm_search_20261026/showcase/interval_rviz.png)

两张 RViz 图使用相同 TopDownOrtho 视角，选取目标 2 开始后约 2 秒，显示实时地图、
引导线、Corridor 和优化轨迹；机器人位置和规划时序不完全相同，不能逐像素判断质量。
截图来自额外的启用 RViz 展示轮（前两个目标），不混入正式性能/质量统计。
`showcase/selected.json` 保存原始帧和选择时间；`screenshots.json` 保存全部截图时间。
斜视展示轮与最初界面调试分别保存在 showcase_oblique/、showcase_initial/，均不计入 A/B。
展示进程与独立 ROS master、Xvfb 已在结束后关闭。
