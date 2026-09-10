# Target exploration 空间解耦：实现与验收记录

日期：2026-09-10。本文记录本轮源码修改及实际运行结果；不替代历史分析或后续拓扑复用设计方案。

## 实现范围

- Target 不再受 coverage boxes 或初始 120 m workspace 容量约束；仍拒绝无效坐标和显式禁入区域。
- Frontier 使用有符号三维 cell key；Topo 根据 odom 和局部观测按需创建区域，保留固定世界索引与历史节点身份。远端目标查询不会预分配沿途空间。
- Coverage 保留原 box 并集、边界过滤、区域预分配及更新顺序。切换模式时交换独立 frontier 生命周期列表，保留 coverage 的 VISITED 状态，避免 target 远端任务混入覆盖任务。
- Target 跳过覆盖框的目标裁剪和 SFC 裁剪，保留障碍、净空及轨迹检查。没有 ROG 或处于未知区域时，不将 LIO 距离回退当作已知自由证据。
- 近目标、全段已知自由时可建立经过安全检查的直接连接。Target 静止短接近动作不再被巡航最小自由长度单独否决；移动时的制动要求及 coverage 策略保留。
- Target 的 `auto_workspace` 兼容参数被忽略，无须设置为 true。Coverage 显式开启它时保留原自动工作空间行为；默认使用配置中的覆盖框。

## 本轮验证

| 验证 | 结果与范围 |
| --- | --- |
| Noetic 编译 | planner_runtime_node、exploration_node 及相关自测构建通过 |
| 稀疏域自测 | 10 km 目标、20,001 个正负 key、725 m 区域增长、历史身份及只读查询通过 |
| Coverage 隔离 | 80,631 个采样点与原覆盖框/禁区规则一致；模式切换保留访问状态 |
| 安全语义 | Target 无局部地图返回 UNKNOWN；coverage 原兼容行为保持 |
| 策略自测 | target_directed_exploration_self_test、coverage_guidance_self_test 通过 |
| 真实 runtime 启动 | Target 与 coverage 两种 auto_workspace 兼容测试通过，收到实际 PlannerStatus |
| 合成走廊 0 → 145 m | 约 69.58 s，最终 SUCCEEDED，跨越原 20 m 边界及 120 m 容量 |
| State2state → target | 从 x=130 m 切换后导航至 145 m，约 12.51 s 成功 |
| Coverage 合成闭环 | 运行 20 s，实际移动且位置保持在原覆盖框内；未验证完整覆盖结束 |

原始日志及 JSON：`/home/diffbot/ros1_ws/real_planner/analysis/20260910_151649/`。
最终长距离结果为 `target_0_to_145_final_result.json`，对应日志为
`target_0_to_145_final_runtime.log`。启动兼容结果为 `smoke_target_compat.log`
与 `smoke_coverage_compat.log`；覆盖闭环为 `coverage_runtime_result.json`。
目录中的较早失败记录保留用于追溯，不代表最终版本的测试结果。

## 复现

在 ROS Noetic 环境 source 工作空间后执行（每个脚本自行创建和清理独立 ROS master）：

```bash
source /root/ws/real_planner/devel/setup.bash
cd /root/ws/real_planner/src/General-Planner
python3 src/Planner/general_planner/tests/target_sparse_runtime_smoke_test.py
TEST_MISSION_MODE=coverage python3 src/Planner/general_planner/tests/target_sparse_runtime_smoke_test.py
python3 src/Planner/general_planner/tests/target_sparse_corridor_integration.py
TEST_INITIAL_MODE=state2state TEST_START_X=130 python3 src/Planner/general_planner/tests/target_sparse_corridor_integration.py
TEST_INITIAL_MODE=exploration TEST_COVERAGE_SECONDS=20 python3 src/Planner/general_planner/tests/target_sparse_corridor_integration.py
```

## 实际边界

合成走廊使用射线点云与理想位置指令跟随器，没有 Unity 飞行动力学；这些结果不能证明复杂障碍下导航完备，也不能替代实际场景回归。其他模式未逐项进行飞行验证。

本轮解除固定几何容量，采用持久稀疏历史；并未实现固定内存预算或冷存储。长时间远行时历史数据仍会增长，数值表示也有范围限制。

源码及 devel 构建已更新。`general_planner_release` 的预编译发布包未重新生成，使用该包时需重新打包；仅修改发布 launch 参数不能获得本轮算法修改。
