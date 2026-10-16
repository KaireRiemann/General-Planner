# State2state 失败链路审查与修复（2026-10-13）

## 证据与范围

分析对象为 `2026-10-13-19-06-23.bag` 和当前源码。该 bag 只有 7.59 秒，
76 条监督器状态均为 `waiting_input / s2s_wait_goal`，`ready_for_new_task=true`；
导航状态均为 `WAIT_GOAL 26 13 IDLE QUIESCENT`。没有录到新目标和最初的
拓扑查询失败，所以不能用它证明状态机死锁，也不能重建此前的任务失败时间线。

飞机约位于 `(0.93,-41.48,7.01)`。8 帧 `inf_occ` 中，对齐附近反馈后的位置
均落入中心 `(0.975,-41.475,6.975)` 的 0.15 m 膨胀占据体素；最近原始障碍
中心距离约 0.35 m。这支持起点碰撞问题，但不能区分真实障碍、残留占据或
点云坐标误差。图含 84 个公开节点、212 条边，最近节点约 3.38 m、3.83 m。
旧代码还会投影起点，因此实际里程计位置不能直接等同于旧代码最终提交给
连接器的位置；缺少失败时的完整日志，不能断言具体哪条旧连接被哪个体素阻断。

此审查覆盖 general_planner 的目标接收、状态机、拓扑前端、命令监督及恢复，
不对 IDE 中另一个 sfm 包作未经证据支持的修改。历史文档
`topology_start_observation_recovery.md` 的“10 秒起点观测等待”等描述与本次
修改前源码不一致；不能把它当作当前程序已具备的功能。

## 风险及处理

| 环节 | 可能的问题 | 本次处理或保留约束 |
| --- | --- | --- |
| 输入与悬停 | 低频 odom 反复重置稳定计时 | 保留已改为 0.5 秒的参数；本 bag 已稳定，不再次扩大 |
| 静止规划起点 | 投影到非占据位置可掩盖真实起点碰撞；非占据也可能是 UNKNOWN | 拓扑图段未完成时先检查真实起点，禁止投影及宽松 escape |
| 起点证据 | raw FREE 不代表 inflated FREE | 分别报告原始占据、膨胀占据、未知、越界、非法坐标 |
| 图可用性 | 有 snapshot 不代表起点可接入，也不代表目的地可达 | 保留接入及执行前缀逐段检查 |
| 连接修复 | 候选半径、最多三个修复候选、20 ms/候选及60 ms总预算会限制成功率 | 不凭短 bag 盲目增大；修复仍须已知自由且满足膨胀安全检查 |
| 图更新 | 更新 focus 只是请求，查询可能读到尚未刷新的 snapshot | 保留有限重试和有界暂态等待；不在规划线程同步重建图 |
| 首次规划 | 默认拓扑等待3秒、规划截止5秒，却被首命令2秒提前终止 | state2state 首次规划由既有规划截止时间负责；网关继续安全HOLD |
| 执行与重规划 | 放宽首命令保护可能意外放宽运动中断流保护 | 收到过命令后仍执行原有 stale-source 保护；其他模式不改变 |
| 失败诊断 | 拓扑失败最终只显示 retries exhausted 或 command timeout | 增加可选 reason=CODE，保留原有状态消息的epoch/sequence/worker字段 |
| 跨线程读写 | 状态回调直接读取 route 会和规划线程竞争 | 仅用独立短临界区传递诊断字符串，不从状态线程读取路线 |
| 任务恢复 | 旧任务迟到的状态可能解锁新任务 | 保留epoch、goal sequence、QUIESCENT、新ARM确认及悬停验证 |
| 恢复期点击 | 新目标覆盖、取消、过期可能复活旧任务 | 保留已有最新目标缓存策略，并运行相关回归 |
| 局部导航 | 强制拓扑检查可能破坏显式local-only和最终局部段 | 检查仅应用于仍需拓扑图段的任务 |

## 修改后的边界

被占据起点仍会安全失败，不会伪造自由空间、缩减膨胀、瞬移起点或自动飞离
障碍。恢复目标是“有限结束、准确诊断、可重新提交任务”，不是无条件产出轨迹。
未知起点使用已有有限失败重试，不新增无限观测等待。拓扑尚未就绪的暂态等待
仍受已有拓扑等待期限和监督器规划截止时间约束。

新增的可选诊断字段由旧接收器忽略；新接收器仍接受没有该字段的旧消息。
仅在现有epoch/goal sequence校验通过后把失败原因写入监督器状态。

## 验证结果

- 真实前端：连续起点保持、膨胀占据、未知、越界、障碍清除后的成功重试。
- 路线策略：保留图绕行、等待实测到达锚点、拒绝未知空间捷径、local-only。
- 超时：首次规划超过首命令宽限仍可继续，但到规划截止必须结束；运动中断流及其他模式不豁免。
- 生命周期：具体失败原因传递、旧epoch拒绝、BUSY不解锁、新ARM确认、新目标接收。
- 缓存：恢复期最新目标、取消、过期；tracking切换与HOLD命令连续性。

ROS1 Noetic 容器内编译通过，15 个测试调用全部通过：

- `planner_command_gateway_policy_self_test`
- `planning_retry_self_test`
- `state2state_topology_frontend_test`
- `state2state_topology_route_self_test`
- `planning_failure_integration_test` 的 `retries`、`frontend_reason`、`deadline`、
  `initial_plan`、`late_command`、`queued`、`queued_cancel`、`queued_expired`、`tracking`
- `planner_command_gateway_behavior_self_test`
- `navigation_goal_admission_integration_test`

测试使用独立 loopback ROS master 11381/11384/11386，结束后均已关闭，
未给正在运行的仿真或车辆发送目标。测试脚本、逐项日志和构建日志位于工作区
`analysis/topology_audit_20261013/`。`git diff --check` 通过。

已重新构建 `planner_runtime_node` 和 `general_planner_runtime_node`，将 strip
后的两个二进制同步到 release/bin；在加载 ROS1 环境后检查动态依赖均可解析。
旧 release 二进制备份也在上述分析目录。没有自动重启用户节点或重新生成发布压缩包。

短片段不替代从建图启动到导航失败/成功的实景闭环验证；本次未声称已验证
原场景自动脱困。现场占据仍存在时，应明确失败并等待新的有效任务/地图条件。
