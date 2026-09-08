# State2state global-topology failure / worker recovery fix

## 已确认的问题与修复

1. **射线遍历可能不终止。** 栅格边界上的浮点误差会使已经到达目标索引的轴继续前进，之后再也无法同时满足三个轴的终止条件。原版可用 `resolution=0.1`、`start=(6.6,-10,7.1)`、`end=(5.6,13.6,-13.3)` 对应的边界测试输入复现（测试中坐标由整数乘分辨率生成，保留实际浮点误差）。修复将已到达终点的轴退出竞争，并用索引曼哈顿距离约束总步数；非法输入不再复用旧射线。碰撞查询额外检查首尾点，避免零长度/同栅格线段和终点漏检。
2. **连通图不代表端点可直线接入。** 常规拓扑查询失败后，复用局部 A* 生成有数量、单次搜索时间、总规划时间限制的端点接入路径。接入采用原始地图已知空闲检查、膨胀邻域检查，并验证最终拼接段；起点修复成功后优先查询，必要时再修复局部目标端点。仍须接入拓扑，不会把严格模式降级成自由空间规划。
3. **旧图可能反复提供同一条已阻塞边。** 在图 A* 松弛边时检查当前占据证据，跳过明确占据的边，允许搜索剩余绕行分支。历史 UNKNOWN 本身不删除历史连通性，当前要执行的局部前缀仍单独检查。进展锚点改为最多 12 个空间分散候选，避免只试同一片区域内的前三个相邻节点。
4. **真实失败和查询限流混在一起。** 初始规划重试使用单调时钟退避；运行配置为 0.6 s 间隔、最多 5 次连续真实失败。单纯查询限流不计入失败；前缀失败后的二次查询限流/成功，不会抹掉本次真实失败。失败路由也记录查询目标/epoch，换目标可立即重查；重新接入失败不再每帧刷新限流时间。
5. **外层取消不足以让规划线程退出。** 每次规划有独立于 ROS 时钟的截止时间（默认 2 s）；拓扑线段采样和 L-BFGS 的迭代/线搜索均检查取消。优化器取消钩子仅作用于当前工作线程。它是协作式退出，不是强杀线程，单次不可中断的第三方调用不具备硬实时抢占保证。
6. **指令与旧规划结果竞争。** 拓扑开关忙碌时也记录原子策略并请求取消；新目标请求取消旧规划。初始规划及滚动重规划均校验任务、目标和策略版本。旧工作线程 BUSY 时不允许再次 ARM；supervisor 保留模式切换，观察 READY 后继续激活。
7. **失败被误报成成功。** navigation status 增加兼容旧格式的 `result=`、`reason=`；重试耗尽/非法目标报告 `blocked`。supervisor 不再把所有 `WAIT_GOAL` 都当作目标到达。诊断在规划返回后提取，状态回调不获取长时间持有的规划锁。

## 配置与可见结果

配置位于 `src/Planner/general_planner/config/task_planner_runtime_state2state.yaml`：

```yaml
fsm:
  state2state_replan_watchdog_timeout: 2.0
  state2state_plan_from_rest_max_failures: 5
  state2state_clear_goal_on_plan_failure: true
  state2state_plan_from_rest_failure_backoff: 0.6
```

没有通过关闭 global topology、关闭 strict route、放开未知空间或缩小安全膨胀来消除报错。若当前位置缺少已知空闲证据、确实没有安全接入或轨迹仍不可行，应安全停止并允许下一任务，而不是承诺任意目标都能到达。

失败状态示例：`WAIT_GOAL <epoch> <sequence> IDLE READY stage=idle result=blocked reason=TOPO_PREFIX_BLOCKED`。另有起点缺少已知空闲证据、接入/图不连通、前缀越出窗口、局部前端失败、轨迹/备份失败和超时等原因。查询失败也记录当次地图/拓扑版本。

## 回归与使用

构建目标：`general_planner_runtime_node`、`fsm_node`。

本次已在 ROS1 Noetic 容器中完成两个节点构建，并通过下列五项及 `boundary_map_self_test`、`planner_command_gateway_policy_self_test`、`planner_command_gateway_behavior_self_test`、`fast_lbfgs_self_test`，共 9 项自测。需要 ROS master 的网关测试使用临时独立 master，结束后已关闭。

新增/扩展自测：

- `raycaster_self_test`：60000 组负坐标、边界、退化射线及无效输入；原版在同一有限步数检查中失败，修复版通过。
- `incremental_topology_self_test`：旧图短路径被占据时改选较长绕行路径；在线段采样期间取消，不返回部分路线。
- `state2state_planning_control_self_test`：优化器无 progress callback 时仍能在线搜索中取消；截止时间、线程隔离、下一次操作恢复。
- `state2state_topology_route_self_test`：严格模式边界、路由切片、策略版本以及查询限流与真实前缀失败的区分。
- `planner_status_self_test`：失败原因、worker READY 与目标生命周期兼容解析。

请在正常停止现有 runtime 后重新启动，使新二进制和配置生效。先在仿真/隔离回放中检查有绕行路径、无安全路径、规划中换目标、切换拓扑、PAUSE/CLEAR 和重新下发任务。没有自动向运行中的机器人发布目标、重启 runtime 或改变 ROS 控制话题。

这些回归证明的是具体代码缺陷及恢复机制；尚不能代替最新 bag 所在完整地图状态的闭环验收，也不能仅凭射线复现断言历史 bag 中每次 BUSY 都由同一个输入触发。
