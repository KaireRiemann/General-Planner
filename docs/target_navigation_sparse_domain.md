# 目标探索：空间解耦与卡死审计

## 本次基线与范围

基线是 7f9dd0e（包含此前地图融合、state2state 取消及目标容量预检修改）。
没有回滚地图、调整 raycaster 或修改历史 bag 参数。目标是撤销覆盖区域对
target navigation 的限制，而非把边界扩大到另一个固定距离。

## 已实施

1. LIO 的任务模式独立于覆盖 boxes。目标模式 IsInBox/IsInMap 仅判断有效
   坐标和显式 dead areas；覆盖模式仍保留原区域。模式切换不改索引原点。
   远端目标无需处于 ROG 当前窗口或已创建的 Bubble 区域。
2. Frontier key 从按初始 box 尺寸打包的 64 位值，改为三个有符号 32 位
   cell 坐标；负坐标和越过原边界不会编码别名。coverage 仍使用原边缘过滤，
   target 不使用覆盖边缘过滤。不是无限数值精度：仍受 float/int32 表示限制。
3. Bubble 区域在串行观测维护中按需创建。只创建当前 odom、截断后的观测
   端点及射线路径经过的区域，活动更新集仍受 max_update_region_num 限制。
   getter 保持只读，避免 OpenMP 搜索线程并发插入 unordered_map。
   区域索引用 floor 而不是向零截断，修正原点负侧归属错误。
4. 保留历史区域、节点及连接身份，不随 odom 改原点或全量清图；ROG 仍负责
   滚动局部占据证据。历史路线是否仍安全必须经当前局部安全检查。
5. target 不再将 SFC 裁剪到 coverage boxes。已有障碍走廊、重叠和最终轨迹
   安全检查保留；goal_refine 不再把目标挤回旧覆盖框。
6. auto_workspace 参数作为兼容入口保留，但不再覆盖 box 参数，也不再等待
   odom 来创建固定容量。target 启动不等待 RViz 覆盖角点选择。
7. 拓扑锚点 shortlist 在可用预算内保留一个绕行候选，避免未验证连通性的
   前向候选完全排除后退分支。单候选配置仍只有一个候选，不提供完备性保证。

## 卡死审计与本次补充

| 路径 | 处理与剩余边界 |
|---|---|
| 固定 box 拒绝 odom/目标 | 移除 target 的覆盖约束，仍检查禁入区 |
| PLAN_TRAJ 无 odom 邻接边直接返回 | 增加 WallTime 超时，受控停止后 BLOCKED；新任务重置计时 |
| 探索 MINCO/LBFGS 耗时失控 | 增加 exploration/optimization_budget_sec，默认 2s；检查迭代、线搜索、重试和提交前；超时不提交新轨迹 |
| LAND 内 while(1)+sleep | 改为 FSM tick 中限频发布，保持终止状态但不占死回调线程 |
| state2state 优化/规划取消 | 基线已有独立队列、取消探针及规划预算，本次保留并回归测试 |
| 未知目标无局部进展 | BLOCKED 是可恢复的任务结果，不是数学上的不可达证明 |

## 尚不能宣称解决的设计问题

- world 队列仍串行处理点云、探索维护和探索任务请求。优化预算不能抢占一次
  PCL/IKD 操作、单次代价函数计算或 OpenMP 区域处理。Supervisor/gateway
  队列独立，但探索确认仍可能延迟。不能简单增加 spinner 线程：FSM/图及前沿
  容器没有相应快照/互斥协议。下一步需要带任务 epoch 的工作快照与提交栅栏。
- 当前为持久稀疏历史，并非内存上限严格固定的地图。持续远行时历史区域、
  frontier labels、IKD 和历史拓扑会增长；需要单独做分块冷存储/安全回收，
  不能直接删仍被 shared_ptr 边引用的区域。这次不宣称无限时长或无限内存。
- 覆盖模式仍有同步 LKH 调用及共享临时文件；target 在 commitTargetDirectedTour
  提前返回，不依赖覆盖 TSP。这不等于覆盖任务的最坏响应时间已受控。
- 锚点评分、有限 shortlist 和局部搜索不提供未知环境导航完备性保证。
  保留绕行候选修正了一种饥饿情况，但复杂迷宫仍需要闭环验证。
- 本次测试不能证明 Unity 飞行闭环已经成功，必须重新启动程序后验证。

## 验证

target_workspace_self_test 使用实际 LIO/TopoGraph：10km 目标合法、禁入区拒绝、
负区域索引、20001 个正负 frontier key、30 次模拟 odom 更新跨越 725m、
历史区域身份不变、切回 coverage 恢复原框、绕行候选保留。
这是结构/策略回归，不含点云融合或动力学执行，不应称作飞行或完整 bag 回放。
launch 测试覆盖 12 个 initial_mode×mission_mode 组合和兼容参数覆盖。

实机/仿真验收需要：原 bag 出生点→框外目标，跨多个局部窗口，U 形障碍，
动态阻挡，规划中换目标/取消，长时间内存和 heartbeat 延迟统计。

本轮结果：planner_runtime_node、exploration_node 编译通过；上述稀疏域测试
在独立的 11379 ROS master 下通过；target_directed_exploration、planner_status、
state2state_planning_control、lio_observation_fusion 自测通过，launch 矩阵通过。
未启动实际 runtime 飞行、未发送飞行命令、未进行 Unity 动力学闭环验收。

## 补充：启动 SIGABRT 回归

用户随后启动报 exit -6。隔离 master 下 GDB 复现为 free(): invalid pointer，
栈在 FastExplorationFSM::init 读取 fsm/task_command_topic，而非点云融合。
状态机头文件曾在编译期间改变布局，部分旧对象文件的完成时间晚于头文件
修改时间，使普通增量构建没有重新编译它们。停止编辑并使该头文件依赖重新
编译后，不改变初始化算法即可恢复启动。这是前一轮构建产物一致性与验证缺口。

新增 tests/runtime_startup_smoke_test.py：自行启动随机端口的独立 ROS master，
启动实际 runtime（state2state，无传感器、Unity、RViz、轨迹消费者），等待真实
PlannerStatus 并查询节点进程存活，再关闭自建进程。本轮通过；不等价于飞行验收。
