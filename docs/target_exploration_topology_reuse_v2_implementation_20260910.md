# Target exploration 拓扑复用 V2：实现与验证记录

实施基线：`60b31ca5ce51932edaa9ad8c06979a47455132a7`，开始实施时 General-Planner 工作树干净。保留此基线的 target 稀疏任务域、取消 target 自动边界框、raw LIO/ROG 单入口，以及 tracking 的最新修改。

对应设计：[V2 分析与方案](target_exploration_topology_reuse_v2_20260910.md)。本次实现 P1/P2/P3 的功能链路，不删除旧探索历史图，不把 coverage 转成全局图导航，也不开展 P4 的大规模类迁移。

## 1. 实际执行链路

| 路径来源 | 证据与入口 | 执行方式 |
| --- | --- | --- |
| `LOCAL_EXPLORATION` | 无可用全局路线、查询预算耗尽或路线冷却 | 原 frontier/Bubble 搜索、评分和局部规划 |
| `KNOWN_GOAL` | 不可变全局搜索快照中的连通路线，连接段和全部图边重新验证 | 保留完整折线；从当前观测区域截取可执行前缀 |
| `KNOWN_ANCHOR` | 可达图节点，portal 方向有实际 UNKNOWN 证据 | 前缀执行、到达出口后交回局部探索 |
| `LOCAL_GOAL` | 目标在局部前缀范围内，整条直达连接在当前 raw ROG 中可通行 | 无需全局图的末端接近，同样采用严格提交门禁 |

`LOCAL_GOAL` 是闭环测试暴露末端问题后的补充：旧 Bubble 路径可能绕进未观测区域，即使目标本身已知也无法完成最后接近。新增连接只接受**整段当前观测已知空闲**，不把“目标点已知”推导成“直线可飞”，也不把这个来源计入全局拓扑复用次数。

`TargetRouteRuntime` 保存完整路线、稳定图节点身份、任务代际、world epoch、实际弧长进度和冷却记录。默认 `prefer_known`；`legacy` 使用旧点引导；`shadow` 只计算新路线，不选择新来源。

模式参数在节点初始化时读取，修改 YAML 后需重启对应入口。`legacy` 回退的是路线选源/执行接入，不撤销公共首命令握手和有界等待修复。

## 2. 地图所有权和查询边界

- 不增加第二份 MapManager/ROG，不修改原始点云融合，不调用旧 `findTopologyPath()` 的同步维护副作用。
- `peekGlobalGridType()` 只读当前原始占据和历史 BoundaryMap，不消耗事件队列，不触发图维护或发布。
- 拓扑结构使用不可变 `SearchSnapshot`。其中当前图边本来就是端点间验证过的直线；完整路线由有序图边组成，不把整条路线压成首尾弦线。
- 查询在现有串行 world 队列中进行，具有墙钟、节点、展开数、采样数限制。**没有新增读取 live ROG 的异步线程**。查询预算是协作式检查，单次底层读取的耗时不能被抢占。
- 历史信息只用于路线选择。局部前缀和最终轨迹必须满足当前 raw `KNOWN_FREE`、局部窗口、膨胀障碍、LIO/ESDF clearance、任务禁区等约束。

## 3. 路线、恢复与安全

- 已认证前缀不再依赖旧 Bubble 根节点或 `global_tour_.size() >= 2`；旧来源仍保留旧前置条件。
- 新来源不经过旧路径的直线化/回折消除；MINCO、corridor、backup、速度限制和提交仍共用原后端。
- 投影只在由实际里程计位移限定的邻近弧长窗口内搜索。接近的远端折返分支不能使进度跳跃；planned prefix 和 future head 不能计入实际进度。
- future replan head 仍由 adapter 唯一生成，但新来源的 head 投影也有弧长上界，重接段还要经过当前地图检查。
- 首版采用保守的**每个局部前缀末端静止**策略；运行中仍可滚动重规划延长，未知边界不带非零终端速度。
- 失败先尝试有界局部重接；图边/anchor 使用稳定身份冷却，最多保留 64 条，任务/目标/world 切换清理。失败不标记覆盖目标或 frontier cluster。
- 后端每次尝试最多调用一次。失败保留既有安全提交，否则进入现有受控停止路径。
- 最终公共门禁检查**实际组合命令**（head/body/backup），覆盖有 backup 和无 backup 两条提交分支；包含 t=0、最后端点、小于采样周期的轨迹和相邻采样点之间的空间检查。

## 4. 启动和状态协议

- 目标任务未发布第一条真实轨迹前，FSM 内部报告 `WAITING_LOCAL_PLAN`，Supervisor 保持 HOLD，`ready_for_new_task=false`。
- 位置轨迹成功发布后，才把任务标记为已开始命令输出；受控停止/安全区恢复发出的真实轨迹也计入。
- 首命令墙钟截止时间默认 15 s，在 START 时启动，重发 START 不续期；监督队列独立于 world/optimizer 队列。
- 有限时间内无动作会进入失败/阻塞及既有悬停恢复协议。旧任务迟到 RUNNING 不能重新授权。
- 原 `/planner/target_exploration/status` 消息与四态接口不变；等待局部规划不是“可以派发新任务”。

## 5. 配置和日志

配置在 `src/Planner/general_planner/config/target_exploration.yaml`：

```yaml
exploration/target_route/mode: prefer_known
exploration/target_route/query_interval: 1.0
exploration/target_route/query_budget_ms: 12.0
exploration/target_route/prefix_length: 12.0
exploration/target_route/progress_timeout: 12.0
exploration/target_route/cooldown: 5.0
exploration_first_command_timeout: 15.0
```

`[target route]` 报告来源状态、route_id、实际进度、剩余弧长、前缀拒绝原因、查询耗时/采样/展开数量和阶段。

`[target route commit] accepted` 报告来源、route_id、traj_id、任务代际、world epoch、duration 和 backup。只有 `source=KNOWN_GOAL/KNOWN_ANCHOR` 才是已执行的全局拓扑复用证据。

## 6. 验证方法

所有编译与测试在已有 `ros1_noetic` 容器，工作区 `/root/ws/real_planner`。测试只使用自己创建的隔离 ROS master，不接管现有 11311/11381 master 或外部飞行消费者。

```bash
source /opt/ros/noetic/setup.bash
cd /root/ws/real_planner
catkin_make '-DCATKIN_WHITELIST_PACKAGES=map_manager;general_planner' -j4
source devel/setup.bash
python3 src/General-Planner/src/Planner/general_planner/tests/run_target_route_regressions.py
python3 src/General-Planner/src/Planner/general_planner/tests/run_target_route_regressions.py --navigation
```

合成闭环脚本 `target_sparse_corridor_integration.py` 支持 `TEST_NO_TOPOLOGY`、`TEST_U_SHAPE`、`TEST_REQUIRE_ROUTE`、`TEST_COVERAGE_SECONDS` 和独立输出目录。U 场景先通过传感器 setup pass 建立地图，再回到起点派发目标，setup 位移不计作任务进展。

### 结果

容器编译及发布构建均通过。`run_target_route_regressions.py` 的 13 个用例、navigation 分组的 3 个用例，以及 SE3 spatial-map / tracking-brake 的 2 个额外 CTest 用例通过。覆盖路线几何、末端/backup 采样、空图、raw 未知、任务域、目标状态、覆盖引导、状态到状态路由、里程计快照、命令网关、首命令握手、截止时间、导航/跟踪恢复协议。

最终另外运行了 `target_sparse_runtime_smoke_test.py`：真实 runtime 初始化、Supervisor 实际状态发布、target 忽略旧 auto_workspace 且保留原覆盖框的检查通过。`git diff --check` 通过；所有本次测试创建的 master/节点已退出，原有 master 保持运行。

| 实际 runtime 合成闭环 | 结果 | 执行证据 |
| --- | --- | --- |
| 无全局图，旧覆盖框外 x=21.5 → 45 | SUCCEEDED，14.52 s；终点 `(44.99884, 0.00007, 1.50089)` | `KNOWN_*` 提交 0 次，`LOCAL_GOAL` 1 次 |
| 源码入口，已知 U 形绕行 `(0,0) → (4,0)` | SUCCEEDED，17.02 s；终点 `(4.00978, 0.01697, 1.50254)` | `KNOWN_GOAL` 提交 14 次；到目标的平面距离曾从 4 m 增至 5.007 m；未穿墙 |
| 原 coverage 模式，20 s 连续运行 | 持续 RUNNING，移动约 14.18 m，始终在原覆盖区域内 | 两种新路线提交均为 0；此测试**不代表完整覆盖率验收** |
| 发布包入口，同一已知 U 形绕行 | SUCCEEDED，15.01 s；终点 `(4.00000, -0.00578, 1.49997)` | `KNOWN_GOAL` 提交 12 次；未穿墙 |

闭环日志和逐时位置记录在 workspace 的 `test_results/target_route_v2_20260910/`：`no_topology_final`、`known_u_route_final`、`coverage`、`release_known_u_route`。测试时原有 11311/11381 master 保持不动；协议测试使用自建 11382/11383，闭环使用自建随机端口。

中间失败记录同样保留：

- `no_topology` 证实原局部图末端绕行会进入未观测段，促成上文的 `LOCAL_GOAL` 修复。
- `known_u_route` 的首次版本因重复 attachment 检查耗尽查询预算而回退。将初始起终点连接数各限制为 2 后，同一 12 ms 预算下完成查询；源码成功场景所记录的几次图查询约 6.20–9.03 ms。该范围是场景测量，不是实时最坏上界。
- `no_topology_local_goal` 保留一次开发中间产物的初始化堆错误日志。随后固定源码、统一重编所有受新头文件影响的目标，才执行上表的最终源码/发布闭环，未把中间增量产物作为发布依据。

不把 helper 单测替代真实 runtime 闭环，也不把理想位置跟随器当成 Unity/真实飞行动力学验收。没有旧版同场景统计基线，因此不宣称速度或成功率提升百分比。

## 7. 发布边界

源码通过后用 `sh_files/build_general_planner_release.sh` 同步发布程序和配置。补齐发布脚本中原先遗漏的 `ExplorationTaskRequest.msg` / `TargetExplorationStatus.msg` 源定义，保持源定义、生成接口和二进制一致。

发布包需单独启动核验。未启动实际机器人或 Unity 控制链；真实场景的跟踪误差、动力学、动态障碍重观测和复杂出口效果仍需要实际仿真/飞行验收，不能由本次合成测试推导性能提升百分比。

本次已执行发布包 U 场景闭环，且确认发布与源码 `target_exploration.yaml` 逐字一致。发布时使用 `GP_SKIP_ARCHIVE=1 GP_CATKIN_ARGS='-j4'`，更新了发布目录，没有额外生成 tar.gz。`planner_runtime_node.bin` 的 SHA-256：

```text
47240490b68d804a57a1cb42957493f95114563c154326a2bcba4184d4d21bcb
```

本次没有自动 git commit。源码、新测试、文档、配置、补齐的发布消息源定义和更新后的发布二进制应作为一组审阅/提交。
