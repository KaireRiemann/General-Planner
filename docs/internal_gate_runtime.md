# General Planner 内部 Gate 任务

Unity 默认使用 `gate/validation_policy: corridor_only`；基础配置默认 `corridor_and_map`。两种策略的含义见文末“Gate corridor 验收策略”。

窄框地图验收、Unity完整姿态、场景配置及VLM离线恢复已更新，参见 [Gate修复与验证](gate_collision_and_unity_fixes.md)。

`planner_runtime_node` 现在把 `gate` 当作内部任务：按需打开迁移的感知节点，锁定洞口，调用 General Planner 的 SE3 优化器，验收后通过 `PlannerCommandGateway` 执行，最后用真实 odometry 确认穿越和悬停。无需启动 drone_squeeze 的规划器，也不再用外部 `START/END` 驱动状态。

## 使用

```bash
roslaunch task_planner planner_runtime.launch initial_mode:=hold
rostopic pub -1 /planner/mode_request_text std_msgs/String "data: 'gate'"
rostopic echo /planner/status
```

运行中的其他内部模式也可发送同一条 `gate` 请求；supervisor 先暂停原任务并验证悬停。`hold` 可取消当前 Gate，`emergency_stop` 沿用现有紧急停止协议。完成或失败后再次发 `gate` 会开启新 epoch，并要求新观测。完成前重复 `gate` 不会重启正在执行的任务。

runtime launch 默认 `perceptor:=true`；需要接入 `/cloud_registered`、`/lidar_slam/odom`，且观测与定位必须在同一世界坐标系。视觉选框默认打开，需要相机图像、标定/外参以及可用的 VLM 服务（检测 launch 默认 `http://127.0.0.1:18000/v1`，不负责启动模型服务）。可用 `perceptor_vlm:=false` 使用纯点云选洞，或 `perceptor:=false` 接入已有 `ApertureObservation` 发布者。

## 状态和数据流

```text
mode_request_text: gate
  → 原任务暂停、HOLD_VERIFY
  → GATE_WAIT_OBSERVATION（owner=HOLD，检测使能）
  → 连续稳定、有效、新鲜、同坐标系的洞口观测锁定
  → GATE_PLANNING（owner=HOLD，独立线程求解）
  → 轨迹/动力学/机体形状/共享地图验收
  → GATE_EXECUTING（owner=GATE，统一网关发布 /planning/pos_cmd）
  → GATE_END_VERIFY（检查实际越过出口、位置、速度和悬停持续时间）
  → GATE_COMPLETE（result=succeeded，owner=HOLD，锚定实际最终位置）
```

观测超时、无效几何、求解超时或失败、验收失败、定位失效、跟踪偏差过大、出口未到达都会中止；正常任务失败显示 `GATE_FAILED`、`task_result=failed`，并收回 HOLD。指令源超时使用现有网关保护并转入 HOLD。旧 `/planner/gate/status` 消息只产生忽略提示，不能取得控制权。

`GateFrontend` 将有序凸洞口边界变成进段、洞口段、出段三个重叠多面体。SE3 后端使用 `PolytopeSpatialMap` 映射内部节点和相邻走廊交集，优化 MINCO 位置轨迹和分段时间；姿态仍由微分平坦映射得到，并通过机体形状、推力、角速度、倾角代价反传梯度。它没有引入独立 SO(3) 姿态优化变量。固定请求时的 yaw，求解、验收和执行使用同一 yaw，避免后续独立 yaw 优化破坏形状验收。

锁定目前依据连续观测的中心、法向与面积一致性，采用检测器选出的候选，不提供多目标身份跟踪或用户指定框 ID。求解期间关闭检测并固定几何；执行期间持续检查共享地图。轨迹可视化：`/planning/gate/trajectory`。

## 地图与参数

全局地图保持原来的直接创建方式：`GlobalMapRuntime` 创建并维护唯一 ROGMap / MapManager / BoundaryMap / 全局 topo，Gate 持有同一个 `MapManager`。`shouldMaintainTopology()` 仍始终返回 true，Gate 不创建第二份全局地图，不暂停 topo，不把洞口检测结果用于重建全局 topo。

Gate 当前针对附近洞口构造局部穿越问题，不执行远距离寻门或主动转动机体搜索。进出段若有障碍，轨迹验收会拒绝，不会自动绕障重试。地图查询使用原始占据体素和有姿态的机体椭球，保守考虑机体安全间距与体素体积。

参数位于 `general_planner/config/gate_runtime.yaml`，可由 launch 的 `gate_config` 替换：

- `body_radius`、`body_height` 为椭球半轴，默认 0.165 m / 0.10 m；`margin=0.03` m。
- `wall_depth=0.30` m 是检测器不能测量厚度时的配置值，不能当作感知测量。实际墙厚需要配置匹配。
- `max_distance=5.0` m；`exit_distance=1.5` m；仅接受有序凸、近似共面的边界，拒绝近水平洞口。
- `require_known_free=true`：整条轨迹的机体占据范围必须在共享地图已知自由空间内；洞后未知空间不会当作可飞行区域。
- `observation_timeout=20` s、`planning_timeout=8` s、`odom_timeout=0.3` s；求解有迭代上限和任务取消回调。
- `speed`、`thrust_min/max`（单位 m/s²）、`body_rate`、`tilt` 定义动态限制，验收以 20 ms 间隔检查；执行命令 `thrust.z` 为总推力加速度，与现有 FSM 的 PositionCommand 合约一致。

## 回归验证

`internal_gate_integration_test` 只允许连接 `http://127.0.0.1:11381` 的隔离 ROS master。它执行真实 supervisor、GateFrontend、SE3 优化、验收和网关，以合成洞口、理想跟踪定位和注入的地图探针验证生命周期；地图提供者 revision 递增并检查维护开关始终打开。它不是实机或真实地图融合的飞行验证。

场景参数：`success`、`timeout`、`stale`、`blocked`、`external`、`cancel`、`tracking_error`。检测器点云/VLM 输入本身由 `aperture_detector/tests` 中的既有独立测试覆盖。

本次在 ROS Noetic 容器中通过：

- `planner_runtime_node`、`fsm_node` 和测试程序构建；launch 参数展开检查。
- SE3 空间映射/梯度（含倾角代价）和取消回归；supervisor 状态、网关策略和 Gate 超时/epoch 拒绝测试。
- 上述 7 个 Gate 场景；原 state2state 重试耗尽/规划超时和 tracking 生命周期回归。
- `internal_gate_detector_test.py`：合成 LiDAR 点云 → 实际迁移检测器 → 内部 SE3 → 网关 → 理想跟踪定位 → 完成反馈。视觉模型服务、真实地图融合和实机控制不在这条测试的覆盖范围内。

结果日志位于 workspace 的 `test_results/internal_gate_20260909/`。复现集成测试时，先在独立终端启动 `roscore -p 11381`，再运行：

```bash
export ROS_MASTER_URI=http://127.0.0.1:11381
rosrun general_planner internal_gate_integration_test success
python3 $(rospack find general_planner)/tests/internal_gate_detector_test.py
```


## Gate corridor 验收策略

`gate/validation_policy` 有两个取值，启动时读取并打印，其他取值会拒绝启动：

- `corridor_only`：检测多边形生成接近、穿框、离开三段 corridor，由 General Planner SE3 后端优化。跳过占用体素收紧、轨迹地图检查和执行期地图检查；不依赖 Gate 的地图就绪探针。仍进行密集的全机体 corridor、动力学验收，以及执行期轨迹预检、里程计时效、跟踪误差、框附近横向误差与实际穿出后的稳定悬停检查。
- `corridor_and_map`：保留上述验收，并使用共享地图收紧穿框 corridor，检查占用体素；`require_known_free: true` 时也拒绝未知体素。

基础 `gate_runtime.yaml` 默认 `corridor_and_map`；`unity_planner_sim.launch` 默认加载的 `gate_unity.yaml` 将其覆盖为 `corridor_only`，对应独立 SE3 穿框测试的空间假设。`require_known_free` 在 `corridor_only` 下不参与验收。修改后需要重启运行节点。

`corridor_only` 将检测多边形及配置的接近/离开盒子视为可通行空间；识别一个框并不证明框前后盒子里没有其他障碍。它不会清除未知体素、缩小机体或关闭全局建图，GlobalMapRuntime 和 topo 的维护策略不变。SE3 仍使用平坦输出与 MINCO，姿态通过加速度等导出，并参与机体形状和动力学约束，不是增加独立 SO(3) 决策变量。

回归场景 `internal_gate_integration_test corridor_only` 使用始终拒绝的地图探针，要求完整执行成功且探针调用数为零；`corridor_unknown` 使用真实但全未知的 ROG 地图，要求成功；`corridor_tracking_error` 验证移除地图验收后跟踪失效仍终止任务。`unknown` 和 `blocked` 继续验证严格模式拒绝未知/不可通行空间。测试仅在隔离的 ROS master 11381 上运行。


## 低频反馈与执行对齐

`gate/feedback_max_delay` 表示反馈到达前允许估计的额外延迟，基础默认0，Unity覆盖为0.20秒，配置范围0..0.3秒。反馈到达后在Gate中等待的时间另外按WallTime补偿。近框跟踪误差采用该有限窗口内的轨迹匹配；指令时钟不随匹配结果改变，总位置误差、消息新鲜度、执行截止与实际完成确认仍有效。实际机体还按测量四元数对corridor集合做椭球形状验收，失败为 `MEASURED_CORRIDOR_VIOLATION`。

不能用此参数掩盖任意长延迟，不能将它理解为已测出的传感器采样时刻。详细源码对比、现场证据、推力单位差异与验证方法见 [drone_squeeze 执行差异与修复](gate_squeeze_execution_comparison.md)。
