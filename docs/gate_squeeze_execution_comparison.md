# drone_squeeze 与 General Planner Gate：执行差异和修复

本次比较以本机 `/home/diffbot/ros1_ws/drone_squeeze_0825/src` 源码和 General Planner 当前工作区为准，不将包名 `gcopter` 当作其内部实现的依据。现场快照见工作区 `test_results/gate_lateral_diagnosis/`。没有在同一输入上重新运行旧求解器作性能对照，也没有向正在运行的 Unity 发送飞行指令。

## 1. 当前故障处于哪一层

现场检测 geometry_valid=true，锁定框中心约 `(1.896,-0.293,0.877)`，通过验收的轨迹约5.04秒，策略 corridor_only。执行约2秒后进入 gate_failed，原因 GATE_LATERAL_TRACKING_ERROR，Gateway转HOLD，实测位置约 `(1.224,-0.110,0.967)`，距离框平面0.672米。不是求解失败，也不是地图拒绝。

该位置距已发布路径的 t=2.08 秒采样点仅0.86毫米，而失败日志距路径发布约2.20秒。静止后实测反馈8.32Hz、最大间隔0.160秒。旧检查将最新反馈位置直接与当前轨迹时刻比较；在下降段，沿轨迹滞后会成为框法向正交方向上的误差，约0.12秒时差可产生3.4厘米，超过2.5厘米阈值。由于未保存失败瞬间的连续指令与反馈，这支持时序解释，但不是精确传输延迟测量。

## 2. 完整链路的区别

| 层 | drone_squeeze 当前源码 | General Planner |
|---|---|---|
| 感知接入 | 点云/VLM检测后缓存JSON，收到 planning_trigger_odom 触发；触发时检查JSON位置与当前odom距离 | 迁移的检测节点发布 ApertureObservation；Gate请求打开检测，锁定新鲜且连续一致的多边形 |
| corridor | start_box、before_local、through、after_local、goal_box 五段，洞口侧面来自多边形 | before、through、after 三段，接近/离开盒子随起终点扩展，穿框段加长以容纳机体和重叠 |
| 参数化 | SplineSFCOptimizer，QuinticSplineND、QuadInvTimeMap、PolytopeSpatialMap | SE3AggressiveTrajOpt，MINCO_S3、PolytopeSpatialMap |
| 姿态 | 由平坦映射导出，形状/倾角/角速度约束的梯度回传样条变量 | 相同思想，姿态由平坦输出导出；两者都不是独立SO(3)优化 |
| 物理模型 | 平坦映射包含水平/竖直/寄生阻力；推力限制以N计 | 无阻力模型，推力限制以m/s²计 |
| 优化验收 | setup、有限代价、已初始化且有段；未见与当前相同的密集独立硬验收 | 密集检查形状、速度、推力、倾角、角速度以及分段边界 |
| 地图 | 此Gate执行路径不逐体素验收 | Unity corridor_only已跳过地图收紧与体素验收；严格模式可选 |
| 发布 | commandTimer 100Hz按now-trajStamp求值，直接发布PositionCommand/SO3Command | Gate按执行时钟求值，通过Supervisor/Gateway按owner和epoch统一发布 |
| 执行保护 | publishCommandsAt不根据实时跟踪横向误差中止 | 里程计超时、总误差、近框横向误差、执行时钟和退出超时等 |
| 完成 | command_t>=spline_t就发布COMPLETED；DetectionPlanningBridge据命令flag判完成 | 实际跨过框平面、到达出口、低速低偏航角速度且持续稳定才完成 |

旧代码关键位置：`gcopter_zuanfeng/gcopter/src/gate_planning.cpp` 的 `buildGateCorridor`、`planTo`、`publishCommandsAt`；`include/gcopter/spline_sfc_optimizer.hpp`；`detector_zuanfeng/src/detection_planning_bridge.cpp` 的命令回调。当前对应 `gate_frontend.cpp`、`gate_runtime.cpp`、`se3_aggressive_traj_opt.hpp`。

因此，旧程序能持续飞完整段而当前中途主动悬停，不足以归因于优化器能力差；本次直接差异是执行保护与低频反馈的结合。corridor段数和动力学模型会改变轨迹形状、姿态及耗时，但当前已有有效轨迹，没有证据表明必须先换成旧求解器或复制全部权重。

## 3. 参数不能照抄

旧默认文件：WeightT=500、SafeMargin=.05、LocalCorridorHalfWidth=.5、Height=.3、IntegralIntervs=16、RelCostTol=1e-5；当前Gate：时间权重5、margin=.03、接近/离开半宽1/半高.8、积分24、容差1e-6。不同代价实现和轨迹参数化下，权重数值不应机械对齐。

两者机体水平半轴.165、竖直半轴.10，速度2、角速度4、倾角.78。旧MaxThrust=22.2 N、mass=1.168，换算约19.0 m/s²，与当前上限19接近；旧MinThrust=0 N，当前下限4 m/s²，并不一致。旧含阻力，转换推力单位也不能使两套平坦映射完全等价。

当前PositionCommand的thrust.z沿用本项目“集体加速度”约定；旧发布世界系力向量。Unity bridge当前从acceleration、jerk、yaw重建姿态，不消费旧力向量格式。接实机控制器时应按实际消费者统一接口，不能为了与旧包字段相同就局部改单位。

## 4. 本次执行修复

1. 按反馈实际到达Gate后的WallTime年龄回退参考时刻，避免把同一条低频反馈在多次定时检查中的等待时间当作偏离。
2. `gate/feedback_max_delay` 提供额外有限延迟搜索窗口。基础默认0，Unity设置0.20秒，允许范围0..0.3秒。仅在 `[elapsed-receipt_age-max_delay, elapsed-receipt_age]` 与轨迹有效时间的交集中寻找匹配位置，采样间隔最多2ms。不改变轨迹发布时钟，不将指令位置伪装成反馈。
3. 近框误差使用匹配参考位置，并根据实际位置或匹配位置进入近框区。保留2.5厘米阈值和当前位置相对当前时刻轨迹的0.5米总误差保护。
4. 新增实际机体椭球对corridor集合的半空间验收（必须完整落在至少一个单元内，避免参考轨迹跨单元而延迟反馈仍在上一单元时误报），使用实测四元数及真实尺寸/安全余量。参考轨迹合法不再等于实测机体合法；大的姿态偏差或位置越界仍失败。
5. 失败日志包含 t、matched_t、receipt_age、max_delay、总误差、横向误差、实测和匹配参考位置。窗口随时间移动，不能通过全轨迹最近点搜索无限容忍停滞；消息新鲜度、执行截止时间、实际出口完成条件继续生效。

这是有界进度估计，不是精确重建Unity传感器采样时间。在低速/悬停时位置不能辨识延迟；消息新鲜度和任务超时仍有必要。Unity bridge目前重写接收时间戳，单纯拿这个stamp减now无法知道此前的网络/仿真延迟。

## 5. 配置与后续验证

Unity使用gate_unity.yaml，保持corridor_only、机体尺寸、margin、near_gate_lateral_error不变，新增feedback_max_delay=.20。必须重启节点使配置和新二进制生效。

长期应提高Unity真实反馈频率（建议至少30–50Hz作为测试目标），记录Unity采样序号/时间以及ROS接收时间，评估排队与时钟映射；本仓库的bridge发送100Hz不等于Unity反馈100Hz。当前没有定位到可修改的Unity C#工程，未声称已经提升反馈频率。

先做隔离回归：接近现场尺寸的低框、8.3Hz反馈、80ms额外延迟；应完整穿出并确认悬停。再注入真实横向偏差、机体旋转和停滞，必须失败。保留原corridor模式、严格地图和取消任务回归。实际Unity复测应同时录制 `/planner/status`、`/planning/pos_cmd`、`/unity_odom`、`/lidar_slam/odom`、`/planning/gate/locked_observation`、`/planning/gate/trajectory`，按新日志确认最大延迟和实际净空，不能仅凭指令轨迹画完宣布成功。

## 6. 本次验证结果

`planner_runtime_node`、`internal_gate_integration_test`、`gate_body_voxel_self_test`编译通过。几何/有限时间窗口单测通过；隔离ROS master 11381上的8项集成回归均通过：corridor_only、delayed、delayed_cross_track、delayed_stall、delayed_attitude、corridor_unknown、unknown、cancel。delayed在近现场尺寸的低框、约8.3Hz反馈和80ms额外延迟条件下完成并稳定悬停；其余故障注入分别由横向误差、总误差、实际机体corridor越界拦截。严格未知地图模式及取消任务保留原行为。日志位于工作区 `test_results/gate_feedback_alignment/`。launch展开确认Unity最终参数为corridor_only、feedback_max_delay=.20、near_gate_lateral_error=.025。

这些是隔离的理想跟踪模型测试，不代表Unity物理场景已复测成功；本次未重启现场节点或触发飞行。
