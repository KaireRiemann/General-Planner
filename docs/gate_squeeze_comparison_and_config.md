# drone_squeeze 与 General Planner Gate：差异和配置设计

本文保留修复前的对照结论。后续已实现的碰撞、走廊、Unity姿态和配置改动，见 [Gate修复与验证](gate_collision_and_unity_fixes.md)。

本次对比基于工作区源码、drone_squeeze_0825 的 gate_planning.yaml，以及上一轮 Unity 现场快照；未重跑旧系统，未修改飞行参数。旧配置具体启动值还可能被其 launch/YAML 选择逻辑改变，下面列出的是当前仓库配置路径。

## 结论

两者都是位置/分段时间优化，借助微分平坦映射把机体姿态约束反传到平移轨迹；均使用 PolytopeSpatialMap，并非旧系统独立优化 SO(3)、新系统没有姿态优化。当前失败原因是 MAP_COLLISION_OR_UNKNOWN，说明已获得稳定洞口并完成求解，进入地图验收。

旧 GatePlanner::planTo 只检查最终代价有限、样条初始化及段数，再直接发布。该路径没有 General Planner 的共享 ROG 占据/未知空间验收。旧系统成功生成和播放轨迹，不能单独证明这条轨迹会通过新系统的地图和动态限制验收。

## 路径和优化模型

| 项目 | drone_squeeze 当前源码/配置 | General Planner 内部 Gate |
|---|---|---|
| 输入 | JSON 缓存 + 触发；允许当前位置距快照 odom 1 m | 请求之后的有效新观测，连续一致性检查，规划前后位置变化 <=0.08 m |
| 走廊 | 起点盒、进段、洞口段、出段、终点盒 | 进段、洞口段、出段 |
| 洞口段半长 | 配置0，实际自动下限约0.171 m | max(0.8, wall_half_depth+2*(radius+margin)+0.15)，当前0.8 m |
| 出口 | 检测 JSON 对中心做水平镜像，保持起点高度 | 沿洞口法向延伸1.5 m，通常停在洞口高度 |
| 样条/映射 | QuinticSplineND<3> + PolytopeSpatialMap | MINCO_S3<3> + PolytopeSpatialMap |
| 平坦模型 | 质量、重力、一阶/寄生阻力 | 无阻力，姿态由 a+g 推导，推力为加速度量 |
| yaw | 优化代价中固定0；播放使用ReplayYaw，默认0 | 求解、验收、指令固定同一个请求起始yaw |
| 验收 | 软惩罚优化，有限代价即可提交 | 动态/形状离散验收，原始地图占据、未知/越界验收 |
| 完成 | elapsed >= trajectory_duration + appended_duration | 实际定位越过出口、位置接近终点、低速持续悬停 |

三段/五段本身不是优劣判据，关键是切换面、重叠区、机体支持半径和起终点条件一致。尤其旧洞口段半长约0.171 m时，机体水平半轴0.165 m加0.05 m余量，水平姿态的法向支持半径为0.215 m；短段端面可能与完整机体包含约束冲突。旧优化器没有最终硬验收，不能用“有轨迹”排除此类软约束残差。

## 参数逐项对照

| 参数 | 旧配置 | 当前Gate | 处理建议 |
|---|---:|---:|---|
| 水平/竖直半轴 | 0.165/0.10 m（由全尺寸折半） | 0.165/0.10 m | 已一致；按Unity碰撞体/实机包络校准，不靠缩机体调通 |
| margin | 0.05 m | 0.03 m | 单独定义机体余量、感知误差、跟踪余量，避免重复叠加 |
| wall depth | 0.30 m | 0.30 m | 当前来自配置，不是检测测量；薄框和厚墙应分场景 |
| 速度 | 2 m/s | 2 m/s | 先保持；减速不解决静态占据包络冲突 |
| 最大角速度 | 4 rad/s | 4 rad/s | 按控制能力校准 |
| 最大倾角 | 0.78 rad | 0.78 rad | 已一致；不要把“倾斜越大越能过框”当成通则 |
| 推力下限 | 0 N | 4 m/s² | 非等价。不能直接将旧0搬来，当前代码要求>0以避免平坦奇异 |
| 推力上限 | 22.2 N | 19 m/s² | 22.2/1.168≈19.01，基本一致 |
| 时间权重 | 500 | 5 | 相差100倍，但目标与模型不同；修好几何后再做5/20/50/100/500离线扫描 |
| 走廊权重 | 2.5e6 | 2.5e6 | 已一致；提升不能修复地图验收与优化模型不一致 |
| 速度/角速度权重 | 1e4/1e4 | 1e5/1e5 | 新系统更重，但要结合目标量纲比较；应暴露配置 |
| 倾角/推力权重 | 1e5/1e5 | 1e5/1e5 | 推力单位不同，数值相同不代表代价等价 |
| 积分采样 | 16/段 | 24/段 | 与独立验收分开配置 |
| 光滑epsilon | 0.01 | 0.01 | 已一致 |
| 收敛阈值 | 1e-5 | 1e-6 | 新值更紧，不是本次地图错误原因 |
| 迭代/时间上限 | 见旧求解器默认 | 400次/8秒 | 都应显式暴露，按目标硬件测试 |
| 全轨迹验收 | 无对应共享地图检查 | 20 ms采样 | 增加最大空间步长，按速度自适应；20ms×2m/s=4cm |
| 跟踪误差上限 | 无对应内部任务验收 | 0.5 m | 对33cm高洞口太宽，应按沿法向/横向/竖向和实时剩余间隙设置 |

纯牛顿力与推力加速度惩罚具有不同尺度；即使对一个线性超限量可按质量换算权重，smooth epsilon和惩罚具体公式仍须同时核对，不能直接复制ChiVec。

## 本次地图冲突及正确修复方向

现场洞口宽约0.705 m、高约0.330 m，机体高度0.20 m、上下各0.03 m安全距离，名义竖向总占用0.26 m，只剩总计0.07 m。0.15 m全局体素已经大于剩余间隙。

当前地图检查又将安全距离和体素外接球通过两次统一比例放大并入椭球，得到约(0.429,0.429,0.260)m半轴。框边占据中心到检测中心约0.214m，小于最短半轴，因此该中心必定被拒绝。这是模型/离散化不一致，不是时间权重不够。

应使用有姿态椭球/真实机体与占据体素AABB的相交或距离检查，正确处理margin；优化器和验收采用同一机体定义。粗网格仍可能无法表达剩余间隙，建议为Gate设计2–3cm级局部碰撞表示作为离线验证起点，精度必须结合实际点云密度/噪声确认。仅把某个YAML resolution改小不会自动产生细地图，需要地图构建与查询代码支持。全局ROG/topo可保持原来的直接创建和维护；局部几何不得直接清空全局障碍或把未知设为自由。

缺测射线不能与确认无障碍混用。若设计局部精细表示，应只在有充分近期观测支撑的区域替代粗体素碰撞，保留外围占据/未知空间验收和同步版本。

## Unity执行链的独立缺口

unity_cmd_odom_bridge.py 的 _on_cmd 调用 _set_yaw(msg.yaw)，后者构造 quaternion_from_euler(0,0,yaw)，丢弃 roll/pitch 和机体角速度x/y。因此目前不能完整验证依赖倾斜的SE3穿框。需要定义明确的姿态指令合约：消费固定约定的RPY/四元数，或用同一平坦模型从a/yaw重建姿态，再由Unity实际反馈核对。普通HOLD没有完整attitude时要有明确fallback，不能无条件使用零姿态覆盖yaw。这不是本次地图验收提前拒绝的直接原因。

## 配置设计

已有 general_planner/config/gate_runtime.yaml，且 planner_runtime.launch 已通过gate_config加载；但unity_planner_sim.launch未透传gate_config及感知配置参数，仍依赖内层默认值。

建议保持一个Gate场景入口，内部按职责分开：

- gate_runtime.yaml：默认任务/观测锁定/几何/优化/验收/执行策略。
- gate_unity.yaml、gate_real.yaml：经验证的场景覆盖，机体、墙厚、控制能力独立。
- aperture_detector/config/gate_perception_unity.yaml：点云累计、ROI、图像/相机参数、VLM策略；加载到检测器命名空间。
- 局部碰撞模型配置：可由Gate配置引用，但必须由对应地图模块实际读取，不能假设它影响现有全局地图。

需要新增或结构化暴露的键（以下是设计，当前代码不读取这些新键）：

```yaml
gate:
  observation:
    stable_frames: 2
    center_tolerance: 0.08
    normal_tolerance: 0.10
    area_relative_tolerance: 0.15
  geometry:
    goal_policy: normal_exit_then_restore_height
    tunnel_half_depth: auto
    overlap: auto_body_support
  optimizer:
    weight_velocity: 100000.0
    weight_body_rate: 100000.0
    weight_tilt: 100000.0
    weight_thrust: 100000.0
    integral_steps: 24
    max_iterations: 400
    relative_cost_tolerance: 0.000001
  validation:
    body_model: ellipsoid
    collision_method: oriented_body_vs_voxel_box
    max_time_step: 0.02
    max_spatial_step: 0.01
    require_known_free: true
    report_first_failure: true
  execution:
    tracking_error_policy: gate_clearance_adaptive
    completion_hold_time: 0.5
```

现有门限中的0.08m中心一致性并不等于最终定位精度足够：它比本例单侧3.5cm名义间隙还大，应在粗关联后加更严格的边界一致性/误差估计，必要时拒绝启动。

感知侧：现有120帧窗口、至少30帧、1Hz处理，适合先累计静态点云，但限制反应速度。先记录实际帧率和噪声，再离线比较10–30帧累计、5–10帧启动、2–5Hz处理；稀疏扫描不可盲目减少累计。Gate允许目标5m而检测ROI前向上限3m，应统一到传感器实际可靠范围。VLM服务缺失时应保持节点并上报/重试；是否强制视觉必须是显式场景策略。

## 实施和验证次序

1. 先增加失败分类（占据/未知/越界、采样t/p/R、相关体素）和锁定观测/候选轨迹留档，不改变飞行限制。
2. 修复同一机体模型下的局部碰撞检查，验证实际窄框、姿态旋转、体素边界、未知区域和失败后的安全HOLD。
3. 完成Unity完整姿态执行合约；验证反馈姿态确实跟随。
4. 整理配置和Unity透传，恢复VLM服务，并核对目标选择。
5. 使用同一洞口快照、同一初终状态比较两后端；记录约束残差、最小机体间隙、时间/峰值速度/推力/角速度/倾角。先几何可行，再调时间权重和动作激进程度。
