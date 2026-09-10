# Tracking 前端与闭环修复（2026-09-10）

## 本次修改

1. 默认前端改用 tracking_detector/scripts/target_state_estimator.py。
   CameraInfo内参、历史odom平移插值/SLERP、采集时钟上的真实dt滤波、
   创新门控、空间关联、多帧确认、协方差和真实观测年龄。
   bbox保留采集stamp，odom输出采用ROS时钟，原始采集stamp保留在/tracking/status。
   网络延迟无法仅靠接收时刻精确消除；本次处理的是已缓存观测与位姿之间的错位，
   不宣称实现硬件时钟同步。

2. YOLOE不再等待odom才检测；保留最新帧队列，不累积推理积压。
   检测图像叠加状态、观测年龄和定位来源。显示不依赖planner是否正在运动。

3. 支持显式配置的几何定位：
   - ground_plane：bbox地面接触点与指定地平面相交，输出指定目标中心高度。
   - depth：需要真实注册到RGB的米制深度，支持16UC1/32FC1与对应无损传输，
     严格检查时间、有效像素、编码；使用中央前景深度统计抑制背景。
   - auto：有效深度优先，否则显式回退地面约束。
   - known_height：仅保留物高模型作对照，不再默认使用0.7m车高。
   默认ground_plane/ground_z=0/target_center_height=0.7。
   这依赖道路平面和相机外参假设；Unity实际相机挂载、道路高度仍需核对。
   未验证的深度不会被自动当作距离使用。

4. 预测上限由4秒改为1.5秒，随年龄与协方差缩短；默认禁用噪声敏感的转向外推。
   估计器最多允许0.65秒无观测外推，同时检查位置不确定度与odom输入失联。
   预测器要求有效性心跳，输入超时或失效时发布空Path。
   planner将空Path当作撤销目标，防止旧预测缓存持续驱动规划；
   迟到odom在Path有效性窗口内不能将撤销的目标复活。
   有时间戳的Path使用实际采样间隔计算速度/加速度，不再盲用本地默认dt。

5. planner新增TRACKING_BRAKING/TRACKING_LOST适配器状态：
   目标失效时生成一次制动轨迹，制动完成后保持tracking模式等待确认重捕获。
   /planner/status分别报告braking/waiting_input及明确原因，不再始终executing。
   新目标在制动结束后恢复规划，避免中途取消制动导致反复切换。
   丢失不关闭detector，也不自动把车辆任务记成完成。

6. tracking hold改为七次多项式制动：
   起点采用已提交指令在衔接时刻的p/v/a/j，终点v/a/j归零。
   自适应增加时长以满足加速度、jerk、yaw速度/加速度约束，
   检查完整制动路径的地图安全性；不安全时走已有emergency分支，
   不以require_safe=false绕过检查。
   这解决的是旧实现将运动指令直接替换为常位置轨迹的突变。

7. yaw优先生成朝向目标位置的轨迹，与目标车体heading区分。
   使用候选轨迹起始时刻的旧yaw状态，minimum-snap插值，
   前端与最终提交均应用tracking专用角速度限制；
   默认yaw_rate_limit=1.2rad/s，yaw_acceleration_limit=2.4rad/s²。
   参数位于规划配置general_planner/tracking，其他任务的yaw限制保持原配置。
   不能满足FOV/动态限制的候选不强行提交。

## 启动

继续使用原Unity或runtime启动命令。进入tracking后自动启动新前端；
state2state、gate与模式启停流程保留。release目录同步同一实现。
可配置：
tracking_camera_info_topic、tracking_range_method、tracking_depth_registered、
tracking_depth_topic、tracking_ground_z、tracking_target_center_height。
完整说明见src/Perceptor/tracking_detector/README.md。

## 已取得的验证证据

- 纯滤波/几何单元测试：非固定dt速度估计、乱序拒绝、异常观测门控、
  相机转动下世界坐标不变、SLERP、深度注册与时间有效性。
- 独立master11329合成闭环：约150ms检测延迟，相机平移并转动，
  静止目标位置最大误差约0.075m；丢失停止发布有效odom，空Path撤销；
  再出现经多帧确认恢复，track_id递增。
- 真实CPU YOLOE全链路通过，包含car检出、估计odom、预测及空检测。
- 旧现场bag前12秒以原接收节奏回放新前端：能生成目标估计，
  无观测后转lost并持续撤销预测。该回放不改变已录制的飞行运动，
  不能用它证明新闭环已经不会跟丢。
- 制动多项式单元测试检查起点p/v/a/j与终点v/a/j，多个时长通过。

## 下一次Unity运行的验收

首先核对camera.yaml外参与道路高度；然后分别进行静止目标/机体转动、
车辆直行、车辆转弯、短遮挡、长遮挡再出现。
同时录制CameraInfo、图像/bbox、实际odom、目标odom/quality、Path、
planner/status和最终pos_cmd。车辆真实世界位姿只作为独立真值对照。
比较视野内检出率、目标坐标误差、观测年龄、视线误差、丢失/恢复时间、
位置指令及yaw的连续性。不能以显示更平滑代替实际位置正确。

尚未宣称完成实际Unity相机挂载标定、新闭环运行或实机验收。
当前重捕获是在安全保持状态下继续检测并等待多帧确认；
没有在未知障碍与未知目标方位下自动旋转飞行器搜索。

## 最终验证记录

- runtime、预测器及release所需二进制构建通过；release目录已同步，未刷新tar.gz归档。
- tracking_brake_self_test、planner_status_self_test、tracking_input_isolation_self_test通过。
  输入测试包含空Path撤销、迟到odom抑制、同模式恢复、按Path消息时间计算速度及坏时间拒绝。
- 独立测试master11381上的planning_failure_integration_test tracking/default/deadline均通过；
  tracking分支覆盖braking→lost→planning→executing，保持mode/epoch不变。
- 最终估计器+预测器ROS回归再次通过，合成运动相机最大误差0.0735m以内。
- Unity/source/release launch解析、Python编译、source/release估计脚本SHA256一致性、
  git diff --check通过。
- 构建日志：容器/tmp/tracking_optimization_release.log；
  supervisor测试日志：/tmp/planning_failure_integration_test*_optimized.log。
- 没有接管用户主master进行合成传感器/飞行指令测试，也没有重启用户的Unity进程。
