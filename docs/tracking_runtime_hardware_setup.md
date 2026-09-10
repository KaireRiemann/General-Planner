# Tracking 仿真与实机配置、修复和验证

同一套 tracking_detector 负责图像 → bbox → 三维目标状态 → 预测路径。模式管理器在 tracking 模式启动它，退出时停止；实机也使用此逻辑。不要同时运行独立前端和 runtime 管理的前端。

## 本轮修复

- HOLD_TRACKING 持续输出制动轨迹到终点；不再导致网关0.3秒超时后硬切悬停。
- FSM 锁忙时，独立指令队列仅采样已提交轨迹，不访问可变任务/机器人状态；采样前后检查执行授权和 task epoch。没有上一条本 epoch 的执行指令则不输出。轨迹耗尽且终点仍在运动时不伪造持续有效命令，交回网关超时保护；正常锁路径仍负责结束状态、诊断和重规划看门狗。
- tracking 默认使用snap轨迹优化，约束初始jerk并提高分段连接连续性；静止起步预留150ms指令所有权交接时间，避免网关开始执行时已经错过轨迹起点。
- tracking 优化器使用独立的 tracking_traj_cfg。运行配置默认速度3 m/s、加速度3 m/s²、jerk12 m/s³、倾角0.45 rad、yaw速度0.8 rad/s、yaw加速度1.6 rad/s²。这些是本场景的保守起点，实机应按控制器能力和目标速度调整。
- 提交轨迹前验证速度/加速度峰值，并以20ms间隔检查jerk/倾角；软惩罚优化不能绕过验证。已有运动边界允许连续接入，避免为了新限值硬跳变。该离散检查不是连续时间jerk/倾角证明。
- tracking参数与state2state参数独立；另根据state2state独立测试的大加速度，主动将当前runtime的traj_opt/boundary调整为速度3、加速度3、jerk12、倾角0.45、角速度1.2，并将通用yaw_dot_max从3降到1.2 rad/s。其他任务配置文件未因此改写。
- 观测总年龄上限1秒、接受检测间隔上限0.45秒、odom接收间隔上限0.4秒；仍使用位置协方差阈值。将推理延迟与连续漏检分开。确认丢失后重新累积3次观测才恢复有效。
- 状态新增 validity_reason、detection_gap、position_std，避免出现 lost/accepted 却不知哪项有效性门限失败。
- 有效目标的预测时域最低0.75秒，最大1.5秒（默认dt0.25）；目标失效立即发布空Path。预测长度不是目标持续可见的保证。
- 支持原始深度16UC1/32FC1以及受支持的无损compressedDepth；JPEG显示深度始终不能当米制深度。
- 实机/Unity使用同一配置接口，启动入口贯通 tracking_rgb_compressed、tracking_camera_config、tracking_estimator_config。

## 参数放在哪里

| 内容 | 位置/入口 |
|---|---|
| 相机内参 | 实际图像对应的 CameraInfo；节点按检测输出尺寸缩放K |
| 相机外参 | camera_config YAML 中 cam2body_R、cam2body_p |
| 世界坐标系、年龄/漏检/协方差门限、已知目标高度 | estimator_config YAML |
| 地面高度、目标中心离地高度 | tracking_ground_z、tracking_target_center_height |
| 图像、CameraInfo、机体odom、深度 | runtime 的 tracking_image_topic、tracking_camera_info_topic、tracking_odom_topic、tracking_depth_topic |
| 图像是否压缩 | tracking_rgb_compressed |
| 测距模式、深度注册 | tracking_range_method、tracking_depth_registered |
| 跟踪动态约束 | planner YAML 的 general_planner/tracking/max_vel、max_acc、max_jerk、max_tilt、max_omg、yaw_rate_limit、yaw_acceleration_limit |

外参定义：p_world = p_body_world + R_body_world * (cam2body_p + cam2body_R * p_camera_optical)。
camera optical 轴为右、下、前；平移单位米。输入odom必须是机体实际反馈，不得使用发给Unity或飞控的命令odom。

estimator_config 的 output_frame 必须与输入odom真实frame一致，并与planner世界系一致。节点不会仅改frame名字来伪造坐标变换；若定位输出是其他坐标系，需要上游做真实TF变换。
图像应已去畸变，匹配的CameraInfo.D应为零；当前估计器对带畸变的原始图像会明确拒绝，避免使用错误投影。这里的raw RGB开关表示ROS未压缩Image传输，不表示允许未去畸变图像。

RGB、深度、odom时间戳必须表示同一时钟下的采集/曝光时刻；ROS输出时间戳不是原始曝光时刻，质量话题保留原始观测时间。

range_method=ground_plane 要求接地点落在可见bbox底边，ground_z对应地面；坡道/遮挡接地点会使其失效。
range_method=known_height 的 object_height 在 estimator_config 中配置，需要完整可见目标且尺寸假设适用。
range_method=depth 要求 depth_registered=true，且实际数据确实已对齐RGB并以米/毫米表示；16UC1默认每单位0.001米。
不能把仿真经验参数直接当作实机标定。

## Unity

正在打开的项目定位为 /home/diffbot/UAV-Diff-0901/UAV-Diff-0826；相关内容已复制到容器 /tmp/tracking_unity_reference 分析。
MainScene 的 RGBCamera0_ros 在机体下局部平移为零，Unity局部yaw为90度。新增 camera_unity_main_scene.yaml；unity_planner_sim 和 release planner_runtime_sim 默认使用它。
其他场景可覆盖 tracking_camera_config；不把本场景相机安装关系写死到估计器代码。

Unity当前RGB纹理读取与发布存在错帧风险。integration/unity/capture_timing.patch 修复：
RGB在采集时手动渲染，避免读取上一渲染帧；CameraImageMsgPublisher在sensor更新事件中发布，避免独立发布定时器重用旧纹理。
补丁匹配本次读取的PackageCache版本，manifest.json记录原文件校验值。PackageCache可能被Unity包更新覆盖，长期应将这些包嵌入项目版本管理。
补丁是否已写回及Unity编译结果见本轮执行记录；仅通过patch匹配不等于Unity编译/闭环通过。

## 实机：由runtime按模式管理（推荐）

源码运行先source /opt/ros/noetic/setup.bash与工作区devel/setup.bash；release建议在新终端仅source ROS和release/setup.bash，避免源码与release的同名包混用。

先准备实测 camera.yaml 和 estimator.yaml。后者可复制tracking_detector/config/estimator.yaml调整；不要复用Unity场景外参。
以下话题仅为示例，必须替换为设备实际话题：

```bash
roslaunch task_planner planner_runtime.launch \
  initial_mode:=state2state \
  odom_topic:=/localization/odom cloud_topic:=/cloud_registered \
  tracking_image_topic:=/camera/color/image_rect_color tracking_rgb_compressed:=false \
  tracking_camera_info_topic:=/camera/color/camera_info \
  tracking_odom_topic:=/localization/odom \
  tracking_depth_topic:=/camera/aligned_depth_to_color/image_raw \
  tracking_range_method:=depth tracking_depth_registered:=true \
  tracking_camera_config:=/absolute/path/to/vehicle/camera.yaml \
  tracking_estimator_config:=/absolute/path/to/vehicle/estimator.yaml
```

release 将命令首行替换为 roslaunch general_planner_release planner_runtime.launch，其余tracking参数相同。
切换到tracking后模式管理器启动前端；state2state下前端不运行。实机还应通过navigation_config设置traj_opt/boundary中的全局运动约束，这些约束控制state2state等模式；general_planner/tracking中的同名参数只覆盖tracking。不能只修改tracking约束而忽略其他模式。实机不运行unity_cmd_odom_bridge，应由已有飞控适配器执行/planning/pos_cmd，并反馈实测odom。

## 实机：独立调试前端

tracking_detector_real.launch 显式要求提供相机标定文件、图像/odom/CameraInfo/深度话题及注册声明，默认raw RGB。
例如：

```bash
roslaunch tracking_detector tracking_detector_real.launch \
  camera_config:=/absolute/path/to/vehicle/camera.yaml \
  estimator_config:=/absolute/path/to/vehicle/estimator.yaml \
  odom_topic:=/localization/odom rgb_topic:=/camera/color/image_rect_color \
  camera_info_topic:=/camera/color/camera_info \
  depth_topic:=/camera/aligned_depth_to_color/image_raw depth_registered:=true
```

只验证前端时先不启用飞控执行。检查 /tracking/status 中有效性原因，再检查目标位置是否随相机转动错误漂移；静止目标测试通过后再做运动目标闭环。

## 验收边界

合成测试覆盖标定缺失、运动相机/静止目标、延迟检测、失效清空预测、重新确认以及输入模式隔离；它不能替代实际标定和实机闭环。
本轮不通过放宽碰撞检查解决规划拒绝。遮挡、米制深度缺失、相机时序或标定未修复时，仍应可靠制动等待，不能输出看似平滑但位置错误的目标。
