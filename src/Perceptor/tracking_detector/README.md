# tracking_detector

独立 tracking 前端：YOLOE bbox → 按观测时间更新的目标状态估计 → 自适应短时预测。
代码和模型位于本包，不依赖 scene_graph 或 elastic-tracker-with-label 的运行节点。

## 随 planner 启动

源码 task_planner/planner_runtime.launch、Unity unity_planner_sim.launch、release runtime 均启用模式管理器。
仅 active_mode=tracking 时运行前端；离开模式或 planner 状态超时后关闭。
目标丢失保持 tracking 模式，检测继续运行以等待重捕获。gate 模式仍使用独立 gate detector。
tracking_detector:=false 可关闭自带前端，接入外部目标输入。

## 几何配置

新估计器强制等待有效 CameraInfo，按bbox输出分辨率缩放内参。
输入必须为无畸变/已校正图像；非零畸变系数会明确拒绝。相机外参仍由camera.yaml中的cam2body_R/p配置。

runtime/Unity/release统一参数：
- tracking_camera_info_topic：默认 /camera0/color/info
- tracking_range_method：默认 ground_plane
- tracking_ground_z：默认0，必须等于实际道路在world中的高度
- tracking_target_center_height：默认0.7，输出点相对地面的高度
- tracking_depth_topic：默认 /camera0/depth/image/compressed
- tracking_depth_registered：默认false

ground_plane 使用bbox下边缘中心与地平面求交，不再假设固定0.7m物高估深。
这是针对平面道路的模型，坡道、悬空目标、错误地面高度或相机外参会造成偏差。
该模型不能代替场景标定。近地平线、接地点出画、距离超限的观测会被拒绝。

深度已经与RGB配准且单位已验证时，设置：
tracking_range_method:=depth tracking_depth_registered:=true
auto 模式优先使用有效深度，失败后显式回退地平面，并在status中报告来源。
depth 模式绝不静默回退。支持原始16UC1(mm)/32FC1(m)、16位PNG及32FC1 compressedDepth反深度格式；
JPEG、8位可视化深度、过期深度会被拒绝。uint16单位换算参数~depth_uint16_scale默认0.001。
深度与RGB必须真正配准，分辨率相同并不能证明已经配准。
known_height 仅用于无深度/地面约束时的对照，~object_height必须按目标标定（默认1.4m）。

独立运行：roslaunch tracking_detector tracking_detector.launch
独立launch参数去掉runtime参数的tracking_前缀，例如 range_method:=depth。
仍可设置device、confidence、target_label、camera_config、rgb_topic、odom_topic等。

## 估计与时间

YOLOE仅订阅图像，使用长度1的最新帧队列，不因缺odom停止检测。
bbox保留图像采集stamp。估计器缓存历史odom，平移插值、四元数SLERP，
将延迟观测变换到同一world坐标，再按实际观测dt更新常速度Kalman滤波。
Joseph协方差更新、创新门控与空间关联抑制异常点和跳目标。
目标必须连续获得3次有效观测才允许规划；失去观测最多外推0.65秒，
同时检查位置不确定度和odom输入超时。乱序观测丢弃，时钟回跳清空缓存。
输入图像/odom必须使用同一采集时钟。滤波状态使用采集时钟；
通过最新odom与接收时间的局部对应关系计算年龄、外推到发布时刻。
该映射不能消除未知网络传输延迟，不是硬件时钟同步。
输出odom使用ROS时钟，真实观测stamp/年龄单独发布，避免“新发布等于新观测”。

## 输出与失效协议

- /tracking/bboxes：本包BoundingBoxes
- /tracking/detections/image：带bbox、tracking状态、观测年龄和定位方式的图像
- /target_ekf_node/yolo_odom：原始三维观测（ROS发布时刻；采集stamp见status）
- /target_ekf_node/target_odom：估计位置/速度及协方差；无效时不发布
- /tracking/target_valid：latched Bool，有效性心跳
- /tracking/observation_age：Float64，真实观测年龄；未观测时为inf
- /tracking/status：JSON String，state/valid/track_id/observation_stamp/observation_age/range_method/reason
- /tracking/target_prediction：Path，默认上限1.5秒、0.25秒等间距采样；
  随年龄和协方差缩短，默认关闭转向外推，先使用稳定的短时匀速模型。

预测器要求有效性心跳与新鲜odom；目标失效/输入超时发布空Path。
planner将空Path解释为显式撤销旧预测，并抑制迟到odom将其复活。
外部输入需采用ROS时钟并提供有限数值；外部预测器同样可发送空Path撤销目标。

保留 target_ekf_node C++ 可执行文件用于旧算法对照，默认launch已改用
target_state_estimator.py，不能同时启动新旧估计器向同一话题发布。

## 验证与边界

tests/test_estimation.py：真实dt滤波、创新门控、世界坐标不变性、位姿插值、深度有效性。
tests/estimator_ros_test.py --predictor：自带独立master11329，检测延迟、ego运动、
丢失撤销和确认重捕获，不发布到用户主master。
tests/end_to_end_smoke.py：真实CPU YOLOE推理全链路，需独立master和本包launch。
新测试与实际运行记录见tests/VALIDATION.md。

模型真实存放于vendor/yoloe；模型文件被.gitignore排除，
仅复制Git源码时需按config/model_assets.json补充权重。
容器已有的PyTorch/YOLOE依赖沿用，不自动安装或升级GPU环境。
源码来源和许可见THIRD_PARTY.md。未宣称已完成未知Unity挂载标定或实机飞行验收。

## 仿真与实机配置

完整参数定义与实机启动见 [RUNTIME_SETUP.md](RUNTIME_SETUP.md)。runtime通过tracking_camera_config、tracking_estimator_config和tracking_rgb_compressed传入配置；实机独立调试使用tracking_detector_real.launch。Unity MainScene使用camera_unity_main_scene.yaml，实机必须提供实测外参。
