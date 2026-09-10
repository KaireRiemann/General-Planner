# tracking_detector

General Planner 的完整 tracking 感知入口：**图像 + 无人机实际 odom → YOLOE bbox → 目标 EKF → target odom → 运动预测 Path**。

包位于 `src/Perceptor/tracking_detector`。ROS节点、bbox消息、YOLOE/MobileCLIP/CLIP运行代码及当前模型都在本包内，不需要启动或source `scene_graph_copy`、`elastic-tracker-with-label`。仍需传感器/Unity bridge发布图像与实际odom；本包不启动planner、不切换任务模式、不发布飞行命令。

## 随 planner runtime 自动启动

`task_planner/planner_runtime.launch`、`unity_planner_bridge/unity_planner_sim.launch`、`general_planner_release/planner_runtime.launch` 默认启用本前端的模式管理器。无需再单独运行本包launch。管理器等待 /planner/status 确认 active_mode 为 tracking 后启动 YOLOE、EKF、预测节点，离开 tracking 后关闭整个前端；state2state 启动不会运行这些检测节点。重新进入 tracking 会重新加载模型并重置估计状态，因此首次目标输出需要等待模型加载。源码 runtime 中 gate 检测同样仅在 gate 模式运行。

可使用 `tracking_detector:=false` 关闭自动启动（例如接入外部目标输入），用 `tracking_device:=cpu`、`tracking_image_topic`、`tracking_odom_topic`、`tracking_target_label`、`tracking_confidence`、`tracking_camera_config` 配置前端。

Unity默认使用原始 `/unity_odom` 与相机时间轴同步，planner继续使用 `/lidar_slam/odom`。基础runtime默认target输出为 `/tracking/target_odom`，Unity/release入口默认为 `/target_ekf_node/target_odom`；两端由同一个launch参数统一连线。

release目录内已包含运行代码、模型、两个C++节点及Python消息，打包脚本会通过 `sh_files/sync_tracking_detector_release.py` 更新。无需加载原感知工作区。

## 单独启动（ros1_noetic 容器内）

```bash
source /opt/ros/noetic/setup.bash
source /root/ws/real_planner/devel/setup.bash
roslaunch tracking_detector tracking_detector.launch
```

从宿主机启动：

```bash
docker exec -it ros1_noetic bash -lc 'source /opt/ros/noetic/setup.bash; source /root/ws/real_planner/devel/setup.bash; roslaunch tracking_detector tracking_detector.launch'
```

默认CPU，对应当前容器已验证的PyTorch 2.4.1+cpu。具备兼容CUDA环境时可传 `device:=cuda:0`，或 `device:=auto`。

不要同时启动原YOLOE/target_ekf/path predictor链路，它们可能向相同目标话题重复发布。

## 输入与输出

| 方向 | 默认话题 | 类型/语义 |
|---|---|---|
| 输入 | `/camera0/color/image/compressed` | sensor_msgs/CompressedImage |
| 输入 | `/unity_odom` | nav_msgs/Odometry，真实机体位姿，与图像同一时间轴 |
| 输出 | `/tracking/bboxes` | tracking_detector/BoundingBoxes，图像时间戳，640×480图像坐标 |
| 输出 | `/tracking/detections/image` | sensor_msgs/Image，检测可视化 |
| 输出 | `/target_ekf_node/yolo_odom` | nav_msgs/Odometry，bbox测距的原始目标位置 |
| 输出 | `/target_ekf_node/target_odom` | nav_msgs/Odometry，滤波目标状态 |
| 输出 | `/tracking/target_prediction` | nav_msgs/Path，默认4秒、0.25秒采样，共17点 |

默认目标话题兼容当前 `unity_planner_bridge/unity_planner_sim.launch`。直接运行默认 `task_planner/planner_runtime.launch` 时，需要给planner传 `tracking_target_odom_topic:=/target_ekf_node/target_odom`，或让本包输出 `target_odom_topic:=/tracking/target_odom`。

bbox使用本包自己的消息类型，避免依赖外部object_detection_msgs。订阅旧 `object_detection_msgs/BoundingBoxes` 的程序需调整类型和话题。nav_msgs/Odometry和nav_msgs/Path接口不变。

节点在 `/tracking_detector` 命名空间内：yoloe、target_ekf、predictor。target odom的话题名是兼容输出名称，不代表仍调用外部target_ekf节点。

## 常用参数

```bash
roslaunch tracking_detector tracking_detector.launch \
  rgb_topic:=/camera/image_raw rgb_compressed:=false \
  odom_topic:=/odometry \
  target_odom_topic:=/tracking/target_odom \
  device:=cpu confidence:=0.6 target_label:=car
```

可配置：`bbox_topic`、`visualization_topic`、`raw_target_odom_topic`、`prediction_topic`、`camera_config`、`prompt_path`、`model_path`、`yoloe_root`、`publish_visualization`、`prediction_horizon`、`prediction_dt`。

`config/prompts.txt`是检测类别清单；`target_label`必须出现在其中。`config/camera.yaml`保存当前640×480相机内参与外参，迁移没有重新标定。

可单独运行EKF/预测：`start_detector:=false`，由外部发布本包类型的bbox；可关闭预测：`start_predictor:=false`。

## 构建与运行依赖

当前容器已构建并验证，可直接启动。已有工作区含catkin白名单，新包已加入。本环境重建命令：

```bash
cd /root/ws/real_planner
source /opt/ros/noetic/setup.bash
catkin_make --pkg tracking_detector \
  -DCATKIN_WHITELIST_PACKAGES='map_manager;general_planner;general_planner_rviz_plugins;aperture_detector;tracking_detector' -j2
source devel/setup.bash
rosrun tracking_detector check_runtime.py
```

其他工作区有不同白名单时，应保留其已有包并追加tracking_detector。ROS依赖在package.xml中；推理还需要与Python/PyTorch匹配的torchvision、YOLOE及MobileCLIP/CLIP依赖。第三方运行代码已经内置，不能仅安装一个不同版本的pip ultralytics来替代。新环境可参考内置 `vendor/yoloe/pyproject.toml` 和各third_party包依赖；`check_runtime.py`检查实际导入路径和模型资产。

当前本包约661MB，其中两个模型约629MiB。模型真实存放在包内，没有指向原工作区的软链接。`.gitignore`排除了大模型；**只复制Git源码到新机器时，还需按config/model_assets.json将这两个模型文件复制到相应路径**。复制整个包目录或执行本包catkin安装规则会包含现有模型。

## 验证结果（2026-09-10）

- 当前工作区C++节点和bbox消息构建通过。
- check_runtime确认ultralytics、mobileclip、clip均从本包vendor加载。
- 独立安装到/tmp/tracking_detector_install后，再次确认三个推理模块均从安装目录加载。
- 独立ROS master端口11329，使用本包车辆图像fixture做真实CPU推理。
- 完整YOLOE → bbox → EKF → Path链路通过，空图像输出空bbox列表验证通过。
- 不曾向主ROS master发布测试传感器或控制消息。

重复测试（分别在三个终端，全部使用独立master）：

```bash
# 终端1
ROS_MASTER_URI=http://127.0.0.1:11329 roscore -p 11329
# 终端2，先source本工作区
ROS_MASTER_URI=http://127.0.0.1:11329 roslaunch tracking_detector tracking_detector.launch
# 终端3，先source本工作区
ROS_MASTER_URI=http://127.0.0.1:11329 python3 $(rospack find tracking_detector)/tests/end_to_end_smoke.py
```

测试源码和fixture供devel/source工作区使用。日志留存在容器 `/tmp/tracking_detector_build.log`、`/tmp/tracking_detector_launch.log`、`/tmp/tracking_detector_smoke.log`。

## 迁移边界

这次完成封装和独立运行，不宣称修复此前的跟踪质量问题。保留原EKF和预测算法，包括bbox高度估深、固定步长、更新速度门控、丢失/重置超时以及4秒预测策略。scene_graph的语义图、mask、VLM和点云模块不在tracking bbox/odom必需链路中，因此没有作为运行依赖搬入。

检测无结果时发布空bbox，保留图像header；模型加载失败会明确报错并结束本launch。源码来源与第三方许可证见THIRD_PARTY.md。后续可以直接在本包内修目标估计和预测问题，与planner执行连续性修复分别回归。
