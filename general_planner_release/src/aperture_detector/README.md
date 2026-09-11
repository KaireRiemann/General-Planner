# Aperture detector / Perceptor

从 `drone_squeeze_0825/src/detector_zuanfeng` 迁移。保留原 LICENSE、视觉选框、点云平面/空洞检测、RViz、检测开关、JSON 快照以及可选旧桥接库。包名为 `aperture_detector`，不依赖原工作空间或 GCOPTER，不包含 rosbag、模型权重或模型服务。

## 启动

```bash
roslaunch aperture_detector detector.launch
# 仅点云检测（不需要模型服务）
roslaunch aperture_detector detector.launch vlm:=false
# Unity 原始输入话题
roslaunch aperture_detector polygon_hole_step_viz_unity_sim.launch
# 随当前 runtime 一起启动，复用 cloud_topic / odom_topic
roslaunch task_planner planner_runtime.launch perceptor:=true perceptor_vlm:=false
```

通用 launch 默认 `/cloud_registered`、`/lidar_slam/odom`、`world`；可覆盖 cloud_topic、odom_topic、frame_id、image_topic、camera_info_topic、config。相机外参和 ROI 在 YAML 中配置。点云必须已经位于目标系或具有观测时间的 TF。定位必须使用同一目标坐标系。

视觉依赖 ROS Python、cv_bridge、numpy、OpenCV、rospkg；VLM 通过 `vlm_base_url` 调用已运行的兼容服务（默认 localhost:18000），迁移不会自动启动或下载模型。完整 prompts 和客户端位于 scripts/windowtec_vlm_window_corners，并随 install 安装。

## 输出

- `/polygon_hole_step_viz/observation`：`aperture_detector/ApertureObservation`，几何边界、中心、法向、面积质量信息；无效/停用/点云超时发布 geometry_valid=false。
- `/polygon_hole_step_viz/hole_polygon_cloud`、`hole_center_cloud`、`step_markers`：保留原接口。
- `/polygon_hole_step_viz/planning_snapshot_json`：旧接口；现在必须检测 PASS 才发布。镜像终点仅用于旧桥接兼容。
- `/windowtec/window_box`：视觉四边形，不等于可通行区域。
- `/polygon_hole_step_viz/detection_enable`：Bool 开关。

观测时间取最新贡献点云时间，未提供时间戳时保持零；消费者应拒绝无法判断时效的观测。cloud_timeout 默认 1.5 秒（墙上时间），停止输入后清除累计窗口。私有 YAML 参数不会污染 Planner 全局参数。相邻观测尚未进行目标 ID 关联；法向符号也应由规划器按进出方向确定。

几何有效不保证机体通过，也不证明洞后无障碍；墙厚未知（depth_known=false）。当前算法仍使用平面空白区域及凸包。通用 runtime launch 默认启动按需感知。发送 `mode_request_text=gate` 后，General Planner 内部 Gate 任务锁定观测、进行 SE3 规划与共享地图验收，并经统一网关执行；详见 [内部 Gate 说明](../../../docs/internal_gate_runtime.md)。

`detection_planning_bridge.launch` 为旧外部规划器保留，未在通用 launch 启动；如使用需显式配置规划触发和控制话题。新 Planner 接口优先消费 observation。

## 验证

- `python3 -m unittest discover -s scripts/windowtec_vlm_window_corners/tests -t scripts/windowtec_vlm_window_corners`：视觉输出解析，7 项。
- 在独立 ROS master 下运行 `python3 tests/synthetic_detection_test.py`：发布合成框/实墙点云，检查中心、法向、有效状态、JSON、停用与超时。测试自己启动并清理检测节点，不启动控制器。
- `python3 tests/vlm_node_smoke_test.py`：独立 ROS master 上使用本地模拟 HTTP 服务验证图像到四边形输出，不调用真实模型。
- C++ 编译目标：`polygon_hole_step_viz`、`detection_planning_bridge`。
