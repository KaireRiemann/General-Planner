# Unity tracking through planner_runtime

**Release：使用 Unity 车辆真值检查 tracking planner**

在 `ros1_noetic` 中启动：

```bash
source /opt/ros/noetic/setup.bash
source /root/ws/real_planner/src/General-Planner/general_planner_release/setup.bash
roslaunch unity_planner_bridge unity_planner_release.launch tracking_ground_truth:=true
```

然后沿用原有模式选择，切换到 tracking：

```bash
rostopic pub -1 /planner/mode_request_text std_msgs/String "data: 'tracking'"
```

`tracking_ground_truth` 默认为 false；设为 true 时，release 自动关闭 tracking 的视觉检测/定位管理器，改用真值适配器和同一个目标预测器。FOV、碰撞检查、加速度和 jerk 约束仍然生效。测试开始时应让车辆位于无人机前方、地图覆盖范围内；真值输入不会自动转向、移动无人机到测试起点，也不会绕过规划器的可见性要求。

Unity 端安装本包 `unity/TrackingGroundTruthPublisher.cs` 到所运行工程的 `Assets/RobotScripts/`，停止 Play、执行 Assets → Refresh 后再 Play。当前工程 `/home/diffbot/UAV-Diff-0901/UAV-Diff-0826` 已安装该脚本。脚本自动附加到带有 RoadPatrol 的 `Car (1)`，同时支持 Neighborhood 的附加场景加载，无需手动加组件。原始真值在视觉模式也可以发布和录制，用于对照定位误差。

| 数据 | 话题 / 约定 |
|---|---|
| 实际车体基准点位姿 | `/unity/car_ground_truth/odom`；Unity 仿真时间；world pose、car frame twist |
| planner 使用的目标中心与世界速度 | `/tracking/ground_truth/target_odom`；ROS 接收时间；z 加 `tracking_target_center_height`，默认 0.7 m |
| 预测输入 | `/tracking/ground_truth/target_prediction`；沿用最新视觉 bag 的 0.75 s / 0.25 s 采样 |
| 输入有效性 | `/tracking/ground_truth/target_valid` |
| 来源、失效原因与输入年龄 | `/tracking/ground_truth/status`、`/tracking/ground_truth/observation_age` |

位置沿用本工程坐标系：ROS `(x,y,z)` = Unity `(x,z,y)`；原始四元数为 `(-qx,-qz,-qy,qw)`。速度由实际 Transform 逐帧位移计算，不复制 `patrolSpeed` 配置，也不使用检测框或 EKF 位置。适配器将原始局部 twist 转为 planner 所需的世界速度。Unity 原始位置是模型基准点；增加 0.7 m 是为了与当前视觉目标中心保持同一跟踪语义。

接收超时为 0.5 s。重复时间戳不会续命；Unity 停止/暂停后目标失效，预测器发空路径，planner 使用原有制动逻辑。Unity 重新 Play 导致源时钟回退时，先撤销旧输入，再接受新时间轴的后续样本。ROS 时间戳采用接收时间，不声称已消除 ROS-TCP 传输延迟。

查看输入：

```bash
rostopic echo /tracking/ground_truth/target_odom
rostopic echo /tracking/ground_truth/status
```

`sh_files/record_tracking.sh` 已加入上述真值话题。恢复视觉输入时使用 `tracking_ground_truth:=false`，或省略该参数。

Use the source workspace in the `ros1_noetic` container:

```bash
source /root/ws/real_planner/devel/setup.bash
roslaunch unity_planner_bridge unity_planner_sim.launch
```

Start the YOLOE and target EKF nodes separately. Switch the existing runtime with:

```bash
rostopic pub -1 /planner/mode_request_text std_msgs/String "data: 'tracking'"
rostopic echo /planner/status
```

The status `active_mode_str` becomes `tracking` after the existing hover verification. With no fresh target input, tracking waits for input. State2state and tracking share the navigation command adapter (`command_owner_str=state2state` while executing); the runtime still has one publisher of the final command bus. Switch back using `state2state` or `hold` on the same request topic. Do not use `/triger` (Elastic Tracker's separate trigger) for this runtime.

The previous supervisor did not recognize `tracking` and logged `ignore unknown mode_request_text='tracking'`. The runtime now exposes mode value 6 in both PlannerModeRequest and PlannerStatus. Re-source the workspace and restart the runtime and any custom nodes that use these changed ROS message types after rebuilding.

## Unity topic connections

| Data | Topic |
|---|---|
| EKF target position/velocity | `/target_ekf_node/target_odom` |
| External target prediction | `/tracking/target_prediction` |
| Unity raw vehicle feedback | `/unity_odom` |
| Restamped vehicle feedback for planning | `/lidar_slam/odom` |
| Final position/yaw command | `/planning/pos_cmd` |
| Unity command transport (not feedback) | `/drone_0_visual_slam/odom`, `/odom` |

`unity_planner_sim.launch` exposes `tracking_target_odom_topic` and `tracking_target_prediction_topic` arguments. The generic `task_planner/planner_runtime.launch` retains its old default target topic for non-Unity callers. ROS topic overrides are applied after loading the FSM YAML.

`unity_endpoint.launch` enables `estimate_feedback_twist`: world-frame linear velocity and yaw rate are estimated from feedback pose timestamps with a 0.15 s exponential time constant. This is needed for Unity feedback whose twist is always zero. The estimator resets for duplicate/reversed timestamps or gaps over 0.5 s; it does not copy commanded velocity into feedback. Set this endpoint argument to false when Unity supplies trustworthy twist.

## Scope and validation

This change connects mode lifecycle and topics. It does not change EKF calibration, FOV penalties, target measurement freshness, or the runtime's selected navigation YAML. In particular, switching to tracking does not automatically load `tracking.yaml`: omitted settings retain their existing defaults.

Validated: runtime build, mode/owner self-test, isolated supervisor transitions (state2state/tracking/hold and stale epoch rejection), existing planning failure/deadline recovery tests, and offline feedback tests (translation, yaw wrap, stopping, time discontinuities, disabling estimation). Supervisor integration tests use ROS_MASTER_URI=http://127.0.0.1:11381 and synthetic status only. Real target tracking and visibility must still be tested in Unity.
