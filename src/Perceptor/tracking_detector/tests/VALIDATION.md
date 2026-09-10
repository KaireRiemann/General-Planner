# Validation 2026-09-10

Container: ros1_noetic. ROS Noetic, Python 3.8, PyTorch 2.4.1+cpu.

- catkin_make --pkg tracking_detector: PASS (messages and both C++ executables).
- Python scripts compile: PASS.
- check_runtime in devel workspace: PASS, all three inference modules within package.
- Real YOLOE inference smoke on isolated ROS master 11329: PASS. Recorded counters: car=4, empty=1, target odom=8, prediction path=7. Checks cover message timestamps, image coordinate bounds, finite target position, world frame and 17-point/4-second prediction.
- Initial smoke fixture from inherited ultralytics/assets/bus.jpg was discovered to be an all-zero placeholder. It correctly produced empty detections. The committed fixture instead uses a recorded vehicle image (see THIRD_PARTY.md).
- CMake package install into /tmp/tracking_detector_install: PASS, including model assets and ROS executables/messages.
- check_runtime in staged install: PASS, ultralytics/mobileclip/clip loaded from installed vendor directory.
- roslaunch --nodes: exactly tracking_detector/yoloe, tracking_detector/target_ekf, tracking_detector/predictor.
- Isolated test launch and master stopped after validation. No test traffic sent to the primary ROS master.

This validates packaging/inference/wiring, not the previously identified tracking quality fixes. EKF and prediction algorithms were preserved.

## 模式生命周期回归（2026-09-10）

在独立 ROS master 11329 使用真实 roslaunch 子进程和轻量 rostopic 检测替身，
发布 general_planner/PlannerStatus，验证：
- state2state 不启动任何 detector。
- tracking 仅启动 tracking 子 launch。
- tracking → gate 关闭 tracking，启动 gate。
- gate → state2state 关闭 gate。
- 再次进入 tracking 能重新启动。
- 管理器 SIGINT 退出会清理子 launch 及节点。

源码 runtime、Unity、release runtime/sim launch 解析通过；关闭 tracking_detector/perceptor 后不加载管理器。
这组回归验证进程生命周期，不代表重新验证 YOLOE 推理或跟踪质量。
日志：容器 /tmp/detector_mode_test_*.log。
