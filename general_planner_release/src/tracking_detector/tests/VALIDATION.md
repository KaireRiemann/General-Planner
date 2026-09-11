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

## 优化版（2026-09-10）
新估计器和预测器的验证以 docs/tracking_frontend_optimization.md 为准。
test_estimation.py覆盖滤波、几何、异常输入；estimator_ros_test.py --predictor
自带master11329，验证移动相机+延迟检测、丢失失效和确认恢复。
旧版4秒/17点Path的固定断言已改为自适应短预测。
真实CPU smoke通过：car=8、empty=1、odom=6、path=5。
优化前后现场bag没有目标真值，未报告位置RMSE改善百分比。

最终验证：4项纯估计测试、移动相机延迟观测ROS回归、
制动C3边界、planner输入撤销/真实采样间隔、
tracking状态转换、state2state失败/超时恢复均通过。
release必要二进制及前端同步完成，tar.gz归档未刷新。
