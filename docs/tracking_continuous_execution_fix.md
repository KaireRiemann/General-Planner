# Tracking 连续执行修复与验证

针对 tracking_20260910_085321_0.bag 的“提交轨迹后约0.2秒反复恢复悬停”问题。
源码工作区修改；未修改、构建或同步 general_planner_release。未重启用户的 Unity/runtime。

## 已修复的原因

1. TrackingCostManager原来忽略jerk及其梯度，优化阶段不约束jerk而提交阶段拒绝超限候选。
   现加入jerk代价和梯度，并加入加速度倾角锥代价；保留最终可行性检查。
2. tracking联合优化中增加yaw速度/加速度代价，并传播系数和时长梯度；
   对精确对准不可行的yaw重建尝试连续的部分转向，仍须通过后续视野检查。
3. 目标预测速度曾直接成为无人机固定终点速度。真实片段出现3.181929m/s的终点要求，
   超过无人机3m/s配置和3.15m/s提交容差。现在每个优化/恢复尝试都限制无人机终点
   速度、加速度、jerk和yaw速度，不改变原始目标预测或当前运动起点。
4. 删除短保留路径按两次keep-old计数直接失败的条件。
   已有轨迹安全、可见时允许继续执行；视野退化只允许原提交开始后最多1秒的起步宽限，
   不以重试刷新计时。仍要求新鲜预测、足够剩余时间及逐段碰撞检查。
5. 提交前FOV检查使用加速度生成的完整机体姿态及相机外参，采样目标中心与尺寸边缘，
   包含地面接地点，避免只看yaw和目标中心。
6. tracking动力学拒绝细分日志给出velocity/acceleration/jerk/tilt实际峰值及各上限；
   yaw拒绝给出角速度/角加速度峰值及上限。它们保存在诊断上下文中。
7. 初始tracking尚未授权navigation时，不再等待不存在的静止终点；运动后的HOLD终点保护保留。

## tracking配置

task_planner_runtime_state2state.yaml只修改general_planner/tracking段：
- keep_old_startup_grace=1.0s；
- 跟随距离3.5m、高度偏移0.7m、高度容差0.3m；
- Unity水平FOV75.178度、垂直60度；
- cam2body_R、cam2body_p使用与Unity前端相同的光学相机到机体约定；
- target_half_height=0.7m、target_half_width=0.5m作为可配置尺寸近似。

实机必须按相机标定同步配置planner的cam2body_R/p与tracking detector相机配置，
按CameraInfo确定FOV，按实际目标设置尺寸。这里的尺寸采样是近似视野检查，不是完整网格遮挡模型。
前端ground-contact裁切拒绝、观测超时和目标valid规则未被关闭或放宽。
state2state优化参数没有变更。

## 验证

容器中编译planner_runtime_node和tracking_dynamics_cost_self_test通过。
- jerk/倾角代价数值梯度测试通过，包括可行状态零代价。
- 持续移动且缓慢转弯的目标，保持有效6秒后撤掉输入：无非预期恢复悬停；
  最大相邻位置指令变化0.04060m，jerk峰值7.650m/s3，最终速度0。
- 使用原bag第一段有效目标预测，重建一个独立、简化地图的闭环执行场景：
  修正终点约束前5次恢复悬停；修正后没有恢复悬停，
  唯一制动为测试主动撤掉目标输入；最大指令间隔0.01553s，
  相邻位置变化0.05968m，最大加速度2.908m/s2，
  jerk约12.017m/s3（提交检查允许配置值的5%容差），最终速度0。
  个别新候选仍可能被安全检查拒绝，但不再因此无谓打断可执行的已有轨迹。
- state2state隔离回归通过：无指令超时，终点速度0，相邻位置变化0.05817m。
- git diff --check通过。

真实预测片段测试不是完整原场景回放：保留目标预测输入，但机体反馈由新指令闭环生成，
地图使用受控地面，未重放检测器图像、原场景动态障碍。不能替代Unity实景或实机验证。

## 如何复测

重启源码工作区的unity_planner_sim.launch加载新程序。
使用sh_files/record_tracking.sh录制，先观察直线，再缓慢转弯，再测试真实遮挡。
分析时提供所有bag分包与同名metadata目录。重点观察：
- 目标valid期间运动是否持续，是否再出现密集recovery_hold；
- 拒绝候选时的具体峰值/阈值；
- 目标到图像边缘的余量、接地点是否裁切；
- 真正丢失后是否连续制动并停留在新终点；
- 若目标真实持续速度高于无人机3m/s上限，仍需另行评估追赶能力，不能以虚假目标valid掩盖。

正式移动目标回归：
source /opt/ros/noetic/setup.bash
source /root/ws/real_planner/devel/setup.bash
python3 src/Perceptor/tracking_detector/tests/tracking_runtime_command_smoke.py --moving-target

该脚本仅使用自己创建的11329隔离ROS master，端口占用时拒绝运行。
诊断产物：/root/ws/real_planner/tracking_diagnostics/continuous_fix/
