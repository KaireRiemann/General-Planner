# Tracking 姿态晃动与指令中断诊断（2026-09-10）

分析和修改均在 ros1_noetic 容器中完成。运行中的主 ROS master 未切换模式、未重启。当前修复需下一次重启 runtime 后才会生效。

## 现场证据

采样：tracking_diagnostics/regression/oscillation.bag，34.5 秒，237 帧实际 odom、3450 条最终 PositionCommand。
指标：同目录 metrics.json；图：attitude_command_timeline.png。

| 指标 | 实测 |
|---|---:|
| 实际 roll 绝对峰值 | 24.64° |
| 实际 pitch 绝对峰值 | 34.27° |
| 加速度重建的指令倾角峰值 | 38.07° |
| 指令加速度模长峰值 | 7.00 m/s² |
| 指令 jerk 模长峰值 | 66.36 m/s³ |
| 相邻指令最大加速度跳变 | 6.17 m/s² / 11.6 ms |
| 目标有效占比（状态心跳） | 64 / 691 ≈ 9.3% |

roll/pitch 的95分位仅约0.59°/0.69°，说明大部分时间悬停，少量运动阶段有严重瞬态；不能用全程平均值掩盖。
旧 live_tracking.bag 的 pitch 峰值也约31.55°，因此不能把所有大倾角都归因于这次新增 EKF，也不能把不同运行直接当作严格 A/B 测试。

## 已确认的直接缺陷

1. 1789012775.0096：target_valid 变为 false，观测年龄0.665秒。
2. 1789012775.0532：planner 已提交1.895秒的平滑制动轨迹，随后进入 HOLD_TRACKING。
3. pubCmdTimerCallback 原本只允许 FOLLOW_TRAJ、STATIC_TRACKING、EMER_STOP，遗漏 HOLD_TRACKING；因此制动轨迹没有持续生成 PositionCommand。
4. 1789012775.3536：网关收到的最后一条导航指令已超过0.302秒，触发 source timeout。
5. 网关将速度约3.91 m/s、加速度约6.17 m/s²的最后指令直接切成当前测量位置、零速度、零加速度。
6. Unity bridge 按 a+[0,0,9.81] 重建机体姿态，因此该切换同时要求机体立即回正。它可以直接造成画面跳动，并进一步降低检测连续性。

注意：网关 owner=state2state 是 tracking 共用导航指令源的名称，并不表示录包时 active_mode 已切换到 state2state。

修复：
- 将 HOLD_TRACKING 纳入持续输出指令的执行状态。
- 不再在制动终点前50毫秒退出；等待剩余时长归零且指令采样器实际输出终点。
- 增加执行状态回归检查，保留 WAIT_GOAL/GENERATE_TRAJ 不输出执行指令的约束。
- 不延长网关超时以掩盖断流，不放宽碰撞检查，不在 Unity 中简单抹掉 roll/pitch。

## 同时存在的前端与规划问题

- 新估计器默认使用 ground_plane，但相机外参、地面高度、车辆接地点没有现场标定。检测框存在时仍可能报 ground ray near/above horizon、ground_contact_clipped、association_gate_rejected。
- 目标失效前有效观测年龄约0.40–0.43秒，默认上限0.65秒；约7 Hz输入下余量只有1–2帧，容易反复跨阈值。需分离检测、接收、处理延迟，并基于时间与协方差共同设计短时预测/再确认。不能仅无限延长超时。
- 当前 depth/compressed 是三通道 uint8 JPEG，可视化而非米制深度；不能直接用于三维测距。
- 目标预测在现场不确定度下经常只剩2–3个点，预测时域短，会加重规划频繁调整。后续应在有效性预算内提供连续的规划时间窗，并为跟踪单独设置速度、加速度、jerk 和视场约束。
- 当前共用配置 max_acc=7、max_jerk=70、max_tilt=1.05 rad，允许较激烈姿态变化。应在规划/轨迹验证层约束平顺性，而非只改显示姿态。
- 已捕获碰撞验证拒绝：OCCUPIED、segment not line-free，inside_local_map=1；不能都解释成跟丢，也不能关闭检查。
- tracking 重规划仍持有 FSM 锁，而指令线程使用 try_lock，慢规划可能造成另一类断流；需要独立的已提交轨迹采样/提交同步设计，不能简单删除锁引入竞态。

## 后续验证顺序

1. 修复生效后复跑相同 tracking 场景，录入导航原始命令、最终命令、实际 odom、target_valid、目标预测、ROS 日志。确认丢失期间制动持续输出到零速度/零加速度终点，不再因 HOLD_TRACKING 导致源超时。
2. 使用静止目标、静止/平移/旋转相机分别核对投影与时钟。运动相机下静止目标的世界坐标应稳定；获取米制深度或完成地面/外参标定后确定默认测距方式。
3. 再调整目标有效性、预测时域、重捕获确认，避免一次漏检触发停走循环，同时保证长期失效可靠停车。
4. 在上述基础上单独降低 tracking 动力学激烈程度，检查视场、避障与响应滞后的权衡；保持位置、速度、加速度、jerk 及 yaw 导数连续。
5. state2state 需另录一次实际运动，核对相同指令源超时、重规划提交处加速度跳变、当前与以前参数/二进制差异。当前这段仅有 tracking 数据，尚不足以认定 state2state 回归的同一根因。

本轮验证：source 与 release 目录构建/同步完成（压缩归档未更新）；tracking_brake_self_test、planner_status_self_test、tracking_input_isolation_self_test（含新增执行状态检查）、隔离 master 11381 上的 tracking supervisor 生命周期测试全部通过。运行中的 Unity 闭环尚未重启验证。上述后续闭环项目尚未验证完成，不能宣称 tracking 已整体修复。

后续处理与最终测试记录见 [tracking_full_fix_validation.md](tracking_full_fix_validation.md)，实机参数见 [tracking_runtime_hardware_setup.md](tracking_runtime_hardware_setup.md)。
