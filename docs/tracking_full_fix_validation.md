# Tracking / state2state 修复验证（2026-09-10）

所有分析、修改和ROS测试在ros1_noetic容器内执行。Unity源文件从正在打开的宿主机项目复制到容器内分析；尚未写回宿主机Unity项目。

## 完成的检查

- 源码构建与release目录同步成功；压缩归档未重新生成。
- tracking_brake_self_test、planner_status_self_test、tracking_input_isolation_self_test通过。
- 隔离master 11381上的tracking supervisor生命周期测试通过。
- 5项估计器单元测试通过，含roll/pitch/yaw相机运动投影、延迟与漏检分离、失效后重新确认。
- 隔离master 11329上的估计器/预测器测试通过：静止目标与运动相机最大误差约0.0735米，丢失停止odom并清空Path，再捕获恢复；预测时域不低于0.75秒。
- 真实CPU YOLOE → bbox → EKF → Path通过，包含空检测：car=10、empty=1、odom=10、path=8。
- Unity入口、实机前端入口、release入口参数展开通过。原始Image输入和raw metric depth配置正确传入。
- git diff --check通过；源码/release前端脚本、launch和runtime YAML一致。

## 完整runtime合成闭环

使用独立master、合成地面点云、理想命令反馈；没有向用户运行的车辆master发布控制指令。
Tracking先运动再撤掉目标输入，state2state执行点到点目标。

| 模式 | 加速度峰值 m/s² | jerk峰值 m/s³ | 相邻加速度最大差 m/s² | 最大指令间隔 ms | 运动后源超时 |
|---|---:|---:|---:|---:|---:|
| tracking | 3.008 | 6.926 | 0.076 | 14.78 | 0 |
| state2state | 2.309 | 3.674 | 0.056 | 13.84 | 0 |

两种模式均有实际运动且终端速度为零。Tracking经历executing → braking → waiting_input；不再由HOLD_TRACKING断流触发硬悬停。
这些数据来自合成闭环，不是与先前Unity录包在相同场景下的严格A/B测试；不能把它当作实机或Unity实际跟踪成功率。

## 尚待现场操作

- 本轮未重启主runtime；收尾检查主master仅见rosout和rviz节点，planner runtime已不在运行。下次启动将加载新文件，主master上的实际Unity闭环尚未验证。
- Unity采集时序补丁已生成并通过patch文件匹配检查，但未写回、未经过Unity重编译与Play验证；写回会触发重编译，需用户确认是否中断当前Play。
- 实机必须填写真实相机外参、世界系、图像/odom时钟、米制深度注册或地面/尺寸标定，然后再验证静止目标、旋转相机和运动目标。
- 轨迹jerk/倾角提交检查是20ms采样验证，非连续时间证明。长期阻塞且已提交轨迹耗尽、终点非静止时仍交给网关超时保护，不会伪造永久有效的运动指令。

参数和启动命令见 [tracking_runtime_hardware_setup.md](tracking_runtime_hardware_setup.md)。
