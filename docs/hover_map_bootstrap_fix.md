# 悬停建图与探索启动修复

> 历史中间方案：其中静态确认限制长期 hit 的设计已被撤销。当前实现与验收方式见 `local_fusion_restore.md`。

## 故障来源

`runtime_20260908_023356_0.bag` 中机器人停在 `(21.5, 0, 1.5)`，原始点云持续输入，但静态过滤同时要求 10 次观测、0.9 秒持续时间和 0.4 米观察者位移。位移门槛无法在悬停时满足。过滤后空云又让 MapManager 提前退出，并阻止探索 LIO 更新，导致空闲空间和长期障碍都无法建立。

探索的 RUNNING 状态仅表示任务已接受，不保证轨迹已发布；旧 supervisor 却据此转交控制权，引发启动指令超时。探索 IKD 地图只增加点、不根据空闲证据删除点，也是独立的拖影来源。

## 修改原则

- 取消观察者位移作为静态确认门槛。旧 YAML 字段保留兼容，但不再参与判定。
- 原始点云始终进入 ROG 射线融合，时间确认只控制长期 hit 写入。过滤结果为空时照常建立空闲空间、清除旧占据。
- 未确认回波不是“没有回波”：射线不越过实际端点，本帧实际端点不接受其他射线的 miss 更新。
- 探索碰撞查询合并长期 IKD 与最新原始帧 KD。当前帧不积累成历史障碍，未通过静态确认的运动物体仍可被查询到。
- 仅在局部 ROG 明确为 KNOWN_FREE 时删除对应长期 IKD 点。遮挡、离开视场、单纯超时都不是删除静态几何的依据。
- 地图尚未就绪时保持 HOLD，缓存最后一个探索目标；地图与里程计就绪、稳定悬停后再启动任务。
- 不把可选的全局 topology 作为探索启动的强制依赖；探索自身 bubble graph 未就绪时报告 WAITING_TOPOLOGY。
- RUNNING / PLAN_TRAJ 不再触发首次控制权转交；EXEC_TRAJ / REORIENT 才允许探索输出。已有轨迹的滚动重规划保持原控制权。
- 地图 readiness 同时检查最近融合时间，避免停更后一直报告 ready。

## 回归验证入口

在 ROS Noetic 工作空间编译以下目标：

```bash
catkin_make -j3 -l8 --make-args planner_runtime_node fsm_node \
  temporal_static_filter_self_test observed_ray_fusion_self_test \
  temporal_filter_bag_replay lio_observation_fusion_self_test \
  planner_status_self_test planner_command_gateway_behavior_self_test
```

- `temporal_static_filter_self_test`：旧非零 baseline、悬停、运动点、观测间断、禁用过滤。
- `observed_ray_fusion_self_test <exploration_rog_map.yaml>`：实际 ROG 融合，验证悬停空闲空间、端点遮挡保护、静态确认和空静态流下旧占据清除。
- `lio_observation_fusion_self_test`：当前障碍即时可查询、当前帧替换不积累、遮挡静态点保留、实测空闲删除旧点。
- `temporal_filter_bag_replay <bag> <exploration_rog_map.yaml>`：只读原 bag，验证零位移静态确认，并用前 30 帧运行实际 ROG 融合。无 ROS master、发布者或执行器输出。

## 验收边界

本次验证结果（2026-09-08）：

- `planner_runtime_node`、`fsm_node`、`exploration_node` 编译通过。
- 上述过滤/融合/LIO/状态测试通过；控制网关行为测试在独立 ROS master 上通过。
- 规划取消、state2state 拓扑路径、射线裁剪、60,000 组边界/负坐标/退化射线测试通过。
- 原 bag 的 385 帧输入中，零位移下 376 帧输出确认点，第一次确认相对首帧时间为 0.963724 秒（消息时间，不是闭环启动延迟承诺）。
- 前 30 帧实际 ROG 融合后，在起点周围 10 × 10 × 1.5 米区域测得 42,541 个空闲体素、364 个占据体素；统计区域避开配置的虚拟地面。
- 本机本轮前 30 帧纯 ROG 融合累计约 5.12 秒，不能据此宣称完整 runtime 已达到 10 Hz；闭环验收还需检查 cloud age 和完整回调耗时。

时间持续性不是语义动态物体识别。物体停留足够久仍可能进入长期地图，之后需要实际射线穿过旧位置才能清除；不能为了“无拖影”凭未观测删除被遮挡的真实墙体，也不能把未确认障碍当作可穿越空间。

离线融合测试不等价于 Unity 闭环飞行验收。重启加载新二进制后，应保持机器人不动验证地图出现、下发目标验证首次轨迹及后续目标切换，再验证动态物体经过/离开后的清除。不要以先推动飞机、关闭安全检查或放宽指令超时替代上述修复。
