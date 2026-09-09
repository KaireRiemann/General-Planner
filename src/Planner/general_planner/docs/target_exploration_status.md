# Target exploration 简化状态

话题 `/planner/target_exploration/status`，类型
`general_planner/TargetExplorationStatus`，与 `/planner/status` 同频发布并锁存。

| result | 上层动作 |
| --- | --- |
| RUNNING=0 | 等待，不下发任务（也包括启动和交接等待） |
| SUCCEEDED=1 | 当前目标已成功且系统就绪，可以推进下一目标 |
| FAILED=2 | 等待恢复，不下发任务 |
| READY=3 | 可以下发任务，包括重试失败目标 |

`reason` 用于诊断，不用于控制分支。模式未启用、地图或里程计不可用
会报告 FAILED。成功后若就绪条件丢失，也不再报告允许派发的 SUCCEEDED。
不要求拓扑预先可达才能接收目标：路径可达性由任务启动后的规划处理。

BLOCKED 至少发布一次 FAILED，然后在悬停、里程计、地图及任务接收条件
满足时发布 READY。READY 的 reason 保留目标受阻信息。周期状态不是可靠
事件队列，订阅者可能错过短暂 FAILED；完整任务结果仍在 `/planner/status`。
硬故障不会仅因停车而自动恢复，需原有 Supervisor 恢复/重新启用流程。

订阅者仅在消息新鲜且 result 为 SUCCEEDED 或 READY 时派发目标。
使用 task_id/task_epoch 去重；发送后等待新任务确认，不能每次收到 READY
就重复发送。消息超时视为等待，避免使用锁存的旧就绪消息继续派发。

该话题只提供状态，不自动重试、不修改飞行控制授权。
