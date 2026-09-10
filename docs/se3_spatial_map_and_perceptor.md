# SE3 空间映射与检测迁移

## SE3

`SE3AggressiveTrajOpt` 现在复用 `spatial_map::PolytopeSpatialMap`：时间变量保持 QuadInvTimeMap；内部点采用多面体顶点数量决定的可变维度，解码和 MINCO 梯度分别经过 toPhysical / backwardGrad，并加入映射范数惩罚。无走廊模式使用 identity 映射。

输入 hpolys 每列 `[外法向 n; 平面上一点 q]`，转换为归一化 `[n, -n.dot(q)]` 后枚举顶点。空间映射保存首顶点和相对偏移。同走廊的内部点约束在本走廊；跨走廊的内部点约束在交集中。不接受非有限几何、走廊跳跃/逆序、空交集、起终点不在首尾走廊等输入。

`piece_to_corridor` 必须与分段数量一致，覆盖从 0 到最后一个走廊的有序序列。两个现有问题构造入口保证 piece_num 至少为走廊数，避免窄缝走廊被编号映射跳过。映射只保证内部点可行，整段曲线、旋转机体和动力学仍由已有采样代价/Manager 验收处理；不改变姿态参数化、yaw 或阻力模型。

验证目标 `se3_spatial_map_self_test` 检查内部点及交集约束、MINCO 全目标梯度有限差分、无效输入拒绝、重初始化后的 identity 回退。

## Perceptor

`src/Perceptor/aperture_detector` 完整包含原检测所需源码、视觉模型客户端/提示词、配置、launch、RViz 和许可文件。详见该包 README。未迁移模型服务、原包后端规划器或控制器。

`planner_runtime.launch` 现在默认启动按需感知；`perceptor_vlm:=false` 可仅使用点云。新 `ApertureObservation` 是几何输出，旧点云和 JSON 输出保留。发送 `mode_request_text=gate` 后，由内部 GateRuntime 完成观测锁定、GateFrontend、共享地图验收、SE3 求解和统一网关执行；原外部 START/END 接管已停用。详见 [内部 Gate 使用说明](internal_gate_runtime.md)。

## 本次验证（2026-09-09）

在现有 ros1_noetic 容器中构建检测节点/桥接节点、SE3 钻框示例、空间映射测试以及 planner_runtime_node / fsm_node。GNU ld 在新测试程序链接阶段持续占用 CPU 超过数分钟，改用容器已有 gold（本机构建参数 `-DCMAKE_EXE_LINKER_FLAGS=-fuse-ld=gold`）；源码未强制指定链接器。原 workspace 使用 package whitelist，本次将 aperture_detector 加入。

- SE3 空间映射 self-test / CTest 通过（包含一次映射变量优化）。
- 视觉解析 7 项测试通过。
- 合成洞口/实墙、停止输入、检测开关和 FAIL 快照门控测试通过。
- 视觉 ROS 节点通过本地模拟模型端点测试；安装目录中的客户端和提示词也通过同一测试。
- launch 解析通过。未运行真实 VLM 识别准确率评测或飞行穿框闭环。
