# Gate 碰撞、配置与 Unity 执行修复

> 配置更新：下述地图收紧及体素拦截仅在 `gate/validation_policy: corridor_and_map` 下启用。Unity 的 `gate_unity.yaml` 现默认 `corridor_only`，采用检测框生成走廊的 SE3 流程，详见 [Gate 验收策略](internal_gate_runtime.md#gate-corridor-验收策略)。

## 已实现

1. **体素碰撞**：使用盒约束二次型的27个活跃集，计算有姿态椭球与体素AABB是否相交，包含相切。移除体素外接球导致的全轴放大。机体半轴保持0.165/0.165/0.10m；安全距离仅通过体素盒各轴外扩一次施加，边角仍保守。占据和未知体素使用相同几何判据；未关闭已知自由空间要求。
2. **优化与地图对齐**：GateFrontend读取共享地图中框附近的原始占据体素，搜索附近的水平安全锚点，并用体素盒的支持平面收紧洞口走廊。重复平面取更紧约束；未清除任何地图障碍。优化器现在能提前看到粗体素表示的框边，而不是只在最后拒绝。找不到水平锚点时保留原SE3问题，让依赖倾斜的解继续接受最终严格验收；不假定点机器人可通行。
3. **查询完整性**：ROG boxSearch略过查询边界索引，因此查询范围额外扩展两个体素，随后精确检测相交。地图外侧采用包围盒边界拒绝。
4. **失败留档**：区分MAP_OCCUPIED、MAP_UNKNOWN、MAP_CENTER_NOT_FREE和MAP_OUT_OF_BOUNDS；输出首个失败时间、机体位置、体素位置及机体z轴。`/planning/gate/locked_observation`保留锁定观测，`/planning/gate/candidate_trajectory`保留求解器产生的候选（验收失败也可查看）。`/planning/gate/trajectory`仍只发布验收通过的轨迹。
5. **Unity姿态**：bridge以acceleration/jerk/yaw/yaw_dot，按Gate相同的无阻力平坦映射重建完整四元数和三轴角速度。HOLD的零加速度自然得到水平机体加指定yaw；不依赖其他模式可能未填写的attitude字段。非法/奇异输入不会更新位置锚点。
6. **VLM启动恢复**：模型连接初始化移至图像工作线程，离线失败由既有异常处理报告，节点保持存活，后续图像处理自动重试。没有启动或安装实际模型服务；当前策略仍允许纯点云回退。
7. **配置和执行**：优化权重、积分数、迭代上限、收敛阈值、观测一致性、洞口段半长、重叠余量、出口高度策略、空间采样步长、近框横向误差和悬停确认时间均可配置。靠近框时额外限制垂直于法向的跟踪误差。默认验收同时限制时间步长20ms和参考空间步长1cm。

全局ROG/MapManager/topo仍由原GlobalMapRuntime直接创建、持续维护，没有新增全局地图或修改全局分辨率。此次不需要通过降低全局分辨率修复已记录的占据碰撞。当前稀疏/未知空间仍可能导致拒绝，这是保留的验收条件。

## 配置与使用

- `general_planner/config/gate_runtime.yaml`：完整默认Gate参数，原扁平键保持兼容。
- `general_planner/config/gate_unity.yaml`：Unity场景覆盖，默认穿越后恢复起始高度。
- `perceptor_config`：检测器配置入口；默认仍为迁移的Unity感知配置，未未经实测减少累计帧数。
- `unity_planner_sim.launch`透传`gate_config`、`gate_overlay_config`、`perceptor_config`、视觉开关、图像/标定话题、模型URL、模型名、请求超时和检测周期。

```bash
# 启动修改后的程序。没有模型服务时可明确使用纯点云。
roslaunch unity_planner_bridge unity_planner_sim.launch initial_mode:=hold perceptor_vlm:=false
rostopic pub -1 /planner/mode_request_text std_msgs/String "data: 'gate'"
```

使用视觉时设置`perceptor_vlm:=true`和`perceptor_vlm_base_url:=...`。默认请求超时10s，可通过`perceptor_vlm_timeout`按实际模型延迟调整。启动时先加载gate_config，再加载gate_overlay_config；将后者设为空字符串可不加载场景覆盖。

新增参数：`map_anchor_search_radius=0.08` / `map_anchor_search_step=0.01`约束局部安全锚点搜索；`tunnel_half_depth=0`选择根据机体尺寸和墙厚计算的下限，`overlap_slack=0.05`保留交集余量。`near_gate_lateral_error=0.025`针对窄框法向垂直平面上的跟踪误差，不替代真实碰撞检查。`restore_start_height`决定法向出口的高度是否恢复为起始高度，仍需通过完整验收。

墙厚、机体碰撞体及安全距离需要与场景匹配。`mass`保留为后端物理配置；当前加速度形式的推力/无阻力姿态不因改质量而改变上限，不能把N单位的旧推力值直接复制过来。

## 验证与边界

- 体素几何测试：窄孔、真实碰撞、相切、旋转、体素/机体包含；随机球体与AABB解析距离对照。
- 使用上次Unity保存的同一洞口及1529个占据体素离线回放。仅替换碰撞检测时仍碰框；收紧走廊后通过验收，轨迹约5.047s，速度峰值约1.288m/s。该快照没有unknown数据，这项测试只验证占据几何和动态/走廊验收。
- Gate正常穿越、地图拒绝、观测超时、过期观测、外部START/END隔离、取消、跟踪失败回归；另以真实ROG全未知地图确认t=0被MAP_CENTER_NOT_FREE拒绝。
- 合成点云经实际迁移检测器到内部SE3、网关和理想跟踪执行的串联回归。
- VLM本地服务先离线再上线的自动恢复测试；Unity完整倾斜姿态、三轴角速度、HOLD yaw和原定位反馈测试。
- 导航规划超时与tracking生命周期回归。

验证均在离线或11381隔离ROS master中进行；未重启用户正在运行的Unity栈，未向11311发出飞行/模式指令。运行中的旧进程需重启才能使用修复。不能将占据快照通过视为真实场景完整自由空间和实机跟踪已验证。
