# Tracking 录包

在 ros1_noetic 容器内，先启动 planner runtime，然后运行：

```bash
source /opt/ros/noetic/setup.bash
source /root/ws/real_planner/devel/setup.bash
cd /root/ws/real_planner/src/General-Planner
bash sh_files/record_tracking.sh
```

开始录包后再触发目标进入画面、切换 tracking 或复现问题。Ctrl+C 正常停止，等待 rosbag 退出。
默认写入 bags/tracking/，LZ4 压缩，每 10 分钟分包，缓冲区 512MB。
分析时提供本次所有分包和同名前缀的 _metadata 目录。未启动或未发布的话题不会产生消息；
订阅列表不是实际录到数据的保证，停止后用 rosbag info 检查。
运行中出现 buffer exceeded 警告可能丢帧；应检查磁盘吞吐和采集负载。

默认包含：
- RGB、深度图、CameraInfo、检测可视化及 bbox；
- 原始和滤波目标 odom、有效性、观测年龄、预测和估计状态；
- Unity 源时间戳 odom、规划器 odom、发往 Unity 的指令 odom；
- supervisor 状态、模式请求、导航/探索源指令、最终位置指令、多项式轨迹、诊断和 ROS 日志；
- 输入点云、地图及 runtime 轨迹可视化；原有实机 MAVROS/控制诊断话题仍保留。

metadata 保存录制启动时的 ROS 参数、话题/节点清单、Git commit/状态/源码差异、配置和 launch，
以及仓库外 Unity bridge 的 launch 与桥接脚本。它不是参数变化历史；
若录制中手动调参，请同时记录调整内容。参数快照可能含设备配置或凭据，对外分享前检查。

常用覆盖：

```bash
# 只预览，不录制、不创建文件
DRY_RUN=1 bash sh_files/record_tracking.sh

# 单文件，指定输出位置
BAG_DIR=/root/ws/real_planner/tracking_diagnostics NO_SPLIT=1 bash sh_files/record_tracking.sh

# 旧独立 FSM
PLANNER_NS=/fsm_node bash sh_files/record_tracking.sh

# 实机相机和目标接口：替换为实际话题
RGB_TOPIC=/camera/color/image_raw RGB_INFO_TOPIC=/camera/color/camera_info DEPTH_TOPIC=/camera/aligned_depth_to_color/image_raw DEPTH_INFO_TOPIC=/camera/depth/camera_info TARGET_ODOM_TOPIC=/tracking/target_odom bash sh_files/record_tracking.sh
```

其他开关：RECORD_IMAGES=0、RECORD_CLOUD=0、RECORD_MAP=0 可减小数据量，但会损失相应分析能力。
RECORD_DEPTH_POINTS=1 加录 Unity 深度点云；RECORD_RAW_LIDAR=1 加录原始雷达；RECORD_SWARM=1 加录集群话题。
EXTRA_TOPICS=' /vehicle/ground_truth /custom/topic ' 可以追加自定义话题（包括车辆真值）。
BBOX_TOPIC、BUFFER_MB、SPLIT_DURATION、COMPRESSION（lz4/bz2/none）、BAG_PREFIX 均可覆盖。

脚本只录已有话题，不启动检测器，不生成目标真值，不改变规划参数。
同名前缀已有录制或 metadata 时拒绝覆盖。跨版本对比请保留每次 metadata。
