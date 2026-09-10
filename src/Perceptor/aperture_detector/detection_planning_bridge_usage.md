# DetectionPlanningBridge 使用文档

`DetectionPlanningBridge` 是检测-规划桥接库，用于在其他 C++ ROS 程序中直接控制 detector 和 gcopter：

- 开启/关闭检测
- 判断检测 JSON 是否可用
- 触发 gcopter 规划
- 判断规划是否开始、运行中、完成或超时
- 同时保留一个可直接运行的 ROS 节点

## 文件位置

库接口：

```cpp
#include "detector/detection_planning_bridge.hpp"
```

实现文件：

```text
src/detector_zuanfeng/src/detection_planning_bridge.cpp
```

薄节点入口：

```text
src/detector_zuanfeng/src/detection_planning_bridge_node.cpp
```

库目标名：

```cmake
detection_planning_bridge_lib
```

## 被桥接的话题

该库默认对接以下已有话题：

| 功能 | 默认话题 | 类型 |
| --- | --- | --- |
| 控制 detector 开关 | `/polygon_hole_step_viz/detection_enable` | `std_msgs/Bool` |
| 接收检测 JSON | `/polygon_hole_step_viz/planning_snapshot_json` | `std_msgs/String` |
| 触发 gcopter 规划 | `/gate_planner/planning_trigger_odom` | `nav_msgs/Odometry` |
| 监听规划控制指令 | `/setpoints_cmd` | `quadrotor_msgs/PositionCommand` |

## 在 C++ 程序中调用

最小示例：

```cpp
#include "detector/detection_planning_bridge.hpp"

#include <ros/ros.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "my_bridge_user");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  detector::DetectionPlanningBridge bridge(nh, pnh);

  bridge.setDetectionEnabled(true);

  ros::Rate rate(20.0);
  bool planning_triggered = false;

  while (ros::ok())
  {
    ros::spinOnce();

    if (!planning_triggered && bridge.detectionReady())
    {
      bridge.startPlanning();
      planning_triggered = true;
    }

    if (bridge.planningCompleted())
    {
      ROS_INFO_THROTTLE(1.0, "Planning completed.");
    }

    rate.sleep();
  }

  return 0;
}
```

不要在构造后立即依赖 `detectionReady()`。它需要 ROS 回调收到有效检测 JSON 后才会变成 `true`。

## 可用方法

```cpp
void setDetectionEnabled(bool enabled);
bool startPlanning();

bool detectionEnabled() const;
bool detectionReady() const;
bool planningActive() const;
bool planningCompleted() const;
std::string lastState() const;
```

方法含义：

- `setDetectionEnabled(true)`：打开 detector 检测
- `setDetectionEnabled(false)`：关闭 detector 检测
- `startPlanning()`：发布 `/gate_planner/planning_trigger_odom`，触发 gcopter
- `detectionReady()`：检测 JSON 在有效时间窗口内，且检测开关开启
- `planningActive()`：已检测到新轨迹控制指令，轨迹正在执行
- `planningCompleted()`：监听到 `trajectory_flag == TRAJECTORY_STATUS_COMPLETED`
- `lastState()`：返回最近一次状态字符串

## CMake 配置

你的包需要依赖 `detector`：

```cmake
find_package(catkin REQUIRED COMPONENTS
  roscpp
  detector
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(my_bridge_user src/my_bridge_user.cpp)

target_link_libraries(my_bridge_user
  detection_planning_bridge_lib
  ${catkin_LIBRARIES}
)
```

如果你的代码直接使用 `nav_msgs`、`quadrotor_msgs` 等消息，也把它们加入 `find_package(catkin REQUIRED COMPONENTS ...)`。

## package.xml 配置

你的包至少需要：

```xml
<build_depend>detector</build_depend>
<run_depend>detector</run_depend>
```

如果你的代码直接 include 或发布订阅这些消息，也补上：

```xml
<build_depend>nav_msgs</build_depend>
<build_depend>quadrotor_msgs</build_depend>
<run_depend>nav_msgs</run_depend>
<run_depend>quadrotor_msgs</run_depend>
```

## 参数

这些参数从传入的私有 `NodeHandle pnh("~")` 读取：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `detector_enable_topic` | `/polygon_hole_step_viz/detection_enable` | detector 检测开关 |
| `planning_snapshot_topic` | `/polygon_hole_step_viz/planning_snapshot_json` | detector 发布的规划 JSON |
| `planning_trigger_topic` | `/gate_planner/planning_trigger_odom` | gcopter 规划触发 |
| `position_command_topic` | `/setpoints_cmd` | gcopter 输出控制指令 |
| `frame_id` | `world` | 触发 Odometry 的坐标系 |
| `initial_detection_enabled` | `true` | 初始化时是否开启检测 |
| `require_detection_ready` | `true` | 未收到有效检测 JSON 时是否拒绝触发规划 |
| `snapshot_timeout` | `1.5` | 检测 JSON 有效时间，单位秒 |
| `planning_start_timeout` | `3.0` | 触发后等待新轨迹开始的超时时间 |
| `command_timeout` | `1.0` | 规划运行中等待控制指令刷新的超时时间 |

## 直接运行节点

如果不想写代码，也可以直接启动薄节点：

```bash
roslaunch aperture_detector detection_planning_bridge.launch
```

控制输入：

```bash
/detection_planning_bridge/detection_enable_cmd  # std_msgs/Bool
/detection_planning_bridge/planning_start_cmd    # std_msgs/Empty
```

状态输出：

```bash
/detection_planning_bridge/detection_enabled     # std_msgs/Bool
/detection_planning_bridge/detection_ready       # std_msgs/Bool
/detection_planning_bridge/planning_active       # std_msgs/Bool
/detection_planning_bridge/planning_completed    # std_msgs/Bool
/detection_planning_bridge/state                 # std_msgs/String
```

常用命令：

```bash
rostopic pub -1 /detection_planning_bridge/detection_enable_cmd std_msgs/Bool "data: false"
rostopic pub -1 /detection_planning_bridge/detection_enable_cmd std_msgs/Bool "data: true"
rostopic pub -1 /detection_planning_bridge/planning_start_cmd std_msgs/Empty "{}"
rostopic echo /detection_planning_bridge/state
```

## 状态说明

`/detection_planning_bridge/state` 是一行字符串，包含状态名和关键变量，例如：

```text
state=PLANNING_STARTED detection_enabled=true detection_ready=true waiting_for_planning_start=false planning_active=true planning_completed=false latest_traj_id=2 active_traj_id=2 latest_trajectory_flag=1
```

常见状态：

| 状态 | 含义 |
| --- | --- |
| `DETECTION_ENABLED` | 已向 detector 发布开启检测 |
| `DETECTION_DISABLED` | 已向 detector 发布关闭检测 |
| `DETECTION_READY` | 收到有效检测 JSON |
| `DETECTION_NOT_READY` | 检测 JSON 超时或检测关闭 |
| `PLANNING_REJECTED_NO_VALID_DETECTION` | 未收到有效检测 JSON，拒绝触发规划 |
| `PLANNING_TRIGGERED` | 已发布规划触发消息 |
| `PLANNING_STARTED` | 监听到新的 `trajectory_id`，规划已开始输出 |
| `PLANNING_COMPLETED` | 收到 `trajectory_flag=3`，轨迹完成 |
| `PLANNING_START_TIMEOUT` | 触发后超时未看到新轨迹 |
| `PLANNING_COMMAND_TIMEOUT` | 规划运行中控制指令超时 |

## 注意事项

- `detectionReady()` 依赖 detector 发布的 `/polygon_hole_step_viz/planning_snapshot_json`，检测失败或未通过时不会变为 ready。
- `startPlanning()` 默认要求检测 ready；如需强制触发，可设置 `require_detection_ready:=false`。
- 规划开始通过 `/setpoints_cmd` 中新的 `trajectory_id` 判断。
- 规划完成通过 `quadrotor_msgs/PositionCommand::TRAJECTORY_STATUS_COMPLETED` 判断，对应 `trajectory_flag=3`。
- 如果 gcopter 拒绝触发，桥接库无法直接拿到拒绝原因，只能在 `planning_start_timeout` 后反馈 `PLANNING_START_TIMEOUT`；具体原因看 `gate_planning` 日志。
