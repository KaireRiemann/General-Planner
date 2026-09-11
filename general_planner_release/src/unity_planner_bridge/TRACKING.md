# Unity tracking through planner_runtime

Use the source workspace in the `ros1_noetic` container:

```bash
source /root/ws/real_planner/devel/setup.bash
roslaunch unity_planner_bridge unity_planner_sim.launch
```

Start the YOLOE and target EKF nodes separately. Switch the existing runtime with:

```bash
rostopic pub -1 /planner/mode_request_text std_msgs/String "data: 'tracking'"
rostopic echo /planner/status
```

The status `active_mode_str` becomes `tracking` after the existing hover verification. With no fresh target input, tracking waits for input. State2state and tracking share the navigation command adapter (`command_owner_str=state2state` while executing); the runtime still has one publisher of the final command bus. Switch back using `state2state` or `hold` on the same request topic. Do not use `/triger` (Elastic Tracker's separate trigger) for this runtime.

The previous supervisor did not recognize `tracking` and logged `ignore unknown mode_request_text='tracking'`. The runtime now exposes mode value 6 in both PlannerModeRequest and PlannerStatus. Re-source the workspace and restart the runtime and any custom nodes that use these changed ROS message types after rebuilding.

## Unity topic connections

| Data | Topic |
|---|---|
| EKF target position/velocity | `/target_ekf_node/target_odom` |
| External target prediction | `/tracking/target_prediction` |
| Unity raw vehicle feedback | `/unity_odom` |
| Restamped vehicle feedback for planning | `/lidar_slam/odom` |
| Final position/yaw command | `/planning/pos_cmd` |
| Unity command transport (not feedback) | `/drone_0_visual_slam/odom`, `/odom` |

`unity_planner_sim.launch` exposes `tracking_target_odom_topic` and `tracking_target_prediction_topic` arguments. The generic `task_planner/planner_runtime.launch` retains its old default target topic for non-Unity callers. ROS topic overrides are applied after loading the FSM YAML.

`unity_endpoint.launch` enables `estimate_feedback_twist`: world-frame linear velocity and yaw rate are estimated from feedback pose timestamps with a 0.15 s exponential time constant. This is needed for Unity feedback whose twist is always zero. The estimator resets for duplicate/reversed timestamps or gaps over 0.5 s; it does not copy commanded velocity into feedback. Set this endpoint argument to false when Unity supplies trustworthy twist.

## Scope and validation

This change connects mode lifecycle and topics. It does not change EKF calibration, FOV penalties, target measurement freshness, or the runtime's selected navigation YAML. In particular, switching to tracking does not automatically load `tracking.yaml`: omitted settings retain their existing defaults.

Validated: runtime build, mode/owner self-test, isolated supervisor transitions (state2state/tracking/hold and stale epoch rejection), existing planning failure/deadline recovery tests, and offline feedback tests (translation, yaw wrap, stopping, time discontinuities, disabling estimation). Supervisor integration tests use ROS_MASTER_URI=http://127.0.0.1:11381 and synthetic status only. Real target tracking and visibility must still be tested in Unity.
