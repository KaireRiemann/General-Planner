# general_planner

## Vertical runtime mode

In the composed planner runtime (`serial_handover=false`), switch to `vertical`,
wait for `/planner/status` to report `active_mode_str: vertical` and
`ready_for_new_task: true`, then send a relative height in metres:

```bash
rostopic pub -1 /planner/mode_request_text std_msgs/String "data: 'vertical'"
# After the mode is ready: rise 0.5 m from the current odometry height.
rostopic pub -1 /planner/vertical/height std_msgs/Float64 "data: 0.5"
# After completion and ready_for_new_task: true: descend 0.3 m.
rostopic pub -1 /planner/vertical/height std_msgs/Float64 "data: -0.3"
```

`height` is a signed displacement, not an absolute altitude. A zero or non-finite
value is ignored. Commands outside idle `vertical` mode, including commands
received while a previous height is being admitted/executed, are rejected rather
than queued or accumulated. `/goal_3d` and click goals do not start this mode.
The typed mode request also supports `PlannerModeRequest.MODE_VERTICAL` (8).
To interrupt a motion, switch to `hold` through the existing mode interface.

The motion fixes x, y and yaw at the hover pose and constructs one seventh-order
z polynomial directly, without path search or trajectory optimization. Velocity,
acceleration and jerk are zero at both ends. The default limits are 0.5 m/s,
0.5 m/s² and 1.0 m/s³; settings are under `fsm/vertical` in
`config/task_planner_runtime_state2state.yaml`. The planner also respects its
configured dynamics/thrust envelope. Requests requiring more than `max_duration`
are rejected rather than sped up.

The complete segment, including endpoints, must pass local-map and inflated
obstacle checks. Vertical motion allows unknown space by default
(`fsm/vertical/unknown_as_occupied: false`); this does not mark unknown map cells
as observed free or change other modes. Set this option to `true` to require an
observed-free segment. Restart the runtime after changing this YAML setting.
No lateral goal projection or detour is attempted. During execution, the FSM
checks the remaining segment, odometry freshness and tracking error. Failure returns through the runtime's
existing HOLD recovery. Completion requires measured arrival and low speed;
the terminal command is held while settling, with a bounded timeout. This is an
airborne height adjustment, not a ground-contact landing/takeoff procedure.

Validation targets: `vertical_trajectory_self_test` and
`vertical_runtime_integration_test`. The latter requires an isolated ROS master
at `http://127.0.0.1:11386` and runs an ideal command-following plant.

```bash
sudo apt-get install ros-noetic-mavros* ros-noetic-octomap* ros-noetic-plotjuggler* ros-noetic-joy 
sudo apt-get install libdw-dev
```


```bash
  mkdir build
  cd build/
  cmake -DBUILD_SHARED_LIBS=TRUE ..
  make
  sudo make install
```
