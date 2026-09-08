#!/usr/bin/env python3
"""Read-only ROS launch expansion; no master, nodes, or flight commands."""
from pathlib import Path
import roslaunch

root = Path(__file__).resolve().parents[2]
launch = root / "task_planner/launch/planner_runtime.launch"
for initial in ("hold", "state2state", "target_exploration", "exploration"):
    for mission in ("target", "target_directed", "coverage"):
        config = roslaunch.config.ROSLaunchConfig()
        roslaunch.xmlloader.XmlLoader().load(
            str(launch), config,
            argv=["initial_mode:=" + initial,
                  "exploration_mission_mode:=" + mission, "rviz:=false"],
            verbose=False)
        def param(suffix):
            matches = [v.value for k, v in config.params.items() if k.endswith(suffix)]
            assert len(matches) == 1, (suffix, matches)
            return matches[0]
        target = mission != "coverage"
        assert param("/target_exploration/auto_workspace/enabled") is False
        assert param("/coverage_guidance/mode") == ("off" if target else "full")
config = roslaunch.config.ROSLaunchConfig()
roslaunch.xmlloader.XmlLoader().load(
    str(launch), config, argv=["initial_mode:=state2state",
                              "target_exploration_auto_workspace:=false"],
    verbose=False)
assert any(k.endswith("/target_exploration/auto_workspace/enabled") and
           v.value is False for k, v in config.params.items())
print("target workspace launch matrix passed (12 combinations + explicit override)")
