#!/usr/bin/env python3
"""Mode-switched RViz for planner_runtime (chosen visualization scheme).

Exploration  -> LIO accumulated cloud (/cloud_registered) + traj markers
State2state  -> sliding ROG (/fsm_node/rog_map/*) + fsm traj markers

RViz is restarted with a dedicated config on mode change. That is cleaner
than one mixed config: the two modes publish different map/traj topics, and
ROS1 RViz cannot toggle displays externally.
"""

from __future__ import print_function

import os
import signal
import subprocess
import time

import rospy
from std_msgs.msg import String

try:
    from general_planner.msg import PlannerStatus
except ImportError:
    PlannerStatus = None


class PlannerRvizSwitcher(object):
    MODE_HOLD = 0
    MODE_STATE2STATE = 1
    MODE_EXPLORATION = 2
    MODE_TARGET_EXPLORATION = 4

    def __init__(self):
        self.rviz_node_name = rospy.get_param(
            "~rviz_node_name", "rvizvisualisation").strip("/")
        self.exploration_config = rospy.get_param("~exploration_rviz_config", "")
        self.state2state_config = rospy.get_param("~state2state_rviz_config", "")
        self.status_topic = rospy.get_param("~status_topic", "/planner/status")
        self.handover_status_topic = rospy.get_param(
            "~handover_status_topic", "/planner/handover_status")
        self.restart_cooldown_s = float(rospy.get_param("~restart_cooldown_s", 2.0))
        self.initial_mode = rospy.get_param("~initial_mode", "exploration").strip().lower()
        # Prefer latched handover in serial mode; status is secondary.
        self.prefer_handover = bool(rospy.get_param("~prefer_handover", True))

        self._desired = self._mode_key_from_name(self.initial_mode)
        self._active = None
        self._proc = None
        self._last_restart = 0.0
        self._handover_lock = None  # "exploration" | "state2state" | None

        for label, path in (
            ("exploration_rviz_config", self.exploration_config),
            ("state2state_rviz_config", self.state2state_config),
        ):
            if not path or not os.path.isfile(path):
                raise RuntimeError("%s missing: %s" % (label, path))

        if PlannerStatus is not None:
            rospy.Subscriber(
                self.status_topic, PlannerStatus, self._on_status, queue_size=1)
        rospy.Subscriber(
            self.handover_status_topic, String, self._on_handover_status, queue_size=10)

        rospy.on_shutdown(self._shutdown)
        rospy.loginfo(
            "[planner_rviz_switcher] scheme=mode-switch "
            "initial=%s exploration=%s state2state=%s",
            self._desired, self.exploration_config, self.state2state_config)

        # First start quickly; later ticks reconcile desired vs active.
        rospy.Timer(rospy.Duration(0.3), self._tick, oneshot=False)

    @staticmethod
    def _mode_key_from_name(name):
        name = (name or "").strip().lower()
        if name in ("state2state", "s2s", "navigation", "nav"):
            return "state2state"
        return "exploration"

    def _config_for(self, mode_key):
        if mode_key == "state2state":
            return self.state2state_config
        return self.exploration_config

    def _set_desired(self, mode_key, source):
        if mode_key not in ("exploration", "state2state"):
            return
        if mode_key == self._desired:
            return
        rospy.loginfo(
            "[planner_rviz_switcher] desire %s -> %s (via %s)",
            self._desired, mode_key, source)
        self._desired = mode_key

    def _on_status(self, msg):
        # During serial handover, trust /planner/handover_status more: supervisor
        # may still report exploration while fsm_node is already up (or vice versa).
        if self.prefer_handover and self._handover_lock is not None:
            return
        mode = int(msg.active_mode)
        if mode == self.MODE_STATE2STATE:
            self._set_desired("state2state", "planner_status")
        elif mode in (self.MODE_EXPLORATION, self.MODE_TARGET_EXPLORATION):
            self._set_desired("exploration", "planner_status")
        # HOLD / EMERGENCY: keep current viz.

    def _on_handover_status(self, msg):
        text = (msg.data or "").strip().lower()
        if text in ("state2state_ready", "switching_to_state2state"):
            self._handover_lock = "state2state"
            self._set_desired("state2state", "handover_status")
        elif text in ("exploration_ready", "switching_to_exploration"):
            self._handover_lock = "exploration"
            self._set_desired("exploration", "handover_status")
        elif "failed" in text:
            rospy.logwarn("[planner_rviz_switcher] handover failed: %s", text)

    def _node_exists(self, name):
        try:
            out = subprocess.check_output(["rosnode", "list"], text=True)
        except Exception:
            return False
        target = name if name.startswith("/") else "/" + name
        return any(line.strip() == target for line in out.splitlines())

    def _kill_rviz(self):
        target = "/" + self.rviz_node_name
        if self._proc is not None and self._proc.poll() is None:
            try:
                os.killpg(self._proc.pid, signal.SIGTERM)
            except Exception:
                try:
                    self._proc.terminate()
                except Exception:
                    pass
            try:
                self._proc.wait(timeout=3.0)
            except Exception:
                try:
                    os.killpg(self._proc.pid, signal.SIGKILL)
                except Exception:
                    pass
            self._proc = None

        if self._node_exists(target):
            try:
                subprocess.call(["rosnode", "kill", target])
            except Exception:
                pass
        try:
            subprocess.call(
                ["pkill", "-f", "rviz.*__name:=%s" % self.rviz_node_name])
        except Exception:
            pass
        time.sleep(0.5)

    def _start_rviz(self, mode_key):
        cfg = self._config_for(mode_key)
        cmd = [
            "rosrun", "rviz", "rviz",
            "__name:=%s" % self.rviz_node_name,
            "-d", cfg,
        ]
        rospy.loginfo(
            "[planner_rviz_switcher] start rviz mode=%s cfg=%s", mode_key, cfg)
        env = os.environ.copy()
        self._proc = subprocess.Popen(
            cmd,
            preexec_fn=os.setsid,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            env=env,
        )
        self._active = mode_key
        self._last_restart = time.time()

    def _alive(self):
        if self._proc is not None:
            if self._proc.poll() is None:
                return True
            self._proc = None
            return False
        return self._node_exists(self.rviz_node_name)

    def _tick(self, _evt):
        if self._desired is None:
            return
        if self._desired == self._active and self._alive():
            return
        if self._desired == self._active and not self._alive():
            rospy.logwarn(
                "[planner_rviz_switcher] rviz exited; restarting mode=%s",
                self._desired)
            self._active = None
        if time.time() - self._last_restart < self.restart_cooldown_s and self._active is not None:
            return
        self._kill_rviz()
        self._start_rviz(self._desired)

    def _shutdown(self):
        self._kill_rviz()


def main():
    rospy.init_node("planner_rviz_switcher")
    PlannerRvizSwitcher()
    rospy.spin()


if __name__ == "__main__":
    main()
