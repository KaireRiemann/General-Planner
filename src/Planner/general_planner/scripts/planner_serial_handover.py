#!/usr/bin/env python3
"""Serial planner handover helper for planner_runtime.

After exploration finishes and the supervisor requests state2state, this node:
  1) kills exploration_node + highspeed_traj_server
  2) starts a standalone click-demo style fsm_node that owns /planning/pos_cmd
  3) reports readiness on /planner/handover_status

Shared world/map ownership is intentionally deferred (M2).
"""

from __future__ import print_function

import os
import shlex
import signal
import subprocess
import time

import rospy
from std_msgs.msg import String


class SerialHandover(object):
    def __init__(self):
        self.handover_command_topic = rospy.get_param(
            "~handover_command_topic", "/planner/handover")
        self.handover_status_topic = rospy.get_param(
            "~handover_status_topic", "/planner/handover_status")
        self.exploration_nodes = rospy.get_param(
            "~exploration_nodes",
            ["exploration_node", "highspeed_traj_server", "traj_server"])
        self.fsm_node_name = rospy.get_param("~fsm_node_name", "fsm_node")
        self.fsm_package = rospy.get_param("~fsm_package", "general_planner")
        self.fsm_type = rospy.get_param("~fsm_type", "fsm_node")
        self.fsm_config_path = rospy.get_param(
            "~fsm_config_path",
            rospy.get_param(
                "~navigation_config",
                "",
            ),
        )
        self.exploration_launch_pkg = rospy.get_param(
            "~exploration_launch_pkg", "task_planner")
        self.exploration_launch_file = rospy.get_param(
            "~exploration_launch_file", "exploration.launch")
        self.exploration_launch_args = rospy.get_param(
            "~exploration_launch_args", "rviz:=false marsim:=false")
        self.kill_wait_s = float(rospy.get_param("~kill_wait_s", 1.0))
        self.ready_wait_s = float(rospy.get_param("~ready_wait_s", 20.0))

        self._fsm_proc = None
        self._exploration_proc = None
        self._mode = "idle"

        self.status_pub = rospy.Publisher(
            self.handover_status_topic, String, queue_size=10, latch=True)
        rospy.Subscriber(self.handover_command_topic, String, self._on_command, queue_size=10)
        rospy.loginfo(
            "[planner_serial_handover] listening on %s (fsm_config=%s)",
            self.handover_command_topic, self.fsm_config_path)

    def _publish_status(self, text):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)
        rospy.loginfo("[planner_serial_handover] status=%s", text)

    def _node_exists(self, name):
        try:
            out = subprocess.check_output(["rosnode", "list"], text=True)
        except Exception:
            return False
        target = name if name.startswith("/") else "/" + name
        return any(line.strip() == target for line in out.splitlines())

    def _kill_nodes(self, names):
        for name in names:
            target = name if name.startswith("/") else "/" + name
            if not self._node_exists(target):
                continue
            rospy.loginfo("[planner_serial_handover] killing %s", target)
            subprocess.call(["rosnode", "kill", target])
        time.sleep(self.kill_wait_s)
        # Force-kill leftover binaries if rosnode kill left zombies.
        for name in names:
            pattern = name if not name.startswith("/") else name[1:]
            subprocess.call(
                ["bash", "-lc",
                 "pkill -f 'devel/lib/general_planner/%s' >/dev/null 2>&1 || true" % pattern])

    def _stop_fsm(self):
        if self._fsm_proc is not None and self._fsm_proc.poll() is None:
            rospy.loginfo("[planner_serial_handover] stopping fsm_node process")
            self._fsm_proc.send_signal(signal.SIGINT)
            try:
                self._fsm_proc.wait(timeout=5)
            except Exception:
                self._fsm_proc.kill()
        self._fsm_proc = None
        self._kill_nodes([self.fsm_node_name])

    def _stop_exploration_proc(self):
        if self._exploration_proc is not None and self._exploration_proc.poll() is None:
            rospy.loginfo("[planner_serial_handover] stopping exploration launch")
            self._exploration_proc.send_signal(signal.SIGINT)
            try:
                self._exploration_proc.wait(timeout=8)
            except Exception:
                self._exploration_proc.kill()
        self._exploration_proc = None

    def _start_fsm(self):
        self._stop_fsm()
        if not self.fsm_config_path:
            self._publish_status("failed: empty fsm_config_path")
            return False
        cmd = [
            "rosrun", self.fsm_package, self.fsm_type,
            "__name:=%s" % self.fsm_node_name,
            "_config_path:=%s" % self.fsm_config_path,
        ]
        rospy.loginfo("[planner_serial_handover] starting: %s", " ".join(cmd))
        self._fsm_proc = subprocess.Popen(cmd, preexec_fn=os.setsid)
        deadline = time.time() + self.ready_wait_s
        while time.time() < deadline and not rospy.is_shutdown():
            if self._node_exists(self.fsm_node_name):
                return True
            if self._fsm_proc.poll() is not None:
                self._publish_status(
                    "failed: fsm_node exited early code=%s" % self._fsm_proc.returncode)
                return False
            time.sleep(0.2)
        self._publish_status("failed: fsm_node did not appear")
        return False

    def _start_exploration_launch(self):
        self._stop_exploration_proc()
        args = shlex.split(self.exploration_launch_args)
        cmd = [
            "roslaunch", self.exploration_launch_pkg, self.exploration_launch_file,
        ] + args
        rospy.loginfo("[planner_serial_handover] starting: %s", " ".join(cmd))
        self._exploration_proc = subprocess.Popen(cmd, preexec_fn=os.setsid)
        deadline = time.time() + self.ready_wait_s
        while time.time() < deadline and not rospy.is_shutdown():
            if self._node_exists("exploration_node"):
                return True
            if self._exploration_proc.poll() is not None:
                self._publish_status(
                    "failed: exploration launch exited code=%s"
                    % self._exploration_proc.returncode)
                return False
            time.sleep(0.2)
        self._publish_status("failed: exploration_node did not appear")
        return False

    def _on_command(self, msg):
        command = (msg.data or "").strip()
        rospy.loginfo("[planner_serial_handover] command=%s", command)
        if command == "start_state2state":
            self._handle_start_state2state()
        elif command == "start_exploration":
            self._handle_start_exploration()
        else:
            rospy.logwarn("[planner_serial_handover] ignore unknown command=%s", command)

    def _handle_start_state2state(self):
        if self._mode == "state2state" and self._node_exists(self.fsm_node_name):
            self._publish_status("state2state_ready")
            return
        self._publish_status("switching_to_state2state")
        self._stop_exploration_proc()
        self._kill_nodes(self.exploration_nodes)
        if not self._start_fsm():
            self._mode = "failed"
            return
        self._mode = "state2state"
        self._publish_status("state2state_ready")

    def _handle_start_exploration(self):
        if self._mode == "exploration" and self._node_exists("exploration_node"):
            self._publish_status("exploration_ready")
            return
        self._publish_status("switching_to_exploration")
        self._stop_fsm()
        # Prefer nodes already brought up by planner_runtime.launch. Only relaunch
        # when exploration_node is absent (e.g. after a prior state2state kill).
        if self._node_exists("exploration_node"):
            self._mode = "exploration"
            self._publish_status("exploration_ready")
            return
        if not self._start_exploration_launch():
            self._mode = "failed"
            return
        self._mode = "exploration"
        self._publish_status("exploration_ready")

    def spin(self):
        rospy.on_shutdown(self._shutdown)
        rospy.spin()

    def _shutdown(self):
        self._stop_fsm()
        self._stop_exploration_proc()


def main():
    rospy.init_node("planner_serial_handover")
    SerialHandover().spin()


if __name__ == "__main__":
    main()
