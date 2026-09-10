#!/usr/bin/env python3
"""Run a detector launch only while its planner mode is active."""
import os
import signal
import subprocess
import time

import rospy
from general_planner.msg import PlannerStatus


class DetectorSwitcher:
    def __init__(self):
        mode = rospy.get_param("~active_mode")
        self.mode = {"tracking": PlannerStatus.MODE_TRACKING,
                     "gate": PlannerStatus.MODE_GATE}[mode]
        self.command = ["roslaunch", rospy.get_param("~launch_file")]
        for key, value in sorted(rospy.get_param("~launch_args", {}).items()):
            if isinstance(value, bool):
                value = str(value).lower()
            self.command.append("{}:={}".format(key, value))
        self.enabled = False
        self.last_status = 0.0
        self.status_timeout = max(0.5, float(rospy.get_param("~status_timeout", 2.0)))
        self.process = None
        self.retry_after = 0.0
        self.subscriber = rospy.Subscriber(
            rospy.get_param("~status_topic", "/planner/status"),
            PlannerStatus, self.on_status, queue_size=1)
        rospy.loginfo("Detector %s waits for active planner mode", mode)

    def on_status(self, message):
        self.last_status = time.monotonic()
        self.enabled = message.active_mode == self.mode

    def stop(self):
        if self.process is None:
            return
        # Include the launched nodes, even if roslaunch exited unexpectedly.
        for sig, timeout in ((signal.SIGINT, 5.0),
                             (signal.SIGTERM, 2.0),
                             (signal.SIGKILL, 1.0)):
            try:
                os.killpg(self.process.pid, sig)
            except ProcessLookupError:
                break
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                self.process.poll()  # Reap the launcher before checking its group.
                try:
                    os.killpg(self.process.pid, 0)
                except ProcessLookupError:
                    break
                time.sleep(0.05)
            else:
                continue
            break
        self.process.wait()
        self.process = None
        rospy.loginfo("Detector launch stopped")

    def run(self):
        try:
            # Wall time also works when simulation time has not started/has paused.
            while not rospy.is_shutdown():
                if self.process is not None and self.process.poll() is not None:
                    rospy.logerr("Detector launch exited; retrying in 5 seconds")
                    self.stop()
                    self.retry_after = time.monotonic() + 5.0
                if not self.enabled or time.monotonic() - self.last_status > self.status_timeout:
                    self.stop()
                    self.retry_after = 0.0
                elif self.process is None and time.monotonic() >= self.retry_after:
                    rospy.loginfo("Starting detector: %s", self.command)
                    try:
                        self.process = subprocess.Popen(
                            self.command, start_new_session=True)
                    except OSError as error:
                        rospy.logerr("Cannot start detector: %s", error)
                        self.retry_after = time.monotonic() + 5.0
                time.sleep(0.1)
        finally:
            self.stop()


if __name__ == "__main__":
    rospy.init_node("planner_detector_switcher")
    try:
        DetectorSwitcher().run()
    except Exception:
        import traceback
        rospy.logfatal("Detector manager failed:\n%s", traceback.format_exc())
        raise
