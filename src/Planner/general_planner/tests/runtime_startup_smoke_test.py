#!/usr/bin/env python3
"""Start the actual runtime without sensors on an owned, isolated ROS master.

Run after sourcing the workspace. No Unity bridge, simulator, RViz, or command
consumer is started. Tests initialization/heartbeat, not flight correctness.
"""
import collections
import os
from pathlib import Path
import signal
import socket
import subprocess
import threading
import time
import xmlrpc.client

with socket.socket() as reservation:
    reservation.bind(("127.0.0.1", 0))
    port = reservation.getsockname()[1]
env = os.environ.copy()
env["ROS_MASTER_URI"] = "http://127.0.0.1:%d" % port
os.environ["ROS_MASTER_URI"] = env["ROS_MASTER_URI"]
processes = []
output = collections.deque(maxlen=200)

def start(args):
    proc = subprocess.Popen(args, env=env, stdout=subprocess.PIPE,
                            stderr=subprocess.STDOUT, text=True,
                            start_new_session=True)
    processes.append(proc)
    def collect():
        for line in proc.stdout:
            output.append(line.rstrip())
    threading.Thread(target=collect, daemon=True).start()
    return proc

try:
    master = start(["roscore", "-p", str(port)])
    deadline = time.monotonic() + 20
    while True:
        try:
            if xmlrpc.client.ServerProxy(env["ROS_MASTER_URI"]).getPid("smoke")[0] == 1:
                break
        except OSError:
            pass
        if master.poll() is not None or time.monotonic() > deadline:
            raise RuntimeError("isolated master did not start")
        time.sleep(.1)
    launch = Path(__file__).resolve().parents[2] / "task_planner/launch/planner_runtime.launch"
    runtime = start(["roslaunch", str(launch), "initial_mode:=state2state",
                     "rviz:=false", "auto_rviz_switch:=false", "traj_server:=false"])
    import rospy
    from general_planner.msg import PlannerStatus
    rospy.init_node("runtime_startup_smoke_test", anonymous=True, disable_signals=True)
    message = rospy.wait_for_message("/planner/status", PlannerStatus, timeout=45)
    time.sleep(.5)
    # A publisher registration alone is insufficient: the FSM must finish
    # initialization and the supervisor timer must emit a real status message.
    code, _, uri = xmlrpc.client.ServerProxy(env["ROS_MASTER_URI"]).lookupNode(
        "smoke", "/planner_runtime_node")
    assert code == 1
    assert xmlrpc.client.ServerProxy(uri).getPid("smoke")[0] == 1
    assert runtime.poll() is None
    print("runtime startup / actual PlannerStatus / process alive: PASS")
except Exception:
    print("\n".join(output))
    raise
finally:
    for proc in reversed(processes):
        if proc.poll() is None:
            os.killpg(proc.pid, signal.SIGINT)
            try:
                proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL)
                proc.wait()
