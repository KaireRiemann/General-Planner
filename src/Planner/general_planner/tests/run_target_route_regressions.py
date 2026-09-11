#!/usr/bin/env python3
"""Own an isolated master; never publish fixture data to a vehicle master.

Run inside ros1_noetic after sourcing devel/setup.bash. --navigation runs the
existing navigation/tracking failure protocol fixtures on their required port.
"""
import argparse
import os
import signal
import socket
import subprocess
import tempfile
import time
import xmlrpc.client
from pathlib import Path

parser = argparse.ArgumentParser()
parser.add_argument("--navigation", action="store_true")
args = parser.parse_args()
port = 11383 if args.navigation else 11382
env = os.environ.copy()
env["ROS_MASTER_URI"] = "http://127.0.0.1:%d" % port
env["ROS_IP"] = "127.0.0.1"
# Fail if any other process owns the fixture master, rather than attach to it.
with socket.socket() as reservation:
    reservation.bind(("127.0.0.1", port))
directory = Path(tempfile.mkdtemp(prefix="target_route_regressions_"))
print("logs:", directory, flush=True)
with (directory / "master.log").open("w") as output:
    master = subprocess.Popen(["roscore", "-p", str(port)], env=env,
                              stdout=output, stderr=subprocess.STDOUT,
                              start_new_session=True)
    try:
        deadline = time.monotonic() + 20
        while True:
            try:
                if xmlrpc.client.ServerProxy(env["ROS_MASTER_URI"]).getPid("route_test")[0] == 1:
                    break
            except OSError:
                pass
            if master.poll() is not None or time.monotonic() >= deadline:
                raise RuntimeError("isolated master startup failed")
            time.sleep(.1)
        if args.navigation:
            cases = [("planning_failure_integration_test", arg) for arg in ("", "deadline", "tracking")]
        else:
            cases = [("target_route_runtime_self_test", ""),
                     ("target_directed_exploration_self_test", ""),
                     ("target_exploration_status_self_test", ""),
                     ("planner_status_self_test", ""),
                     ("state2state_topology_route_self_test", ""),
                     ("coverage_guidance_self_test", ""),
                     ("coverage_recovery_identity_self_test", ""),
                     ("planner_command_gateway_policy_self_test", ""),
                     ("planner_command_gateway_behavior_self_test", ""),
                     ("odometry_snapshot_self_test", ""),
                     ("target_sparse_domain_self_test", ""),
                     ("target_route_handshake_integration_test", ""),
                     ("target_route_handshake_integration_test", "deadline")]
        for binary, arg in cases:
            command = ["rosrun", "general_planner", binary] + ([arg] if arg else [])
            result = subprocess.run(command, env=env, capture_output=True, text=True, timeout=45)
            log = directory / (binary + ("_" + arg if arg else "") + ".log")
            log.write_text(result.stdout + result.stderr)
            print(binary, arg, "PASS" if result.returncode == 0 else "FAIL", flush=True)
            if result.returncode:
                print(result.stdout[-8000:] + result.stderr[-4000:])
                raise RuntimeError("fixture failed: " + str(log))
    finally:
        if master.poll() is None:
            os.killpg(master.pid, signal.SIGINT)
            try:
                master.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(master.pid, signal.SIGKILL)
                master.wait()
