#!/usr/bin/env python3
"""Run synthetic failure tests on a private master, never the vehicle master."""
import os
import signal
import socket
import subprocess
import sys
import tempfile
import time
import xmlrpc.client

env = dict(os.environ, ROS_MASTER_URI="http://127.0.0.1:11381",
           ROS_HOSTNAME="127.0.0.1", ROS_IP="127.0.0.1")
with socket.socket() as probe:
    probe.bind(("127.0.0.1", 11381))  # Refuse to interfere with an existing master.
with tempfile.TemporaryFile() as log:
    master = subprocess.Popen(["roscore", "-p", "11381"], env=env,
                              stdout=log, stderr=log, start_new_session=True)
    try:
        for _ in range(50):
            try:
                xmlrpc.client.ServerProxy(env["ROS_MASTER_URI"]).getPid("/failure_test")
                break
            except OSError:
                if master.poll() is not None:
                    raise RuntimeError("isolated master exited")
                time.sleep(.1)
        else:
            raise RuntimeError("isolated master did not start")
        for scenario in ["failure", "deadline"]:
            subprocess.run([sys.argv[1], scenario], env=env, check=True, timeout=15)
    finally:
        os.killpg(master.pid, signal.SIGINT)
        try:
            master.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(master.pid, signal.SIGKILL)
            master.wait()
