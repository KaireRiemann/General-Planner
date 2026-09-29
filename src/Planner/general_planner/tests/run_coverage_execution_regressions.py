#!/usr/bin/env python3
"""Run coverage policy and real MINCO regressions on an isolated ROS master."""
import argparse
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import time
import xmlrpc.client

parser = argparse.ArgumentParser()
parser.add_argument('--output', required=True)
args = parser.parse_args()
output = Path(args.output)
output.mkdir(parents=True, exist_ok=True)
package = Path(__file__).resolve().parents[1]
with socket.socket() as reservation:
    reservation.bind(('127.0.0.1', 0))
    port = reservation.getsockname()[1]
os.environ['ROS_MASTER_URI'] = 'http://127.0.0.1:%d' % port
os.environ['ROS_IP'] = '127.0.0.1'
results = {}
with (output / 'master.log').open('w') as log:
    master = subprocess.Popen(['roscore', '-p', str(port)], stdout=log,
                              stderr=subprocess.STDOUT, start_new_session=True)
    try:
        deadline = time.monotonic() + 20
        while True:
            try:
                if xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI']).getPid('coverage_regression')[0] == 1:
                    break
            except OSError:
                pass
            if master.poll() is not None or time.monotonic() > deadline:
                raise RuntimeError('isolated ROS master did not start')
            time.sleep(.1)
        import rosparam
        import yaml
        config = yaml.safe_load((package / 'config/exploration_house.yaml').read_text())
        config.update(yaml.safe_load((package / 'tests/config/legacy_coverage_parameters.yaml').read_text()))
        config.update({'box_num': 1, 'box_0/down': [-12., -6., -.2],
                       'box_0/up': [12., 6., 4.2], 'dead_area_num': 0,
                       'RogMapConfigPath': str(package / 'config/exploration_rog_map.yaml')})
        rosparam.upload_params('/coverage_observation_integration_test', config)
        cases = [(name, []) for name in [
            'coverage_motion_policy_self_test', 'coverage_recovery_identity_self_test',
            'coverage_route_policy_self_test', 'coverage_guidance_self_test',
            'exploration_tour_self_test', 'topology_update_budget_self_test',
            'target_sparse_domain_self_test', 'target_route_runtime_self_test']]
        cases += [('coverage_observation_integration_test', []),
                  ('coverage_observation_integration_test', ['--continuity'])]
        for name, options in cases:
            label = name + ('_continuity' if options else '')
            started = time.monotonic()
            with (output / (label + '.log')).open('w') as case_log:
                process = subprocess.Popen(['rosrun', 'general_planner', name] + options,
                                           stdout=case_log, stderr=subprocess.STDOUT,
                                           start_new_session=True)
                try:
                    code = process.wait(timeout=120)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
                    raise
            results[label] = {'exit_code': code, 'seconds': time.monotonic() - started}
            (output / 'results.json').write_text(json.dumps(results, indent=2))
            print(label, 'PASS' if code == 0 else 'FAIL', flush=True)
            if code:
                raise RuntimeError('regression failed: ' + label)
    finally:
        if master.poll() is None:
            os.killpg(master.pid, signal.SIGINT)
            try:
                master.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(master.pid, signal.SIGKILL)
                master.wait()
