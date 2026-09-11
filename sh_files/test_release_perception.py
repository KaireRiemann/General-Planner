#!/usr/bin/env python3
"""Validate a relocated release without source overlays or a flight controller.

Run inside ROS Noetic: python3 tests/test_release_perception.py --runtime
Runtime checks own master 11329 and fail if it is occupied. No planner is started.
"""
import argparse
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import tempfile
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--release', type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument('--runtime', action='store_true')
    args = parser.parse_args()
    release = args.release.resolve()
    if not (release / 'setup.bash').is_file():
        raise RuntimeError('Pass --release /path/to/general_planner_release')
    env = os.environ.copy()
    env.update(ROS_PACKAGE_PATH=str(release / 'src') + ':/opt/ros/noetic/share',
               CMAKE_PREFIX_PATH='/opt/ros/noetic',
               PYTHONPATH=str(release / 'lib/python3/dist-packages') + ':/opt/ros/noetic/lib/python3/dist-packages',
               LD_LIBRARY_PATH=str(release / 'lib') + ':/opt/ros/noetic/lib',
               PYTHONDONTWRITEBYTECODE='1', ROS_MASTER_URI='http://127.0.0.1:11329',
               ROS_HOSTNAME='127.0.0.1', ROS_IP='127.0.0.1')
    # Re-exec so even this test's message imports cannot leak from devel.
    if os.environ.get('_GP_PERCEPTION_TEST_ROOT') != str(release):
        env['_GP_PERCEPTION_TEST_ROOT'] = str(release)
        os.execve('/usr/bin/python3', ['/usr/bin/python3', str(Path(__file__).resolve())] + sys.argv[1:], env)

    import rospkg
    import roslib.packages
    import roslaunch
    rp = rospkg.RosPack()
    for name in ('general_planner_release', 'tracking_detector', 'aperture_detector',
                 'unity_planner_bridge', 'ros_tcp_endpoint'):
        assert Path(rp.get_path(name)).resolve() == release / 'src' / name, name
    targets = [('aperture_detector', 'polygon_hole_step_viz'),
               ('aperture_detector', 'windowtec_vlm_box_node.py'),
               ('tracking_detector', 'yoloe_bbox_node.py'),
               ('tracking_detector', 'target_state_estimator.py'),
               ('tracking_detector', 'target_path_predictor_node'),
               ('general_planner_release', 'planner_detector_release'),
               ('unity_planner_bridge', 'unity_cmd_odom_bridge.py'),
               ('ros_tcp_endpoint', 'default_server_endpoint.py')]
    for package, name in targets:
        paths = roslib.packages.find_node(package, name, rospack=rp)
        assert paths and all(release in Path(p).resolve().parents for p in paths), (name, paths)
    for path in (release / 'src/aperture_detector/polygon_hole_step_viz',
                 release / 'src/tracking_detector/target_path_predictor_node',
                 release / 'src/general_planner_release/bin/planner_runtime_node.bin'):
        output = subprocess.check_output(['ldd', str(path)], env=env, text=True)
        assert 'not found' not in output, output
        assert '/devel/' not in output and '/build/' not in output, output

    def config(path, argv=()):
        result = roslaunch.config.ROSLaunchConfig()
        roslaunch.xmlloader.XmlLoader().load(str(path), result, argv=list(argv), verbose=False)
        return result

    runtime_launch = release / 'src/general_planner_release/launch/planner_runtime.launch'
    for path in (runtime_launch, release / 'src/unity_planner_bridge/launch/unity_planner_release.launch'):
        cfg = config(path, ['rviz:=false'])
        managers = [n for n in cfg.nodes if n.type == 'planner_detector_release']
        assert {n.name for n in managers} == {'gate_detector_switcher', 'tracking_detector_switcher'}
        for name in ('gate', 'tracking'):
            launch_file = cfg.params['/' + name + '_detector_switcher/launch_file'].value
            assert release in Path(launch_file).resolve().parents, launch_file
        assert '/planner_runtime_node/gate/observation_topic' in cfg.params
        off = config(path, ['rviz:=false', 'perceptor:=false', 'tracking_detector:=false'])
        assert not any(n.type == 'planner_detector_release' for n in off.nodes)
    config(release / 'src/general_planner_release/launch/planner_runtime_sim.launch', ['rviz:=false'])
    subprocess.run(['/usr/bin/python3', str(release / 'src/tracking_detector/scripts/check_runtime.py')],
                   env=env, check=True, timeout=90)
    print('PASS release-only packages, binaries, messages, assets and launch wiring', flush=True)
    if not args.runtime:
        return

    # Never connect to an existing ROS master, even if its process is unrelated.
    with socket.socket() as probe:
        # Permit an earlier owned test's TIME_WAIT sockets, never a listener.
        probe.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        probe.bind(('127.0.0.1', 11329))
    processes = []
    logs = []
    with tempfile.TemporaryDirectory(prefix='gp-release-perception-') as tmp:
        env['ROS_LOG_DIR'] = tmp

        def spawn(command, child_env=None):
            log = tempfile.TemporaryFile(mode='w+')
            logs.append(log)
            proc = subprocess.Popen(command, env=child_env or env, stdout=log, stderr=log,
                                    start_new_session=True)
            processes.append(proc)
            return proc

        def run_test(path, timeout=90):
            subprocess.run(['/usr/bin/python3', str(path)], env=env, check=True, timeout=timeout)

        try:
            spawn(['roscore', '-p', '11329'])
            import rosgraph
            master = rosgraph.Master('/release_test')
            deadline = time.monotonic() + 15
            while True:
                try:
                    master.getPid()
                    break
                except Exception:
                    if time.monotonic() > deadline:
                        raise RuntimeError('owned master startup timeout')
                    time.sleep(.1)
            run_test(release / 'src/aperture_detector/tests/synthetic_detection_test.py')
            run_test(release / 'src/aperture_detector/tests/vlm_node_smoke_test.py')

            import rospy
            import rosnode
            import xmlrpc.client
            from general_planner.msg import PlannerStatus
            rospy.init_node('release_detector_lifecycle', disable_signals=True)
            status = rospy.Publisher('/planner/status', PlannerStatus, queue_size=1)

            class LocalTransport(xmlrpc.client.Transport):
                def make_connection(self, host):
                    connection = super().make_connection(host)
                    connection.timeout = .5
                    return connection

            def node_pid(name):
                try:
                    uri = master.lookupNode(name)
                    response = xmlrpc.client.ServerProxy(uri, transport=LocalTransport()).getPid('/release_test')
                    return response[2] if response[0] == 1 else None
                except Exception:
                    return None

            wrapper = release / 'src/general_planner_release/planner_detector_release'
            for mode, package, launch in [('gate', 'aperture_detector', 'detector.launch'),
                                          ('tracking', 'tracking_detector', 'tracking_detector.launch')]:
                name = 'release_' + mode + '_switcher'
                rospy.set_param('/' + name + '/launch_args', {'vlm': False} if mode == 'gate' else {'device': 'cpu'})
                # Deliberately contaminate the parent's search paths. The release
                # wrapper must strip these before it launches any detector.
                contaminated = env.copy()
                contaminated['CMAKE_PREFIX_PATH'] = '/root/ws/real_planner/devel:/opt/ros/noetic'
                contaminated['ROS_PACKAGE_PATH'] = '/root/ws/real_planner/src/General-Planner/src/Perceptor:' + env['ROS_PACKAGE_PATH']
                spawn([str(wrapper), '__name:=' + name, '_active_mode:=' + mode,
                       '_launch_file:=' + str(release / 'src' / package / 'launch' / launch)], contaminated)

            def wait_mode(mode, present=(), absent=(), timeout=30):
                msg = PlannerStatus()
                msg.active_mode = mode
                deadline = time.monotonic() + timeout
                while time.monotonic() < deadline:
                    status.publish(msg)
                    # roscore may still list a just-stopped geometry fixture.
                    # Require a live XMLRPC endpoint, not stale registration.
                    names = {name for name in set(present) | set(absent) if node_pid(name)}
                    if set(present) <= names and not set(absent) & names:
                        return
                    time.sleep(.1)
                raise AssertionError(('detector lifecycle timeout', mode, names))

            gate = '/polygon_hole_step_viz'
            tracking = ('/tracking_detector/yoloe', '/tracking_detector/target_ekf', '/tracking_detector/predictor')
            wait_mode(PlannerStatus.MODE_GATE, [gate], tracking)
            pid = node_pid(gate)
            assert release in Path('/proc/%s/exe' % pid).resolve().parents
            wait_mode(PlannerStatus.MODE_TRACKING, tracking, [gate], timeout=60)
            for name in tracking:
                pid = node_pid(name)
                command = Path('/proc/%s/cmdline' % pid).read_bytes().replace(b'\0', b' ').decode()
                assert str(release) in command and '/devel/' not in command, command
            # Keep the selected mode heartbeat while the real CPU inference
            # fixture drives only perception topics, never flight commands.
            import threading
            stop_heartbeat = threading.Event()

            def heartbeat():
                msg = PlannerStatus()
                msg.active_mode = PlannerStatus.MODE_TRACKING
                while not stop_heartbeat.wait(.1):
                    status.publish(msg)

            thread = threading.Thread(target=heartbeat, daemon=True)
            thread.start()
            try:
                run_test(release / 'src/tracking_detector/tests/end_to_end_smoke.py', timeout=100)
            finally:
                stop_heartbeat.set()
                thread.join()
            wait_mode(PlannerStatus.MODE_STATE2STATE, absent=[gate] + list(tracking))
            wait_mode(PlannerStatus.MODE_GATE, [gate], tracking)
            # No heartbeat must also stop the detector.
            deadline = time.monotonic() + 20
            while node_pid(gate) and time.monotonic() < deadline:
                time.sleep(.1)
            assert not node_pid(gate), 'stale status did not stop detector'
            print('PASS Gate/Tracking mode lifecycle, source-overlay isolation, real inference and stale-status shutdown', flush=True)
        except BaseException:
            for log in logs:
                log.flush()
                log.seek(0)
                print(log.read()[-16000:], file=sys.stderr)
            raise
        finally:
            for proc in reversed(processes):
                if proc.poll() is None:
                    os.killpg(proc.pid, signal.SIGINT)
                    try:
                        proc.wait(timeout=20)
                    except subprocess.TimeoutExpired:
                        os.killpg(proc.pid, signal.SIGKILL)
                        proc.wait()
            for log in logs:
                log.close()


if __name__ == '__main__':
    main()
