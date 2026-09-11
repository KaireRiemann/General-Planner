#!/usr/bin/env python3
"""Bundle Gate perception and Python-only Unity transport; never copy devel wrappers."""
import argparse
import shutil
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workspace', type=Path, required=True)
    parser.add_argument('--repo', type=Path, required=True)
    args = parser.parse_args()
    release = args.repo / 'general_planner_release'
    aperture = args.repo / 'src/Perceptor/aperture_detector'
    unity = args.workspace / 'src/unity_planner_bridge'
    tcp = args.workspace / 'src/ROS-TCP-Endpoint'
    binary = args.workspace / 'devel/lib/aperture_detector/polygon_hole_step_viz'
    messages = args.workspace / 'devel/lib/python3/dist-packages/aperture_detector'
    headers = args.workspace / 'devel/include/aperture_detector'
    required = [binary, messages / 'msg/_ApertureObservation.py',
                headers / 'ApertureObservation.h', aperture / 'package.xml',
                release / 'src/unity_planner_bridge/launch/unity_planner_release.launch',
                unity / 'launch/unity_endpoint.launch', tcp / 'LICENSE']
    for path in required:
        if not path.is_file():
            raise RuntimeError('Missing release input: ' + str(path))
    ignore = shutil.ignore_patterns('__pycache__', '*.pyc', '.git')

    def tree(source, destination):
        shutil.copytree(source, destination, dirs_exist_ok=True, ignore=ignore)

    dst = release / 'src/aperture_detector'
    dst.mkdir(parents=True, exist_ok=True)
    for name in ('launch', 'config', 'rviz_cfg', 'scripts', 'msg', 'tests'):
        tree(aperture / name, dst / name)
    # Keep the optional legacy bridge deployable, but never start it from the
    # composed runtime. Internal Gate consumes observations via its own gateway.
    for name in ('package.xml', 'LICENSE', 'README.md'):
        shutil.copy2(aperture / name, dst / name)
    shutil.copy2(binary, dst / binary.name)
    shutil.copy2(binary.parent / 'detection_planning_bridge', dst)
    shutil.copy2(args.workspace / 'devel/lib/libdetection_planning_bridge_lib.so', release / 'lib')
    tree(messages, release / 'lib/python3/dist-packages/aperture_detector')
    tree(headers, release / 'include/aperture_detector')
    for name in ('gate_runtime.yaml', 'gate_unity.yaml'):
        shutil.copy2(args.repo / 'src/Planner/general_planner/config' / name,
                     release / 'src/general_planner_release/config' / name)

    unity_dst = release / 'src/unity_planner_bridge'
    unity_dst.mkdir(parents=True, exist_ok=True)
    (unity_dst / 'launch').mkdir(exist_ok=True)
    # The release launch is owned here, not mirrored in the source package:
    # catkin/roslaunch searches both roots and rejects duplicate launch names.
    # Keep the source simulation entry and copy only its shared endpoint.
    shutil.copy2(unity / 'launch/unity_endpoint.launch', unity_dst / 'launch/unity_endpoint.launch')
    (unity_dst / 'scripts').mkdir(exist_ok=True)
    shutil.copy2(unity / 'scripts/unity_cmd_odom_bridge.py', unity_dst / 'scripts')
    for name in ('package.xml', 'TRACKING.md'):
        shutil.copy2(unity / name, unity_dst / name)
    tcp_dst = release / 'src/ros_tcp_endpoint'
    tcp_dst.mkdir(parents=True, exist_ok=True)
    tree(tcp / 'launch', tcp_dst / 'launch')
    tree(tcp / 'src/ros_tcp_endpoint', release / 'lib/python3/dist-packages/ros_tcp_endpoint')
    for name in ('package.xml', 'LICENSE', 'README.md'):
        shutil.copy2(tcp / name, tcp_dst / name)
    tcp_entry = tcp_dst / 'default_server_endpoint.py'
    shutil.copy2(args.repo / 'sh_files/release_ros_tcp_endpoint.py', tcp_entry)
    tcp_entry.chmod(0o755)
    # Include the small perception-only regression fixtures, not planner source.
    tree(args.repo / 'src/Perceptor/tracking_detector/tests',
         release / 'src/tracking_detector/tests')
    (release / 'tests').mkdir(exist_ok=True)
    shutil.copy2(args.repo / 'sh_files/test_release_perception.py', release / 'tests')
    print('Bundled aperture_detector, Unity bridge, ROS-TCP endpoint and perception tests')


if __name__ == '__main__':
    main()
