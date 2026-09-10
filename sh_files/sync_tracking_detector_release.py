#!/usr/bin/env python3
"""Bundle the built tracking frontend into the relocatable ROS release."""
import argparse
import shutil
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workspace', type=Path, required=True)
    parser.add_argument('--repo', type=Path, required=True)
    args = parser.parse_args()
    src = args.repo / 'src/Perceptor/tracking_detector'
    release = args.repo / 'general_planner_release'
    dst = release / 'src/tracking_detector'
    binaries = args.workspace / 'devel/lib/tracking_detector'
    messages = args.workspace / 'devel/lib/python3/dist-packages/tracking_detector'
    required = [src / 'package.xml', messages / '__init__.py']
    required += [binaries / n for n in ('target_ekf_node', 'target_path_predictor_node')]
    required += [src / 'vendor/yoloe/mobileclip_blt.pt', src / 'vendor/yoloe/prompt/yoloe_pretrain/yoloe-11m-seg.pt']
    for path in required:
        if not path.is_file():
            raise RuntimeError('Build tracking_detector / provision assets before packaging: ' + str(path))
    dst.mkdir(parents=True, exist_ok=True)
    ignore = shutil.ignore_patterns('__pycache__', '*.pyc', '.git')
    for name in ('launch', 'config', 'msg', 'scripts', 'vendor', 'integration'):
        shutil.copytree(src / name, dst / name, dirs_exist_ok=True, ignore=ignore)
    for name in ('package.xml', 'README.md', 'RUNTIME_SETUP.md', 'THIRD_PARTY.md', 'LICENSE.target_ekf', '.gitignore'):
        shutil.copy2(src / name, dst / name)
    # Source Python scripts are portable; catkin devel wrappers embed source paths.
    for name in ('target_ekf_node', 'target_path_predictor_node'):
        shutil.copy2(binaries / name, dst / name)
    shutil.copytree(messages, release / 'lib/python3/dist-packages/tracking_detector', dirs_exist_ok=True, ignore=ignore)
    print('Bundled tracking_detector nodes, Python messages and model assets into', dst)


if __name__ == '__main__':
    main()
