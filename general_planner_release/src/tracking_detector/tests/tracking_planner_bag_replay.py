#!/usr/bin/env python3
"""Replay recorded planner inputs on an isolated master; recorded odometry is open loop."""
import argparse
import collections
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag', type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--port', type=int, default=11339)
    parser.add_argument('--tracking-config', type=Path)
    parser.add_argument('--release', action='store_true')
    parser.add_argument('--regenerate-prediction',action='store_true',
        help='Run the current estimator and predictor from recorded boxes and camera/vehicle poses')
    args = parser.parse_args()
    os.environ['ROS_MASTER_URI'] = 'http://127.0.0.1:' + str(args.port)
    import rosgraph
    import rospy
    import rosbag
    from general_planner.msg import PlannerStatus
    from quadrotor_msgs.msg import PositionCommand
    from std_msgs.msg import String
    from rosgraph_msgs.msg import Log
    try:
        rosgraph.Master('/probe').getPid()
    except Exception:
        pass
    else:
        raise RuntimeError('test master port already in use')
    args.output.mkdir(parents=True, exist_ok=True)
    processes, logs, events, statuses, commands, messages = [], [], [], [], [], []

    def spawn(command, name):
        handle = (args.output / (name + '.log')).open('w')
        logs.append(handle)
        process = subprocess.Popen(command, stdout=handle, stderr=subprocess.STDOUT, start_new_session=True)
        processes.append(process)
        return process

    try:
        spawn(['roscore', '-p', str(args.port)], 'master')
        for _ in range(120):
            try:
                rosgraph.Master('/probe').getPid()
                break
            except Exception:
                time.sleep(.05)
        rospy.set_param('/use_sim_time', True)
        rospy.init_node('tracking_planner_bag_replay', disable_signals=True)
        rospy.Subscriber('/planning/diagnostics/events', String, lambda m: events.append(m.data), queue_size=5000)
        rospy.Subscriber('/planner/status', PlannerStatus,
            lambda m: statuses.append(dict(t=m.header.stamp.to_sec(), phase=m.phase_str,
                owner=m.command_owner_str, reason=m.reason)), queue_size=2000)
        rospy.Subscriber('/planning/pos_cmd', PositionCommand,
            lambda m: commands.append(dict(t=m.header.stamp.to_sec(), p=[m.position.x,m.position.y,m.position.z],
                v=[m.velocity.x,m.velocity.y,m.velocity.z], a=[m.acceleration.x,m.acceleration.y,m.acceleration.z],
                j=[m.jerk.x,m.jerk.y,m.jerk.z], yaw=m.yaw,yaw_rate=m.yaw_dot)),queue_size=2000)
        rospy.Subscriber('/rosout_agg', Log,
            lambda m: messages.append(dict(t=m.header.stamp.to_sec(),level=m.level,msg=m.msg)),queue_size=3000)
        command=['roslaunch','general_planner_release' if args.release else 'task_planner',
            'planner_runtime.launch','initial_mode:=tracking','tracking_detector:=false','perceptor:=false',
            'rviz:=false','auto_rviz_switch:=false','enable_exploration:=false']
        if args.tracking_config:
            command.append('tracking_config:='+str(args.tracking_config.resolve()))
        runtime=spawn(command,'runtime')
        for _ in range(300):
            subscribers=dict(rospy.get_master().getSystemState()[2][1])
            if '/tracking/target_prediction' in subscribers and '/cloud_registered' in subscribers:
                break
            assert runtime.poll() is None, 'runtime exited during startup'
            time.sleep(.05)
        else:
            raise RuntimeError('planner input subscribers did not start')
        topics=['/lidar_slam/odom','/cloud_registered','/tracking/target_prediction']
        if args.regenerate_prediction:
            detector=spawn(['roslaunch','tracking_detector','tracking_detector.launch',
                'start_detector:=false','range_method:=auto','camera_config:='+
                str(Path(__file__).resolve().parents[1]/'config/camera_unity_main_scene.yaml')],'estimator')
            for _ in range(200):
                subscribers=dict(rospy.get_master().getSystemState()[2][1])
                if '/tracking/bboxes' in subscribers: break
                assert detector.poll() is None, 'estimator exited during startup'
                time.sleep(.05)
            else: raise RuntimeError('estimator subscribers did not start')
            topics=['/lidar_slam/odom','/cloud_registered','/unity_odom','/tracking/bboxes',
                    '/camera0/color/info','/camera0/capture_pose']
        with rosbag.Bag(str(args.bag)) as bag:
            duration=bag.get_end_time()-bag.get_start_time()
        player=spawn(['rosbag','play','--clock','--delay=1',str(args.bag),'--topics']+topics, 'player')
        deadline=time.monotonic()+duration+60
        while player.poll() is None and time.monotonic()<deadline:
            assert runtime.poll() is None, 'runtime exited during replay'
            time.sleep(.1)
        assert player.poll()==0, 'bag playback failed or timed out'
        time.sleep(.3)
        contexts=[dict(f.split('=',1) for f in event.split(';') if '=' in f) for event in events]
        contexts=[e for e in contexts if e.get('event') in ('tracking_plan_from_rest_context','tracking_replan_context')]
        assert len(commands)>100 and contexts, 'planner did not execute'
        assert all(math.isfinite(v) for c in commands for v in c['p']+c['v']+c['a']+c['j']+[c['yaw'],c['yaw_rate']])
        result=dict(validation='PASS',feedback='recorded open-loop odometry',bag=str(args.bag),
            commands=len(commands), regenerated_prediction=args.regenerate_prediction, planning_attempts=len(contexts),
            planning_phases=dict(collections.Counter(e.get('phase') for e in contexts)),
            rejection_stages=dict(collections.Counter(e.get('last_commit_reject_reason') for e in contexts)),
            maximum_keep_old_count=max(int(e.get('keep_old_count',0)) for e in contexts),
            hold_commits=sum('TRACKING_HOLD_COMMITTED' in m['msg'] for m in messages),
            guard_events=sum('predictive target guard:' in m['msg'] and 'TRACKING_HOLD' not in m['msg'] for m in messages))
        (args.output/'summary.json').write_text(json.dumps(result,indent=2))
        (args.output/'trace.json').write_text(json.dumps(dict(events=events,statuses=statuses,commands=commands,rosout=messages)))
        print(json.dumps(result,indent=2),flush=True)
    finally:
        for process in reversed(processes):
            if process.poll() is None:
                os.killpg(process.pid,signal.SIGINT)
                try:
                    process.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid,signal.SIGKILL)
                    process.wait()
        for handle in logs:
            handle.close()


if __name__=='__main__':
    main()
