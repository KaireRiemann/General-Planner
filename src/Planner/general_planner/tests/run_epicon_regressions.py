#!/usr/bin/env python3
"""Run point-cloud frontend, corridor and General backend tests on an isolated master."""
import argparse
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import time
import xmlrpc.client

parser=argparse.ArgumentParser()
parser.add_argument('--manifest',required=True)
parser.add_argument('--output',required=True)
parser.add_argument('--only',help='Run only this labeled regression case')
args=parser.parse_args()
out=Path(args.output);out.mkdir(parents=True,exist_ok=True)
with socket.socket() as sock:
    sock.bind(('127.0.0.1',0));port=sock.getsockname()[1]
os.environ['ROS_MASTER_URI']='http://127.0.0.1:%d'%port
os.environ['ROS_IP']='127.0.0.1'
master=None
try:
    with (out/'master.log').open('w') as log:
        master=subprocess.Popen(['roscore','-p',str(port)],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    api=xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI'])
    deadline=time.monotonic()+20
    while time.monotonic()<deadline:
        try:
            if api.getPid('epicon_test')[0]==1:break
        except OSError:pass
        time.sleep(.1)
    import rospy
    import rosparam
    import yaml
    package=Path(__file__).resolve().parents[1]
    backend=yaml.safe_load((package/'config/exploration_house.yaml').read_text())
    backend.update(yaml.safe_load((package/'config/exploration_unity_overlay.yaml').read_text()))
    backend['RogMapEnable']=False
    rosparam.upload_params('/epicon_backend_replay_test',backend)
    params={'box_num':1,'box_0/down':backend['box_0/down'],'box_0/up':backend['box_0/up'],
            'dead_area_num':0,'MaxVelMag':2.,'MaxTiltAngle':.5,'GravAcc':9.8,'yaw_max_vel':1.2,
            'odometry_topic':'/unused/odom','cloud_topic':'/unused/cloud',
            'lidar_perception/fov_up':30.,'lidar_perception/fov_down':-30.,
            'lidar_perception/fov_viewpoint_up':28.,'lidar_perception/fov_viewpoint_down':-28.,
            'lidar_perception/lidar_pitch':0.,'lidar_perception/max_ray_length':30.}
    for key,value in params.items():rospy.set_param('/epicon_frontend_replay_test/'+key,value)
    results=[]
    for name,extra in [('epicon_execution_policy_self_test',[]),('epicon_corridor_self_test',[]),
                       ('epicon_tour_self_test',[]),('epicon_frontend_replay_test',[args.manifest]),
                       ('epicon_frontend_replay_test',['--shifted-box']),('epicon_backend_replay_test',[args.manifest]),
                       ('epicon_backend_replay_test',['--recovery'])]:
        label=name+('_shifted' if extra==['--shifted-box'] else '_recovery' if extra==['--recovery'] else '')
        if args.only and args.only!=label:continue
        began=time.monotonic()
        with (out/(label+'.log')).open('w') as log:
            process=subprocess.Popen(['rosrun','general_planner',name]+extra,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            try:code=process.wait(timeout=180)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid,signal.SIGKILL);process.wait();raise
        results.append({'test':label,'exit_code':code,'elapsed':time.monotonic()-began})
        (out/'results.json').write_text(json.dumps(results,indent=2))
        print(results[-1],flush=True)
        if code:raise RuntimeError(name+' failed; see '+str(out/(label+'.log')))
    assert results,'No matching regression case'
    assert not any('RogMapConfigPath' in key for key in rospy.get_param_names())
    print('PASS: selected EPICON regressions completed without a ROG map',flush=True)
finally:
    if master and master.poll() is None:
        os.killpg(master.pid,signal.SIGINT)
        try:master.wait(timeout=10)
        except subprocess.TimeoutExpired:os.killpg(master.pid,signal.SIGKILL);master.wait()
