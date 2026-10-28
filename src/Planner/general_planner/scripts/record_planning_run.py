#!/usr/bin/env python3
"""Run a local planner command in an isolated session and analyze it on exit.
Example: record_planning_run.py --config config/click.yaml -- roslaunch pkg demo.launch
Run inside the same host/container as the planner so GP_RUN_DIR is accessible.
"""
import argparse
import datetime
import hashlib
import json
import os
from pathlib import Path
import platform
import shutil
import signal
import subprocess
import sys
import time
import uuid
from analyze_planning_run import analyze


def utc():
    return datetime.datetime.now(datetime.timezone.utc).isoformat()


def command_output(args, cwd=None):
    result=subprocess.run(args,cwd=cwd,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,check=False)
    return result.stdout


def stop_group(process, timeout=15):
    if process is None or process.poll() is not None:return False
    os.killpg(process.pid,signal.SIGINT)
    try:process.wait(timeout=timeout);return False
    except subprocess.TimeoutExpired:
        os.killpg(process.pid,signal.SIGKILL);process.wait();return True


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,help='New directory; existing directories are never reused')
    parser.add_argument('--config',type=Path,action='append',default=[],help='Snapshot an external config or launch file; repeatable')
    parser.add_argument('--duration',type=float,default=0,help='Optional wall-time limit in seconds; 0 means wait for exit/Ctrl-C')
    parser.add_argument('--bag-topic',action='append',default=[],help='Optional ROS1 bag topic; repeat for clock, inputs, odometry, status, etc.')
    parser.add_argument('command',nargs=argparse.REMAINDER)
    args=parser.parse_args();cmd=args.command
    if cmd and cmd[0]=='--':cmd=cmd[1:]
    if not cmd:parser.error('a command after -- is required')
    if args.duration<0:parser.error('--duration must be nonnegative')
    for path in args.config:
        if not path.is_file():parser.error(f'config not found: {path}')
    if args.bag_topic and shutil.which('rosbag') is None:parser.error('rosbag is unavailable')
    repo=Path(__file__).resolve().parents[4]
    output=(args.output or repo/'analysis'/'runs'/(datetime.datetime.now().strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:8])).resolve()
    output.mkdir(parents=True,exist_ok=False)
    package=Path(__file__).resolve().parents[1]
    shutil.copytree(package/'config',output/'config_snapshot'/'planner')
    shutil.copytree(package/'launch',output/'config_snapshot'/'launch')
    (output/'tools_snapshot').mkdir()
    for name in ['record_planning_run.py','analyze_planning_run.py']:
        shutil.copy2(Path(__file__).parent/name,output/'tools_snapshot'/name)
    configs=[]
    for i,path in enumerate(args.config):
        dest=output/'config_snapshot'/f'external_{i}_{path.name}';shutil.copy2(path,dest)
        configs.append({'source':str(path.resolve()),'snapshot':str(dest.relative_to(output))})
    (output/'source.patch').write_text(command_output(['git','diff','HEAD'],repo))
    manifest={'schema_version':1,'state':'running','started_utc':utc(),'command':cmd,'cwd':os.getcwd(),
              'repository':str(repo),'git_commit':command_output(['git','rev-parse','HEAD'],repo).strip(),
              'git_status':command_output(['git','status','--short'],repo),'external_configs':configs,
              'platform':platform.platform(),'python':sys.version,'bag_topics':args.bag_topic,
              'clock_contract':{'utc':'session lifecycle','steady':'executor and launcher durations','ros_sim':'cycle/event/command correlation'},
              'coverage':'FSM artifacts per process; other nodes via console, roslogs and optional explicit-topic bag. Not a deterministic replay bundle.'}
    def save():
        tmp=output/'manifest.tmp';tmp.write_text(json.dumps(manifest,indent=2,ensure_ascii=False));tmp.replace(output/'manifest.json')
    manifest['snapshot_sha256']={str(f.relative_to(output)):hashlib.sha256(f.read_bytes()).hexdigest() for f in (output/'config_snapshot').rglob('*') if f.is_file()}
    save()
    env=dict(os.environ,GP_RUN_DIR=str(output),ROS_LOG_DIR=str(output/'roslogs'))
    interrupted=[False]
    old={sig:signal.signal(sig,lambda signum,frame:interrupted.__setitem__(0,True)) for sig in (signal.SIGINT,signal.SIGTERM)}
    process=bag=None;begin=time.monotonic();next_params=begin+3;params_saved=False
    print(f'Planning session: {output}',flush=True)
    try:
        with (output/'console.log').open('w') as console, (output/'bag.log').open('w') as baglog:
            process=subprocess.Popen(cmd,env=env,stdout=console,stderr=subprocess.STDOUT,start_new_session=True)
            if args.bag_topic:
                bag=subprocess.Popen(['rosbag','record','-O',str(output/'topics.bag')]+args.bag_topic,env=env,stdout=baglog,stderr=subprocess.STDOUT,start_new_session=True)
            while process.poll() is None:
                elapsed=time.monotonic()-begin
                if interrupted[0] or (args.duration and elapsed>=args.duration):
                    manifest['state']='interrupted' if interrupted[0] else 'duration_limit'
                    manifest['forced_kill']=stop_group(process)
                    break
                if not params_saved and time.monotonic()>=next_params and shutil.which('rosparam'):
                    try:
                        result=subprocess.run(['rosparam','dump',str(output/'rosparams.yaml')],env=env,stdout=console,stderr=console,timeout=2,check=False)
                        params_saved=result.returncode==0
                    except subprocess.TimeoutExpired:pass
                    next_params=time.monotonic()+5
                time.sleep(.1)
            manifest['returncode']=process.returncode
            if manifest['state']=='running':manifest['state']='finished' if process.returncode==0 else 'failed'
            if bag is not None:
                manifest['bag_forced_kill']=stop_group(bag)
                manifest['bag_returncode']=bag.returncode
    except OSError as error:
        manifest['state']='failed';manifest['error']=str(error)
    finally:
        stop_group(process);stop_group(bag)
        for sig,handler in old.items():signal.signal(sig,handler)
        manifest['ended_utc']=utc();manifest['elapsed_wall_s']=time.monotonic()-begin;manifest['rosparams_captured']=params_saved
        manifest['artifacts']=[{'path':str(f.relative_to(output)),'bytes':f.stat().st_size} for f in output.rglob('*') if f.is_file()]
        save();analyze(output)
        manifest['artifacts']=[{'path':str(f.relative_to(output)),'bytes':f.stat().st_size} for f in output.rglob('*') if f.is_file()]
        save()
    print(f'Report: {output / "report.html"}',flush=True)
    return 0 if manifest['state']=='finished' else (124 if manifest['state']=='duration_limit' else 1)

if __name__=='__main__':sys.exit(main())
