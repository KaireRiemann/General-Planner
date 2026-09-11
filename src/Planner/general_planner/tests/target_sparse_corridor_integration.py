#!/usr/bin/env python3
"""Isolated runtime + ideal position-command follower + raycast corridor.
This is a synthetic software closed loop, not Unity/dynamics flight validation.
"""
import os,socket,subprocess,time,signal,threading,json,math,xmlrpc.client,tempfile
from pathlib import Path
import numpy as np
output_dir=os.environ.get('TARGET_TEST_OUTPUT_DIR')
base=Path(output_dir) if output_dir else Path(tempfile.mkdtemp(prefix='target_corridor_'))
base.mkdir(parents=True,exist_ok=True)
print('test output:',base,flush=True)
with socket.socket() as s:s.bind(('127.0.0.1',0));port=s.getsockname()[1]
os.environ['ROS_MASTER_URI']='http://127.0.0.1:%d'%port
os.environ['ROS_IP']='127.0.0.1'
procs=[]; files=[]
def start(args,name):
 f=open(base/name,'w');files.append(f)
 p=subprocess.Popen(args,stdout=f,stderr=subprocess.STDOUT,start_new_session=True);procs.append(p);return p
try:
 master=start(['roscore','-p',str(port)],'synthetic_master.log')
 deadline=time.monotonic()+20
 while time.monotonic()<deadline:
  try:
   if xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI']).getPid('test')[0]==1:break
  except OSError:pass
  time.sleep(.1)
 import rospy
 from nav_msgs.msg import Odometry
 from geometry_msgs.msg import PoseStamped
 from quadrotor_msgs.msg import PositionCommand
 from general_planner.msg import PlannerStatus
 from rosgraph_msgs.msg import Log
 from sensor_msgs.msg import PointCloud2
 from sensor_msgs.point_cloud2 import create_cloud_xyz32
 from std_msgs.msg import Header, String
 rospy.init_node('synthetic_corridor_test',anonymous=True,disable_signals=True)
 u_shape=bool(os.environ.get('TEST_U_SHAPE'))
 follow_commands=not u_shape
 pos=np.array([float(os.environ.get('TEST_START_X','0')),0.,1.5]); vel=np.zeros(3);yaw=0.;cmd=None;status=None;logs=[];history=[];running=True
 def command(m):
  global cmd
  cmd=m
 def state(m):
  global status
  status=m
 def log(m):
  if m.name=='/planner_runtime_node':logs.append(m.msg)
 rospy.Subscriber('/planning/pos_cmd',PositionCommand,command,queue_size=1)
 rospy.Subscriber('/planner/status',PlannerStatus,state,queue_size=10)
 rospy.Subscriber('/rosout',Log,log,queue_size=200)
 odom_pub=rospy.Publisher('/lidar_slam/odom',Odometry,queue_size=1)
 cloud_pub=rospy.Publisher('/cloud_registered',PointCloud2,queue_size=1)
 goal_pub=rospy.Publisher('/planner/exploration/trigger',PoseStamped,queue_size=1)
 mode_pub=rospy.Publisher('/planner/mode_request_text',String,queue_size=1)
 az,el=np.meshgrid(np.linspace(-math.pi,math.pi,240,endpoint=False),np.linspace(-math.radians(float(os.environ.get('TEST_HALF_FOV','30'))),math.radians(float(os.environ.get('TEST_HALF_FOV','30'))),64))
 directions=np.c_[np.cos(el).ravel()*np.cos(az).ravel(),np.cos(el).ravel()*np.sin(az).ravel(),np.sin(el).ravel()]
 def sensor_loop():
  global pos,vel,yaw
  tick=0
  while running and not rospy.is_shutdown():
   if cmd is not None and follow_commands:
    desired=np.array([cmd.position.x,cmd.position.y,cmd.position.z]);delta=desired-pos
    pos=pos+delta*min(1.,.12/max(float(np.linalg.norm(delta)),1e-9))
    vel=np.array([cmd.velocity.x,cmd.velocity.y,cmd.velocity.z]);yaw=cmd.yaw
   stamp=rospy.Time.now();o=Odometry();o.header.stamp=stamp;o.header.frame_id='world';o.child_frame_id='body'
   o.pose.pose.position.x,o.pose.pose.position.y,o.pose.pose.position.z=pos.tolist()
   o.pose.pose.orientation.z=math.sin(yaw/2);o.pose.pose.orientation.w=math.cos(yaw/2)
   o.twist.twist.linear.x,o.twist.twist.linear.y,o.twist.twist.linear.z=vel.tolist();odom_pub.publish(o)
   if tick%10==0:
    distances=np.full(len(directions),np.inf)
    for axis,boundary in [(0,-15.),(0,180.),(1,-6.),(1,6.),(2,0.),(2,4.5)]:
     with np.errstate(divide='ignore',invalid='ignore'): t=(boundary-pos[axis])/directions[:,axis]
     distances=np.minimum(distances,np.where(t>0,t,np.inf))
    if u_shape:
     # A wall between start (0,0) and goal (4,0), open only above y=3.
     # Exact ray/AABB intersection, not an artificial point waypoint barrier.
     with np.errstate(divide='ignore',invalid='ignore'):
      t1=(np.array([1.5,-6.,0.])-pos)/directions
      t2=(np.array([2.5,3.,4.5])-pos)/directions
     enter=np.max(np.minimum(t1,t2),axis=1);leave=np.min(np.maximum(t1,t2),axis=1)
     distances=np.minimum(distances,np.where((enter>0)&(leave>=enter),enter,np.inf))
    valid=np.isfinite(distances) & (distances < 30.)
    pts=pos+directions[valid]*distances[valid,None]
    cloud_pub.publish(create_cloud_xyz32(Header(stamp=stamp,frame_id='world'),pts.tolist()))
   tick+=1;time.sleep(.01)
 sensor=threading.Thread(target=sensor_loop,daemon=True);sensor.start()
 launch_args=['roslaunch',os.environ.get('TEST_LAUNCH_PACKAGE','task_planner'),'planner_runtime.launch','initial_mode:='+os.environ.get('TEST_INITIAL_MODE','target_exploration'),'marsim:=false','rviz:=false','auto_rviz_switch:=false','tracking_detector:=false','perceptor:=false','cloud_odom_mode:=latest_odom','target_exploration_auto_workspace:=false', 'exploration_mission_mode:='+('coverage' if os.environ.get('TEST_COVERAGE_SECONDS') else 'target')]
 if os.environ.get('TEST_NO_TOPOLOGY'):
  launch_args.append('global_topology_config:='+str(Path(__file__).resolve().parent/'config/target_route_no_topology.yaml'))
 launch=start(launch_args,'synthetic_runtime.log')
 deadline=time.monotonic()+60
 while time.monotonic()<deadline:
  if status and status.map_ready and status.odom_valid and goal_pub.get_num_connections()>0:break
  if launch.poll() is not None:raise RuntimeError('runtime exited')
  time.sleep(.1)
 assert status and status.map_ready,'map not ready'
 if u_shape:
  # Sensor-only setup pass: accumulate a world route, then restore the initial
  # pose before admitting the task. This phase is not counted as task motion.
  setup=[(0,0),(0,1.5),(0,3),(0,4.5),(1.5,4.5),(3,4.5),(4,4.5),(4,3),(4,1.5),(4,0)]
  for x,y in setup+list(reversed(setup[:-1])):
   pos=np.array([x,y,1.5]);vel=np.zeros(3);time.sleep(1.2)
  follow_commands=True
 time.sleep(3)
 if os.environ.get('TEST_INITIAL_MODE') == 'state2state':
  mode_pub.publish(String(data='target_exploration'))
  until=time.monotonic()+15
  while time.monotonic()<until and status.active_mode_str != 'target_exploration':time.sleep(.1)
  assert status.active_mode_str == 'target_exploration','mode switch failed'
 goal=PoseStamped();goal.header.frame_id='world';goal.header.stamp=rospy.Time.now();goal.pose.position.x=float(os.environ.get('TEST_GOAL_X','4' if u_shape else '145'));goal.pose.position.z=1.5;goal.pose.orientation.w=1;goal_pub.publish(goal)
 began=time.monotonic();success=False
 coverage_seconds=float(os.environ.get("TEST_COVERAGE_SECONDS","0"))
 coverage_moved=False
 while time.monotonic()-began<180:
  history.append({'t':time.monotonic()-began,'p':pos.tolist(),'result':status.task_result_str,'phase':status.phase_str})
  if any('outside the active exploration capacity' in x for x in logs):raise AssertionError('old capacity rejection')
  if u_shape:assert not (1.3<pos[0]<2.7 and pos[1]<3.2),'command crossed U-wall safety margin'
  if coverage_seconds:
   in_low=(-20<=pos[0]<=20 and -20<=pos[1]<=20.2 and -.2<=pos[2]<=3.8)
   in_high=(-9.5<=pos[0]<=9.5 and -9.5<=pos[1]<=9.5 and 2.8<=pos[2]<=6.4)
   assert in_low or in_high,'coverage command escaped its boxes'
   coverage_moved=coverage_moved or np.linalg.norm(pos-np.array([0.,0.,1.5]))>1.
   if time.monotonic()-began>=coverage_seconds:
    success=coverage_moved and status.active_mode_str=='exploration';break
  elif status.task_result_str=='succeeded' and pos[0]>goal.pose.position.x-1.:success=True;break
  if status.task_result_str in ['blocked','failed'] and time.monotonic()-began>3:break
  if launch.poll() is not None:raise RuntimeError('runtime exited')
  time.sleep(.5)
 route_commits=sum('[target route commit] accepted' in x and 'source=KNOWN_' in x for x in logs)
 local_goal_commits=sum('[target route commit] accepted' in x and 'source=LOCAL_GOAL' in x for x in logs)
 result={'success':success,'final':pos.tolist(),'elapsed':time.monotonic()-began,'result':status.task_result_str,'reason':status.reason,'known_route_commits':route_commits,'local_goal_commits':local_goal_commits,'history':history}
 (base/'synthetic_result.json').write_text(json.dumps(result,indent=2))
 print(json.dumps({k:v for k,v in result.items() if k!='history'}),flush=True)
 assert success,'synthetic corridor not completed; inspect runtime log'
 if os.environ.get('TEST_REQUIRE_ROUTE'):assert route_commits>0,'task finished without executing a known route'
 if os.environ.get('TEST_NO_TOPOLOGY'):assert route_commits==0,'disabled topology unexpectedly executed a known route'
 if u_shape:
  assert route_commits>0,'known U-route was not used'
  assert max(np.linalg.norm(np.array(h['p'][:2])-np.array([goal.pose.position.x,0.])) for h in history)>4.25,'U-detour did not demonstrate Euclidean retreat'
finally:
 running=False
 for p in reversed(procs):
  if p.poll() is None:
   os.killpg(p.pid,signal.SIGINT)
   try:p.wait(timeout=10)
   except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
 for f in files:f.close()
