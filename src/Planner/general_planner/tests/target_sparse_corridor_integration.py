#!/usr/bin/env python3
"""Isolated runtime + ideal position-command follower + raycast corridor.
This is a synthetic software closed loop, not Unity/dynamics flight validation.
"""
import os,socket,subprocess,time,signal,threading,json,math,xmlrpc.client,tempfile,re
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
 epicon=bool(os.environ.get('TEST_EPICON'))
 expect_epicon_blocked=epicon and bool(os.environ.get('TEST_EXPECT_EPICON_BLOCKED'))
 expect_epicon_empty=epicon and bool(os.environ.get('TEST_EXPECT_EPICON_EMPTY'))
 native_audits=[]; native_results=[]
 def native_audit(m):
  native_audits.append(json.loads(m.data))
 def native_result(m):
  native_results.append(json.loads(m.data))
 rospy.Subscriber('/planning/exploration/epicon_status',String,native_audit,queue_size=100)
 rospy.Subscriber('/planning/coverage_result',String,native_result,queue_size=10)
 u_shape=bool(os.environ.get('TEST_U_SHAPE'))
 epicon_boxes=[(np.array(low),np.array(high)) for low,high in [([-8.,-4.,0.],[-3.,4.,5.]),([3.,-8.,0.],[8.,-3.,4.]),([3.,3.,0.],[8.,8.,5.])]]
 minimum_clearance=float('inf')
 measured_height=[float('inf'),float('-inf')]
 coverage_boxes=[]; coverage_bounds_violation=False
 follow_commands=not u_shape and not expect_epicon_blocked
 pos=np.array([float(os.environ.get('TEST_START_X','0')),0.,float(os.environ.get('TEST_START_Z','1.5'))]); vel=np.zeros(3);yaw=0.;cmd=None;status=None;logs=[];history=[];running=True
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
  global pos,vel,yaw,minimum_clearance,coverage_bounds_violation
  tick=0
  while running and not rospy.is_shutdown():
   if cmd is not None and follow_commands:
    desired=np.array([cmd.position.x,cmd.position.y,cmd.position.z]);delta=desired-pos
    pos=pos+delta*min(1.,.12/max(float(np.linalg.norm(delta)),1e-9))
    vel=np.array([cmd.velocity.x,cmd.velocity.y,cmd.velocity.z]);yaw=cmd.yaw
   if epicon:
    minimum_clearance=min(minimum_clearance,float(pos[2]),*(float(np.linalg.norm(np.maximum(np.maximum(low-pos,pos-high),0.))) for low,high in epicon_boxes))
    measured_height[0]=min(measured_height[0],float(pos[2]))
    measured_height[1]=max(measured_height[1],float(pos[2]))
    if coverage_boxes and not any(np.all(pos>=low) and np.all(pos<=high) for low,high in coverage_boxes):
     coverage_bounds_violation=True
   stamp=rospy.Time.now();o=Odometry();o.header.stamp=stamp;o.header.frame_id='world';o.child_frame_id='body'
   o.pose.pose.position.x,o.pose.pose.position.y,o.pose.pose.position.z=pos.tolist()
   o.pose.pose.orientation.z=math.sin(yaw/2);o.pose.pose.orientation.w=math.cos(yaw/2)
   o.twist.twist.linear.x,o.twist.twist.linear.y,o.twist.twist.linear.z=vel.tolist();odom_pub.publish(o)
   if tick%10==0:
    distances=np.full(len(directions),np.inf)
    planes=[(0,-12.),(0,12.),(1,-12.),(1,12.),(2,0.),(2,8.)] if epicon else [(0,-15.),(0,180.),(1,-6.),(1,6.),(2,0.),(2,float(os.environ.get('TEST_CEILING_Z','4.5')))]
    for axis,boundary in planes:
     with np.errstate(divide='ignore',invalid='ignore'): t=(boundary-pos[axis])/directions[:,axis]
     distances=np.minimum(distances,np.where(t>0,t,np.inf))
    if epicon:
     # Occluding buildings make native point-cloud frontiers; a featureless
     # corridor is insufficient to exercise the EPICON observation model.
     for low,high in epicon_boxes:
      with np.errstate(divide='ignore',invalid='ignore'):
       first=(np.array(low)-pos)/directions;second=(np.array(high)-pos)/directions
      enter=np.max(np.minimum(first,second),axis=1);leave=np.min(np.maximum(first,second),axis=1)
      distances=np.minimum(distances,np.where((enter>0)&(leave>=enter),enter,np.inf))
    if u_shape:
     # A wall between start (0,0) and goal (4,0), open only above y=3.
     # Exact ray/AABB intersection, not an artificial point waypoint barrier.
     with np.errstate(divide='ignore',invalid='ignore'):
      t1=(np.array([1.5,-6.,0.])-pos)/directions
      t2=(np.array([2.5,3.,4.5])-pos)/directions
     enter=np.max(np.minimum(t1,t2),axis=1);leave=np.min(np.maximum(t1,t2),axis=1)
     distances=np.minimum(distances,np.where((enter>0)&(leave>=enter),enter,np.inf))
    valid=np.isfinite(distances) & (distances < float(os.environ.get('TEST_SENSOR_RANGE','30')))
    pts=pos+directions[valid]*distances[valid,None]
    cloud_pub.publish(create_cloud_xyz32(Header(stamp=stamp,frame_id='world'),pts.tolist()))
   tick+=1;time.sleep(.01)
 sensor=threading.Thread(target=sensor_loop,daemon=True);sensor.start()
 launch_args=['roslaunch',os.environ.get('TEST_LAUNCH_PACKAGE','task_planner'),'planner_runtime.launch','initial_mode:='+os.environ.get('TEST_INITIAL_MODE','target_exploration'),'marsim:=false','rviz:=false','auto_rviz_switch:=false','tracking_detector:=false','perceptor:=false','cloud_odom_mode:=latest_odom','target_exploration_auto_workspace:=false', 'exploration_mission_mode:='+('coverage' if os.environ.get('TEST_COVERAGE_SECONDS') else 'target')]
 # Release/Unity and source launch files have different odometry defaults.
 # Always connect the isolated fixture's own sensor topics explicitly.
 launch_args.extend(['odom_topic:=/lidar_slam/odom','cloud_topic:=/cloud_registered'])
 for key,arg in [('TEST_ROG_MAP_CONFIG','rog_map_config'),('TEST_TRACKING_ROG_MAP_CONFIG','tracking_rog_map_config')]:
  if os.environ.get(key):launch_args.append(arg+':='+os.environ[key])
 if os.environ.get('TEST_STARTUP_GRACE'):
  launch_args.append('source_startup_grace_duration:='+os.environ['TEST_STARTUP_GRACE'])
 if os.environ.get('TEST_EXPLORATION_OVERLAY'):
  overlay=os.environ['TEST_EXPLORATION_OVERLAY']
  if expect_epicon_blocked or expect_epicon_empty:
   import yaml
   parameters=yaml.safe_load(Path(overlay).read_text())
   if expect_epicon_blocked:
    parameters.setdefault('epicon',{}).update({'execution/stall_timeout':12.0,'execution/stall_reselect':3.0})
   if expect_epicon_empty:
    # Deliberately make every cluster dormant to exercise an audited empty
    # pool. This fixture tests termination, not physical coverage quality.
    parameters.setdefault('epicon',{})['FrontierManager/cluster_min_size']=1000.0
   overlay=str(base/'termination_fixture.yaml')
   Path(overlay).write_text(yaml.safe_dump(parameters))
  launch_args.append('exploration_overlay_config:='+overlay)
 if os.environ.get('TEST_NO_TOPOLOGY'):
  launch_args.append('global_topology_config:='+str(Path(__file__).resolve().parent/'config/target_route_no_topology.yaml'))
 if os.environ.get('TEST_TARGET_ROUTE_CONFIG'):
  launch_args.append('target_exploration_config:='+os.environ['TEST_TARGET_ROUTE_CONFIG'])
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
 goal=PoseStamped();goal.header.frame_id='world';goal.header.stamp=rospy.Time.now();goal.pose.position.x=float(os.environ.get('TEST_GOAL_X','4' if u_shape else '145'));goal.pose.position.z=float(os.environ.get('TEST_GOAL_Z','1.5'));goal.pose.orientation.w=1;goal_pub.publish(goal)
 began=time.monotonic();success=False
 return_trip=bool(os.environ.get('TEST_RETURN_TRIP'));return_sent=False;return_log_start=0
 coverage_seconds=float(os.environ.get("TEST_COVERAGE_SECONDS","0"))
 coverage_boxes=[]
 if coverage_seconds:
  for index in range(rospy.get_param('/planner_runtime_node/box_num')):
   prefix='/planner_runtime_node/box_%d/'%index
   coverage_boxes.append((np.array(rospy.get_param(prefix+'down')),np.array(rospy.get_param(prefix+'up'))))
  if epicon:
   for index,(low,high) in enumerate(coverage_boxes):
    prefix='/planner_runtime_node/epicon/box_%d/'%index
    assert np.array_equal(low,rospy.get_param(prefix+'down')) and np.array_equal(high,rospy.get_param(prefix+'up')),'native frontend task bounds differ from runtime'
 coverage_moved=False
 while time.monotonic()-began<float(os.environ.get('TEST_MAX_SECONDS','180')):
  history.append({'t':time.monotonic()-began,'p':pos.tolist(),'speed':float(np.linalg.norm(vel)),'result':status.task_result_str,'phase':status.phase_str,
                  'world_epoch':status.world_epoch,'map_revision':status.map_revision,'topo_revision':status.topo_revision,'topology_ready':status.topology_ready})
  if any('outside the active exploration capacity' in x for x in logs):raise AssertionError('old capacity rejection')
  if u_shape:assert not (1.3<pos[0]<2.7 and pos[1]<3.2),'command crossed U-wall safety margin'
  if coverage_seconds:
   assert any(np.all(pos>=low) and np.all(pos<=high) for low,high in coverage_boxes),'coverage command escaped its boxes'
   coverage_moved=coverage_moved or bool(np.linalg.norm(pos-np.array([0.,0.,1.5]))>1.)
   if expect_epicon_empty and status.task_result_str=='succeeded':
    success=True;break
   if os.environ.get('TEST_REQUIRE_COVERAGE_FINISH') and status.task_result_str=='succeeded':
    success=coverage_moved;break
   if time.monotonic()-began>=coverage_seconds:
    success=coverage_moved and status.active_mode_str=='exploration' and (
        not os.environ.get('TEST_REQUIRE_COVERAGE_FINISH') or status.task_result_str=='succeeded');break
  elif status.task_result_str=='succeeded' and np.linalg.norm(pos[:2]-np.array([goal.pose.position.x,0.]))<1.:
   if return_trip and not return_sent:
    if status.ready_for_new_task:
     return_sent=True;return_log_start=len(logs)
     goal.pose.position.x=float(os.environ.get('TEST_START_X','0'))
     goal.header.stamp=rospy.Time.now();goal_pub.publish(goal)
   else:success=True;break
  if status.task_result_str in ['blocked','failed'] and time.monotonic()-began>3:
   success=expect_epicon_blocked and status.task_result_str=='blocked'
   break
  if launch.poll() is not None:raise RuntimeError('runtime exited')
  time.sleep(.5)
 handover_passed=False
 if epicon and success:
  mode_pub.publish(String(data='hold'))
  deadline=time.monotonic()+20
  while time.monotonic()<deadline:
   if status.active_mode_str=='hold' and status.ready_for_new_task and np.linalg.norm(vel)<.15:
    handover_passed=True;break
   if launch.poll() is not None:raise RuntimeError('runtime exited during handover')
   time.sleep(.1)
 route_commits=sum('[target route commit] accepted' in x and 'source=KNOWN_' in x for x in logs)
 local_goal_commits=sum('[target route commit] accepted' in x and 'source=LOCAL_GOAL' in x for x in logs)
 return_commits=sum('[target route commit] accepted' in x and 'source=KNOWN_' in x for x in logs[return_log_start:]) if return_sent else 0
 validation_pending=sum('[target route]' in x and 'ROUTE_VALIDATION_PENDING' in x for x in logs)
 result={'success':success,'final':pos.tolist(),'elapsed':time.monotonic()-began,'result':status.task_result_str,'reason':status.reason,'known_route_commits':route_commits,'local_goal_commits':local_goal_commits,'return_route_commits':return_commits,'validation_pending':validation_pending,'history':history}
 if os.environ.get('TEST_REQUIRE_WORLD_MAP'):
  result['world_map']={'world_epochs':sorted({h['world_epoch'] for h in history}),
   'map_revision_start':history[0]['map_revision'],'map_revision_end':status.map_revision,
   'topo_revision_start':history[0]['topo_revision'],'topo_revision_end':status.topo_revision,
   'topology_ready':status.topology_ready}
 if coverage_seconds:
  result['coverage_execution']={
   'mixed_candidate_updates':sum('[coverage candidate]' in x and 'joint=1' in x and
       re.search(r'executable_frontiers=[1-9]',x) is not None for x in logs),
   'moving_candidate_updates':sum('[coverage candidate]' in x and
       float(re.search(r'speed=([\d.e+-]+)',x).group(1))>.5 for x in logs),
   'measured_passages':sum('measured observation passage' in x for x in logs),
   'coverage_passages':sum('measured observation passage' in x and 'coverage=1' in x for x in logs),
   'active_goal_finish_guard':sum('retain active observation after' in x for x in logs),
   'speed_admission_waits':sum('blocker=vehicle_speed' in x for x in logs)}
 if epicon:
  result['epicon']={
   'minimum_geometry_clearance':minimum_clearance,
   'measured_height_range':measured_height,
   'coverage_bounds_violation':coverage_bounds_violation,
   'hold_handover_passed':handover_passed,
   'global_updates':sum('[EPICON] global result=SUCCEED' in x for x in logs),
   'trajectory_commits':sum('[EPICON] trajectory backend result=3' in x for x in logs),
   'moving_global_updates':sum('[EPICON] global result=SUCCEED' in x and
       float(re.search(r'speed=([\d.e+-]+)',x).group(1))>.5 for x in logs),
   'legacy_coverage_updates':sum('[coverage candidate]' in x or '[coverage recovery] select' in x for x in logs)}
  if expect_epicon_blocked:
   result['epicon']['actuator_fault']=True
  if expect_epicon_empty:
   result['epicon']['empty_pool_fixture']=True
   result['epicon']['independent_empty_audits']=len({x['cloud_revision'] for x in native_audits if x.get('audited') and x['result']=='NO_FRONTIER'})
   result['epicon']['termination_evidence']=native_results
 (base/'synthetic_result.json').write_text(json.dumps(result,indent=2))
 print(json.dumps({k:v for k,v in result.items() if k!='history'}),flush=True)
 assert success,'synthetic corridor not completed; inspect runtime log'
 if os.environ.get('TEST_REQUIRE_WORLD_MAP'):
  assert len(result['world_map']['world_epochs'])==1 and status.world_epoch==history[0]['world_epoch'],'task or mode change replaced the world'
  assert status.map_revision>history[0]['map_revision'],'shared ROG stopped fusing during exploration'
  assert status.topology_ready and status.topo_revision>0,'shared global topology unavailable'
 if epicon:
  assert not coverage_bounds_violation,'100 Hz follower escaped exploration bounds'
  assert handover_passed,'EPICON did not release command ownership into a verified hold'
  assert minimum_clearance>=.30,'trajectory crossed synthetic obstacle clearance'
  if not expect_epicon_empty:
   assert result['epicon']['trajectory_commits']>=2,'native paths did not reach General trajectory optimizer'
  assert result['epicon']['legacy_coverage_updates']==0,'legacy coverage selection remained active'
  if expect_epicon_blocked:
   assert not any(h['result']=='succeeded' for h in history),'stationary actuator fault reported successful exploration'
   assert any('[EPICON] defer failed goal' in x for x in logs),'stationary recovery never tried another goal'
   assert history[-1]['t']<30,'recovery exceeded its configured bounded timeout'
  if expect_epicon_empty:
   assert result['epicon']['independent_empty_audits']>=4,'completion reused sensor evidence'
   assert any(x.get('result')=='COMPLETE' and x.get('verified_frames',0)>=4 for x in native_results),'completion lacked settled fresh audits'
 if os.environ.get('TEST_REQUIRE_JOINT_COVERAGE'):
  assert result['coverage_execution']['mixed_candidate_updates']>0,'joint candidate pipeline was not exercised'
  assert result['coverage_execution']['speed_admission_waits']==0,'joint coverage waited for low-speed admission'
 if os.environ.get('TEST_REQUIRE_ROUTE'):assert route_commits>0,'task finished without executing a known route'
 if return_trip:assert return_sent and return_commits>0,'return task did not execute the existing topology route'
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
