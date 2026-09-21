"""Position-only adapter around the checked-in IMM-MOT motion code.
No NuScenes data, ground truth, measured yaw or detector velocity is injected.
"""
import os
os.environ['OPENBLAS_NUM_THREADS']='1'
os.environ['OMP_NUM_THREADS']='1'
import ast,copy,importlib.util,math,pickle,sys,types
from pathlib import Path
import numpy as np
import corrected_mixing
import process_noise
from pyquaternion import Quaternion
UPSTREAM=None
POSITION_STD=0.16
BASE_TRANSITION=np.array([[.95,.02,.02,.01],[.02,.95,.01,.02],[.02,.01,.95,.02],[.01,.02,.02,.95]])
TRANSITION_EIGENVALUES,TRANSITION_EIGENVECTORS=np.linalg.eigh(BASE_TRANSITION)

def set_options(position_std):
 global POSITION_STD
 if not math.isfinite(position_std) or position_std<=0:
  raise ValueError('position_std must be finite and positive')
 POSITION_STD=position_std

def transition_for_dt(dt):
 # Symmetric stochastic M: exact fractional power, M(0.5 s)=original M.
 matrix=(TRANSITION_EIGENVECTORS*(TRANSITION_EIGENVALUES**(dt/.5)))@TRANSITION_EIGENVECTORS.T
 if matrix.min() < -1e-10:raise ValueError('nonstochastic time-scaled transition')
 matrix=np.maximum(matrix,0.)
 return matrix/matrix.sum(axis=1,keepdims=True)


def set_process_noise(acceleration_psd,jerk_psd,turn_acceleration_psd,vertical_position_psd):
 values=dict(acceleration_psd=acceleration_psd,jerk_psd=jerk_psd,turn_acceleration_psd=turn_acceleration_psd,vertical_position_psd=vertical_position_psd)
 process_noise.validate(values);process_noise.DEFAULTS.update(values)

def configure(root):
 global UPSTREAM
 root=Path(root).resolve();sys.path.insert(0,str(root))
 # Load constants without executing unrelated NuScenes data package imports.
 for name, relative in [('data','data'),('data.script','data/script')]:
  package=types.ModuleType(name);package.__path__=[str(root/relative)];sys.modules[name]=package
 # Drop ONLY the NuScenes box-format import. Frame bookkeeping is unused in
 # a single selected-track backend; all filtering/mixing code is loaded intact.
 path=root/'motion_module/kalman_filter.py';tree=ast.parse(path.read_text(),str(path))
 tree.body=[n for n in tree.body if not isinstance(n,ast.ImportFrom) or n.module!='pre_processing']
 module=types.ModuleType('motion_module.kalman_filter');module.__package__='motion_module'
 sys.modules[module.__name__]=module;exec(compile(tree,str(path),'exec'),module.__dict__);UPSTREAM=module
 module.KalmanFilter.addFrameObject=lambda *args:None
 module.KalmanFilter.getMeasureInfo=lambda self,det:np.mat(det['np_array'][:3]).T
 # Remove unavailable heading, box size and velocity from observation space.
 for cls in [module.CV,module.CA,module.CTRV,module.CTRA]:
  cls.getMeasureDim=lambda self:3
  old_r=cls.getMeaNoiseR;old_h=cls.getMeaStateH
  cls.getMeaNoiseR=lambda self,old=old_r:old(self)[:3,:3]
  cls.getMeaStateH=lambda self,*args,old=old_h:old(self,*args)[:3,:]
  cls.StateToMeasure=lambda self,state:state[:3].copy()
  cls.warpResYawToPi=staticmethod(lambda residual:residual)
 original_combine=module.IMMFilter._compute_state_estimate
 def combine(self):
  if getattr(self,'corrected',False):corrected_mixing.combine(self)
  else:original_combine(self)
 module.IMMFilter._compute_state_estimate=combine
 module.IMMFilter.StateToMeasure=lambda self,state:state[:3].copy()
 module.IMMFilter.warpResYawToPi=staticmethod(lambda residual:residual)

def detection(position,size):
 q=Quaternion(axis=(0,0,1),radians=0.)
 array=np.array([*position,*size,0.,0.,*q.q,1.,4.])
 return dict(seq_id=0,np_array=array,nusc_box=types.SimpleNamespace(yaw=q.radians))

def filters(obj):return obj.filters if isinstance(obj,UPSTREAM.IMMFilter) else [obj]
def refresh(obj):
 if isinstance(obj,UPSTREAM.IMMFilter):obj._compute_state_estimate()
def set_dt(obj,dt):
 for f in filters(obj):
  f.dt=dt;f.model.dt=dt
  # Original Q is per 0.5 s scan; scale its variance to actual elapsed time.
  if not hasattr(obj,'physical_noise'):f.Q=f.model.getProcessNoiseQ()*(dt/.5)
  if isinstance(f,UPSTREAM.LinearKalmanFilter):f.F=f.model.getTransitionF()

def summary(ctx):
 f=ctx['filter'];x=np.asarray(f.state).ravel()
 if isinstance(f,UPSTREAM.IMMFilter):
  state=[x[0],x[1],x[6],x[7],x[9],x[10],x[13]];mu=list(f.mu)
  indices=[0,1,6,7];cov=np.asarray(f.P)[np.ix_(indices,indices)]
 else:
  speed,acc,yaw,omega=x[6:10];c,s=math.cos(yaw),math.sin(yaw)
  state=[x[0],x[1],speed*c,speed*s,acc*c,acc*s,omega];mu=[0.,0.,0.,1.]
  jac=np.zeros((4,10));jac[0,0]=jac[1,1]=1;jac[2,6]=c;jac[2,8]=-speed*s;jac[3,6]=s;jac[3,8]=speed*c
  cov=jac@np.asarray(f.P)@jac.T
 if not np.isfinite(state).all() or not np.isfinite(cov).all() or not np.isfinite(mu).all():
  raise FloatingPointError('IMM-MOT produced nonfinite state/covariance/probabilities')
 return pickle.dumps(ctx,protocol=4),state,cov.ravel().tolist(),mu

def create(mode,position,size,stamp):
 cfg={'basic':{'LiDAR_interval':.1,'has_velo':False},'motion_model':{'model':{4:'CTRA'},'mu':{4:[.4,.4,.1,.1]},'M':{4:[[.95,.02,.02,.01],[.02,.95,.01,.02],[.02,.01,.95,.02],[.01,.02,.02,.95]]}}}
 cls=UPSTREAM.IMMFilter if mode in ('imm_mot','imm_mot_r','imm_mot_rt','imm_mot_fixed','imm_mot_fixed_q') else UPSTREAM.ExtendKalmanFilter
 f=cls(0,cfg,0,detection(position,size))
 if mode in ('imm_mot_r','imm_mot_rt','imm_mot_fixed','imm_mot_fixed_q'):
  for sub in filters(f):sub.R=np.mat(np.eye(3)*POSITION_STD**2)
 if mode in ('imm_mot_fixed','imm_mot_fixed_q'):f.corrected=True;refresh(f)
 if mode=='imm_mot_fixed_q':f.physical_noise=process_noise.DEFAULTS.copy()
 return summary(dict(mode=mode,filter=f,last_position=np.array(position),last_stamp=stamp,stamp=stamp,size=list(size),seeded=False))

def advance(ctx,dt):
 if dt<=0:return
 if ctx.get('mode')=='imm_mot_fixed_q' and dt>.10000001:
  remaining=dt
  while remaining>1e-8:
   h=min(.1,remaining);advance(ctx,h);remaining-=h
  return
 f=ctx['filter']
 set_dt(f,dt)
 if ctx.get('mode') in ('imm_mot_fixed','imm_mot_fixed_q'):
  f.M=transition_for_dt(dt);corrected_mixing.predict(f);ctx['stamp']+=dt;return
 if ctx.get('mode')=='imm_mot_rt':
  f.M=transition_for_dt(dt)
  f._compute_mixing_probabilities()
 f.predict(0)
 if ctx.get('mode')=='imm_mot_rt':
  # On missing observations the mode posterior is its propagated prior.
  # update() still uses this step's cbar, so no transition is counted twice.
  f.mu=f.cbar.copy();refresh(f)
 ctx['stamp']+=dt

def operate(blob,operation,values):
 ctx=pickle.loads(blob);f=ctx['filter'];values=list(values)
 if operation=='predict':advance(ctx,values[0])
 elif operation=='update':
  position=np.array(values[:3]);stamp=values[3];delta=position-ctx['last_position'];dt=stamp-ctx['last_stamp']
  # Polar models need an initial motion direction. Derive a one-time prior
  # from consecutive accepted positions, never simulator orientation/velocity.
  if not ctx['seeded'] and .02<=dt<=.5 and np.linalg.norm(delta[:2])>.05:
   velocity=delta[:2]/dt;speed=np.linalg.norm(velocity)
   if speed<=3.:
    yaw=math.atan2(velocity[1],velocity[0])
    for sub in filters(f):
     if isinstance(sub.model,(UPSTREAM.CV,UPSTREAM.CA)):
      sub.state[6:8,0]=np.mat(velocity).T;sub.state[-1,0]=yaw
     else:sub.state[6,0]=speed;sub.state[-2,0]=yaw
    ctx['seeded']=True;refresh(f)
  if ctx.get('mode') in ('imm_mot_fixed','imm_mot_fixed_q'):corrected_mixing.update(f,position)
  else:f.update(0,detection(position,ctx['size']))
  ctx['last_position']=position;ctx['last_stamp']=stamp
 elif operation=='snap':
  shift=np.array(values[:3])-np.asarray(f.state[:3]).ravel()
  for sub in filters(f):sub.state[:3,0]+=np.mat(shift).T
  refresh(f)
 elif operation in ('damp','limit','blend'):
  for sub in filters(f):
   linear=isinstance(sub.model,(UPSTREAM.CV,UPSTREAM.CA))
   if operation=='damp':
    if linear:sub.state[6:9]*=values[0]
    else:sub.state[6]*=values[0];sub.state[-1]*=values[0]
    if isinstance(sub.model,UPSTREAM.CA):sub.state[9:12]*=values[0]
    if isinstance(sub.model,UPSTREAM.CTRA):sub.state[7]*=values[0]
   elif operation=='limit':
    if linear:
     speed=np.linalg.norm(sub.state[6:8]);sub.state[6:8]*=min(1.,values[0]/max(speed,1e-9))
    else:sub.state[6,0]=np.clip(sub.state[6,0],-values[0],values[0])
   else:
    old=np.asarray(sub.state[6:8]).ravel() if linear else np.array([math.cos(float(sub.state[-2])),math.sin(float(sub.state[-2]))])*float(sub.state[6])
    v=(1-values[2])*old+values[2]*np.array(values[:2])
    if linear:sub.state[6:8,0]=np.mat(v).T
    else:sub.state[6,0]=np.linalg.norm(v);sub.state[-2,0]=math.atan2(v[1],v[0])
  refresh(f)
 elif operation=='future':
  # Forecast on private deserialized state, without touching live posterior.
  remaining=values[0]
  while remaining>1e-8:
   h=min(.1,remaining);advance(ctx,h);remaining-=h
 else:raise ValueError(operation)
 return summary(ctx)
