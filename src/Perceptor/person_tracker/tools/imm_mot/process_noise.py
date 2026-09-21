"""Continuous-time driving-noise PSD -> discrete covariance.
Polar models use a local frozen Jacobian and exact Van Loan integration for it.
This is not an exact discretization of the nonlinear stochastic system.
"""
import math
import numpy as np
from scipy.linalg import expm
DEFAULTS=dict(acceleration_psd=6.25,jerk_psd=16.,turn_acceleration_psd=.36,vertical_position_psd=.01)

def validate(config):
 for name in DEFAULTS:
  if not math.isfinite(config[name]) or config[name]<0:raise ValueError(name+' must be finite and nonnegative')

def polar_linearization(state,kind,config):
 # Reduced coordinates [x,y,v,(a),theta,omega].
 indices=[0,1,6,7,8,9] if kind=='CTRA' else [0,1,6,7,8]
 x=np.asarray(state).ravel();n=len(indices);a=np.zeros((n,n));d=np.zeros((n,n))
 theta=x[-2];v=x[6];heading=n-2;omega=n-1
 a[0,2]=math.cos(theta);a[1,2]=math.sin(theta)
 a[0,heading]=-v*math.sin(theta);a[1,heading]=v*math.cos(theta)
 a[heading,omega]=1.
 if kind=='CTRA':a[2,3]=1.;d[3,3]=config['jerk_psd']
 else:d[2,2]=config['acceleration_psd']
 d[omega,omega]=config['turn_acceleration_psd']
 return indices,a,d

def van_loan(a,d,dt):
 n=len(a);block=np.zeros((2*n,2*n));block[:n,:n]=a;block[:n,n:]=d;block[n:,n:]=-a.T
 e=expm(block*dt);q=e[:n,n:]@e[:n,:n].T
 return (q+q.T)/2

def discrete(kind,state,dt,config=None):
 config=DEFAULTS if config is None else config
 if not math.isfinite(dt) or dt<0:raise ValueError('dt must be finite and nonnegative')
 validate(config);q=np.zeros((len(state),len(state)))
 if dt==0:return q
 if kind=='CV':
  block=config['acceleration_psd']*np.array([[dt**3/3,dt**2/2],[dt**2/2,dt]])
  for axis in (0,1):q[np.ix_([axis,6+axis],[axis,6+axis])]=block
 elif kind=='CA':
  block=config['jerk_psd']*np.array([[dt**5/20,dt**4/8,dt**3/6],[dt**4/8,dt**3/3,dt**2/2],[dt**3/6,dt**2/2,dt]])
  for axis in (0,1):q[np.ix_([axis,6+axis,9+axis],[axis,6+axis,9+axis])]=block
 elif kind in ('CTRV','CTRA'):
  indices,a,d=polar_linearization(state,kind,config);q[np.ix_(indices,indices)]=van_loan(a,d,dt)
 else:raise ValueError(kind)
 # Shared small vertical-position random walk. Shape dimensions are static.
 q[2,2]=config['vertical_position_psd']*dt
 return q
