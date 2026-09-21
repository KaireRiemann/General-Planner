"""Opt-in corrected interaction/update for upstream CV/CA/CTRV/CTRA.
Common acceleration is physical Cartesian acceleration, including centripetal terms.
Unmodelled deterministic coordinates have zero mapped covariance, not artificial 1000.
"""
import math
import numpy as np
import process_noise

def wrap(x):return math.atan2(math.sin(x),math.cos(x))
def common(x,kind):
 y=np.zeros(14);y[:6]=x[:6]
 if kind in ('CV','CA'):
  y[6:9]=x[6:9];y[12]=math.atan2(x[7],x[6]) if np.linalg.norm(x[6:8])>.05 else x[-1]
  if kind=='CA':
   y[9:12]=x[9:12]
   if np.linalg.norm(x[6:8])>.05:y[13]=(x[6]*x[10]-x[7]*x[9])/(x[6]**2+x[7]**2)
 else:
  v=x[6];a=x[7] if kind=='CTRA' else 0.;theta,omega=x[-2:]
  c,s=math.cos(theta),math.sin(theta)
  y[6:8]=[v*c,v*s];y[9:11]=[a*c-v*omega*s,a*s+v*omega*c];y[12:]=[theta,omega]
 return y

def native(y,kind,reference_yaw=None):
 if kind=='CV':return np.r_[y[:9],y[12]]
 if kind=='CA':return y[:13].copy()
 # Choose an equivalent signed-speed chart near the destination heading.
 # Taking abs(speed) while retaining heading reverses negative low velocities.
 reference=y[12] if reference_yaw is None else reference_yaw
 speed=np.linalg.norm(y[6:8])
 if speed>1e-12:
  theta=math.atan2(y[7],y[6])
  if math.cos(theta-reference)<0:
   speed=-speed;theta=wrap(theta+math.pi)
 else:
  speed=0.;theta=reference
 # Signed tangential acceleration; never hypot(ax,ay).
 a=y[9]*math.cos(theta)+y[10]*math.sin(theta)
 return np.r_[y[:6],speed,a,theta,y[13]] if kind=='CTRA' else np.r_[y[:6],speed,theta,y[13]]

def transform(x,p,fn,angle):
 y=fn(x);j=np.empty((len(y),len(x)))
 for k in range(len(x)):
  h=1e-5*max(1.,abs(x[k]));a=x.copy();b=x.copy();a[k]+=h;b[k]-=h
  d=fn(a)-fn(b);d[angle]=wrap(d[angle]);j[:,k]=d/(2*h)
 q=j@p@j.T;return y,(q+q.T)/2

def moments(xs,ps,weights,angle):
 weights=np.asarray(weights)
 mean=np.sum(np.array(xs)*weights[:,None],axis=0)
 # Unwrap around the highest-weight hypothesis, avoiding +/-pi arithmetic averaging.
 ref=xs[int(np.argmax(weights))][angle]
 mean[angle]=wrap(ref+sum(w*wrap(x[angle]-ref) for w,x in zip(weights,xs)))
 p=np.zeros_like(ps[0])
 for w,x,q in zip(weights,xs,ps):
  d=x-mean;d[angle]=wrap(d[angle]);p+=w*(q+np.outer(d,d))
 return mean,(p+p.T)/2

def mapped(bank):
 values=[]
 for f in bank.filters:
  x=np.asarray(f.state).ravel();kind=type(f.model).__name__;j=common_jacobian(x,kind)
  p=j@np.asarray(f.P)@j.T;values.append((common(x,kind),(p+p.T)/2))
 return [v[0] for v in values],[v[1] for v in values]

def combine(bank):
 xs,ps=mapped(bank);x,p=moments(xs,ps,bank.mu,12)
 bank.state=np.mat(x).T;bank.P=np.mat(p)

def predict(bank):
 """Mix in each destination model's coordinates, then predict once.

 Keeping the self component in its native coordinates preserves its full
 covariance, including heading uncertainty at zero speed. A polar -> Cartesian
 -> polar round trip is singular at rest and must not replace that component.
 """
 bank._compute_mixing_probabilities()
 sources=[(np.asarray(f.state).ravel().copy(),np.asarray(f.P).copy(),type(f.model).__name__) for f in bank.filters]
 for j,f in enumerate(bank.filters):
  kind=type(f.model).__name__
  angle=len(sources[j][0])-(1 if kind in ('CV','CA') else 2)
  reference=sources[j][0][angle]
  converted=[]
  for i,(state,cov,source_kind) in enumerate(sources):
   if i==j:
    converted.append((state,cov))
   else:
    shared=common(state,source_kind)
    jac=native_jacobian(shared,kind,reference)@common_jacobian(state,source_kind)
    mapped_cov=jac@cov@jac.T
    converted.append((native(shared,kind,reference),(mapped_cov+mapped_cov.T)/2))
  y,q=moments([z[0] for z in converted],[z[1] for z in converted],bank.omega[:,j],angle)
  f.state=np.mat(y).T;f.P=np.mat(q)
  if hasattr(bank,'physical_noise'):
   f.Q=np.mat(process_noise.discrete(kind,y,f.dt,bank.physical_noise))
  f.predict(0)
 bank.mu=bank.cbar.copy();combine(bank)

def update(bank,position):
 logs=[]
 for f in bank.filters:
  x=np.asarray(f.state).ravel();p=np.asarray(f.P);r=np.asarray(f.R)
  h=np.zeros((3,len(x)));h[:,:3]=np.eye(3)
  residual=np.asarray(position)-x[:3];s=h@p@h.T+r
  sign,logdet=np.linalg.slogdet(s)
  if sign<=0:raise FloatingPointError('non-positive innovation covariance')
  gain=np.linalg.solve(s,h@p).T;x+=gain@residual
  a=np.eye(len(x))-gain@h;p=a@p@a.T+gain@r@gain.T
  f.model.warpStateYawToPi(np.mat(x).T)
  kind=type(f.model).__name__;idx=-1 if kind in ('CV','CA') else -2;x[idx]=wrap(x[idx])
  f.state=np.mat(x).T;f.P=np.mat((p+p.T)/2)
  logs.append(-.5*(residual@np.linalg.solve(s,residual)+logdet+3*math.log(2*math.pi)))
 weights=np.log(np.maximum(bank.mu,1e-300))+logs;weights=np.exp(weights-max(weights));bank.mu=weights/sum(weights)
 combine(bank)


def common_jacobian(x,kind):
 """Jacobian of native -> common coordinates away from chart boundaries."""
 j=np.zeros((14,len(x)));j[:6,:6]=np.eye(6)
 if kind in ('CV','CA'):
  j[6:9,6:9]=np.eye(3);vx,vy=x[6:8];s2=vx*vx+vy*vy
  if s2>.05**2:j[12,6:8]=[-vy/s2,vx/s2]
  else:j[12,-1]=1.
  if kind=='CA':
   j[9:12,9:12]=np.eye(3)
   if s2>.05**2:
    ax,ay=x[9:11];cross=vx*ay-vy*ax
    j[13,6]=(ay*s2-2*vx*cross)/s2**2
    j[13,7]=(-ax*s2-2*vy*cross)/s2**2
    j[13,9:11]=[-vy/s2,vx/s2]
 else:
  v=x[6];a=x[7] if kind=='CTRA' else 0.;theta,omega=x[-2:];c,s=math.cos(theta),math.sin(theta)
  j[6:8,6]=[c,s];j[6:8,-2]=[-v*s,v*c]
  j[9:11,6]=[-omega*s,omega*c]
  j[9:11,-2]=[-a*s-v*omega*c,a*c-v*omega*s]
  j[9:11,-1]=[-v*s,v*c]
  if kind=='CTRA':j[9:11,7]=[c,s]
  j[12,-2]=1.;j[13,-1]=1.
 return j


def native_jacobian(y,kind,reference_yaw):
 n=native(y,kind,reference_yaw);j=np.zeros((len(n),14));j[:6,:6]=np.eye(6)
 if kind=='CV':j[:9,:9]=np.eye(9);j[-1,12]=1.
 elif kind=='CA':j[:13,:13]=np.eye(13)
 else:
  theta=n[-2];c,s=math.cos(theta),math.sin(theta);s2=float(y[6:8]@y[6:8])
  j[6,6:8]=[c,s]
  if s2>1e-24:j[-2,6:8]=[-y[7]/s2,y[6]/s2]
  else:j[-2,12]=1. # At rest retain the destination heading chart.
  j[-1,13]=1.
  if kind=='CTRA':
   j[7,9:11]=[c,s];j[7]+=(-y[9]*s+y[10]*c)*j[-2]
 return j
