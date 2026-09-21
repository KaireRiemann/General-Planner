#include "tracking/imm_filter.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace person_tracker {
void ImmFilter::initialize(const Eigen::Vector2d & position, double q, double r, const Config & config) {
  if (!position.allFinite() || !std::isfinite(q) || q<=0 || !std::isfinite(r) || r<=0 ||
      !std::isfinite(config.jerk_noise) || config.jerk_noise<=0 ||
      !std::isfinite(config.turn_noise) || config.turn_noise<=0 ||
      !std::isfinite(config.stay_probability) || config.stay_probability<=1./3. || config.stay_probability>=1.)
    throw std::invalid_argument("Invalid IMM initial state/noise/transition probability");
  config_=config; acceleration_noise_=q; measurement_noise_=r;
  probabilities_={{.45,.30,.25}};
  for (auto & m:modes_) {
    m.x.setZero(); m.x.head<2>()=position;
    m.p.setZero(); m.p.diagonal()<<.09,.09,1.,1.,4.,4.,1.;
  }
  combine();
}
ImmFilter::State ImmFilter::transition(const State & x,double dt,int mode) {
  State y=x;
  if(mode==0) {y.head<2>()+=x.segment<2>(2)*dt;y.tail<3>().setZero();}
  else if(mode==1) {
    y.head<2>()+=x.segment<2>(2)*dt+.5*x.segment<2>(4)*dt*dt;
    y.segment<2>(2)+=x.segment<2>(4)*dt;y(6)=0.;
  } else {
    const double angle=x(6)*dt,s=std::sin(angle),c=std::cos(angle);
    const double a=std::abs(angle)<1e-4 ? dt*(1-angle*angle/6) : s/x(6);
    const double b=std::abs(angle)<1e-4 ? dt*(angle/2-angle*angle*angle/24) : (1-c)/x(6);
    y(0)+=a*x(2)-b*x(3);y(1)+=b*x(2)+a*x(3);
    y(2)=c*x(2)-s*x(3);y(3)=s*x(2)+c*x(3);
    y(4)=-x(6)*y(3);y(5)=x(6)*y(2);
  }
  return y;
}
void ImmFilter::combine() {
  mean_.setZero();covariance_.setZero();
  for(int i=0;i<3;++i) mean_+=probabilities_[i]*modes_[i].x;
  for(int i=0;i<3;++i) {
    const State d=modes_[i].x-mean_;
    covariance_+=probabilities_[i]*(modes_[i].p+d*d.transpose());
  }
  covariance_=(.5*(covariance_+covariance_.transpose())).eval();
}
void ImmFilter::predict(double dt) {
  if(!std::isfinite(dt) || dt<0.) throw std::invalid_argument("Invalid IMM dt");
  // Accurate elapsed time and stable nonlinear covariance on scan gaps.
  while(dt>1e-9) {const double h=std::min(dt,.1);step(h);dt-=h;}
}
void ImmFilter::step(double dt) {
  // Symmetric continuous-time Markov chain, calibrated at dt=0.1 seconds.
  const double e=std::pow((3*config_.stay_probability-1)/2,dt/.1);
  const double stay=(1+2*e)/3,change=(1-e)/3;
  const auto old=modes_; const auto mu=probabilities_;
  for(int j=0;j<3;++j) {
    double prior=0;std::array<double,3> weights{};
    for(int i=0;i<3;++i) {weights[i]=mu[i]*(i==j?stay:change);prior+=weights[i];}
    Mode mixed; mixed.p.setZero();
    for(int i=0;i<3;++i) {weights[i]/=prior;mixed.x+=weights[i]*old[i].x;}
    for(int i=0;i<3;++i) {State d=old[i].x-mixed.x;mixed.p+=weights[i]*(old[i].p+d*d.transpose());}
    Covariance f;
    for(int k=0;k<7;++k) {
      const double eps=1e-5*std::max(1.,std::abs(mixed.x(k)));
      State plus=mixed.x,minus=mixed.x;plus(k)+=eps;minus(k)-=eps;
      f.col(k)=(transition(plus,dt,j)-transition(minus,dt,j))/(2*eps);
    }
    Covariance q=Covariance::Identity()*1e-10;
    for(int axis=0;axis<2;++axis) {
      State g=State::Zero();
      if(j==1) {g(axis)=dt*dt*dt/6;g(axis+2)=dt*dt/2;g(axis+4)=dt;}
      else {g(axis)=dt*dt/2;g(axis+2)=dt;}
      const double noise=j==1?config_.jerk_noise:acceleration_noise_;
      q+=noise*noise*g*g.transpose();
    }
    if(j==2) q(6,6)+=config_.turn_noise*config_.turn_noise*dt;
    modes_[j].x=transition(mixed.x,dt,j);
    modes_[j].p=f*mixed.p*f.transpose()+q;
    modes_[j].p=(.5*(modes_[j].p+modes_[j].p.transpose())).eval();
    probabilities_[j]=prior;
  }
  combine();
}
void ImmFilter::update(const Eigen::Vector2d & z) {
  if(!z.allFinite()) throw std::invalid_argument("Invalid IMM measurement");
  std::array<double,3> log_weights{};
  const Eigen::Matrix2d r=Eigen::Matrix2d::Identity()*measurement_noise_*measurement_noise_;
  for(int i=0;i<3;++i) {
    auto & m=modes_[i];const Eigen::Vector2d innovation=z-m.x.head<2>();
    const Eigen::Matrix2d s=m.p.topLeftCorner<2,2>()+r;
    const Eigen::LLT<Eigen::Matrix2d> llt(s);
    if(llt.info()!=Eigen::Success) throw std::runtime_error("IMM innovation covariance not positive definite");
    const Eigen::Matrix2d l=llt.matrixL();
    const double log_det=2*(std::log(l(0,0))+std::log(l(1,1)));
    log_weights[i]=std::log(std::max(probabilities_[i],1e-300))-.5*(innovation.dot(llt.solve(innovation))+log_det+2*std::log(2*std::acos(-1.)));
    const Eigen::Matrix<double,7,2> k=llt.solve(m.p.leftCols<2>().transpose()).transpose();
    m.x+=k*innovation;
    Covariance residual=Covariance::Identity();residual.leftCols<2>()-=k;
    m.p=residual*m.p*residual.transpose()+k*r*k.transpose();
    m.p=(.5*(m.p+m.p.transpose())).eval();
  }
  double sum=0;const double peak=*std::max_element(log_weights.begin(),log_weights.end());
  for(int i=0;i<3;++i) {probabilities_[i]=std::max(1e-12,std::exp(log_weights[i]-peak));sum+=probabilities_[i];}
  for(auto & p:probabilities_) p/=sum;
  combine();
}
void ImmFilter::snap(const Eigen::Vector2d & position) {
  const Eigen::Vector2d shift=position-mean_.head<2>();
  for(auto & m:modes_) m.x.head<2>()+=shift;
  // External association correction: preserve uncertainty, never shrink it
  // as though an additional independent measurement had been received.
  combine();
}
void ImmFilter::damp(double factor) {
  factor=std::clamp(factor,0.,1.);
  Covariance t=Covariance::Identity();t.bottomRightCorner<5,5>()*=factor;
  for(auto & m:modes_) {m.x=t*m.x;m.p=(t*m.p*t.transpose()).eval();m.p.diagonal().array()+=1e-10;}
  combine();
}
void ImmFilter::limitSpeed(double maximum) {
  for(auto & m:modes_) {double s=m.x.segment<2>(2).norm();if(s>maximum && s>1e-9)m.x.segment<2>(2)*=maximum/s;}
  combine();
}
void ImmFilter::blendVelocity(const Eigen::Vector2d & velocity,double factor) {
  for(auto & m:modes_) m.x.segment<2>(2)=(1-factor)*m.x.segment<2>(2)+factor*velocity;
  combine();
}
}
