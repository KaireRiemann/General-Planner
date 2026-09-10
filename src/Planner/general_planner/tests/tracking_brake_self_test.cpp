#include <general_core/tracking/tracking_brake.hpp>
#include <iostream>
Eigen::Vector3d derivative(const Eigen::Matrix<double,3,8>& c,double t,int order){
  Eigen::Vector3d v=Eigen::Vector3d::Zero();
  for(int k=0;k<8;++k){int power=7-k;if(power<order)continue;
    double factor=1;for(int n=0;n<order;++n)factor*=power-n;
    v+=c.col(k)*factor*std::pow(t,power-order);
  }return v;
}
int main(){
  Eigen::Matrix<double,3,4> start;
  start << 1,3,.2,.1, 2,-1,.1,-.2, 1.5,.2,0,.1;
  for(double T:{.8,1.5,3.0}){
    auto c=general_planner::trackingBrakeCoefficients(start,T);
    for(int d=0;d<4;++d)
      if((derivative(c,0,d)-start.col(d)).norm()>1e-8)return 1;
    for(int d=1;d<4;++d)
      if(derivative(c,T,d).norm()>1e-7)return 2;
    auto end=start.col(0)+start.col(1)*T/2+start.col(2)*T*T/12;
    if((derivative(c,T,0)-end).norm()>1e-7)return 3;
  }
  std::cout<<"tracking brake C3 boundaries passed\n";
}
