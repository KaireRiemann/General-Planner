#include "utils/registration.h"

namespace ICPAlgorithm {


bool plane_fit(Eigen::Matrix<float, 3, Eigen::Dynamic> A, Eigen::Vector4f &plane_coeff)
{
    Eigen::Vector3f centroid = A.rowwise().mean();
    Eigen::MatrixXf centered = A.colwise() - centroid;
    Eigen::Matrix3f cov = centered * centered.transpose();
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eig(cov);
    Eigen::Vector3f normal = eig.eigenvectors().col(0);
    float d = -normal.dot(centroid);
    Eigen::Vector4f plane_eq;
    plane_coeff << normal, d; // std::cout << "Equation of plane1: " << plane_coeff(0) << "x + " << plane_coeff(1) << "y + " << plane_coeff(2) << "z + " << plane_coeff(3) << " = 0" << std::endl;

    // Compute residuals
    Eigen::VectorXf residuals = centered.transpose() * normal;

    // Check for large residuals
    for (int i = 0; i < residuals.size(); i++)
    {
        if (fabs(residuals(i)) > 0.1f)
            return false;
    }
    return true;
}

void addNormal(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
               pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_with_normals,
               int num)
{
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

  pcl::search::KdTree<pcl::PointXYZI>::Ptr searchTree(new pcl::search::KdTree<pcl::PointXYZI>);
  searchTree->setInputCloud(cloud);

  pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> normalEstimator;
  normalEstimator.setInputCloud(cloud);
  normalEstimator.setSearchMethod(searchTree);
  normalEstimator.setKSearch(num);
  normalEstimator.compute(*normals);

  pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
}

// Eigen::Matrix4f GDPointToPlane(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud1,
//                                pcl::PointCloud<pcl::PointXYZI>::Ptr cloud2,
//                                Eigen::Matrix4f& guess,
//                                std::vector<double> &ret,
//                                pcl::Correspondences& cor,
//                                std::vector<double> params)
// {
//     pcl::PointCloud<pcl::PointXYZI>::Ptr pc_src_world(new pcl::PointCloud<pcl::PointXYZI>());
//     pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
//     kdtree.setInputCloud(cloud2);

//     bool isconverge = false;
//     int iter_conter = 0;
//     Eigen::Matrix4f transform = guess;
//     Eigen::Vector3f transition = guess.block<3,1>(0,3);

//     Eigen::Quaternionf _q(guess.block<3,3>(0,0));
//     _q = _q.normalized();
//     pcl::transformPointCloud(*cloud1, *pc_src_world, guess);

//     while(!isconverge)
//     {
//         iter_conter++;
//         int effct_feat_num = 0;
//         std::vector<Eigen::Vector4f> plane_seq;       // norm(a,b,c) + d => ax + by + cz + d =0
//         std::vector<Eigen::Vector3f> world_point_seq; // norm(a,b,c) + d => ax + by + cz + d =0
//         std::vector<Eigen::Vector3f> plane_point_seq; // norm(a,b,c) + d => ax + by + cz + d =0
//         plane_seq.clear();
//         world_point_seq.clear();
//         plane_point_seq.clear();
//         float error_R= 0;
//         float error_t= 0;
//         float error_Rt = 999.9;

//         // Matching features
//         sensor_msgs::PointCloud2 scan_pc_msg_val;
//         pcl::PointCloud<pcl::PointXYZI> valid_point;
//         valid_point.clear();

//         for (int i = 0; i < pc_src_world->size(); i++)
//         {
//             pcl::PointXYZI point_world = pc_src_world->points[i];
//             pcl::PointCloud<pcl::PointXYZI> pc_vis;
//             pc_vis.clear();
            
//             std::vector<int> pointIdxNKNSearch(int(params[0]));
//             //保存对象点与邻近点的距离平方值
//             std::vector<float> pointNKNSquaredDistance(int(params[0]));            
//             kdtree.nearestKSearch (point_world, int(params[0]), pointIdxNKNSearch, pointNKNSquaredDistance);
//             float maxValue = *max_element(pointNKNSquaredDistance.begin(),pointNKNSquaredDistance.end()); 
            
//             if (maxValue > params[1])
//             {
//                 // cout<<"[TEST] >0.5 continue!"<<endl;
//                 continue; // 距离太远，退出
//             }

//             Eigen::Matrix<float, 3, int(params[0])> A;

//             for (size_t j = 0; j < pointIdxNKNSearch.size(); ++j)
//             {
//                 pcl::PointXYZI point = cloud2->at(pointIdxNKNSearch[j]);
//                 A(0, j) = point.x;
//                 A(1, j) = point.y;
//                 A(2, j) = point.z;
//                 pc_vis.push_back(point);
//             }

//             Eigen::Vector4f plane_coeff;
//             if (!plane_fit(A, plane_coeff))
//             {
//                 // cout<<"[TEST] bad fit continue!"<<endl; //平面度不好，退出
//                 continue;
//             }
//             valid_point.push_back(point_world);
//             Eigen::Vector3f centroid = A.rowwise().mean();
//             plane_point_seq.push_back(centroid);
//             plane_seq.push_back(plane_coeff);
//             world_point_seq.push_back(Eigen::Vector3f(point_world.x, point_world.y, point_world.z));
//         }

//         if (world_point_seq.size() != plane_seq.size())
//             cout << "[NEVER reach] seq error continue!" << endl;

//         effct_feat_num = world_point_seq.size();

//         Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> h_x = Eigen::MatrixXf::Zero(effct_feat_num, 1);    // b
//         Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> J_x = Eigen::MatrixXf::Zero(effct_feat_num, 6);    // A
//         Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> H_x = Eigen::MatrixXf::Zero(6, 6);                 // H
//         Eigen::Matrix<float, 6, 1> x_opt1 = Eigen::MatrixXf::Zero(6, 1); //x_opt^1

//         for (int i = 0; i < effct_feat_num; i++)
//         {
//             Eigen::Vector3f norm = plane_seq[i].block<3, 1>(0, 0);
//             Eigen::Vector3f w_point = world_point_seq[i] - transition;
//             Eigen::Vector3f cross_vc = w_point.cross(norm);
//             Eigen::Matrix3f H_tmp = Eigen::MatrixXf::Zero(3, 3);

//             Eigen::Vector3f L_point = _q.inverse()*(world_point_seq[i] - transition);
//             Eigen::Matrix3f L_point_hat;
//             L_point_hat << 0,-L_point(2),L_point(1),L_point(2),0,-L_point(0),-L_point(1),L_point(0),0;
//             Eigen::Matrix3f R_q(_q.matrix());
//             Eigen::Vector3f J_tmp_last3term = L_point_hat*R_q.transpose()*norm;
//             h_x(i) = norm.transpose()*(world_point_seq[i]-plane_point_seq[i]);

//             // right disturb
//             Eigen::Matrix3f RTu_hat = Eigen::MatrixXf::Zero(3, 3);
//             Eigen::Vector3f RTu = R_q.transpose()*norm;
//             J_x.row(i) << norm(0), norm(1), norm(2), J_tmp_last3term(0), J_tmp_last3term(1), J_tmp_last3term(2);
//             RTu_hat << 0,-RTu(2),RTu(1),RTu(2),0,-RTu(0),-RTu(1),RTu(0),0;
//             H_tmp = L_point_hat*RTu_hat;
//             H_x.block(3,3,3,3) = H_x.block(3,3,3,3) + H_tmp.transpose()*H_tmp;
//         }

//         // // assessment
//         Eigen::MatrixXf JTJ = J_x.transpose() * J_x;
//         Eigen::MatrixXf HTH = H_x.transpose() * H_x;

//         // update
//         // cout<<"*********"<<endl;
//         Eigen::Matrix<float,6,1> delta_x = JTJ.ldlt().solve(-J_x.transpose()*h_x);   //[deltat, delta_phi_left]
//         // cout<<"delta_x"<<delta_x.transpose()<<endl;
//         // cout << "cost = " << h_x.norm() << endl;

//         float alpha = 0.5;  // learning rate
//         delta_x = alpha*delta_x;

//         Eigen::Vector3f delta_phi = delta_x.block<3,1>(3,0);
//         Eigen::AngleAxisf delta_phi_left(delta_phi.norm(),delta_phi.normalized());
//         Eigen::Matrix3f delta_R = delta_phi_left.matrix();
//         Eigen::Vector3f delta_t = delta_x.block<3,1>(0,0);

//         // transform
//         transition +=  delta_t;
//         _q =_q* delta_R;
//         _q = _q.normalized();

//         pc_src_world->clear();

//         transform.block<3,3>(0,0) = _q.toRotationMatrix();
//         transform.block<3,1>(0,3) << transition;
//         pcl::transformPointCloud(*cloud1, *pc_src_world, transform);

//         if(iter_conter>params[2])
//         {
//             break;
//         }
//     }
//     return transform;
//     // std::cout << "transform = \n" << transform << std::endl;

// }


Eigen::Matrix4f PointToPlane(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud1,
                             pcl::PointCloud<pcl::PointXYZI>::Ptr cloud2,
                             Eigen::Matrix4f& guess,
                           std::vector<double> &ret,
                           pcl::Correspondences& cor,
                           std::vector<double> params)
{
  pcl::PointCloud<pcl::PointNormal>::Ptr src(new pcl::PointCloud<pcl::PointNormal>);
  pcl::copyPointCloud(*cloud1, *src);
  pcl::PointCloud<pcl::PointNormal>::Ptr tgt(new pcl::PointCloud<pcl::PointNormal>);
  pcl::copyPointCloud(*cloud2, *tgt);

  pcl::NormalEstimation<pcl::PointNormal, pcl::PointNormal> norm_est;
  norm_est.setSearchMethod(pcl::search::KdTree<pcl::PointNormal>::Ptr(new pcl::search::KdTree<pcl::PointNormal>));
  norm_est.setKSearch(int(params[0]));
  norm_est.setInputCloud(tgt);
  norm_est.compute(*tgt);

  pcl::IterativeClosestPoint<pcl::PointNormal, pcl::PointNormal> icp;
  typedef pcl::registration::TransformationEstimationPointToPlane<pcl::PointNormal, pcl::PointNormal> PointToPlane;
  boost::shared_ptr<PointToPlane> point_to_plane(new PointToPlane);
  icp.setTransformationEstimation(point_to_plane);  // key

  // cor_rej_var->setOverlapRatio(params[7]);
  icp.setInputSource(src);
  icp.setInputTarget(tgt);

  icp.setMaxCorrespondenceDistance(params[1]);  // 1500
  icp.setMaximumIterations(params[2]);
  icp.setRANSACIterations(params[3]);  
  // icp.addCorrespondenceRejector(cor_rej_sac);
  // icp.setRANSACOutlierRejectionThreshold(params[4]);

  icp.setTransformationEpsilon(1e-2);

  pcl::PointCloud<pcl::PointNormal> output;
  icp.align(output,guess); 

  ret.clear();
  ret.push_back(icp.hasConverged());
  ret.push_back(icp.getFitnessScore());

  cor = *(icp.correspondences_);
  // cout << "cor: " << icp.correspondences_->size() << endl;    
  // cout << "rej: " << icp.getCorrespondenceRejectors().size() << endl; 
  // cout << "converge: " << icp.hasConverged() << endl;

  return icp.getFinalTransformation();
}

Eigen::Matrix4f PointToPlaneWithNormal(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud1,
                             pcl::PointCloud<pcl::PointNormal>::Ptr cloud2,
                             Eigen::Matrix4f guess,
                             std::vector<double> &ret,
                             pcl::Correspondences& cor,
                             std::vector<double> params)
{
  pcl::PointCloud<pcl::PointNormal>::Ptr src(new pcl::PointCloud<pcl::PointNormal>);
  pcl::copyPointCloud(*cloud1, *src);
  pcl::PointCloud<pcl::PointNormal>::Ptr tgt(new pcl::PointCloud<pcl::PointNormal>);
  pcl::copyPointCloud(*cloud2, *tgt);

  // pcl::NormalEstimation<pcl::PointNormal, pcl::PointNormal> norm_est;
  // norm_est.setSearchMethod(pcl::search::KdTree<pcl::PointNormal>::Ptr(new pcl::search::KdTree<pcl::PointNormal>));
  // norm_est.setKSearch(int(params[0]));
  // norm_est.setInputCloud(tgt);
  // norm_est.compute(*tgt);

  pcl::IterativeClosestPoint<pcl::PointNormal, pcl::PointNormal> icp;
  typedef pcl::registration::TransformationEstimationPointToPlaneLLS<pcl::PointNormal, pcl::PointNormal> PointToPlane;
  boost::shared_ptr<PointToPlane> point_to_plane(new PointToPlane);
  icp.setTransformationEstimation(point_to_plane);  // key

  icp.setInputSource(src);
  icp.setInputTarget(tgt);

  icp.setMaxCorrespondenceDistance(params[1]);  // 1500
  icp.setMaximumIterations(params[2]);
  icp.setRANSACIterations(params[3]);  

  // icp.addCorrespondenceRejector (cor_rej_o2o);
  
  icp.setTransformationEpsilon(1e-2);

  pcl::PointCloud<pcl::PointNormal> output;
  icp.align(output,guess); 

  ret.clear();
  ret.push_back(icp.hasConverged());
  ret.push_back(icp.getFitnessScore());

  cor = *(icp.correspondences_);

  return icp.getFinalTransformation();
}


Eigen::Matrix4f PointToPoint(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud1,
                             pcl::PointCloud<pcl::PointXYZI>::Ptr cloud2,
                             Eigen::Matrix4f& guess,
                               std::vector<double> &ret,
                               pcl::Correspondences& cor,
                               std::vector<double> params)
{
  // ICP
  pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;
  // pcl::IterativeClosestPointWithNormals<pcl::PointXYZI, pcl::PointXYZI> icp;
  pcl::search::KdTree<pcl::PointXYZI>::Ptr tree1(new pcl::search::KdTree<pcl::PointXYZI>);
  tree1->setInputCloud(cloud1);
  pcl::search::KdTree<pcl::PointXYZI>::Ptr tree2(new pcl::search::KdTree<pcl::PointXYZI>);
  tree2->setInputCloud(cloud2);
  icp.setSearchMethodSource(tree1);
  icp.setSearchMethodTarget(tree2);
  icp.setInputSource(cloud1);
  icp.setInputTarget(cloud2);

  icp.setMaxCorrespondenceDistance(params[1]);  // 1500
  icp.setMaximumIterations(params[2]);
  icp.setRANSACIterations(params[3]);  

  icp.setTransformationEpsilon(1e-2);
  icp.setEuclideanFitnessEpsilon(1e-2);  // 1e-2

  pcl::PointCloud<pcl::PointXYZI> output;
  icp.align(output,guess);
  ret.clear();
  ret.push_back(icp.hasConverged());
  ret.push_back(icp.getFitnessScore());  

  cor = *(icp.correspondences_);  
  return icp.getFinalTransformation();
}

Eigen::Matrix4f icpPlaneToPlane(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud1, 
                                pcl::PointCloud<pcl::PointXYZI>::Ptr cloud2, 
                                Eigen::Matrix4f& guess,
                               std::vector<double> &ret,
                               pcl::Correspondences& cor,
                               std::vector<double> params)
{
  pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;  // GICP 泛化的ICP，或者叫Plane to Plane
  icp.setInputTarget(cloud2);
  icp.setInputSource(cloud1);

  
  icp.setMaxCorrespondenceDistance(params[1]);  // 1500
  icp.setMaximumIterations(params[2]);
  icp.setRANSACIterations(params[3]);  

  icp.setTransformationEpsilon(1e-2);
  

  pcl::PointCloud<pcl::PointXYZI> unused_result;
  icp.align(unused_result, guess);

  cor = *(icp.correspondences_);

  ret.clear();
  ret.push_back(icp.hasConverged());
  ret.push_back( icp.getFitnessScore());  
  
  return icp.getFinalTransformation();
}

Eigen::Matrix4f icpPointToPlane(PointCloud<pcl::PointXYZI>::Ptr cloud1, 
                                PointCloud<pcl::PointXYZI>::Ptr cloud2, 
                                Eigen::Matrix4f& guess,
                                std::vector<double> &ret,
                                pcl::Correspondences& cor,
                                std::vector<double> params)
{
  pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_source_normals(new pcl::PointCloud<pcl::PointXYZINormal>());
  pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_target_normals(new pcl::PointCloud<pcl::PointXYZINormal>());
  pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_source_trans_normals(new pcl::PointCloud<pcl::PointXYZINormal>());

  addNormal(cloud2, cloud_target_normals, int(params[0]));
  addNormal(cloud1, cloud_source_normals, int(params[0]));

  pcl::IterativeClosestPointWithNormals<pcl::PointXYZINormal, pcl::PointXYZINormal> icp;
  icp.setMaxCorrespondenceDistance(params[1]);  // 1500
  icp.setMaximumIterations(params[2]);
  icp.setRANSACIterations(params[3]);  

  icp.setTransformationEpsilon(1e-2);

  icp.setInputSource(cloud_source_normals);  //
  icp.setInputTarget(cloud_target_normals);
  icp.align(*cloud_source_trans_normals, guess);  //

  cor = *(icp.correspondences_);

  ret.clear();
  ret.push_back(icp.hasConverged());
  ret.push_back( icp.getFitnessScore());  
  return icp.getFinalTransformation();
}

Eigen::Matrix4f pclNDT(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud1,
                       pcl::PointCloud<pcl::PointXYZI>::Ptr cloud2,
                       Eigen::Matrix4f& guess,
                       std::vector<double> &ret,
                       pcl::Correspondences& cor,
                       std::vector<double> params)
{
  // NDT
  pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> ndt;

  ndt.setInputSource(cloud1);
  ndt.setInputTarget(cloud2);
  ndt.setMaximumIterations (params[2]);
  ndt.setStepSize(params[5]);              
  ndt.setResolution(params[4]);            
  ndt.setTransformationEpsilon (1e-3);
  ndt.setEuclideanFitnessEpsilon(1e-3);  // 1e-2

  pcl::PointCloud<pcl::PointXYZI> output;
  ndt.align(output,guess);
  ret.clear();
  ret.push_back(ndt.hasConverged());
  ret.push_back(ndt.getFitnessScore());  

  // cor = *(ndt.correspondences_);  
  return ndt.getFinalTransformation();
} 

// Eigen::Matrix4f fastGICPST(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud1,
//                        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud2,
//                        Eigen::Matrix4f& guess,
//                        std::vector<double> &ret,
//                        pcl::Correspondences& cor,
//                        std::vector<double> params)
// {
//   fast_gicp::FastGICPSingleThread<pcl::PointXYZI, pcl::PointXYZI> icp;

//   icp.setInputSource(cloud1);
//   icp.setInputTarget(cloud2);

//   icp.setMaxCorrespondenceDistance(params[1]);  // 1500
//   icp.setMaximumIterations(params[2]);
//   icp.setRANSACIterations(params[3]);  

//   icp.setTransformationEpsilon(1e-2);

//   pcl::PointCloud<pcl::PointXYZI> output;
//   icp.align(output,guess); 

//   ret.clear();
//   ret.push_back(icp.hasConverged());
//   ret.push_back(icp.getFitnessScore());

//   return icp.getFinalTransformation();
// }

// Eigen::Matrix4f fastGICPMT(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud1,
//                        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud2,
//                        Eigen::Matrix4f& guess,
//                        std::vector<double> &ret,
//                        pcl::Correspondences& cor,
//                        std::vector<double> params)
// {
//   fast_gicp::FastGICP<pcl::PointXYZI, pcl::PointXYZI> icp;

//   icp.setInputSource(cloud1);
//   icp.setInputTarget(cloud2);

//   icp.setMaxCorrespondenceDistance(params[1]);  // 1500
//   icp.setMaximumIterations(params[2]);
//   icp.setRANSACIterations(params[3]);  

//   icp.setTransformationEpsilon(1e-2);

//   pcl::PointCloud<pcl::PointXYZI> output;
//   icp.align(output,guess); 

//   ret.clear();
//   ret.push_back(icp.hasConverged());
//   ret.push_back(icp.getFitnessScore());

//   return icp.getFinalTransformation();
// }


}