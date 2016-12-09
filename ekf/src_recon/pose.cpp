#include "pose.h"

Eigen::Vector3d R_to_rpy(const Eigen::Matrix3d& R)
{
  Eigen::Vector3d rpy;
  double r = asin(R(2,1));
  double p = atan2(-R(2,0)/cos(r), R(2,2)/cos(r));
  double y = atan2(-R(0,1)/cos(r), R(1,1)/cos(r));
  rpy(0) = r;
  rpy(1) = p;
  rpy(2) = y;

  return rpy;
}

Eigen::Matrix3d rpy_to_R(const Eigen::Vector3d& rpy)
{
      Eigen::MatrixXd R = Eigen::MatrixXd::Zero(3,3);
      double r = rpy(0), p = rpy(1), y = rpy(2);
      R(0,0) =  cos(y)*cos(p) - sin(r)*sin(p)*sin(y);
      R(0,1) = -cos(r)*sin(y);
      R(0,2) =  cos(y)*sin(p) + cos(p)*sin(r)*sin(y);
      R(1,0) =  cos(p)*sin(y) + cos(y)*sin(r)*sin(p);
      R(1,1) =  cos(r)*cos(y);
      R(1,2) =  sin(y)*sin(p) - cos(y)*cos(p)*sin(r);
      R(2,0) = -cos(r)*sin(p);  
      R(2,1) =  sin(r);
      R(2,2) =  cos(r)*cos(p); 

      return R;
}

