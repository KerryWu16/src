#ifndef POSE_H
#define POSE_H

#include <Eigen/Eigen>

Eigen::Vector3d R_to_rpy(const Eigen::Matrix3d& R);
Eigen::Matrix3d rpy_to_R(const Eigen::Vector3d& rpy);

#endif
