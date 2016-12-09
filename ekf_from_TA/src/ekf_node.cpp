#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <fstream>
#include "ekf.h"

EKF ekf;
ros::Publisher odom_pub;
ros::Publisher path_ekf;
ros::Publisher path_ref;
ros::Publisher reference_pub;
nav_msgs::Path pathE;
nav_msgs::Path pathR;

void imu_callback(const sensor_msgs::Imu::ConstPtr &msg)
{
    //propagation
    ros::Time Time_imu = msg->header.stamp;
    VectorXd u = VectorXd::Zero(6);

    u(0)   =  msg->angular_velocity.x;
    u(1)   =  msg->angular_velocity.y;
    u(2)   =  msg->angular_velocity.z;
    u(3)   =  msg->linear_acceleration.x;
    u(4)   =  msg->linear_acceleration.y;
    u(5)   =  msg->linear_acceleration.z;

    if(ekf.isInit() == false)
        return;

    ekf.ImuPropagation(u, Time_imu);
    VectorXd mean = ekf.GetState();
    Vector3d X_rpy;
    X_rpy(0) = mean(3);
    X_rpy(1) = mean(4);
    X_rpy(2) = mean(5);
    Quaterniond X_quat;
    X_quat = rpy_to_R(X_rpy);

    nav_msgs::Odometry Pose_ekf;
    Pose_ekf.pose.pose.position.x = mean(0);
    Pose_ekf.pose.pose.position.y = mean(1);
    Pose_ekf.pose.pose.position.z = mean(2);
    Pose_ekf.header.frame_id = string("/my_frame");
    Pose_ekf.header.stamp = Time_imu;
    Pose_ekf.pose.pose.orientation.w = X_quat.w();
    Pose_ekf.pose.pose.orientation.x = X_quat.x();
    Pose_ekf.pose.pose.orientation.y = X_quat.y();
    Pose_ekf.pose.pose.orientation.z = X_quat.z();
    Pose_ekf.twist.twist.linear.x = mean(6);
    Pose_ekf.twist.twist.linear.y = mean(7);
    Pose_ekf.twist.twist.linear.z = mean(8);
    odom_pub.publish(Pose_ekf);

    pathE.header.frame_id = string("/my_frame");
    pathE.header.stamp = Time_imu;
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = mean(0);
    pose.pose.position.y = mean(1);
    pose.pose.position.z = mean(2);
    pose.pose.orientation.w = X_quat.w();
    pose.pose.orientation.x = X_quat.x();
    pose.pose.orientation.y = X_quat.y();
    pose.pose.orientation.z = X_quat.z();
    pose.header = pathE.header;
    pathE.poses.push_back(pose);
    path_ekf.publish(pathE);
}

Matrix3d Rcam = Quaterniond(0, 0, 1, 0).toRotationMatrix();

void odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    //return;
    //update
    //camera position in the IMU frame = (0, -0.05, +0.02)
    //camera orientaion in the IMU frame = Quaternion(0, 0, -1, 0); w x y z, respectively

    Quaterniond quater;
    Vector3d Tcw, Twi, Tic, rpy_wi;
    Matrix3d Riw, Rwi, Rcw;
    quater.w() = msg->pose.pose.orientation.w; 
    quater.x() = msg->pose.pose.orientation.x; 
    quater.y() = msg->pose.pose.orientation.y; 
    quater.z() = msg->pose.pose.orientation.z;
    Tcw(0)= msg->pose.pose.position.x; //Tcw
    Tcw(1)= msg->pose.pose.position.y; 
    Tcw(2)= msg->pose.pose.position.z;
    ros::Time Time_update = msg->header.stamp;
    VectorXd Z = VectorXd::Zero(6);

    Tic << 0,
          -0.04,
          -0.02;

    Rcw = quater.toRotationMatrix();
    Rwi = Rcw.transpose() * Rcam.transpose(); 
    Twi = -Rcw.transpose() * (Rcam.transpose() * Tic + Tcw);
    rpy_wi = R_to_rpy(Rwi);

    Z << Twi,
         rpy_wi;
    
    if(ekf.isInit() == false){   
        ekf.SetInit(Z, Time_update);
        return;
    }


    VectorXd mean = ekf.GetState();
    nav_msgs::Odometry reference;
    Eigen::Quaterniond Q_ref(Rwi);
    reference.header.stamp = Time_update;
    reference.header.frame_id = "/my_frame";
    reference.pose.pose.position.x = Twi(0);
    reference.pose.pose.position.y = Twi(1);
    reference.pose.pose.position.z = Twi(2);
    reference.pose.pose.orientation.w = Q_ref.w();
    reference.pose.pose.orientation.x = Q_ref.x();
    reference.pose.pose.orientation.y = Q_ref.y();
    reference.pose.pose.orientation.z = Q_ref.z();
    reference_pub.publish(reference);

    pathR.header.frame_id = string("/my_frame");
    pathR.header.stamp = Time_update;
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = Twi(0);
    pose.pose.position.y = Twi(1);
    pose.pose.position.z = Twi(2);
    pose.pose.orientation.w = Q_ref.w();
    pose.pose.orientation.x = Q_ref.x();
    pose.pose.orientation.y = Q_ref.y();
    pose.pose.orientation.z = Q_ref.z();
    pose.header = pathR.header;
    pathR.poses.push_back(pose);
    path_ref.publish(pathR);

    ekf.OdomUpdate(Z, Time_update);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ekf");
    ros::NodeHandle n("~");
    double var_g = 0.01, var_a = 0.01, var_bg = 0.01, var_ba = 0.01, var_p = 0.2, var_q = 0.2;
    ekf.SetParam(var_g, var_a, var_bg, var_ba, var_p, var_q);
    //Rcam = Eigen::Quaterniond(0, 0, 1, 0).toRotationMatrix();
    ros::Subscriber s1 = n.subscribe("imu", 100, imu_callback);
    ros::Subscriber s2 = n.subscribe("tag_odom", 100, odom_callback);
    odom_pub = n.advertise<nav_msgs::Odometry>("ekf_odom", 100);
    reference_pub = n.advertise<nav_msgs::Odometry>("ref_odom", 100);
    path_ekf = n.advertise<nav_msgs::Path>("path_ekf",100);
    path_ref = n.advertise<nav_msgs::Path>("path_ref",100);
    ros::spin();
}
