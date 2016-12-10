#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <fstream>
#include "ekf.h"

using namespace std;

EKF ekf;

ros::Publisher odom_pub;
// ros::Publisher reference_pub;

void imu_callback(const sensor_msgs::Imu::ConstPtr &msg)
{
	ros::Time time_imu = msg->header.stamp;

	// store input u: acceleration, angular_velocity
	VectorXd u = VectorXd::Zero(6);
	u(0) = msg->linear_acceleration.x;
	u(1) = msg->linear_acceleration.y;
	u(2) = msg->linear_acceleration.z;
	u(3) = msg->angular_velocity.x;
	u(4) = msg->angular_velocity.y;
	u(5) = msg->angular_velocity.z;

	/* 	 Shift all the heavy lifting in Eigen to EKF class			*/
	if (!ekf.isInit()) {
		return;
	}
	ekf.IMU_Propagation(u, time_imu);
	VectorXd mean_ns = ekf.Get_State();

	/* 															*/
	/*		Update the odometry everytime IMU comes 			*/
	/* 		Instead of in camera callback						*/
	/* 															*/
	Vector3d X_rpy;
	X_rpy(0) = mean_ns(3);
	X_rpy(1) = mean_ns(4);
	X_rpy(2) = mean_ns(5);
	Quaterniond Q_output;
	Q_output = rpy_to_R(X_rpy);

    nav_msgs::Odometry ekf_odom;
	ekf_odom.header.seq = msg->header.seq;
	ekf_odom.header.stamp = time_imu;
	ekf_odom.header.frame_id = "world";
	ekf_odom.pose.pose.position.x = mean_ns(0);
	ekf_odom.pose.pose.position.y = mean_ns(1);
	ekf_odom.pose.pose.position.z = mean_ns(2);
	ekf_odom.pose.pose.orientation.w = Q_output.w();
	ekf_odom.pose.pose.orientation.x = Q_output.x();
	ekf_odom.pose.pose.orientation.y = Q_output.y();
	ekf_odom.pose.pose.orientation.z = Q_output.z();
	ekf_odom.twist.twist.linear.x = mean_ns(6);
	ekf_odom.twist.twist.linear.y = mean_ns(7);
	ekf_odom.twist.twist.linear.z = mean_ns(8);

    odom_pub.publish(ekf_odom);
}

void odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
	ros::Time time_update = msg->header.stamp;

	// store observation model position and orientation
	VectorXd zt = VectorXd::Zero(6);
	/*	Frame transformation from camera frame to IMU frame 				*/
	/* 	1. Get the world frame in camera frame transformation from the msg 	*/
	/* 	2. Record the camera frame in the IMU frame from TA 				*/
	Matrix3d R_cw, R_ic, R_wi;
	Vector3d T_cw, T_ic, T_wi, rpy_wi;

	geometry_msgs::Quaternion q_cw = msg->pose.pose.orientation;

	R_cw = Quaterniond(q_cw.w, q_cw.x, q_cw.y, q_cw.z).toRotationMatrix();
	T_cw << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;
	R_ic = Quaterniond(0, 0, 1, 0).toRotationMatrix();
	T_ic = Vector3d(0, -0.04, -0.02);
	R_wi =  R_cw.transpose() *  R_ic.transpose();
	T_wi = -R_cw.transpose() * (R_ic.transpose() * T_ic + T_cw);
	rpy_wi = R_to_rpy(R_wi);

	zt << 	T_wi,
			rpy_wi;

	/* 																	*/
	/*		Process the odometry everytime camera signal comes 			*/
	/* 		And re-propogate the IMU data if the camera is delayed		*/
	/* 																	*/
	if (!ekf.isInit()) {
		ekf.SetInit(zt, time_update);
		return;
	}

	ekf.Odom_Update(zt, time_update);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ekf");
    ros::NodeHandle n("~");

	double var_g = 0.01, var_a = 0.01, var_bg = 0.01, var_ba = 0.01;
	double var_p = 0.5, var_q = 0.5;
	ekf.SetParam(var_g, var_a, var_bg, var_ba, var_p, var_q);

	ros::Subscriber s1 = n.subscribe("imu", 100, imu_callback);
	ros::Subscriber s2 = n.subscribe("tag_odom", 100, odom_callback);
	odom_pub = n.advertise<nav_msgs::Odometry>("ekf_odom", 100);
	// reference_pub = n.advertise<nav_msgs::Odometry>("ref_odom", 100);

	ros::spin();
	return 0;
}
