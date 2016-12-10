#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Eigen>
#include "pose.h"

using namespace std;
using namespace Eigen;
ros::Publisher odom_pub;
MatrixXd Q = MatrixXd::Identity(12, 12);
MatrixXd Rt = MatrixXd::Identity(6,6);
MatrixXd x = MatrixXd::Zero(15,1);
MatrixXd cov = MatrixXd::Identity(15,15);
Vector3d pos_la;

double cur_t = 0.0;
bool cam_ready = 0;

void imu_callback(const sensor_msgs::Imu::ConstPtr &msg)
{
    //your code for propagation
    double dT = msg->header.stamp.toSec()-cur_t;
    if(dT>1||!cam_ready)
    	dT = 0;
	//cout<<"dT: "<<dT<<endl;
	MatrixXd At = MatrixXd::Zero(15,15);
	MatrixXd Ut = MatrixXd::Zero(15,12);
	MatrixXd Ft = MatrixXd::Zero(15,15);
	MatrixXd Vt = MatrixXd::Zero(15,12);
	MatrixXd f = MatrixXd::Zero(15,1);
	Vector3d ang_v(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
	Vector3d lin_a(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
	Vector3d x4(x(9,0), x(10,0), x(11,0));
	Vector3d x5(x(12,0), x(13,0), x(14,0));
    Matrix3d G_inv_dot, R_dot, G, R;
    Vector3d g(0, 0, 9.8);

	At.block<3,3>(0,6) = Matrix3d::Identity(3,3);
	
    G_inv_dot<<
		0, ang_v(2)*cos(x(4,0)) - ang_v(0)*sin(x(4,0)), 0,
    	ang_v(0)*sin(x(4,0)) - ang_v(2)*cos(x(4,0)) - (ang_v(2)*cos(x(4,0))*sin(x(3,0))*sin(x(3,0)))/(cos(x(3,0))*cos(x(3,0))) + (ang_v(0)*sin(x(3,0))*sin(x(3,0))*sin(x(4,0)))/(cos(x(3,0))*cos(x(3,0))), (ang_v(0)*cos(x(4,0))*sin(x(3,0)))/cos(x(3,0)) + (ang_v(2)*sin(x(3,0))*sin(x(4,0)))/cos(x(3,0)), 0,
    	(ang_v(2)*cos(x(4,0))*sin(x(3,0)))/(cos(x(3,0))*cos(x(3,0))) - (ang_v(0)*sin(x(3,0))*sin(x(4,0)))/(cos(x(3,0))*cos(x(3,0))), -(ang_v(0)*cos(x(4,0)))/cos(x(3,0)) - (ang_v(2)*sin(x(4,0)))/cos(x(3,0)), 0;
    At.block<3,3>(3,3) = G_inv_dot;
    
    R_dot<<
        sin(x(5,0))*((lin_a(1,0)-x(13,0))*sin(x(3,0)) + (lin_a(2,0)-x(14,0))*cos(x(3,0))*cos(x(4,0)) - (lin_a(0,0)-x(12,0))*cos(x(3,0))*sin(x(4,0))), (lin_a(2,0)-x(14,0))*(cos(x(5,0))*cos(x(4,0)) - sin(x(3,0))*sin(x(5,0))*sin(x(4,0))) - (lin_a(0,0)-x(12,0))*(cos(x(5,0))*sin(x(4,0)) + cos(x(4,0))*sin(x(3,0))*sin(x(5,0))),  -(lin_a(0,0)-x(12,0))*(cos(x(4,0))*sin(x(5,0)) + cos(x(5,0))*sin(x(3,0))*sin(x(4,0))) - (lin_a(2,0)-x(14,0))*(sin(x(5,0))*sin(x(4,0)) - cos(x(5,0))*cos(x(4,0))*sin(x(3,0))) - (lin_a(1,0)-x(13,0))*cos(x(3,0))*cos(x(5,0)),
        -cos(x(5,0))*((lin_a(1,0)-x(13,0))*sin(x(3,0)) + (lin_a(2,0)-x(14,0))*cos(x(3,0))*cos(x(4,0)) - (lin_a(0,0)-x(12,0))*cos(x(3,0))*sin(x(4,0))), (lin_a(2,0)-x(14,0))*(cos(x(4,0))*sin(x(5,0)) + cos(x(5,0))*sin(x(3,0))*sin(x(4,0))) - (lin_a(0,0)-x(12,0))*(sin(x(5,0))*sin(x(4,0)) - cos(x(5,0))*cos(x(4,0))*sin(x(3,0))),   (lin_a(0,0)-x(12,0))*(cos(x(5,0))*cos(x(4,0)) - sin(x(3,0))*sin(x(5,0))*sin(x(4,0))) + (lin_a(2,0)-x(14,0))*(cos(x(5,0))*sin(x(4,0)) + cos(x(4,0))*sin(x(3,0))*sin(x(5,0))) - (lin_a(1,0)-x(13,0))*cos(x(3,0))*sin(x(5,0)),
        (lin_a(1,0)-x(13,0))*cos(x(3,0)) - (lin_a(2,0)-x(14,0))*cos(x(4,0))*sin(x(3,0)) + (lin_a(0,0)-x(12,0))*sin(x(3,0))*sin(x(4,0)),           -cos(x(3,0))*((lin_a(0,0)-x(12,0))*cos(x(4,0)) + (lin_a(2,0)-x(14,0))*sin(x(4,0))),                                                                            0;
    At.block<3,3>(6,3) = R_dot;
	G<<
		cos(x(4,0)), 0, -cos(x(3,0))*sin(x(4,0)),
        0, 1, sin(x(3,0)),
        sin(x(4,0)), 0, cos(x(3,0))*cos(x(4,0));
    At.block<3,3>(3,9) = -G.inverse();
    
    R<<
    	cos(x(4,0))*cos(x(5,0)) - sin(x(3,0))*sin(x(4,0))*sin(x(5,0)), -cos(x(3,0))*sin(x(5,0)), cos(x(5,0))*sin(x(4,0)) + cos(x(4,0))*sin(x(3,0))*sin(x(5,0)),
		cos(x(4,0))*sin(x(5,0)) + cos(x(5,0))*sin(x(3,0))*sin(x(4,0)),  cos(x(3,0))*cos(x(5,0)), sin(x(4,0))*sin(x(5,0)) - cos(x(4,0))*cos(x(5,0))*sin(x(3,0)),
	   -cos(x(3,0))*sin(x(4,0)), sin(x(3,0)), cos(x(3,0))*cos(x(4,0));
	At.block<3,3>(6,12) = -R;

    Ut.block<3,3>(3,0) = -G.inverse();
    Ut.block<3,3>(6,3) = -R;
    Ut.block<6,6>(9,6) = MatrixXd::Identity(6,6);

    Ft = MatrixXd::Identity(15,15) + dT*At;
    Vt = dT*Ut;
    cov = Ft*cov*Ft.transpose()+Vt*Q*Vt.transpose();

	f.block<3,1>(0,0) = x.block<3,1>(6,0);
	f.block<3,1>(3,0) = G.inverse()*(ang_v-x4);
	f.block<3,1>(6,0) = g + R*(lin_a-x5);
	x += f*dT;

	cur_t = msg->header.stamp.toSec();


    Matrix3d Rmw(3,3);
    Rmw<<
        0, 1, 0,
        1, 0, 0,
        0, 0, -1;
    Vector3d Xmw(x(0,0), x(1,0), x(2,0));
    Vector3d Vmw;
    Xmw=Rmw*Xmw;
    double x2x=x(3,0);
    double x2y=x(4,0);
    double x2z=x(5,0);
    Matrix3d Ro(3,3);
    Ro<<
        cos(x2y)*cos(x2z) - sin(x2x)*sin(x2y)*sin(x2z), -cos(x2x)*sin(x2z), cos(x2z)*sin(x2y) + cos(x2y)*sin(x2x)*sin(x2z),
        cos(x2y)*sin(x2z) + cos(x2z)*sin(x2x)*sin(x2y),  cos(x2x)*cos(x2z), sin(x2y)*sin(x2z) - cos(x2y)*cos(x2z)*sin(x2x),
        -cos(x2x)*sin(x2y),                              sin(x2x),                              cos(x2x)*cos(x2y);
    Ro=Rmw*Ro;

    Quaterniond Qu(Ro);
    
    Vmw = Xmw - pos_la;
    pos_la = Xmw;
    
    nav_msgs::Odometry odom_me;
    odom_me.header.stamp = msg->header.stamp;
    odom_me.header.frame_id = "world";
    odom_me.pose.pose.position.x = Xmw(0,0);
    odom_me.pose.pose.position.y = Xmw(1,0);
    odom_me.pose.pose.position.z = Xmw(2,0);
    odom_me.twist.twist.linear.x = Vmw(0,0);
    odom_me.twist.twist.linear.y = Vmw(1,0);
    odom_me.twist.twist.linear.z = Vmw(2,0);
    //cout<<"twist"<<endl;
    //cout<<Vmw<<endl;
    odom_me.pose.pose.orientation.w = Qu.w();
    odom_me.pose.pose.orientation.x = Qu.x();
    odom_me.pose.pose.orientation.y = Qu.y();
    odom_me.pose.pose.orientation.z = Qu.z();
    odom_pub.publish(odom_me);

    
}

//Rotation from the camera frame to the IMU frame
Eigen::Matrix3d Rcam;
void odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
{

       VectorXd Zt = VectorXd::Zero(6);
/********************************************get Zt********************************************************/
    geometry_msgs::Pose camera_pose;
    camera_pose = msg->pose.pose;

    geometry_msgs::Quaternion q_wi = camera_pose.orientation;

    MatrixXd CamZt(6,1);//camera position in world frame. What we need is the IMU orientation in world frame.
    CamZt(0) = camera_pose.position.x;
    CamZt(1) = camera_pose.position.y;
    CamZt(2) = camera_pose.position.z;
    
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
    

    Tic << 0,//from camera to IMU = camera in IMU
          -0.04,
          -0.02;
    //Rcam from camera to IMU = iRc
    Rcw = quater.toRotationMatrix();//from world to camera = world in camera
    Rwi = Rcw.transpose() * Rcam.transpose();//Rwi = wRi = wRc *  cRi
    Twi = -Rcw.transpose() * (Rcam.transpose() * Tic + Tcw);
    rpy_wi = R_to_rpy(Rwi);
  

    Zt << Twi,
         rpy_wi;
    MatrixXd zta(6,1);
    zta<<Zt(0,0)-x(0,0), Zt(1,0)-x(1,0), Zt(2,0)-x(2,0), Zt(3,0)-x(3,0), Zt(4,0)-x(4,0), Zt(5,0)-x(5,0);

    if(zta(3,0)>3.14)
    	zta(3,0)=zta(3,0)-2*3.14;
    else if(zta(3,0)<-3.14)
    	zta(3,0)=zta(3,0)+2*3.14;

    if(zta(4,0)>3.14)
    	zta(4,0)=zta(4,0)-2*3.14;
    else if(zta(4,0)<-3.14)
    	zta(4,0)=zta(4,0)+2*3.14;

    if(zta(5,0)>3.14)
    	zta(5,0)=zta(5,0)-2*3.14;
    else if(zta(5,0)<-3.14)
    	zta(5,0)=zta(5,0)+2*3.14;



    MatrixXd Ct(6,15);
	Ct <<      1  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,
	           0  ,  1  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,
	           0  ,  0  ,  1  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,
		   0  ,  0  ,  0  ,  1  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,
		   0  ,  0  ,  0  ,  0  ,  1  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,
		   0  ,  0  ,  0  ,  0  ,  0  ,  1  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ;

	
    MatrixXd Wt = MatrixXd::Identity(6,6);


    MatrixXd Kt(15,6);
    Kt= cov *Ct.transpose()*(Ct* cov *Ct.transpose()+Wt*Rt*Wt.transpose()).inverse();
       
    x += Kt*zta; // update the pose
    cov -= Kt*Ct*cov; // update covariance	

    cam_ready = 1;
    
    
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ekf");
    ros::NodeHandle n("~");
    ros::Subscriber s1 = n.subscribe("imu", 10, imu_callback);
    ros::Subscriber s2 = n.subscribe("tag_odom", 10, odom_callback);
    odom_pub = n.advertise<nav_msgs::Odometry>("ekf_odom", 10);
    Rcam = Quaterniond(0, 0, -1, 0).toRotationMatrix();

    // Q imu covariance matrix; Rt visual odomtry covariance matrix
    Q.topLeftCorner(6, 6) = 0.01 * Q.topLeftCorner(6, 6);   
    Q.bottomRightCorner(6, 6) = 0.01 * Q.bottomRightCorner(6, 6); 
    Rt.topLeftCorner(3, 3) = 0.1 * Rt.topLeftCorner(3, 3);  
    Rt.bottomRightCorner(3, 3) = 0.1 * Rt.bottomRightCorner(3, 3); 
    Rt.bottomRightCorner(1, 1) = 0.02 * Rt.bottomRightCorner(1, 1); 

    ros::spin();
}
