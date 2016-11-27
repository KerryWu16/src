#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <ros/time.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <math.h>

#define DEBUG true

using namespace std;
using namespace Eigen;
ros::Publisher odom_pub;
MatrixXd Q = MatrixXd::Identity(12, 12); // For propogate, IMU noise
MatrixXd Rt = MatrixXd::Identity(6,6);   // For update, Odometry noise


// ros::Time current_time, last_time;
//ros::Time current_time = ros::Time::now();
//ros::Time last_time = ros::Time::now();
    //ROS_INFO("write your code here" ); 
/*  Mean and covariance matrixs
    ps: present state
    ba: state propogated
    ns: next state
    [x1] x(0)  x(1)  x(2) = position, x y z
    [x2] x(3)  x(4)  x(5) = orientation, roll pitch yaw, ZXY Euler
    [x3] x(6)  x(7)  x(8) = linear velocity, x y z
    [x4] x(9)  x(10) x(11)= gyroscope bias
    [x5] x(12) x(13) x(14)= accelerator bias
*/
VectorXd x     = VectorXd::Identity(15, 1);
VectorXd x_dot = VectorXd::Identity(15, 1);
VectorXd ut    = VectorXd::Identity(15, 1);//get 
VectorXd ut_1  = VectorXd::Identity(15, 1);
VectorXd u_t   = VectorXd::Identity(15, 1);
VectorXd fuu0  = VectorXd::Identity(15, 1);
VectorXd z     = VectorXd::Identity(15, 1);
VectorXd gu0   = VectorXd::Identity(15, 1);
/*MatrixXd cov_ps = MatrixXd::Identity(15, 15);
MatrixXd cov_ba = MatrixXd::Identity(15, 15);
MatrixXd cov_ns = MatrixXd::Identity(15, 15);*/

// Initially use a constant, later need to read from the environment
double g = 9.81;

/*  Process model, since IMU is an internal measurement
    the gyro bias and accelerator bias is part to the state
    From the process model, IMU measurement - bias = real state
    Nov. 16th, 2016 note:
    1. Everything IMU has an input, propogate once;
    2. Getting g ~ 9.8 m.s^-2 is tricky
    3. Use MATLAB to from the A, B, and U
    Optimal:
    4. The best way to handle timestamp difference is to store the propogated
       and origin data, and trace back to the most recent propogated value
       and repropogate after the odometry reading comes
cd ../src && cp ekf_node_chongzi.cpp ekf_node.cpp && cd ../../../ && catkin_make && cd src/ekf/launch && roslaunch 25000709.launch  

*/
void imu_callback(const sensor_msgs::Imu::ConstPtr &msg)
{

    
    #ifdef DEBUG
    // ROS_INFO("Imu Seq: [%d]", msg->header.seq);
    // ROS_INFO("Imu Orientation x: [%f], y: [%f], z: [%f], w: [%f]", \
    // msg->orientation.x,msg->orientation.y,msg->orientation.z,msg->orientation.w);
    #endif
    // noise n: linear_acceleration_covariance, angular_velocity_covariance,
    //          acc bias noise, gyro bias noise
    //          assume the covariance is diagonalized and x, y, z are independent
    //          Given by the TA
    //Vector3d p = 
    //double na[3]  = {Q(0, 0), Q(1, 1), Q(2, 2)};
    //double ng[3]  = {Q(3, 3), Q(4, 4), Q(5, 5)};
    //double nba[3] = {Q(6, 6), Q(7, 7), Q(8, 8)};
    //double nbg[3] = {Q(9, 9), Q(10,10), Q(11,11)};

    
    //Ft = MatrixXd::Identity(15, 15) + dt * At;
    //Vt = dt * Ut;

    // Prediction Step

}

//Rotation from the camera frame to the IMU frame
Eigen::Matrix3d Rcam;
void odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    //your code for update
    //camera position in the IMU frame = (0, -0.04, -0.02)
    //camera orientaion in the IMU frame = Quaternion(0, 0, 1, 0); w x y z, respectively
    //			   RotationMatrix << -1, 0, 0,
    //							      0, 1, 0,
    //                                0, 0, -1;

    
    

    /* Update, with C and W matrix
        Linear for this case, use Kalman Filter
    */
    MatrixXd Kt = MatrixXd::Identity(15, 15); // Kalman
    MatrixXd Ct = MatrixXd::Identity(6, 15);
    MatrixXd Wt = MatrixXd::Identity(6, 6);
    /*
    Kt = cov_ba * Ct.transpose() * ((Ct * cov_ba * Ct.transpose()).inverse());
    mean_ns = mean_ba + Kt * (zt - Ct * mean_ba);
    cov_ns  = cov_ba  + Kt * Ct * cov_ba;
    mean_ps = mean_ns;
    cov_ps  = cov_ns;

    AngleAxisd rollAngle(mean_ns(3), Vector3d::UnitX());
    AngleAxisd pitchAngle(mean_ns(4), Vector3d::UnitY());
    AngleAxisd yawAngle(mean_ns(5), Vector3d::UnitZ());

    Quaternion<double> Q_output = yawAngle * rollAngle * pitchAngle;

    nav_msgs::Odometry odom_yourwork;
    odom_yourwork.header.stamp = msg->header.stamp;
    odom_yourwork.header.frame_id = "ekf_odom";
    odom_yourwork.pose.pose.position.x = mean_ns(0);
    odom_yourwork.pose.pose.position.y = mean_ns(1);
    odom_yourwork.pose.pose.position.z = mean_ns(2);
    odom_yourwork.pose.pose.orientation.w = Q_output.w();
    odom_yourwork.pose.pose.orientation.x = Q_output.x();
    odom_yourwork.pose.pose.orientation.y = Q_output.y();
    odom_yourwork.pose.pose.orientation.z = Q_output.z();
    odom_pub.publish(odom_yourwork);*/
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ekf");
    ros::NodeHandle n("~");
    ros::Subscriber s1 = n.subscribe("imu", 1000, imu_callback);
    ros::Subscriber s2 = n.subscribe("tag_odom", 1000, odom_callback);
    odom_pub = n.advertise<nav_msgs::Odometry>("ekf_odom", 100);
    Rcam = Quaterniond(0, 0, -1, 0).toRotationMatrix();
    cout << "R_cam" << endl << Rcam << endl;
    // Q imu covariance matrix; Rt visual odomtry covariance matrix
    Q.topLeftCorner(6, 6) = 0.01 * Q.topLeftCorner(6, 6);
    Q.bottomRightCorner(6, 6) = 0.01 * Q.bottomRightCorner(6, 6);
    Rt.topLeftCorner(3, 3) = 0.5 * Rt.topLeftCorner(3, 3);
    Rt.bottomRightCorner(3, 3) = 0.5 * Rt.bottomRightCorner(3, 3);
    Rt.bottomRightCorner(1, 1) = 0.1 * Rt.bottomRightCorner(1, 1);

    ros::spin();
}
