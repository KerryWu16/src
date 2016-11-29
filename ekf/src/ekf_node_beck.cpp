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

#define DEBUG_ODOM false
#define DEBUG_IMU false
#define DEBUG_TF true

using namespace std;
using namespace Eigen;
ros::Publisher odom_pub;
MatrixXd Q = MatrixXd::Identity(12, 12); // For propogate, IMU noise
MatrixXd Rt = MatrixXd::Identity(6, 6);   // For update, Odometry noise


// ros::Time current_time, last_time;
ros::Time current_time;
ros::Time last_time;

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
VectorXd mean_ps(15);
VectorXd mean_ba(15);
VectorXd mean_ns(15);
MatrixXd cov_ps = MatrixXd::Identity(15, 15);
MatrixXd cov_ba = MatrixXd::Identity(15, 15);
MatrixXd cov_ns = MatrixXd::Identity(15, 15);

// Initially use a constant, later need to read from the environment
float g = 9.81;

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
*/
void imu_callback(const sensor_msgs::Imu::ConstPtr &msg)
{
	#if DEBUG_IMU
	cout << "Before process, the mean_ps is: " << endl << mean_ps << endl;
	cout << "The cov_ps is: " << endl << cov_ps << endl;
    	// ROS_INFO("Imu Seq: [%d]", msg->header.seq);
    	// ROS_INFO("Imu linear acceleration x: [%f], y: [%f], z: [%f]", \
		// msg->linear_acceleration.x,msg->linear_acceleration.y,msg->linear_acceleration.z);
    	// ROS_INFO("Imu angular velocity x: [%f], y: [%f], z: [%f]", \
    	// msg->angular_velocity.x,msg->angular_velocity.y,msg->angular_velocity.z);
	#endif
    MatrixXd At = MatrixXd::Identity(15, 15);
    MatrixXd Bt = MatrixXd::Identity(15, 6);
    MatrixXd Ut = MatrixXd::Identity(15, 12);
    MatrixXd Ft = MatrixXd::Identity(15, 15);
    MatrixXd Vt = MatrixXd::Identity(15, 12);
    last_time = current_time;
	float dt = 0.0;
	if	(current_time < msg->header.stamp) { // Check for its first time
		current_time = msg->header.stamp;
    	dt = current_time.toSec() - last_time.toSec();
	}  else  {
		current_time = msg->header.stamp;
		dt = 0.0;
		return;
	}

    // input u : acceleration, augular_velocity
    float am[3], wm[3];
    am[0] = msg->linear_acceleration.x;
    am[1] = msg->linear_acceleration.y;
    am[2] = msg->linear_acceleration.z;
    wm[0] = msg->angular_velocity.x;
    wm[1] = msg->angular_velocity.y;
    wm[2] = msg->angular_velocity.z;

    // noise n: linear_acceleration_covariance, angular_velocity_covariance,
    //          acc bias noise, gyro bias noise
    //          assume the covariance is diagonalized and x, y, z are independent
    //          Given by the TA
    double na[3]  = {Q(0, 0), Q(1, 1), Q(2, 2)};
    double ng[3]  = {Q(3, 3), Q(4, 4), Q(5, 5)};
    double nba[3] = {Q(6, 6), Q(7, 7), Q(8, 8)};
    double nbg[3] = {Q(9, 9), Q(10,10), Q(11,11)};

    // updated to f(mu_t_1, u_t, 0)
    VectorXd F_t_1(15);
    F_t_1 << \
            mean_ns(6),\
            mean_ns(7),\
            mean_ns(8),\
            wm[0]*cos(mean_ns(4)) - 0*cos(mean_ns(4)) - mean_ns(9)*cos(mean_ns(4)) - 0*sin(mean_ns(4)) + wm[2]*sin(mean_ns(4)) - mean_ns(11)*sin(mean_ns(4)),\
            -(0*cos(mean_ns(3)) - wm[1]*cos(mean_ns(3)) + mean_ns(10)*cos(mean_ns(3)) - 0*cos(mean_ns(4))*sin(mean_ns(3)) + wm[2]*cos(mean_ns(4))*sin(mean_ns(3)) - mean_ns(11)*cos(mean_ns(4))*sin(mean_ns(3)) + 0*sin(mean_ns(3))*sin(mean_ns(4)) - wm[0]*sin(mean_ns(3))*sin(mean_ns(4)) + mean_ns(9)*sin(mean_ns(3))*sin(mean_ns(4)))/cos(mean_ns(3)),\
            -(0*cos(mean_ns(4)) - wm[2]*cos(mean_ns(4)) + mean_ns(11)*cos(mean_ns(4)) - 0*sin(mean_ns(4)) + wm[0]*sin(mean_ns(4)) - mean_ns(9)*sin(mean_ns(4)))/cos(mean_ns(3)),\
            (cos(mean_ns(5))*sin(mean_ns(4)) - cos(mean_ns(4))*sin(mean_ns(3))*sin(mean_ns(5)))*(0 - am[2] + mean_ns(14)) - (cos(mean_ns(4))*cos(mean_ns(5)) + sin(mean_ns(3))*sin(mean_ns(4))*sin(mean_ns(5)))*(0 - am[0] + mean_ns(12)) - cos(mean_ns(3))*sin(mean_ns(5))*(0 - am[1] + mean_ns(13)),\
            (cos(mean_ns(4))*sin(mean_ns(5)) - cos(mean_ns(5))*sin(mean_ns(3))*sin(mean_ns(4)))*(0 - am[0] + mean_ns(12)) - (sin(mean_ns(4))*sin(mean_ns(5)) + cos(mean_ns(4))*cos(mean_ns(5))*sin(mean_ns(3)))*(0 - am[2] + mean_ns(14)) - cos(mean_ns(3))*cos(mean_ns(5))*(0 - am[1] + mean_ns(13)),\
            sin(mean_ns(3))*(0 - am[1] + mean_ns(13)) - cos(mean_ns(3))*cos(mean_ns(4))*(0 - am[2] + mean_ns(14)) - cos(mean_ns(3))*sin(mean_ns(4))*(0 - am[0] + mean_ns(12)) - g,\
            0,\
            0,\
            0,\
            0,\
            0,\
            0;


    // Jacobian Matrix from matlab code
    At << \
     0, 0, 0,                                                                                                                                 0,                                                                                                                                         0,                                                                                                                                                                             0, 1, 0, 0,                             0,  0,                            0,                                                0,                  0,                                                0,
     0, 0, 0,                                                                                                                                 0,                                                                                                                                         0,                                                                                                                                                                             0, 0, 1, 0,                             0,  0,                            0,                                                0,                  0,                                                0,
     0, 0, 0,                                                                                                                                 0,                                                                                                                                         0,                                                                                                                                                                             0, 0, 0, 1,                             0,  0,                            0,                                                0,                  0,                                                0,
     0, 0, 0,                                                                                                                                 0,                                                   wm[2]*cos(mean_ns(4)) - ng[2]*cos(mean_ns(4)) - mean_ns(11)*cos(mean_ns(4)) + ng[0]*sin(mean_ns(4)) - wm[0]*sin(mean_ns(4)) + mean_ns(9)*sin(mean_ns(4)),                                                                                                                                                                             0, 0, 0, 0,                     -cos(mean_ns(4)),  0,                    -sin(mean_ns(4)),                                                0,                  0,                                                0,
     0, 0, 0,                              (ng[2]*cos(mean_ns(4)) - wm[2]*cos(mean_ns(4)) + mean_ns(11)*cos(mean_ns(4)) - ng[0]*sin(mean_ns(4)) + wm[0]*sin(mean_ns(4)) - mean_ns(9)*sin(mean_ns(4)))/pow(cos(mean_ns(3)), 2),                            -(sin(mean_ns(3))*(ng[0]*cos(mean_ns(4)) - wm[0]*cos(mean_ns(4)) + mean_ns(9)*cos(mean_ns(4)) + ng[2]*sin(mean_ns(4)) - wm[2]*sin(mean_ns(4)) + mean_ns(11)*sin(mean_ns(4))))/cos(mean_ns(3)),                                                                                                                                                                             0, 0, 0, 0, -(sin(mean_ns(3))*sin(mean_ns(4)))/cos(mean_ns(3)), -1, (cos(mean_ns(4))*sin(mean_ns(3)))/cos(mean_ns(3)),                                                0,                  0,                                                0,
     0, 0, 0,                  -(sin(mean_ns(3))*(ng[2]*cos(mean_ns(4)) - wm[2]*cos(mean_ns(4)) + mean_ns(11)*cos(mean_ns(4)) - ng[0]*sin(mean_ns(4)) + wm[0]*sin(mean_ns(4)) - mean_ns(9)*sin(mean_ns(4))))/pow(cos(mean_ns(3)), 2),                                        (ng[0]*cos(mean_ns(4)) - wm[0]*cos(mean_ns(4)) + mean_ns(9)*cos(mean_ns(4)) + ng[2]*sin(mean_ns(4)) - wm[2]*sin(mean_ns(4)) + mean_ns(11)*sin(mean_ns(4)))/cos(mean_ns(3)),                                                                                                                                                                             0, 0, 0, 0,             sin(mean_ns(4))/cos(mean_ns(3)),  0,           -cos(mean_ns(4))/cos(mean_ns(3)),                                                0,                  0,                                                0,
     0, 0, 0, sin(mean_ns(3))*sin(mean_ns(5))*(na[1] - am[1] + mean_ns(13)) - cos(mean_ns(3))*cos(mean_ns(4))*sin(mean_ns(5))*(na[2] - am[2] + mean_ns(14)) - cos(mean_ns(3))*sin(mean_ns(4))*sin(mean_ns(5))*(na[0] - am[0] + mean_ns(12)),   (cos(mean_ns(5))*sin(mean_ns(4)) - cos(mean_ns(4))*sin(mean_ns(3))*sin(mean_ns(5)))*(na[0] - am[0] + mean_ns(12)) + (cos(mean_ns(4))*cos(mean_ns(5)) + sin(mean_ns(3))*sin(mean_ns(4))*sin(mean_ns(5)))*(na[2] - am[2] + mean_ns(14)), (cos(mean_ns(4))*sin(mean_ns(5)) - cos(mean_ns(5))*sin(mean_ns(3))*sin(mean_ns(4)))*(na[0] - am[0] + mean_ns(12)) - (sin(mean_ns(4))*sin(mean_ns(5)) + cos(mean_ns(4))*cos(mean_ns(5))*sin(mean_ns(3)))*(na[2] - am[2] + mean_ns(14)) - cos(mean_ns(3))*cos(mean_ns(5))*(na[1] - am[1] + mean_ns(13)), 0, 0, 0,                             0,  0,                            0, - cos(mean_ns(4))*cos(mean_ns(5)) - sin(mean_ns(3))*sin(mean_ns(4))*sin(mean_ns(5)), -cos(mean_ns(3))*sin(mean_ns(5)),   cos(mean_ns(5))*sin(mean_ns(4)) - cos(mean_ns(4))*sin(mean_ns(3))*sin(mean_ns(5)),
     0, 0, 0, cos(mean_ns(5))*sin(mean_ns(3))*(na[1] - am[1] + mean_ns(13)) - cos(mean_ns(3))*cos(mean_ns(5))*sin(mean_ns(4))*(na[0] - am[0] + mean_ns(12)) - cos(mean_ns(3))*cos(mean_ns(4))*cos(mean_ns(5))*(na[2] - am[2] + mean_ns(14)), - (sin(mean_ns(4))*sin(mean_ns(5)) + cos(mean_ns(4))*cos(mean_ns(5))*sin(mean_ns(3)))*(na[0] - am[0] + mean_ns(12)) - (cos(mean_ns(4))*sin(mean_ns(5)) - cos(mean_ns(5))*sin(mean_ns(3))*sin(mean_ns(4)))*(na[2] - am[2] + mean_ns(14)), (cos(mean_ns(4))*cos(mean_ns(5)) + sin(mean_ns(3))*sin(mean_ns(4))*sin(mean_ns(5)))*(na[0] - am[0] + mean_ns(12)) - (cos(mean_ns(5))*sin(mean_ns(4)) - cos(mean_ns(4))*sin(mean_ns(3))*sin(mean_ns(5)))*(na[2] - am[2] + mean_ns(14)) + cos(mean_ns(3))*sin(mean_ns(5))*(na[1] - am[1] + mean_ns(13)), 0, 0, 0,                             0,  0,                            0,   cos(mean_ns(4))*sin(mean_ns(5)) - cos(mean_ns(5))*sin(mean_ns(3))*sin(mean_ns(4)), -cos(mean_ns(3))*cos(mean_ns(5)), - sin(mean_ns(4))*sin(mean_ns(5)) - cos(mean_ns(4))*cos(mean_ns(5))*sin(mean_ns(3)),
     0, 0, 0,                            cos(mean_ns(3))*(na[1] - am[1] + mean_ns(13)) + cos(mean_ns(4))*sin(mean_ns(3))*(na[2] - am[2] + mean_ns(14)) + sin(mean_ns(3))*sin(mean_ns(4))*(na[0] - am[0] + mean_ns(12)),                                                                 cos(mean_ns(3))*sin(mean_ns(4))*(na[2] - am[2] + mean_ns(14)) - cos(mean_ns(3))*cos(mean_ns(4))*(na[0] - am[0] + mean_ns(12)),                                                                                                                                                                             0, 0, 0, 0,                             0,  0,                            0,                               -cos(mean_ns(3))*sin(mean_ns(4)),           sin(mean_ns(3)),                               -cos(mean_ns(3))*cos(mean_ns(4)),
     0, 0, 0,                                                                                                                                 0,                                                                                                                                         0,                                                                                                                                                                             0, 0, 0, 0,                             0,  0,                            0,                                                0,                  0,                                                0,
     0, 0, 0,                                                                                                                                 0,                                                                                                                                         0,                                                                                                                                                                             0, 0, 0, 0,                             0,  0,                            0,                                                0,                  0,                                                0,
     0, 0, 0,                                                                                                                                 0,                                                                                                                                         0,                                                                                                                                                                             0, 0, 0, 0,                             0,  0,                            0,                                                0,                  0,                                                0,
     0, 0, 0,                                                                                                                                 0,                                                                                                                                         0,                                                                                                                                                                             0, 0, 0, 0,                             0,  0,                            0,                                                0,                  0,                                                0,
     0, 0, 0,                                                                                                                                 0,                                                                                                                                         0,                                                                                                                                                                             0, 0, 0, 0,                             0,  0,                            0,                                                0,                  0,                                                0,
     0, 0, 0,                                                                                                                                 0,                                                                                                                                         0,                                                                                                                                                                             0, 0, 0, 0,                             0,  0,                            0,                                                0,                  0,                                                0;


	Bt <<
                                                  0,                 0,                                              0,                            0, 0,                             0,
                                                  0,                 0,                                              0,                            0, 0,                             0,
                                                  0,                 0,                                              0,                            0, 0,                             0,
                                                  0,                 0,                                              0,                     cos(mean_ns(4)), 0,                      sin(mean_ns(4)),
                                                  0,                 0,                                              0, (sin(mean_ns(3))*sin(mean_ns(4)))/cos(mean_ns(3)), 1, -(cos(mean_ns(4))*sin(mean_ns(3)))/cos(mean_ns(3)),
                                                  0,                 0,                                              0,           -sin(mean_ns(4))/cos(mean_ns(3)), 0,             cos(mean_ns(4))/cos(mean_ns(3)),
     cos(mean_ns(4))*cos(mean_ns(5)) + sin(mean_ns(3))*sin(mean_ns(4))*sin(mean_ns(5)), cos(mean_ns(3))*sin(mean_ns(5)), cos(mean_ns(4))*sin(mean_ns(3))*sin(mean_ns(5)) - cos(mean_ns(5))*sin(mean_ns(4)),                            0, 0,                             0,
     cos(mean_ns(5))*sin(mean_ns(3))*sin(mean_ns(4)) - cos(mean_ns(4))*sin(mean_ns(5)), cos(mean_ns(3))*cos(mean_ns(5)), sin(mean_ns(4))*sin(mean_ns(5)) + cos(mean_ns(4))*cos(mean_ns(5))*sin(mean_ns(3)),                            0, 0,                             0,
                                  cos(mean_ns(3))*sin(mean_ns(4)),         -sin(mean_ns(3)),                              cos(mean_ns(3))*cos(mean_ns(4)),                            0, 0,                             0,
                                                  0,                 0,                                              0,                            0, 0,                             0,
                                                  0,                 0,                                              0,                            0, 0,                             0,
                                                  0,                 0,                                              0,                            0, 0,                             0,
                                                  0,                 0,                                              0,                            0, 0,                             0,
                                                  0,                 0,                                              0,                            0, 0,                             0,
                                                  0,                 0,                                              0,                            0, 0,                             0;


    Ut <<
                                                    0,                  0,                                                0,                             0,  0,                            0, 0, 0, 0, 0, 0, 0,
                                                    0,                  0,                                                0,                             0,  0,                            0, 0, 0, 0, 0, 0, 0,
                                                    0,                  0,                                                0,                             0,  0,                            0, 0, 0, 0, 0, 0, 0,
                                                    0,                  0,                                                0,                     -cos(mean_ns(4)),  0,                    -sin(mean_ns(4)), 0, 0, 0, 0, 0, 0,
                                                    0,                  0,                                                0, -(sin(mean_ns(3))*sin(mean_ns(4)))/cos(mean_ns(3)), -1, (cos(mean_ns(4))*sin(mean_ns(3)))/cos(mean_ns(3)), 0, 0, 0, 0, 0, 0,
                                                    0,                  0,                                                0,             sin(mean_ns(4))/cos(mean_ns(3)),  0,           -cos(mean_ns(4))/cos(mean_ns(3)), 0, 0, 0, 0, 0, 0,
     - cos(mean_ns(4))*cos(mean_ns(5)) - sin(mean_ns(3))*sin(mean_ns(4))*sin(mean_ns(5)), -cos(mean_ns(3))*sin(mean_ns(5)),   cos(mean_ns(5))*sin(mean_ns(4)) - cos(mean_ns(4))*sin(mean_ns(3))*sin(mean_ns(5)),                             0,  0,                            0, 0, 0, 0, 0, 0, 0,
       cos(mean_ns(4))*sin(mean_ns(5)) - cos(mean_ns(5))*sin(mean_ns(3))*sin(mean_ns(4)), -cos(mean_ns(3))*cos(mean_ns(5)), - sin(mean_ns(4))*sin(mean_ns(5)) - cos(mean_ns(4))*cos(mean_ns(5))*sin(mean_ns(3)),                             0,  0,                            0, 0, 0, 0, 0, 0, 0,
                                   -cos(mean_ns(3))*sin(mean_ns(4)),           sin(mean_ns(3)),                               -cos(mean_ns(3))*cos(mean_ns(4)),                             0,  0,                            0, 0, 0, 0, 0, 0, 0,
                                                    0,                  0,                                                0,                             0,  0,                            0, 0, 0, 0, 1, 0, 0,
                                                    0,                  0,                                                0,                             0,  0,                            0, 0, 0, 0, 0, 1, 0,
                                                    0,                  0,                                                0,                             0,  0,                            0, 0, 0, 0, 0, 0, 1,
                                                    0,                  0,                                                0,                             0,  0,                            0, 1, 0, 0, 0, 0, 0,
                                                    0,                  0,                                                0,                             0,  0,                            0, 0, 1, 0, 0, 0, 0,
                                                    0,                  0,                                                0,                             0,  0,                            0, 0, 0, 1, 0, 0, 0;

	Ft = MatrixXd::Identity(15, 15) + dt * At;
    Vt = dt * Ut;

    // Calculate the propogagted mean and covariance
    mean_ba = mean_ps + dt * F_t_1;
    cov_ba = Ft * cov_ps * Ft.transpose() + Vt * Q * Vt.transpose();
	#if DEBUG_IMU
		cout<< "After process, the mean_ba became" << endl << mean_ba << endl;
		cout<< "The cov_ba became" << endl << cov_ba << endl;
	#endif
}


void odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
{


    //your code for update
    //camera position in the IMU frame = (0, -0.04, -0.02)
    //camera orientaion in the IMU frame = Quaternion(0, 0, 1, 0); w x y z, respectively
    //			   RotationMatrix << -1, 0, 0,
    //							      0, 1, 0,
    //                                0, 0, -1;
	#if DEBUG_TF
		// static tf::TransformBroadcaster br;

		// Get the world frame in camera frame transformation from the msg
		tf::Transform camera_pose_cw;
		tf::poseMsgToTF(msg->pose.pose, camera_pose_cw);

		// Record the camera frame in the IMU frame from TA
		tf::Transform transform_ic;
		transform_ic.setOrigin( tf::Vector3(0, -0.04, -0.02) );
		transform_ic.setRotation( tf::Quaternion(0, 0, -1, 0) );
		tf::Transform camera_pose_wi = (transform_ic * camera_pose_cw).inverse();

		geometry_msgs::Pose camera_pose_wi_geo;
		tf::transformTFToMsg(camera_pose_wi, camera_pose_wi_geo);

		cout << "camera_pose_wi_geo transformation from TF is: " << endl;
		cout << camera_pose_wi_geo.position.x << endl;
		cout << camera_pose_wi_geo.position.y << endl;
		cout << camera_pose_wi_geo.position.z << endl;
		cout << "quaternion from TF is: " << endl;
		cout << camera_pose_wi_geo.orientation.w << endl;
		cout << camera_pose_wi_geo.orientation.x << endl;
		cout << camera_pose_wi_geo.orientation.y << endl;
		cout << camera_pose_wi_geo.orientation.z << endl;

		VectorXd zt(6); // Camera reading in x,y,z, ZXY Euler
		zt(0) = camera_pose_wi_geo.position.x;
		zt(1) = camera_pose_wi_geo.position.y;
		zt(2) = camera_pose_wi_geo.position.z;
		// From quaternion to rotation matrix and then ZXY Euler
		Eigen::Quaterniond R_wi_quat;
		R_wi_quat.w() = camera_pose_wi_geo.orientation.w;
		R_wi_quat.x() = camera_pose_wi_geo.orientation.x;
		R_wi_quat.y() = camera_pose_wi_geo.orientation.y;
		R_wi_quat.z() = camera_pose_wi_geo.orientation.z;
		Matrix3d R_wi_2 = R_wi_quat.toRotationMatrix();
		zt(3) = asin(R_wi_2(1, 2));
		zt(4) = atan2(-R_wi_2(1, 0), R_wi_2(1, 1)); // pitch_wi
		zt(5) = atan2(-R_wi_2(0, 2), R_wi_2(2, 2)); // yaw_wi

		Vector3d T_cw;
		Quaterniond R_cw_quat;
		// camera to tag world
		R_cw_quat.w() = msg->pose.pose.orientation.w;
		R_cw_quat.x() = msg->pose.pose.orientation.x;
		R_cw_quat.y() = msg->pose.pose.orientation.y;
		R_cw_quat.z() = msg->pose.pose.orientation.z;
		T_cw[0] = msg->pose.pose.position.x;
		T_cw[1] = msg->pose.pose.position.y;
		T_cw[2] = msg->pose.pose.position.z;
		Matrix3d R_cw = R_cw_quat.toRotationMatrix();

		// IMU to camera frame
		Matrix3d R_ic = Quaterniond(0, 0, 1, 0).toRotationMatrix();
	    Vector3d T_ic = Vector3d(0, -0.04, -0.02);

		// IMU to tag world
		Matrix3d R_iw = R_ic * R_cw;
		Vector3d T_iw = R_ic * T_cw + T_ic;

		// tag world to IMU
		Matrix3d R_wi = R_iw.inverse();
		Vector3d T_wi = -R_iw.inverse()*T_iw;
		Quaterniond R_wi_q(R_wi);

		cout << "camera_pose_wi_geo transformation from Eigen is: " << endl;
		cout << T_wi(0) << endl;
		cout << T_wi(1) << endl;
		cout << T_wi(2) << endl;
		cout << "quaternion from TF is: " << endl;
		cout << R_wi_q.w() << endl;
		cout << R_wi_q.x() << endl;
		cout << R_wi_q.y() << endl;
		cout << R_wi_q.z() << endl;

		// VectorXd zt(6); // Camera reading in x,y,z, ZXY Euler
		// zt(0) = T_wi(0);
		// zt(1) = T_wi(1);
		// zt(2) = T_wi(2);
		// // Quaternino -> rotation matrix -> ZXY Euler
		// zt(3) = asin(R_wi(1, 2));
		// zt(4) = atan2(-R_wi(1, 0), R_wi(1, 1)); // pitch_wi
		// zt(5) = atan2(-R_wi(0, 2), R_wi(2, 2)); // yaw_wi
	#endif


    #if DEBUG_ODOM
        cout<<" The x of camera: " << zt(0) <<endl;
        cout<<" The y of camera: " << zt(1) <<endl;
        cout<<" The z of camera: " << zt(2) <<endl;
        cout<<" The roll of camera in ZXY Euler angle: " << zt(3) <<endl;
        cout<<" The pitch of camera in ZXY Euler angle: " << zt(4) <<endl;
        cout<<" The yaw of camera in ZXY Euler angle: " << zt(5) <<endl;
     // Need to rostopic echo tag_odom
        cout<<" The pose of camera is in the coordinate frame: " <<  msg->header.frame_id <<endl;
        cout<<" The twist of camera is in the child frame: " <<  msg->child_frame_id <<endl;
    #endif

    /* Update, with C and W matrix
        Linear for this case, use Kalman Filter
    */
    MatrixXd Kt = MatrixXd::Identity(15, 15); // Kalman
    MatrixXd Ct = MatrixXd::Identity(6, 15);
    MatrixXd Wt = MatrixXd::Identity(6, 6);

    Kt = cov_ba * Ct.transpose() * ((Ct * cov_ba * Ct.transpose()).inverse());
    mean_ns = mean_ba + Kt * (zt - Ct * mean_ba);
    cov_ns  = cov_ba  + Kt * Ct * cov_ba;
    mean_ps = mean_ns;
    cov_ps  = cov_ns;

    AngleAxisd rollAngle(mean_ns(3), Vector3d::UnitX());
    AngleAxisd pitchAngle(mean_ns(4), Vector3d::UnitY());
    AngleAxisd yawAngle(mean_ns(5), Vector3d::UnitZ());

    Quaternion<double> Q_output = yawAngle * rollAngle * pitchAngle;

    nav_msgs::Odometry ekf_odom;
	ekf_odom.header.seq = msg->header.seq;
    ekf_odom.header.stamp = msg->header.stamp;
    ekf_odom.header.frame_id = "world";
    ekf_odom.pose.pose.position.x = mean_ns(0);
    ekf_odom.pose.pose.position.y = mean_ns(1);
    ekf_odom.pose.pose.position.z = mean_ns(2);
    ekf_odom.pose.pose.orientation.w = Q_output.w();
    ekf_odom.pose.pose.orientation.x = Q_output.x();
    ekf_odom.pose.pose.orientation.y = Q_output.y();
    ekf_odom.pose.pose.orientation.z = Q_output.z();

    odom_pub.publish(ekf_odom);

	#if DEBUG_TF
//		tf::Pose ekf_odom_pose;
//		tf::poseMsgToTF(ekf_odom.pose.pose, ekf_odom_pose);

//		br.sendTransform( tf::StampedTransform(ekf_odom_pose, msg->header.stamp, "world", "ekf_odom"));
	#endif

	#if DEBUG_ODOM
		cout<<" The Kalman gain is:" << endl << Kt << endl;
		cout<<" The mean_ns is:" << endl << mean_ns << endl;
		cout<<" The cov_ns is:" << endl << cov_ns << endl;
		cout<<" The odometry before EKF:" << endl;
        ROS_INFO("Seq: [%d]", msg->header.seq);
		ROS_INFO("Position-> x: [%f], y: [%f], z: [%f]", camera_pose_wi_geo.position.x,camera_pose_wi_geo.position.y, camera_pose_wi_geo.position.z);
		ROS_INFO("Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]", camera_pose_wi_geo.orientation.x, camera_pose_wi_geo.orientation.y, camera_pose_wi_geo.orientation.z, camera_pose_wi_geo.orientation.w);
		cout<<" The odometry after EKF:" << endl;
        ROS_INFO("Seq: [%d]", ekf_odom.header.seq);
		ROS_INFO("Position-> x: [%f], y: [%f], z: [%f]", ekf_odom.pose.pose.position.x,ekf_odom.pose.pose.position.y, ekf_odom.pose.pose.position.z);
		ROS_INFO("Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]", ekf_odom.pose.pose.orientation.x, ekf_odom.pose.pose.orientation.y, ekf_odom.pose.pose.orientation.z, ekf_odom.pose.pose.orientation.w);
		ROS_INFO("Vel-> Linear: [%f], Angular: [%f]", ekf_odom.twist.twist.linear.x,ekf_odom.twist.twist.angular.z);
    #endif
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ekf");
    ros::NodeHandle n("~");
	ros::Time::init();
	current_time = ros::Time::now();
    ros::Subscriber s1 = n.subscribe("imu", 1000, imu_callback);
    ros::Subscriber s2 = n.subscribe("tag_odom", 1000, odom_callback);
    odom_pub = n.advertise<nav_msgs::Odometry>("ekf_odom", 100);
    // Q imu covariance matrix; Rt visual odomtry covariance matrix
    Q.topLeftCorner(6, 6) = 0.01 * Q.topLeftCorner(6, 6);
    Q.bottomRightCorner(6, 6) = 0.01 * Q.bottomRightCorner(6, 6);
    Rt.topLeftCorner(3, 3) = 0.5 * Rt.topLeftCorner(3, 3);
    Rt.bottomRightCorner(3, 3) = 0.5 * Rt.bottomRightCorner(3, 3);
    Rt.bottomRightCorner(1, 1) = 0.1 * Rt.bottomRightCorner(1, 1);

    ros::spin();
}
