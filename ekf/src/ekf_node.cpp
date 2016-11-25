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
ros::Time current_time = ros::Time::now();
ros::Time last_time = ros::Time::now();

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
VectorXd mean_ps = VectorXd::Identity(15, 1);
VectorXd mean_ba = VectorXd::Identity(15, 1);
VectorXd mean_ns = VectorXd::Identity(15, 1);
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
    ROS_INFO("Imu Seq: [%d]", msg->header.seq);
    ROS_INFO("Imu Orientation x: [%f], y: [%f], z: [%f], w: [%f]", \
    msg->orientation.x,msg->orientation.y,msg->orientation.z,msg->orientation.w);
    MatrixXd At = MatrixXd::Identity(15, 15);
    MatrixXd Bt = MatrixXd::Identity(15, 6);
    MatrixXd Ut = MatrixXd::Identity(15, 12);
    MatrixXd Ft = MatrixXd::Identity(15, 15);
    MatrixXd Vt = MatrixXd::Identity(15, 12);
    float dt = 0.0;
    last_time = current_time;
    current_time = msg->header.stamp;
    dt = (float)(current_time.nsec - last_time.nsec) / 1E9;
    ROS_INFO("dt: [%f]", dt);
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
    VectorXd F_t_1;
    F_t_1 <<
            mean_ns(6),
            mean_ns(7),
            mean_ns(8),
            wm[0]*cos(mean_ns(4)) - 0*cos(mean_ns(4)) - mean_ns(9)*cos(mean_ns(4)) - 0*sin(mean_ns(4)) + wm[2]*sin(mean_ns(4)) - mean_ns(11)*sin(mean_ns(4)),
            -(0*cos(mean_ns(3)) - wm[1]*cos(mean_ns(3)) + mean_ns(10)*cos(mean_ns(3)) - 0*cos(mean_ns(4))*sin(mean_ns(3)) + wm[2]*cos(mean_ns(4))*sin(mean_ns(3)) - mean_ns(11)*cos(mean_ns(4))*sin(mean_ns(3)) + 0*sin(mean_ns(3))*sin(mean_ns(4)) - wm[0]*sin(mean_ns(3))*sin(mean_ns(4)) + mean_ns(9)*sin(mean_ns(3))*sin(mean_ns(4)))/cos(mean_ns(3)),
            -(0*cos(mean_ns(4)) - wm[2]*cos(mean_ns(4)) + mean_ns(11)*cos(mean_ns(4)) - 0*sin(mean_ns(4)) + wm[0]*sin(mean_ns(4)) - mean_ns(9)*sin(mean_ns(4)))/cos(mean_ns(3)),
            (cos(mean_ns(5))*sin(mean_ns(4)) - cos(mean_ns(4))*sin(mean_ns(3))*sin(mean_ns(5)))*(0 - am[2] + mean_ns(14)) - (cos(mean_ns(4))*cos(mean_ns(5)) + sin(mean_ns(3))*sin(mean_ns(4))*sin(mean_ns(5)))*(0 - am[0] + mean_ns(12)) - cos(mean_ns(3))*sin(mean_ns(5))*(0 - am[1] + mean_ns(13)) + g,
            (cos(mean_ns(4))*sin(mean_ns(5)) - cos(mean_ns(5))*sin(mean_ns(3))*sin(mean_ns(4)))*(0 - am[0] + mean_ns(12)) - (sin(mean_ns(4))*sin(mean_ns(5)) + cos(mean_ns(4))*cos(mean_ns(5))*sin(mean_ns(3)))*(0 - am[2] + mean_ns(14)) - cos(mean_ns(3))*cos(mean_ns(5))*(0 - am[1] + mean_ns(13)) + g,
            sin(mean_ns(3))*(0 - am[1] + mean_ns(13)) - cos(mean_ns(3))*cos(mean_ns(4))*(0 - am[2] + mean_ns(14)) - cos(mean_ns(3))*sin(mean_ns(4))*(0 - am[0] + mean_ns(12)) + g,
            0,
            0,
            0,
            0,
            0,
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

    // Get the world frame in camera frame transformation from the msg
    tf::Pose camera_pose_cw;
    tf::poseMsgToTF(msg->pose.pose, camera_pose_cw);

    // Record the camera frame in the IMU frame from TA
    tf::Transform transform_ic;
    transform_ic.setOrigin( tf::Vector3(0, -0.04, -0.02) );
    transform_ic.setRotation( tf::Quaternion(0, 0, 1, 0) );
    tf::Pose camera_pose_iw = transform_ic * camera_pose_cw;

    // Calculate the IMU frame in the world frame
    tf::Pose camera_pose_wi = camera_pose_iw.inverse();
    geometry_msgs::Pose camera_pose_wi_geo;
    tf::poseTFToMsg(camera_pose_wi, camera_pose_wi_geo);

    VectorXd zt = VectorXd::Identity(6, 1); // Camera reading in x,y,z, ZXY Euler
    zt(0) = camera_pose_wi_geo.position.x;
    zt(1) = camera_pose_wi_geo.position.y;
    zt(2) = camera_pose_wi_geo.position.z;
    geometry_msgs::Quaternion q_wi = camera_pose_wi_geo.orientation;
    // Quaternino -> rotation matrix -> ZXY Euler
    zt(3) = asin(2*(q_wi.y * q_wi.z + q_wi.x * q_wi.w));
    zt(4) = acos((1 - 2 * pow(q_wi.x,2) - 2 * pow(q_wi.y,2)) / cos(zt(3))); // pitch_wi
    zt(5) = acos((1 - 2 * pow(q_wi.x,2) - 2 * pow(q_wi.z,2))  / cos(zt(3))); // yaw_wi

    #ifdef DEBUG
        cout<<" The x of camera: " << endl << zt(0) <<endl;
        cout<<" The y of camera: " << endl << zt(1) <<endl;
        cout<<" The z of camera: " << endl << zt(2) <<endl;
        cout<<" The roll of camera in ZXY Euler angle: " << endl << zt(3) <<endl;
        cout<<" The pitch of camera in ZXY Euler angle: " << endl << zt(4) <<endl;
        cout<<" The yaw of camera in ZXY Euler angle: " << endl << zt(5) <<endl;
    #endif

    /* Update, with C and W matrix
        Linear for this case, use Kalman Filter
    */
    MatrixXd Kt = MatrixXd::Identity(15, 15); // Kalman
    MatrixXd Ct = MatrixXd::Identity(6, 15);
    MatrixXd Wt = MatrixXd::Identity(6, 6);
    #ifdef DEBUG
        // Need to rostopic echo tag_odom
        cout<<" The pose of camera is in the coordinate frame: " << endl << msg->header.frame_id <<endl;
        cout<<" The twist of camera is in the child frame: " << endl << msg->child_frame_id <<endl;
    #endif
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
    odom_pub.publish(odom_yourwork);
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
