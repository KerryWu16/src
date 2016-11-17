#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Eigen>

#include <ros/time.h>

using namespace std;
using namespace Eigen;
ros::Publisher odom_pub;
MatrixXd Q = MatrixXd::Identity(12, 12); // For propogate, IMU noise
MatrixXd Rt = MatrixXd::Identity(6,6);   // For update, Odometry noise


ros::Time current_time, last_time;
current_time = ros::Time::now();
last_time = ros::Time::now();

// Mean and covariance matrixs
VectorXd mean_current = VectorXd::Identity(15);
VectorXd mean_last = VectorXd::Identity(15);
MatrixXd covar_current = MatrixXd::Identity(15, 15);
MatrixXd covar_last = MatrixXd::Identity(15, 15);

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
    float na[3], ng[3], nba[3], nbg[3];
    if (msg->linear_acceleration_covariance.begin() != -1) {
        float linear_cov[9] = msg->linear_acceleration_covariance;
        na[0] = linear_cov[0];
        na[1] = linear_cov[4];
        na[2] = linear_cov[8];
        Map<MatrixXd> Q.topLeftCorner(3, 3)(linear_cov);
    } else { na = {0}; }

    if (msg->angular_velocity_covariance.begin() != -1) {
        float angular_cov[9] = msg->angular_velocity_covariance;
        ng[0] = angular_cov[0];
        ng[1] = angular_cov[4];
        ng[2] = angular_cov[8];
        Map<MatrixXd> Q.topLeftCorner(6, 6).bottomRightCorner(3,3)(angular_cov);
    } else { ng = {0}; }
    nba = {0};
    nbg = {0};
    Q.bottomRightCorner(6, 6) = {0};
    // Q(6,6) = nba[0];
    // Q(7,7) = nba[1];
    // Q(8,8) = nba[2];
    // Q(9,9) = nbg[0];
    // Q(10,10) = nbg[1];
    // Q(11,11) = nbg[2];

    // TODO: updated to f(mu_t_1, u_t, 0)
    VectorXd F_t_1;
    F_t_1 <<
            x31,
            x32,
            x33,
            wm1*cos(x22) - ng1*cos(x22) - x41*cos(x22) - ng3*sin(x22) + wm3*sin(x22) - x43*sin(x22),
            -(ng2*cos(x21) - wm2*cos(x21) + x42*cos(x21) - ng3*cos(x22)*sin(x21) + wm3*cos(x22)*sin(x21) - x43*cos(x22)*sin(x21) + ng1*sin(x21)*sin(x22) - wm1*sin(x21)*sin(x22) + x41*sin(x21)*sin(x22))/cos(x21),
            -(ng3*cos(x22) - wm3*cos(x22) + x43*cos(x22) - ng1*sin(x22) + wm1*sin(x22) - x41*sin(x22))/cos(x21),
            (cos(x23)*sin(x22) - cos(x22)*sin(x21)*sin(x23))*(na3 - am3 + x53) - (cos(x22)*cos(x23) + sin(x21)*sin(x22)*sin(x23))*(na1 - am1 + x51) - cos(x21)*sin(x23)*(na2 - am2 + x52) + g,
            (cos(x22)*sin(x23) - cos(x23)*sin(x21)*sin(x22))*(na1 - am1 + x51) - (sin(x22)*sin(x23) + cos(x22)*cos(x23)*sin(x21))*(na3 - am3 + x53) - cos(x21)*cos(x23)*(na2 - am2 + x52) + g,
            sin(x21)*(na2 - am2 + x52) - cos(x21)*cos(x22)*(na3 - am3 + x53) - cos(x21)*sin(x22)*(na1 - am1 + x51) + g,
            nbg1,
            nbg2,
            nbg3,
            nba1,
            nba2,
            nba3;

    // Jacobian Matrix from matlab code
    At << \
     0, 0, 0,                                                                                                                                 0,                                                                                                                                         0,                                                                                                                                                                             0, 1, 0, 0,                             0,  0,                            0,                                                0,                  0,                                                0,
     0, 0, 0,                                                                                                                                 0,                                                                                                                                         0,                                                                                                                                                                             0, 0, 1, 0,                             0,  0,                            0,                                                0,                  0,                                                0,
     0, 0, 0,                                                                                                                                 0,                                                                                                                                         0,                                                                                                                                                                             0, 0, 0, 1,                             0,  0,                            0,                                                0,                  0,                                                0,
     0, 0, 0,                                                                                                                                 0,                                                   wm[2]*cos(x22) - ng[2]*cos(x22) - x43*cos(x22) + ng[0]*sin(x22) - wm[0]*sin(x22) + x41*sin(x22),                                                                                                                                                                             0, 0, 0, 0,                     -cos(x22),  0,                    -sin(x22),                                                0,                  0,                                                0,
     0, 0, 0,                              (ng[2]*cos(x22) - wm[2]*cos(x22) + x43*cos(x22) - ng[0]*sin(x22) + wm[0]*sin(x22) - x41*sin(x22))/cos(x21)^2,                            -(sin(x21)*(ng[0]*cos(x22) - wm[0]*cos(x22) + x41*cos(x22) + ng[2]*sin(x22) - wm[2]*sin(x22) + x43*sin(x22)))/cos(x21),                                                                                                                                                                             0, 0, 0, 0, -(sin(x21)*sin(x22))/cos(x21), -1, (cos(x22)*sin(x21))/cos(x21),                                                0,                  0,                                                0,
     0, 0, 0,                  -(sin(x21)*(ng[2]*cos(x22) - wm[2]*cos(x22) + x43*cos(x22) - ng[0]*sin(x22) + wm[0]*sin(x22) - x41*sin(x22)))/cos(x21)^2,                                        (ng[0]*cos(x22) - wm[0]*cos(x22) + x41*cos(x22) + ng[2]*sin(x22) - wm[2]*sin(x22) + x43*sin(x22))/cos(x21),                                                                                                                                                                             0, 0, 0, 0,             sin(x22)/cos(x21),  0,           -cos(x22)/cos(x21),                                                0,                  0,                                                0,
     0, 0, 0, sin(x21)*sin(x23)*(na[1] - am[1] + x52) - cos(x21)*cos(x22)*sin(x23)*(na[2] - am[2] + x53) - cos(x21)*sin(x22)*sin(x23)*(na[0] - am[0] + x51),   (cos(x23)*sin(x22) - cos(x22)*sin(x21)*sin(x23))*(na[0] - am[0] + x51) + (cos(x22)*cos(x23) + sin(x21)*sin(x22)*sin(x23))*(na[2] - am[2] + x53), (cos(x22)*sin(x23) - cos(x23)*sin(x21)*sin(x22))*(na[0] - am[0] + x51) - (sin(x22)*sin(x23) + cos(x22)*cos(x23)*sin(x21))*(na[2] - am[2] + x53) - cos(x21)*cos(x23)*(na[1] - am[1] + x52), 0, 0, 0,                             0,  0,                            0, - cos(x22)*cos(x23) - sin(x21)*sin(x22)*sin(x23), -cos(x21)*sin(x23),   cos(x23)*sin(x22) - cos(x22)*sin(x21)*sin(x23),
     0, 0, 0, cos(x23)*sin(x21)*(na[1] - am[1] + x52) - cos(x21)*cos(x23)*sin(x22)*(na[0] - am[0] + x51) - cos(x21)*cos(x22)*cos(x23)*(na[2] - am[2] + x53), - (sin(x22)*sin(x23) + cos(x22)*cos(x23)*sin(x21))*(na[0] - am[0] + x51) - (cos(x22)*sin(x23) - cos(x23)*sin(x21)*sin(x22))*(na[2] - am[2] + x53), (cos(x22)*cos(x23) + sin(x21)*sin(x22)*sin(x23))*(na[0] - am[0] + x51) - (cos(x23)*sin(x22) - cos(x22)*sin(x21)*sin(x23))*(na[2] - am[2] + x53) + cos(x21)*sin(x23)*(na[1] - am[1] + x52), 0, 0, 0,                             0,  0,                            0,   cos(x22)*sin(x23) - cos(x23)*sin(x21)*sin(x22), -cos(x21)*cos(x23), - sin(x22)*sin(x23) - cos(x22)*cos(x23)*sin(x21),
     0, 0, 0,                            cos(x21)*(na[1] - am[1] + x52) + cos(x22)*sin(x21)*(na[2] - am[2] + x53) + sin(x21)*sin(x22)*(na[0] - am[0] + x51),                                                                 cos(x21)*sin(x22)*(na[2] - am[2] + x53) - cos(x21)*cos(x22)*(na[0] - am[0] + x51),                                                                                                                                                                             0, 0, 0, 0,                             0,  0,                            0,                               -cos(x21)*sin(x22),           sin(x21),                               -cos(x21)*cos(x22),
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
                                                  0,                 0,                                              0,                     cos(x22), 0,                      sin(x22),
                                                  0,                 0,                                              0, (sin(x21)*sin(x22))/cos(x21), 1, -(cos(x22)*sin(x21))/cos(x21),
                                                  0,                 0,                                              0,           -sin(x22)/cos(x21), 0,             cos(x22)/cos(x21),
     cos(x22)*cos(x23) + sin(x21)*sin(x22)*sin(x23), cos(x21)*sin(x23), cos(x22)*sin(x21)*sin(x23) - cos(x23)*sin(x22),                            0, 0,                             0,
     cos(x23)*sin(x21)*sin(x22) - cos(x22)*sin(x23), cos(x21)*cos(x23), sin(x22)*sin(x23) + cos(x22)*cos(x23)*sin(x21),                            0, 0,                             0,
                                  cos(x21)*sin(x22),         -sin(x21),                              cos(x21)*cos(x22),                            0, 0,                             0,
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
                                                    0,                  0,                                                0,                     -cos(x22),  0,                    -sin(x22), 0, 0, 0, 0, 0, 0,
                                                    0,                  0,                                                0, -(sin(x21)*sin(x22))/cos(x21), -1, (cos(x22)*sin(x21))/cos(x21), 0, 0, 0, 0, 0, 0,
                                                    0,                  0,                                                0,             sin(x22)/cos(x21),  0,           -cos(x22)/cos(x21), 0, 0, 0, 0, 0, 0,
     - cos(x22)*cos(x23) - sin(x21)*sin(x22)*sin(x23), -cos(x21)*sin(x23),   cos(x23)*sin(x22) - cos(x22)*sin(x21)*sin(x23),                             0,  0,                            0, 0, 0, 0, 0, 0, 0,
       cos(x22)*sin(x23) - cos(x23)*sin(x21)*sin(x22), -cos(x21)*cos(x23), - sin(x22)*sin(x23) - cos(x22)*cos(x23)*sin(x21),                             0,  0,                            0, 0, 0, 0, 0, 0, 0,
                                   -cos(x21)*sin(x22),           sin(x21),                               -cos(x21)*cos(x22),                             0,  0,                            0, 0, 0, 0, 0, 0, 0,
                                                    0,                  0,                                                0,                             0,  0,                            0, 0, 0, 0, 1, 0, 0,
                                                    0,                  0,                                                0,                             0,  0,                            0, 0, 0, 0, 0, 1, 0,
                                                    0,                  0,                                                0,                             0,  0,                            0, 0, 0, 0, 0, 0, 1,
                                                    0,                  0,                                                0,                             0,  0,                            0, 1, 0, 0, 0, 0, 0,
                                                    0,                  0,                                                0,                             0,  0,                            0, 0, 1, 0, 0, 0, 0,
                                                    0,                  0,                                                0,                             0,  0,                            0, 0, 0, 1, 0, 0, 0;

    Ft = MatrixXd::Identity(15, 15) + dt * At;
    Vt = dt * Ut;

    // Calculate the propogagted mean and covariance
    mean_current = mean_last + dt * F_t_1;
    covar_current = Ft * covar_last * Ft.transpose() + Vt * Qt * Vt.transpose();
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

    // Construct the published odemetry
    nav_msgs::Odometry odom_yourwork;
    odom_yourwork.header.stamp = frame_time;
    odom_yourwork.header.frame_id = "world";
    odom_yourwork.pose.pose.position.x = T(0);
    odom_yourwork.pose.pose.position.y = T(1);
    odom_yourwork.pose.pose.position.z = T(2);
    odom_yourwork.pose.pose.orientation.w = Q_yourwork.w();
    odom_yourwork.pose.pose.orientation.x = Q_yourwork.x();
    odom_yourwork.pose.pose.orientation.y = Q_yourwork.y();
    odom_yourwork.pose.pose.orientation.z = Q_yourwork.z();
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
