#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Eigen>

using namespace std;
using namespace Eigen;
ros::Publisher odom_pub;
MatrixXd Q = MatrixXd::Identity(12, 12);
MatrixXd Rt = MatrixXd::Identity(6,6);

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
*/
void imu_callback(const sensor_msgs::Imu::ConstPtr &msg)
{
    /*
    Header header

    geometry_msgs/Quaternion orientation
    float64[9] orientation_covariance # Row major about x, y, z axes

    geometry_msgs/Vector3 angular_velocity
    float64[9] angular_velocity_covariance # Row major about x, y, z axes

    geometry_msgs/Vector3 linear_acceleration
    float64[9] linear_acceleration_covariance # Row major x, y z
    */
    ROS_INFO("Imu Seq: [%d]", msg->header.seq);
    ROS_INFO("Imu Orientation x: [%f], y: [%f], z: [%f], w: [%f]", \
    msg->orientation.x,msg->orientation.y,msg->orientation.z,msg->orientation.w);
    MatrixXd At = MatrixXd::Identity(15, 15);
    MatrixXd Bt = MatrixXd::Identity(15, 6);
    MatrixXd Ut = MatrixXd::Identity(15, 12);
    MatrixXd Ft = MatrixXd::Identity(15, 15);
    MatrixXd Vt = MatrixXd::Identity(15, 12);
    float dt = 0.0;

    // Jacobian Matrix from matlab code
    At << \
     0, 0, 0,                                                                                                                                 0,                                                                                                                                         0,                                                                                                                                                                             0, 1, 0, 0,                             0,  0,                            0,                                                0,                  0,                                                0,
     0, 0, 0,                                                                                                                                 0,                                                                                                                                         0,                                                                                                                                                                             0, 0, 1, 0,                             0,  0,                            0,                                                0,                  0,                                                0,
     0, 0, 0,                                                                                                                                 0,                                                                                                                                         0,                                                                                                                                                                             0, 0, 0, 1,                             0,  0,                            0,                                                0,                  0,                                                0,
     0, 0, 0,                                                                                                                                 0,                                                   wm3*cos(x22) - ng3*cos(x22) - x43*cos(x22) + ng1*sin(x22) - wm1*sin(x22) + x41*sin(x22),                                                                                                                                                                             0, 0, 0, 0,                     -cos(x22),  0,                    -sin(x22),                                                0,                  0,                                                0,
     0, 0, 0,                              (ng3*cos(x22) - wm3*cos(x22) + x43*cos(x22) - ng1*sin(x22) + wm1*sin(x22) - x41*sin(x22))/cos(x21)^2,                            -(sin(x21)*(ng1*cos(x22) - wm1*cos(x22) + x41*cos(x22) + ng3*sin(x22) - wm3*sin(x22) + x43*sin(x22)))/cos(x21),                                                                                                                                                                             0, 0, 0, 0, -(sin(x21)*sin(x22))/cos(x21), -1, (cos(x22)*sin(x21))/cos(x21),                                                0,                  0,                                                0,
     0, 0, 0,                  -(sin(x21)*(ng3*cos(x22) - wm3*cos(x22) + x43*cos(x22) - ng1*sin(x22) + wm1*sin(x22) - x41*sin(x22)))/cos(x21)^2,                                        (ng1*cos(x22) - wm1*cos(x22) + x41*cos(x22) + ng3*sin(x22) - wm3*sin(x22) + x43*sin(x22))/cos(x21),                                                                                                                                                                             0, 0, 0, 0,             sin(x22)/cos(x21),  0,           -cos(x22)/cos(x21),                                                0,                  0,                                                0,
     0, 0, 0, sin(x21)*sin(x23)*(na2 - am2 + x52) - cos(x21)*cos(x22)*sin(x23)*(na3 - am3 + x53) - cos(x21)*sin(x22)*sin(x23)*(na1 - am1 + x51),   (cos(x23)*sin(x22) - cos(x22)*sin(x21)*sin(x23))*(na1 - am1 + x51) + (cos(x22)*cos(x23) + sin(x21)*sin(x22)*sin(x23))*(na3 - am3 + x53), (cos(x22)*sin(x23) - cos(x23)*sin(x21)*sin(x22))*(na1 - am1 + x51) - (sin(x22)*sin(x23) + cos(x22)*cos(x23)*sin(x21))*(na3 - am3 + x53) - cos(x21)*cos(x23)*(na2 - am2 + x52), 0, 0, 0,                             0,  0,                            0, - cos(x22)*cos(x23) - sin(x21)*sin(x22)*sin(x23), -cos(x21)*sin(x23),   cos(x23)*sin(x22) - cos(x22)*sin(x21)*sin(x23),
     0, 0, 0, cos(x23)*sin(x21)*(na2 - am2 + x52) - cos(x21)*cos(x23)*sin(x22)*(na1 - am1 + x51) - cos(x21)*cos(x22)*cos(x23)*(na3 - am3 + x53), - (sin(x22)*sin(x23) + cos(x22)*cos(x23)*sin(x21))*(na1 - am1 + x51) - (cos(x22)*sin(x23) - cos(x23)*sin(x21)*sin(x22))*(na3 - am3 + x53), (cos(x22)*cos(x23) + sin(x21)*sin(x22)*sin(x23))*(na1 - am1 + x51) - (cos(x23)*sin(x22) - cos(x22)*sin(x21)*sin(x23))*(na3 - am3 + x53) + cos(x21)*sin(x23)*(na2 - am2 + x52), 0, 0, 0,                             0,  0,                            0,   cos(x22)*sin(x23) - cos(x23)*sin(x21)*sin(x22), -cos(x21)*cos(x23), - sin(x22)*sin(x23) - cos(x22)*cos(x23)*sin(x21),
     0, 0, 0,                            cos(x21)*(na2 - am2 + x52) + cos(x22)*sin(x21)*(na3 - am3 + x53) + sin(x21)*sin(x22)*(na1 - am1 + x51),                                                                 cos(x21)*sin(x22)*(na3 - am3 + x53) - cos(x21)*cos(x22)*(na1 - am1 + x51),                                                                                                                                                                             0, 0, 0, 0,                             0,  0,                            0,                               -cos(x21)*sin(x22),           sin(x21),                               -cos(x21)*cos(x22),
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

}

//Rotation from the camera frame to the IMU frame
Eigen::Matrix3d Rcam;
void odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    //your code for update
    //camera position in the IMU frame = (0, -0.04, -0.02)
    //camera orientaion in the IMU frame = Quaternion(0, 0, 1, 0); w x y z, respectively
    //					   RotationMatrix << -1, 0, 0,
    //							      0, 1, 0,
    //                                                        0, 0, -1;
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
