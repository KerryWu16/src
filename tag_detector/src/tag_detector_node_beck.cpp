#include <iostream>

#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/Odometry.h>
#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>
#include <Eigen/SVD>
// Added dependency for inverse and cross product
#include <Eigen/Geometry>
//EIgen SVD libnary, may help you solve SVD
//JacobiSVD<MatrixXd> svd(A, ComputeThinU | ComputeThinV);

#define DEBUG false

using namespace cv;
using namespace aruco;
using namespace Eigen;

//global varialbles for aruco detector
aruco::CameraParameters CamParam;
MarkerDetector MDetector;
vector<Marker> Markers;
float MarkerSize = 0.20 / 1.5 * 1.524;
float MarkerWithMargin = MarkerSize * 1.2;
BoardConfiguration TheBoardConfig;
BoardDetector TheBoardDetector;
Board TheBoardDetected;
ros::Publisher pub_odom_yourwork;
ros::Publisher pub_odom_ref;
cv::Mat K, D;

// Beck's insert for non-linear 3D-2D pose estimation
Vector3d euler_past; // Euler angle in ZYX from last frame
Vector3d T_rec_past; // recursive T from last frame

// test function, can be used to verify your estimation
void calculateReprojectionError(const vector<cv::Point3f> &pts_3, const vector<cv::Point2f> &pts_2, const cv::Mat R, const cv::Mat t)
{
    puts("calculateReprojectionError begins");
    vector<cv::Point2f> un_pts_2;
    cv::undistortPoints(pts_2, un_pts_2, K, D);
    for (unsigned int i = 0; i < pts_3.size(); i++)
    {
        cv::Mat p_mat(3, 1, CV_64FC1);
        p_mat.at<double>(0, 0) = pts_3[i].x;
        p_mat.at<double>(1, 0) = pts_3[i].y;
        p_mat.at<double>(2, 0) = pts_3[i].z;
        cv::Mat p = (R * p_mat + t);
        printf("(%f, %f, %f) -> (%f, %f) and (%f, %f)\n",
               pts_3[i].x, pts_3[i].y, pts_3[i].z,
               un_pts_2[i].x, un_pts_2[i].y,
               p.at<double>(0) / p.at<double>(2), p.at<double>(1) / p.at<double>(2));
    }
    puts("calculateReprojectionError ends");
}

// the main function you need to work with
// pts_id: id of each point
// pts_3: 3D position (x, y, z) in world frame
// pts_2: 2D position (u, v) in image frame
void process(const vector<int> &pts_id, const vector<cv::Point3f> &pts_3, const vector<cv::Point2f> &pts_2, const ros::Time& frame_time)
{
    //version 1, as reference
    cv::Mat r, rvec, t;
    cv::solvePnP(pts_3, pts_2, K, D, rvec, t);
    cv::Rodrigues(rvec, r);
    Matrix3d R_ref;
    for(int i=0;i<3;i++)
        for(int j=0;j<3;j++)
        {
            R_ref(i,j) = r.at<double>(i, j);
        }
    Quaterniond Q_ref;
    Q_ref = R_ref;
    nav_msgs::Odometry odom_ref;
    odom_ref.header.stamp = frame_time;
    odom_ref.header.frame_id = "world";
    odom_ref.pose.pose.position.x = t.at<double>(0, 0);
    odom_ref.pose.pose.position.y = t.at<double>(1, 0);
    odom_ref.pose.pose.position.z = t.at<double>(2, 0);
    odom_ref.pose.pose.orientation.w = Q_ref.w();
    odom_ref.pose.pose.orientation.x = Q_ref.x();
    odom_ref.pose.pose.orientation.y = Q_ref.y();
    odom_ref.pose.pose.orientation.z = Q_ref.z();
    pub_odom_ref.publish(odom_ref);

    // version 2, your work
    Matrix3d R;
    Vector3d T;
    R.setIdentity();
    T.setZero();
    ROS_INFO("write your code here!");
    // Version 1 Linear 3D-2D pose estimation on planar scene
    // H = K (r1, r2, t), Hf stands for H flat
    // Using four points to build the H first
    // preprocessing the points
    vector<cv::Point2f> un_pts_2;
    cv::undistortPoints(pts_2, un_pts_2, K, D);
    int pointsNumber = pts_2.size();

    MatrixXd P0(2*pointsNumber, 9);
    MatrixXd P(8,9);

    for (int i = 0; i < pointsNumber; i++) {
        float X, Y, u, v;
        X = pts_3[i].x;
        Y = pts_3[i].y;
        u = un_pts_2[i].x;
        v = un_pts_2[i].y;
        P0.row(2*i  ) << X, Y, 1, 0, 0, 0, -X*u, -Y*u, -u;
        P0.row(2*i+1) << 0, 0, 0, X, Y, 1, -X*v, -Y*v, -v;
    }
    P = P0.topRows(8);

    // Using all of the points to find the linear pose estimation
    JacobiSVD<MatrixXd> svd(P0, ComputeThinU | ComputeThinV);

    MatrixXd V(9,9);
    V = svd.matrixV();

    VectorXd Hf = V.col(8);
    Matrix3d H;

    H << Hf(0), Hf(1), Hf(2),
         Hf(3), Hf(4), Hf(5),
         Hf(6), Hf(7), Hf(8);

    // Find the orthogonal matrix R, with the estimate h1_bar and h2_bar
    Vector3d h1_bar;
    Vector3d h2_bar;
    Vector3d h3_bar;
    Vector3d h12_bar_cross;
    h1_bar = H.col(0);
    h2_bar = H.col(1);
    h3_bar = H.col(2);
    h12_bar_cross = h1_bar.cross(h2_bar);
    Matrix3d R_approx;
    R_approx << h1_bar, h2_bar, h12_bar_cross;
    JacobiSVD<MatrixXd> svd_min(R_approx, ComputeThinU | ComputeThinV);

    Matrix3d R_linear;
    Vector3d T_linear;
    R_linear = svd_min.matrixU() * svd_min.matrixV().transpose();

    double h1_norm = h1_bar.norm();
    T_linear = h3_bar / h1_norm;

    // Since the direction of the camera depends on the z axis
    // Normalize themt
    if (T_linear(2) < 0) {
        T_linear = -T_linear;
        R_linear.col(0) = -R_linear.col(0);
        R_linear.col(1) = -R_linear.col(1);
    }

    // Second phase: using nonlinear 3D-2D Newton Method
    Vector2d b_sum;
    MatrixXd A_sum(2,6);

    Vector3d euler_init;// Euler angle in ZYX, initial
    Vector3d T_init;  // initial T
    Vector3d euler;// Euler angle in ZYX, initial
    Vector3d T_rec;  // recursive T

    euler_init = R_linear.eulerAngle(2,1,0); // Euler angle in ZYX
    T_init = T_linear；
    // Step 1: compute the initial Euler angle and T
    for (int i = 0; i < pointsNumber; i++) {
        Vector2d f0;
        Vector3d p0 << pts_3[i].x, pts_3[i].y, pts_3[i].z;
        Vector3d pos3d_rot = R_linear * p0 + T_linear;
        Vector2d pos3d_rot_2d;
        pos3d_rot_2d << pos3d_rot(0) / pos3d_rot(2), pos3d_rot(1) / pos3d_rot(2); // x/z, y/z
        f0 << un_pts_2[i].x - pos3d_rot_2d(0), un_pts_2[i].y - pos3d_rot_2d(1);
        b_sum += f0;
    }

    // Step 2: get the derivative matrix from Matlab, and get the A
    for (int i = 0; i < pointsNumber; i++) {
        MatrixXd Jacobian(2,6);
        R =

        [ cos(a)*cos(b), cos(a)*sin(b)*sin(y) - sin(a)*cos(y), sin(a)*sin(y) + cos(a)*sin(b)*cos(y)]
        [ cos(b)*sin(a), cos(a)*cos(y) + sin(a)*sin(b)*sin(y), sin(a)*sin(b)*cos(y) - cos(a)*sin(y)]
        [       -sin(b),                        cos(b)*sin(y),                        cos(b)*cos(y)]


        dRda =

        [ -cos(b)*sin(a), - cos(a)*cos(y) - sin(a)*sin(b)*sin(y), cos(a)*sin(y) - sin(a)*sin(b)*cos(y)]
        [  cos(a)*cos(b),   cos(a)*sin(b)*sin(y) - sin(a)*cos(y), sin(a)*sin(y) + cos(a)*sin(b)*cos(y)]
        [              0,                                      0,                                    0]


        dRdb =

        [ -cos(a)*sin(b), cos(a)*cos(b)*sin(y), cos(a)*cos(b)*cos(y)]
        [ -sin(a)*sin(b), cos(b)*sin(a)*sin(y), cos(b)*sin(a)*cos(y)]
        [        -cos(b),       -sin(b)*sin(y),       -sin(b)*cos(y)]


        dRdy =

        [ 0, sin(a)*sin(y) + cos(a)*sin(b)*cos(y),   sin(a)*cos(y) - cos(a)*sin(b)*sin(y)]
        [ 0, sin(a)*sin(b)*cos(y) - cos(a)*sin(y), - cos(a)*cos(y) - sin(a)*sin(b)*sin(y)]
        [ 0,                        cos(b)*cos(y),                         -cos(b)*sin(y)]  
    }
    // Step 3: calculate the increment
    VectorXd increment(6,1);
    increment = b_sum / A_sum;
    euler += increment.segment(0,2);
    T_rec += increment.segment(3,5);

    // Step 4: convert the incremented euler angle to rotation matrix
    double roll = euler(0);
    double yaw  = euler(1);
    double pitch= euler(2);
    AngleAxisd rollAngle(roll, Vector3d::UnitZ());
    AngleAxisd yawAngle(yaw, Vector3d::UnitY());
    AngleAxisd pitchAngle(pitch, Vector3d::UnitX());

    Quaternion<double> quaternion = rollAngle * yawAngle * pitchAngle;

    Matrix3d rotationMatrix = quaternion.matrix();

    R = rotationMatrix;
    T = T_rec;

    Quaterniond Q_yourwork;
    Q_yourwork = R;
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
    pub_odom_yourwork.publish(odom_yourwork);
}

cv::Point3f getPositionFromIndex(int idx, int nth)
{
    int idx_x = idx % 6, idx_y = idx / 6;
    double p_x = idx_x * MarkerWithMargin - (3 + 2.5 * 0.2) * MarkerSize;
    double p_y = idx_y * MarkerWithMargin - (12 + 11.5 * 0.2) * MarkerSize;
    return cv::Point3f(p_x + (nth == 1 || nth == 2) * MarkerSize, p_y + (nth == 2 || nth == 3) * MarkerSize, 0.0);
}

void img_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    double t = clock();
    cv_bridge::CvImagePtr bridge_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);
    MDetector.detect(bridge_ptr->image, Markers);
    float probDetect = TheBoardDetector.detect(Markers, TheBoardConfig, TheBoardDetected, CamParam, MarkerSize);
    ROS_DEBUG("p: %f, time cost: %f\n", probDetect, (clock() - t) / CLOCKS_PER_SEC);

    vector<int> pts_id;
    vector<cv::Point3f> pts_3;
    vector<cv::Point2f> pts_2;
    for (unsigned int i = 0; i < Markers.size(); i++)
    {
        int idx = TheBoardConfig.getIndexOfMarkerId(Markers[i].id);

        char str[100];
        sprintf(str, "%d", idx);
        cv::putText(bridge_ptr->image, str, Markers[i].getCenter(), CV_FONT_HERSHEY_COMPLEX, 0.4, cv::Scalar(-1));
        for (unsigned int j = 0; j < 4; j++)
        {
            sprintf(str, "%d", j);
            cv::putText(bridge_ptr->image, str, Markers[i][j], CV_FONT_HERSHEY_COMPLEX, 0.4, cv::Scalar(-1));
        }

        for (unsigned int j = 0; j < 4; j++)
        {
            pts_id.push_back(Markers[i].id * 4 + j);
            pts_3.push_back(getPositionFromIndex(idx, j));
            pts_2.push_back(Markers[i][j]);
        }
    }

    //begin your function
    if (pts_id.size() > 5)
        process(pts_id, pts_3, pts_2, img_msg->header.stamp);

    cv::imshow("in", bridge_ptr->image);
    cv::waitKey(10);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tag_detector");
    ros::NodeHandle n("~");

    ros::Subscriber sub_img = n.subscribe("image_raw", 100, img_callback);
    pub_odom_yourwork = n.advertise<nav_msgs::Odometry>("odom_yourwork",10);
    pub_odom_ref = n.advertise<nav_msgs::Odometry>("odom_ref",10);
    //init aruco detector
    string cam_cal, board_config;
    n.getParam("cam_cal_file", cam_cal);
    n.getParam("board_config_file", board_config);
    CamParam.readFromXMLFile(cam_cal);
    TheBoardConfig.readFromFile(board_config);

    //init intrinsic parameters
    cv::FileStorage param_reader(cam_cal, cv::FileStorage::READ);
    param_reader["camera_matrix"] >> K;
    param_reader["distortion_coefficients"] >> D;

    //init window for visualization
    cv::namedWindow("in", 1);

    ros::spin();
}
