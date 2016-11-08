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
    Matrix3d KH;
    int pointsNumber = pts_id.size();

    MatrixXd P0(2*pointsNumber, 9);
    MatrixXd P(8,9);
    vector<cv::Point2f> un_pts_2;
    cv::undistortPoints(pts_2, un_pts_2, K, D);
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
    // Using the top four points to SVD first, then use all of the points
    // TODO: later using any four points of SVD, and eliminate the outliers
    //       Then find the optimal
    JacobiSVD<MatrixXd> svd(P, ComputeThinU | ComputeThinV);
    MatrixXd V = svd.matrixV();
    VectorXd Hf = V.col(8);
    H.row(0) = Hf.segment(0,2).transpose();
    H.row(1) = Hf.segment(3,5).transpose();
    H.row(2) = Hf.segment(6,8).transpose();
    // Matrix3d H = Map<Matrix3d>(Hf.data(), Hf.size());
    // cv::Mat K_inv = K.inv();
    // Matrix3d K_inv_Eigen = Map<Matrix3d>(K_inv.data());
    #ifdef DEBUG
        cout<<"Checking if mapping from Hf to H is correct:" << endl;
        cout<<"H: " << endl << H <<endl;
        cout<<"Hf:" << endl << Hf <<endl;
        // cout<<"Checking if mapping from cv::Mat to Eigen::Matrix is correct:" << endl;
        // cout<<"K_inv: " << endl << K_inv <<endl;
        // cout<<"K_inv_Eigen:" << endl << K_inv_Eigen <<endl;
    #endif

    // KH = K_inv_Eigen * H;
    KH = H;

    // Find the orthogonal matrix R, with the estimate h1_bar and h2_bar
    Vector3d h1_bar;
    Vector3d h2_bar;
    Vector3d h3_bar;
    Vector3d h12_bar_cross;
    h1_bar = KH.col(0);
    h2_bar = KH.col(1);
    h3_bar = KH.col(2);
    h12_bar_cross = h1_bar.cross(h2_bar);
    Matrix3d R_approx;
    R_approx << h1_bar, h2_bar, h12_bar_cross;
    JacobiSVD<MatrixXd> svd_min(R_approx, ComputeThinU | ComputeThinV);
    R = svd_min.matrixU() * svd_min.matrixV().transpose();

    double h1_norm = h1_bar.norm();
    T = h3_bar / h1_norm;

    // Since the direction of the camera depends on the z axis
    // Normalize them
    if (T(2) < 0) {
        T = -T;
        R(0) = -R(0);
        R(1) = -R(1);
    }
    //...
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
