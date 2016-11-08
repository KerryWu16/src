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
//#include <Eigen/LU>
//EIgen SVD libnary, may help you solve SVD
//JacobiSVD<MatrixXd> svd(A, ComputeThinU | ComputeThinV);
#include <Eigen/Geometry> //h1.cross(h2);

#include <string.h>

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
    ROS_INFO("write your code here" ); 
    cout << "here is K:" << endl << K <<endl;
    
    //cout << "here is inverse of K" << endl << K.inverse() <<endl;
    //..................................................................................chongzi starts
    //...................At first find what kind of data we can use!
    int num_pts3_point=0;
    int num_pts2_point=0;
    int num_ptsid_point=0;
    num_pts3_point=pts_3.size();
    num_pts3_point=pts_2.size();
    num_ptsid_point=pts_id.size();
    ROS_INFO("number of (pts_3, pts_2, pts_id) is %ld %ld %ld .", num_pts3_point, num_pts3_point, num_ptsid_point);
    ROS_INFO("id of point 0      : %ld .", pts_id[0]);
    ROS_INFO("3D in world frame  : %4f %4f %4f .", pts_3[0].x, pts_3[0].y, pts_3[0].z); 
    ROS_INFO("2D in world picture: %4f %4f .", pts_2[0].x, pts_2[0].y);
    //...................Try to calculate the R and T! Use eigen!! T^T
    //choose 4 points
    int i;
    int size_id = pts_id.size();
    MatrixXd Po(2*size_id,9); //Use this kind of format to declare the Matrix (Dynamic declare)
    for(i=0; i<size_id ;i++)
    {
    cout<<"i" << endl << i <<endl;
    Po.row(2*i)   <<   pts_3[i].x, pts_3[i].y,    1,          0,          0,     0,  -pts_3[i].x*pts_2[i].x,  -pts_3[i].y*pts_2[i].x,  -pts_2[i].x;
    Po.row(2*i+1) <<            0,          0,    0, pts_3[i].x, pts_3[i].y,     1,  -pts_3[i].x*pts_2[i].y,  -pts_3[i].y*pts_2[i].y,  -pts_2[i].y;
    }  
    cout<<"Here" << endl << i <<endl;
    cout<<"Po" << endl << Po <<endl;
    JacobiSVD<MatrixXd> svd(Po, ComputeThinU | ComputeThinV);
    cout<<"svd.singularValues()" << endl << svd.singularValues() <<endl;
    cout<<"svd.matrixV()" << endl << svd.matrixV() <<endl;
    Matrix<double, 9,9> Vpo;
    Vpo=svd.matrixV();
    VectorXd Hp;
    //Hp = svd.solve(An);//get the answer of equation: Po*??=An; Because An={0},it does not works.
    //cout<< "Hp:" << endl << Hp <<endl;
    Hp=Vpo.col(8);
    cout<<" Po * Hp " << endl << Po * Hp <<endl;
    Matrix3d H;
    Matrix3d KH;
    Matrix3d Kin;
    Matrix3d nearR;
    Vector3d h1;
    Vector3d h2;
    Vector3d h3;
    H << Hp[0],  Hp[1], Hp[2],
         Hp[3],  Hp[4], Hp[5],
         Hp[6],  Hp[7], Hp[8];
    Kin << 0.0041,      0,     -0.7088,
                0, 0.0041,     -0.4678,
                0,      0,           1;
    //Kin = K.inverse(); //need #include <Eigen/LU>
    cout << "Kin" << endl << Kin <<endl;
    KH = Kin * H;//(the inverse of K) * H
    h1 << KH.col(0);
    h2 << KH.col(1);
    h3 << KH.col(2);
    nearR.col(0) << h1;
    nearR.col(1) << h2;
    nearR.col(2) << h1.cross(h2); //need #include <Eigen/Geometry>
    cout<< "nearR:" << endl << nearR <<endl;
    JacobiSVD<MatrixXd> svd2(nearR, ComputeThinU | ComputeThinV);
    R=svd2.matrixU() * svd2.matrixV().transpose();
    cout << "R" << endl << R <<endl;
    cout << "|R|" << endl << R.determinant() <<endl;
    cout << "h1" << endl << h1 <<endl;
    double abs_h1=h1.norm();
    cout << "abs_h1" << endl << abs_h1 <<endl;
    T= h3/abs_h1;
    cout << "T" << endl << T <<endl;
    
    //..................................................................................chongzi ends
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
