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
MatrixXd ut = MatrixXd::Identity(15, 1);
MatrixXd u_t(15,1);
MatrixXd Sigmat = MatrixXd::Identity(15, 15);
MatrixXd Sigma_t(15,15);
bool img_checktemp;
double dt;
double time_old;

//ros::Publisher pub_odom_ekfwork;

// Initially use a constant, later need to read from the environment


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

    // noise n: linear_acceleration_covariance, angular_velocity_covariance,
    //          acc bias noise, gyro bias noise
    //          assume the covariance is diagonalized and x, y, z are independent
    //          Given by the TA
        MatrixXd Sigmat_1(15,15);
	Sigmat_1 = Sigmat;
	MatrixXd ut_1(15,1);
	ut_1 = ut;

	    double x11= ut_1(0), x12= ut_1(1), x13= ut_1(2);
	    double x21= ut_1(3), x22= ut_1(4), x23= ut_1(5);
	    double x31= ut_1(6), x32= ut_1(7), x33= ut_1(8);
	    double x41= ut_1(9), x42= ut_1(10),x43= ut_1(11);
	    double x51= ut_1(12),x52= ut_1(13),x53= ut_1(14);
        double am1= msg->linear_acceleration.x;
        double am2= msg->linear_acceleration.y;
        double am3= msg->linear_acceleration.z;
        double wm1= msg->angular_velocity.x;
        double wm2= msg->angular_velocity.y;
        double wm3= msg->angular_velocity.z;
        double na1= Q(0, 0) , na2= Q(1, 1), na3= Q(2, 2);
        double ng1= Q(3, 3) , ng2= Q(4, 4), ng3= Q(5, 5);
        double nba1=Q(6, 6) , nba2=Q(7, 7), nba3=Q(8, 8);
        double nbg1=Q(9, 9) , nbg2=Q(10,10),nbg3=Q(11,11);
        //cout<< "linear_acceleration.x:" << endl<< am1 <<endl;
/****************************************get the dt from stemp***************************************/
if(!img_checktemp)
{
  time_old = msg->header.stamp.toSec()-0.03;
  img_checktemp=1;
}
dt = msg->header.stamp.toSec()-time_old;
time_old = msg->header.stamp.toSec();
       //cout <<"dt"<<endl<<dt<<endl;
/**************************************get the dt from stemp***************************************/
/********************************************f********************************************************/
	MatrixXd f(15,1);
      f<<     
x31,
                                                                                                                                                                                     x32,
                                                                                                                                                                                     x33,
                                                                       - (cos(x22)*(ng1 - wm1 + x41))/(cos(x22)*cos(x22) + sin(x22)*sin(x22)) - (sin(x22)*(ng3 - wm3 + x43))/(cos(x22)*cos(x22) + sin(x22)*sin(x22)),
 wm2 - ng2 - x42 - (sin(x21)*sin(x22)*(ng1 - wm1 + x41))/(cos(x21)*cos(x22)*cos(x22) + cos(x21)*sin(x22)*sin(x22)) + (cos(x22)*sin(x21)*(ng3 - wm3 + x43))/(cos(x21)*cos(x22)*cos(x22) + cos(x21)*sin(x22)*sin(x22)),
                                     (sin(x22)*(ng1 - wm1 + x41))/(cos(x21)*cos(x22)*cos(x22) + cos(x21)*sin(x22)*sin(x22)) - (cos(x22)*(ng3 - wm3 + x43))/(cos(x21)*cos(x22)*cos(x22) + cos(x21)*sin(x22)*sin(x22)),
           cos(x21)*sin(x22)*(na3 - am3 + x53) - (cos(x22)*sin(x23) + cos(x23)*sin(x21)*sin(x22))*(na2 - am2 + x52) - (cos(x22)*cos(x23) - sin(x21)*sin(x22)*sin(x23))*(na1 - am1 + x51),
                                                                                  cos(x21)*sin(x23)*(na1 - am1 + x51) - cos(x21)*cos(x23)*(na2 - am2 + x52) - sin(x21)*(na3 - am3 + x53),
 -982/100 - (sin(x22)*sin(x23) - cos(x22)*cos(x23)*sin(x21))*(na2 - am2 + x52) - cos(x21)*cos(x22)*(na3 - am3 + x53) - (cos(x23)*sin(x22) + cos(x22)*sin(x21)*sin(x23))*(na1 - am1 + x51),
                                                                                                                                                                                    nbg1,
                                                                                                                                                                                    nbg2,
                                                                                                                                                                                    nbg3,
                                                                                                                                                                                    nba1,
                                                                                                                                                                                    nba2,
                                                                                                                                                                                    nba3; 

/********************************************f********************************************************/
/********************************************At********************************************************/
MatrixXd At(15,15);
	At <<  0, 0, 0,                                                                                                                                       0,                                                                                                                                 0,                                                                                                                                                                             0, 1, 0, 0,                             0,  0,                            0,                  0,                                                0,                                                0,
           0, 0, 0,                                                                                                                                       0,                                                                                                                                 0,                                                                                                                                                                             0, 0, 1, 0,                             0,  0,                            0,                  0,                                                0,                                                0,
           0, 0, 0,                                                                                                                                       0,                                                                                                                                 0,                                                                                                                                                                             0, 0, 0, 1,                             0,  0,                            0,                  0,                                                0,                                                0,
           0, 0, 0,                                                                                                                                       0,                                           wm3*cos(x22) - ng3*cos(x22) - x43*cos(x22) + ng1*sin(x22) - wm1*sin(x22) + x41*sin(x22),                                                                                                                                                                             0, 0, 0, 0,                     -cos(x22),  0,                    -sin(x22),                  0,                                                0,                                                0,
           0, 0, 0,                                    (ng3*cos(x22) - wm3*cos(x22) + x43*cos(x22) - ng1*sin(x22) + wm1*sin(x22) - x41*sin(x22))/(cos(x21)*cos(x21)),                    -(sin(x21)*(ng1*cos(x22) - wm1*cos(x22) + x41*cos(x22) + ng3*sin(x22) - wm3*sin(x22) + x43*sin(x22)))/cos(x21),                                                                                                                                                                             0, 0, 0, 0, -(sin(x21)*sin(x22))/cos(x21), -1, (cos(x22)*sin(x21))/cos(x21),                  0,                                                0,                                                0,
           0, 0, 0,                        -(sin(x21)*(ng3*cos(x22) - wm3*cos(x22) + x43*cos(x22) - ng1*sin(x22) + wm1*sin(x22) - x41*sin(x22)))/(cos(x21)*cos(x21)),                                (ng1*cos(x22) - wm1*cos(x22) + x41*cos(x22) + ng3*sin(x22) - wm3*sin(x22) + x43*sin(x22))/cos(x21),                                                                                                                                                                             0, 0, 0, 0,             sin(x22)/cos(x21),  0,           -cos(x22)/cos(x21),                  0,                                                0,                                                0,
           0, 0, 0, (sin(x21)*sin(x23) - cos(x21)*cos(x23)*sin(x22))*(na2 - am2 + x52) - (cos(x21)*sin(x23) + cos(x23)*sin(x21)*sin(x22))*(na3 - am3 + x53), cos(x23)*sin(x22)*(na1 - am1 + x51) - cos(x22)*cos(x23)*sin(x21)*(na2 - am2 + x52) + cos(x21)*cos(x22)*cos(x23)*(na3 - am3 + x53), cos(x22)*sin(x23)*(na1 - am1 + x51) - (cos(x23)*sin(x21) + cos(x21)*sin(x22)*sin(x23))*(na3 - am3 + x53) - (cos(x21)*cos(x23) - sin(x21)*sin(x22)*sin(x23))*(na2 - am2 + x52), 0, 0, 0,                             0,  0,                            0, -cos(x22)*cos(x23), - cos(x21)*sin(x23) - cos(x23)*sin(x21)*sin(x22),   cos(x21)*cos(x23)*sin(x22) - sin(x21)*sin(x23),
           0, 0, 0, (cos(x23)*sin(x21) + cos(x21)*sin(x22)*sin(x23))*(na2 - am2 + x52) - (cos(x21)*cos(x23) - sin(x21)*sin(x22)*sin(x23))*(na3 - am3 + x53), cos(x22)*sin(x21)*sin(x23)*(na2 - am2 + x52) - cos(x21)*cos(x22)*sin(x23)*(na3 - am3 + x53) - sin(x22)*sin(x23)*(na1 - am1 + x51), (cos(x21)*sin(x23) + cos(x23)*sin(x21)*sin(x22))*(na2 - am2 + x52) + (sin(x21)*sin(x23) - cos(x21)*cos(x23)*sin(x22))*(na3 - am3 + x53) + cos(x22)*cos(x23)*(na1 - am1 + x51), 0, 0, 0,                             0,  0,                            0,  cos(x22)*sin(x23),   sin(x21)*sin(x22)*sin(x23) - cos(x21)*cos(x23), - cos(x23)*sin(x21) - cos(x21)*sin(x22)*sin(x23),
           0, 0, 0,                                                               cos(x21)*cos(x22)*(na2 - am2 + x52) + cos(x22)*sin(x21)*(na3 - am3 + x53),                            cos(x21)*sin(x22)*(na3 - am3 + x53) - cos(x22)*(na1 - am1 + x51) - sin(x21)*sin(x22)*(na2 - am2 + x52),                                                                                                                                                                             0, 0, 0, 0,                             0,  0,                            0,          -sin(x22),                                cos(x22)*sin(x21),                               -cos(x21)*cos(x22),
           0, 0, 0,                                                                                                                                       0,                                                                                                                                 0,                                                                                                                                                                             0, 0, 0, 0,                             0,  0,                            0,                  0,                                                0,                                                0,
           0, 0, 0,                                                                                                                                       0,                                                                                                                                 0,                                                                                                                                                                             0, 0, 0, 0,                             0,  0,                            0,                  0,                                                0,                                                0,
           0, 0, 0,                                                                                                                                       0,                                                                                                                                 0,                                                                                                                                                                             0, 0, 0, 0,                             0,  0,                            0,                  0,                                                0,                                                0,
           0, 0, 0,                                                                                                                                       0,                                                                                                                                 0,                                                                                                                                                                             0, 0, 0, 0,                             0,  0,                            0,                  0,                                                0,                                                0,
           0, 0, 0,                                                                                                                                       0,                                                                                                                                 0,                                                                                                                                                                             0, 0, 0, 0,                             0,  0,                            0,                  0,                                                0,                                                0,
           0, 0, 0,                                                                                                                                       0,                                                                                                                                 0,                                                                                                                                                                             0, 0, 0, 0,                             0,  0,                            0,                  0,                                                0,                                                0; 
/********************************************At********************************************************/
/********************************************Ut********************************************************/
MatrixXd Ut(15,12);
	Ut <<                   0,                                                0,                                                0,                             0,  0,                            0, 0, 0, 0, 0, 0, 0,
                            0,                                                0,                                                0,                             0,  0,                            0, 0, 0, 0, 0, 0, 0,
                            0,                                                0,                                                0,                             0,  0,                            0, 0, 0, 0, 0, 0, 0,
                            0,                                                0,                                                0,                     -cos(x22),  0,                    -sin(x22), 0, 0, 0, 0, 0, 0,
                            0,                                                0,                                                0, -(sin(x21)*sin(x22))/cos(x21), -1, (cos(x22)*sin(x21))/cos(x21), 0, 0, 0, 0, 0, 0,
                            0,                                                0,                                                0,             sin(x22)/cos(x21),  0,           -cos(x22)/cos(x21), 0, 0, 0, 0, 0, 0,
           -cos(x22)*cos(x23), - cos(x21)*sin(x23) - cos(x23)*sin(x21)*sin(x22),   cos(x21)*cos(x23)*sin(x22) - sin(x21)*sin(x23),                             0,  0,                            0, 0, 0, 0, 0, 0, 0,
            cos(x22)*sin(x23),   sin(x21)*sin(x22)*sin(x23) - cos(x21)*cos(x23), - cos(x23)*sin(x21) - cos(x21)*sin(x22)*sin(x23),                             0,  0,                            0, 0, 0, 0, 0, 0, 0,
                    -sin(x22),                                cos(x22)*sin(x21),                               -cos(x21)*cos(x22),                             0,  0,                            0, 0, 0, 0, 0, 0, 0,
                            0,                                                0,                                                0,                             0,  0,                            0, 0, 0, 0, 1, 0, 0,
                            0,                                                0,                                                0,                             0,  0,                            0, 0, 0, 0, 0, 1, 0,
                            0,                                                0,                                                0,                             0,  0,                            0, 0, 0, 0, 0, 0, 1,
                            0,                                                0,                                                0,                             0,  0,                            0, 1, 0, 0, 0, 0, 0,
                            0,                                                0,                                                0,                             0,  0,                            0, 0, 1, 0, 0, 0, 0,
                            0,                                                0,                                                0,                             0,  0,                            0, 0, 0, 1, 0, 0, 0;
/********************************************Ut********************************************************/
	//Ft=I+dt*At
	MatrixXd Ft(15,15);
	MatrixXd I15 = MatrixXd::Identity(15, 15);
	Ft = I15 + dt * At;
	//Vt=dt*Ut
	MatrixXd Vt(15,12);
	Vt = dt*Ut;

        // Prediction Step
        u_t = ut_1 + dt * f;
	//Sigma_t=Ft*Sigmat_1*Ft' + Vt*Q*Vt'
        Sigma_t = Ft * Sigmat_1 * Ft.adjoint() + Vt * Q * Vt.adjoint();

}

//Rotation from the camera frame to the IMU frame
Eigen::Matrix3d Rcam;
void odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    //your code for update
    //camera position in the IMU frame = (0, -0.04, -0.02)//from camera to IMU
    //camera orientaion in the IMU frame = Quaternion(0, 0, 1, 0); w x y z, respectively
    //			   RotationMatrix << -1, 0, 0,
    //							      0, 1, 0,
    //                                0, 0, -1;

    
/*    Rcam << -1   ,   0   ,   0   ,
            0    ,   1   ,   0   ,
            0    ,   0   ,   -1  ;
*/
    MatrixXd Tcam(3,1);
    Tcam << 0    , -0.04 , -0.02 ;//from camera to IMU
/********************************************Zt********************************************************/
    geometry_msgs::Pose camera_pose;
    camera_pose = msg->pose.pose;
    cout <<"Pose"<<endl<<camera_pose<<endl;
    geometry_msgs::Quaternion q_wi = camera_pose.orientation;
    cout <<"Quaternion"<<endl<<q_wi<<endl;
    MatrixXd CamZt(6,1);//camera position in world frame. What we need is the IMU orientation in world frame.
    CamZt(0) = camera_pose.position.x;
    CamZt(1) = camera_pose.position.y;
    CamZt(2) = camera_pose.position.z;
    
    // Quaternino -> rotation matrix -> ZXY Euler
    // camera orientation in world frame. What we need is the IMU orientation in world frame.
    /*CamZt(3) = asin(2*(q_wi.y * q_wi.z + q_wi.x * q_wi.w));
    CamZt(4) = acos((1 - 2 * pow(q_wi.x,2) - 2 * pow(q_wi.y,2)) / cos(CamZt(3))); // pitch_wi
    CamZt(5) = acos((1 - 2 * pow(q_wi.x,2) - 2 * pow(q_wi.z,2))  / cos(CamZt(3))); // yaw_wi
    */
    MatrixXd cRw(3,3);//from world to camera
    Vector3d cTw;     
    MatrixXd wRc(3,3);
    //Vector3d wTc;
    MatrixXd iRw(3,3);
    //Vector3d iTw;
    MatrixXd wRi(3,3);
    Vector3d wTi;
    MatrixXd iRc(3,3);
    Vector3d iTc;    //from camera to IMU
    MatrixXd cRi(3,3);
    //Vector3d cTi;
    cRw = Quaterniond(q_wi.x,q_wi.y,q_wi.z,q_wi.w).toRotationMatrix();//CRW from world frame to camera frame.
    cTw <<     CamZt(0), CamZt(1), CamZt(2) ;
    wRc = cRw.adjoint();
    iRc = Rcam;
    cRi = iRc.adjoint();
    iTc = Tcam;
    wRi = wRc * cRi;
    iRw = wRi.adjoint();
    wTi = -1 * wRc * cRw *iTc + wRc * cTw;                                       
    MatrixXd ZR(3,3);
    ZR = wRi; 
    cout << "wRi" << endl << ZR <<endl; 
    //cout << "|ZR|" << endl << ZR.determinant() <<endl;   
    /*cout << "Angles" << endl << ZR.eulerAngles(1,0,2) <<endl;  */ 
    //Vector3d Angle = ZR.eulerAngles(2, 0, 1);

    MatrixXd Zt(6,1);
    Zt(0) = CamZt(0) - Tcam(0);
    Zt(1) = CamZt(1) - Tcam(1);
    Zt(2) = CamZt(2) - Tcam(2); 
 
    double roll = asin(ZR(1,2));
    double pitch = atan(-ZR(0,2)/ZR(2,2));
    double yaw = atan(-ZR(1,0)/ZR(1,1));

    Zt(3) = roll;
    Zt(4) = pitch;
    Zt(5) = yaw;
    cout <<"Zt"<<endl<<Zt<<endl; //Zt is ok. I have see it in rviz.
    
/********************************************Zt********************************************************/

   MatrixXd g(6,1);
	g << u_t(0) , u_t(1), u_t(2), u_t(3), u_t(4), u_t(5);

   MatrixXd Ct(6,15);
	Ct <<      1  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,
	           0  ,  1  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,
	           0  ,  0  ,  1  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,
		       0  ,  0  ,  0  ,  1  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,
		       0  ,  0  ,  0  ,  0  ,  1  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,
		       0  ,  0  ,  0  ,  0  ,  0  ,  1  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ;
    
	//Wt=
	MatrixXd Wt = MatrixXd::Identity(6,6);
	//Kt=Sigma_t * Ct' * (Ct * Sigma_t * Ct' + Wt * R *Wt')
    MatrixXd Kt(15,6);
    MatrixXd MidMatrix(6,6);
    MidMatrix = Ct * Sigma_t * Ct.adjoint() + Wt * Rt * Wt.adjoint();
	Kt = Sigma_t * Ct.adjoint() *(MidMatrix.inverse());
	//ut=u_t + 
	ut = u_t + Kt * (Zt - g);
	Sigmat = Sigma_t - Kt * Ct * Sigma_t;
        cout <<"ut"<<endl<<ut<<endl; 


 
    

    AngleAxisd rollAngle(Zt(3), Vector3d::UnitX());
    AngleAxisd pitchAngle(Zt(4), Vector3d::UnitY());
    AngleAxisd yawAngle(Zt(5), Vector3d::UnitZ());
  

    Quaternion<double> Q_ekfwork = yawAngle * rollAngle * pitchAngle;

    nav_msgs::Odometry odom_ekfwork;
    odom_ekfwork.header.stamp = msg->header.stamp;
    odom_ekfwork.header.frame_id = "world";
    odom_ekfwork.pose.pose.position.x = Zt(0);
    odom_ekfwork.pose.pose.position.y = Zt(1);
    odom_ekfwork.pose.pose.position.z = Zt(2);
    odom_ekfwork.pose.pose.orientation.w = Q_ekfwork.w();
    odom_ekfwork.pose.pose.orientation.x = Q_ekfwork.x();
    odom_ekfwork.pose.pose.orientation.y = Q_ekfwork.y();
    odom_ekfwork.pose.pose.orientation.z = Q_ekfwork.z();
    odom_pub.publish(odom_ekfwork);  
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ekf");
    ros::NodeHandle n("~");
    ros::Subscriber s1 = n.subscribe("imu", 1000, imu_callback);
    ros::Subscriber s2 = n.subscribe("tag_odom", 1000, odom_callback);
    odom_pub = n.advertise<nav_msgs::Odometry>("ekf_odom", 100);
    //pub_odom_ekfwork = n.advertise<nav_msgs::Odometry>("odom_ekfwork",10);
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