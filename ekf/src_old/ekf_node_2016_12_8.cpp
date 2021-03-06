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
ros::Publisher pub_odom_ekf_obs;
MatrixXd Q = MatrixXd::Identity(12, 12); // For propogate, IMU noise
MatrixXd Rt = MatrixXd::Identity(6,6);   // For update, Odometry noise
MatrixXd ut = MatrixXd::Identity(15, 1);
MatrixXd u_t(15,1);
MatrixXd Sigmat = MatrixXd::Identity(15, 15);
MatrixXd Sigma_t(15,15);
MatrixXd Zt_1 = MatrixXd::Identity(6, 1);

bool img_checktemp;
double dt;
double time_old;



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
/****************************************get the dt from stemp***************************************/
if(!img_checktemp)
{
  time_old = msg->header.stamp.toSec()-0.03;
  ut.setZero();
  Sigmat.setIdentity();
}
dt = msg->header.stamp.toSec()-time_old;
time_old = msg->header.stamp.toSec();
       cout <<"dt"<<endl<<dt<<endl;
/**************************************get the dt from stemp***************************************/
        MatrixXd Sigmat_1(15,15);
    Sigmat_1 = Sigmat;
        cout <<"ut"<<endl<<ut<<endl;
    MatrixXd ut_1 = MatrixXd::Identity(15, 1);
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
        double na1= 0 , na2= 0, na3= 0;
        double ng1= 0 , ng2= 0, ng3= 0;
        double nba1=0 , nba2=0, nba3=0;
        double nbg1=0 , nbg2=0, nbg3=0;
        //cout<< "linear_acceleration.x:" << endl<< am1 <<endl;

/********************************************f********************************************************/
    MatrixXd f(15,1);
      f<<     
x31,
                                                                                                                                                                                                    x32,
                                                                                                                                                                                                    x33,
                                                                                                                wm1*cos(x22) - ng1*cos(x22) - x41*cos(x22) - ng3*sin(x22) + wm3*sin(x22) - x43*sin(x22),
 -(ng2*cos(x21) - wm2*cos(x21) + x42*cos(x21) - ng3*cos(x22)*sin(x21) + wm3*cos(x22)*sin(x21) - x43*cos(x22)*sin(x21) + ng1*sin(x21)*sin(x22) - wm1*sin(x21)*sin(x22) + x41*sin(x21)*sin(x22))/cos(x21),
                                                                                                    -(ng3*cos(x22) - wm3*cos(x22) + x43*cos(x22) - ng1*sin(x22) + wm1*sin(x22) - x41*sin(x22))/cos(x21),
                          (cos(x23)*sin(x22) - cos(x22)*sin(x21)*sin(x23))*(na3 - am3 + x53) - (cos(x22)*cos(x23) + sin(x21)*sin(x22)*sin(x23))*(na1 - am1 + x51) - cos(x21)*sin(x23)*(na2 - am2 + x52),
                          (cos(x22)*sin(x23) - cos(x23)*sin(x21)*sin(x22))*(na1 - am1 + x51) - (sin(x22)*sin(x23) + cos(x22)*cos(x23)*sin(x21))*(na3 - am3 + x53) - cos(x21)*cos(x23)*(na2 - am2 + x52),
                                                                                       sin(x21)*(na2 - am2 + x52) - cos(x21)*cos(x22)*(na3 - am3 + x53) - cos(x21)*sin(x22)*(na1 - am1 + x51) - 982/100,
                                                                                                                                                                                                   nbg1,
                                                                                                                                                                                                   nbg2,
                                                                                                                                                                                                   nbg3,
                                                                                                                                                                                                   nba1,
                                                                                                                                                                                                   nba2,
                                                                                                                                                                                                   nba3;

/********************************************f********************************************************/
/********************************************At********************************************************/
MatrixXd At(15,15);
    At <<   0, 0, 0,                                                                                                                                 0,                                                                                                                                         0,                                                                                                                                                                             0, 1, 0, 0,                             0,  0,                            0,                                                0,                  0,                                                0,
 0, 0, 0,                                                                                                                                 0,                                                                                                                                         0,                                                                                                                                                                             0, 0, 1, 0,                             0,  0,                            0,                                                0,                  0,                                                0,
 0, 0, 0,                                                                                                                                 0,                                                                                                                                         0,                                                                                                                                                                             0, 0, 0, 1,                             0,  0,                            0,                                                0,                  0,                                                0,
 0, 0, 0,                                                                                                                                 0,                                                   wm3*cos(x22) - ng3*cos(x22) - x43*cos(x22) + ng1*sin(x22) - wm1*sin(x22) + x41*sin(x22),                                                                                                                                                                             0, 0, 0, 0,                     -cos(x22),  0,                    -sin(x22),                                                0,                  0,                                                0,
 0, 0, 0,                              (ng3*cos(x22) - wm3*cos(x22) + x43*cos(x22) - ng1*sin(x22) + wm1*sin(x22) - x41*sin(x22))/(cos(x21)*cos(x21)),                            -(sin(x21)*(ng1*cos(x22) - wm1*cos(x22) + x41*cos(x22) + ng3*sin(x22) - wm3*sin(x22) + x43*sin(x22)))/cos(x21),                                                                                                                                                                             0, 0, 0, 0, -(sin(x21)*sin(x22))/cos(x21), -1, (cos(x22)*sin(x21))/cos(x21),                                                0,                  0,                                                0,
 0, 0, 0,                  -(sin(x21)*(ng3*cos(x22) - wm3*cos(x22) + x43*cos(x22) - ng1*sin(x22) + wm1*sin(x22) - x41*sin(x22)))/(cos(x21)*cos(x21)),                                        (ng1*cos(x22) - wm1*cos(x22) + x41*cos(x22) + ng3*sin(x22) - wm3*sin(x22) + x43*sin(x22))/cos(x21),                                                                                                                                                                             0, 0, 0, 0,             sin(x22)/cos(x21),  0,           -cos(x22)/cos(x21),                                                0,                  0,                                                0,
 0, 0, 0, sin(x21)*sin(x23)*(na2 - am2 + x52) - cos(x21)*cos(x22)*sin(x23)*(na3 - am3 + x53) - cos(x21)*sin(x22)*sin(x23)*(na1 - am1 + x51),   (cos(x23)*sin(x22) - cos(x22)*sin(x21)*sin(x23))*(na1 - am1 + x51) + (cos(x22)*cos(x23) + sin(x21)*sin(x22)*sin(x23))*(na3 - am3 + x53), (cos(x22)*sin(x23) - cos(x23)*sin(x21)*sin(x22))*(na1 - am1 + x51) - (sin(x22)*sin(x23) + cos(x22)*cos(x23)*sin(x21))*(na3 - am3 + x53) - cos(x21)*cos(x23)*(na2 - am2 + x52), 0, 0, 0,                             0,  0,                            0, - cos(x22)*cos(x23) - sin(x21)*sin(x22)*sin(x23), -cos(x21)*sin(x23),   cos(x23)*sin(x22) - cos(x22)*sin(x21)*sin(x23),
 0, 0, 0, cos(x23)*sin(x21)*(na2 - am2 + x52) - cos(x21)*cos(x23)*sin(x22)*(na1 - am1 + x51) - cos(x21)*cos(x22)*cos(x23)*(na3 - am3 + x53), - (sin(x22)*sin(x23) + cos(x22)*cos(x23)*sin(x21))*(na1 - am1 + x51) - (cos(x22)*sin(x23) - cos(x23)*sin(x21)*sin(x22))*(na3 - am3 + x53), (cos(x22)*cos(x23) + sin(x21)*sin(x22)*sin(x23))*(na1 - am1 + x51) - (cos(x23)*sin(x22) - cos(x22)*sin(x21)*sin(x23))*(na3 - am3 + x53) + cos(x21)*sin(x23)*(na2 - am2 + x52), 0, 0, 0,                             0,  0,                            0,   cos(x22)*sin(x23) - cos(x23)*sin(x21)*sin(x22), -cos(x21)*cos(x23), - sin(x22)*sin(x23) - cos(x22)*cos(x23)*sin(x21),
 0, 0, 0,                            cos(x21)*(na2 - am2 + x52) + cos(x22)*sin(x21)*(na3 - am3 + x53) + sin(x21)*sin(x22)*(na1 - am1 + x51),                                                                 cos(x21)*sin(x22)*(na3 - am3 + x53) - cos(x21)*cos(x22)*(na1 - am1 + x51),                                                                                                                                                                             0, 0, 0, 0,                             0,  0,                            0,                               -cos(x21)*sin(x22),           sin(x21),                               -cos(x21)*cos(x22),
 0, 0, 0,                                                                                                                                 0,                                                                                                                                         0,                                                                                                                                                                             0, 0, 0, 0,                             0,  0,                            0,                                                0,                  0,                                                0,
 0, 0, 0,                                                                                                                                 0,                                                                                                                                         0,                                                                                                                                                                             0, 0, 0, 0,                             0,  0,                            0,                                                0,                  0,                                                0,
 0, 0, 0,                                                                                                                                 0,                                                                                                                                         0,                                                                                                                                                                             0, 0, 0, 0,                             0,  0,                            0,                                                0,                  0,                                                0,
 0, 0, 0,                                                                                                                                 0,                                                                                                                                         0,                                                                                                                                                                             0, 0, 0, 0,                             0,  0,                            0,                                                0,                  0,                                                0,
 0, 0, 0,                                                                                                                                 0,                                                                                                                                         0,                                                                                                                                                                             0, 0, 0, 0,                             0,  0,                            0,                                                0,                  0,                                                0,
 0, 0, 0,                                                                                                                                 0,                                                                                                                                         0,                                                                                                                                                                             0, 0, 0, 0,                             0,  0,                            0,                                                0,                  0,                                                0;
 
 
/********************************************At********************************************************/
/********************************************Ut********************************************************/
MatrixXd Ut(15,12);
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
 
/********************************************Ut********************************************************/
    //Ft=I+dt*At
    MatrixXd Ft(15,15);
    MatrixXd I15 = MatrixXd::Identity(15, 15);
    Ft = I15 + dt * At;
    //Vt=dt*Ut
    MatrixXd Vt(15,12);
    Vt = dt*Ut;

        // Prediction Step
if(!img_checktemp)
{
        u_t = dt * f;
        img_checktemp=1;
}
else
{
        u_t = ut_1 + dt * f;
}



        //cout <<"ut_1 in prediction:" << endl << ut_1 <<endl; 
        //cout <<"u_t in prediction:" << endl << u_t <<endl; 
    //Sigma_t=Ft*Sigmat_1*Ft' + Vt*Q*Vt'
        Sigma_t = Ft * Sigmat_1 * Ft.transpose() + Vt * Q * Vt.transpose();
/*Mu_bar     = Mu_prev + dt * process_model;
    //get Sigma_bar
    Ft         = MatrixXd::Identity(15, 15) + dt * At;
    Vt         = dt * Ut;
    Sigma_bar  = Ft * Sigma_prev * Ft.transpose() + Vt * Q * Vt.transpose();*/
}

//Rotation from the camera frame to the IMU frame
Eigen::Matrix3d Rcam;
void odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    //your code for update
    //camera position in the IMU frame = (0, -0.04, -0.02)//from camera to IMU
    //camera orientaion in the IMU frame = Quaternion(0, 0, 1, 0); w x y z, respectively
    //             RotationMatrix << -1, 0, 0,
    //                                0, 1, 0,
    //                                0, 0, -1;

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
    /*MatrixXd cRw(3,3);//from world to camera
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
    cRw = Quaterniond(q_wi.w,q_wi.x,q_wi.y,q_wi.z).toRotationMatrix();//CRW from world frame to camera frame.
    cTw <<     CamZt(0), CamZt(1), CamZt(2) ;
    wRc = cRw.inverse();
    iRc = Rcam;
    cRi = iRc.inverse();
    iTc = Tcam;
    wRi = wRc * cRi;
    iRw = wRi.inverse();
    wTi = -1 * wRc * cRi *iTc - wRc * cTw;                                        
    MatrixXd ZR(3,3);
    ZR = wRi; 
    cout << "wRc" << endl << wRc <<endl; 
    cout << "cRi" << endl << cRi <<endl;
    cout << "wRi" << endl << wRi <<endl;
    //cout << "|ZR|" << endl << ZR.determinant() <<endl;   
    //Vector3d Angle = ZR.eulerAngles(2, 0, 1);

    
    MatrixXd Zt(6,1);
    Zt(0) = wTi(0);
    Zt(1) = wTi(1);
    Zt(2) = wTi(2);
/************************2333*************************/
    //Quaternion<double> Q_ekf_obs= Quaterniond(q_wi.y,-q_wi.z,q_wi.w,q_wi.x).toRotationMatrix();
   /*geometry_msgs::Quaternion Q_ekf_obs;
    Q_ekf_obs.w = q_wi.y();
    Q_ekf_obs.x = -1*q_wi.z();
    Q_ekf_obs.y = q_wi.w();
    Q_ekf_obs.z = q_wi.x(); 
/************************2333*************************/
    //float q0=(float)Q_ekf_obs.w();
    //float q1=(float)Q_ekf_obs.x();
    //float q2=(float)Q_ekf_obs.y();
    //float q3=(float)Q_ekf_obs.z();
    //MatrixXd QQQ(4,1);
    //QQQ << Q_ekf_obs.w, Q_ekf_obs.x, Q_ekf_obs.y, Q_ekf_obs.z;
    /*double roll = asin(ZR(2,1));
    double yaw = atan(-ZR(1,1)/ZR(0,1)); 
    double pitch = atan(-ZR(2,2)/ZR(2,0));
    //double yaw2 = atan((2*q1*q2-2*q0*q3)/(2*q0*q0+2*q1*q1-1));
    //double pitch2 = -asin(2*q2*q3+2*q0*q1);
    //double roll2 = atan((2*q2*q3-2*q0*q3)/(2*q0*q0+2*q3*q3-1)); 
    //double yaw2 = atan((2*Q_ekf_obs.x*Q_ekf_obs.y-2*Q_ekf_obs.w*Q_ekf_obs.z)/(2*Q_ekf_obs.w*Q_ekf_obs.w+2*Q_ekf_obs.x*Q_ekf_obs.x-1));
    //double pitch2 = -asin(2*Q_ekf_obs.y*Q_ekf_obs.z+2*Q_ekf_obs.w*Q_ekf_obs.x);
    //double roll2 = atan((2*Q_ekf_obs.y*Q_ekf_obs.z-2*Q_ekf_obs.w*Q_ekf_obs.z)/(2*Q_ekf_obs.w*Q_ekf_obs.w+2*Q_ekf_obs.z*Q_ekf_obs.z-1)); 
    Zt(3) = roll;
    Zt(4) = pitch;
    Zt(5) = yaw;
    cout <<"Zt"<<endl<<Zt<<endl; 
/**********************************************************************************************************************************************************/
    MatrixXd R_WorldinCAM(3,3);
    MatrixXd H_WorldinCAM(4,4);
    MatrixXd H_CAMinIMU(4,4);
    Vector3d T_WorldinCAM(3);
    T_WorldinCAM << CamZt(0), CamZt(1), CamZt(2) ;
    R_WorldinCAM = Quaterniond(q_wi.w,q_wi.x,q_wi.y,q_wi.z).toRotationMatrix();
    
    H_WorldinCAM.col(0) << R_WorldinCAM.col(0), 0;
    H_WorldinCAM.col(1) << R_WorldinCAM.col(1), 0;
    H_WorldinCAM.col(2) << R_WorldinCAM.col(2), 0;
    H_WorldinCAM.col(3) << T_WorldinCAM       , 1;
    
    H_CAMinIMU << -1, 0, 0,  0,
                   0, 1, 0, -0.04,
                   0, 0,-1, -0.02,
                   0, 0, 0, 1;
    MatrixXd H_IMUinWorld(4,4);
    H_IMUinWorld =  H_WorldinCAM.inverse() * H_CAMinIMU.inverse();//????????????????

    MatrixXd R_IMUinWorld(3,3);
    Vector3d T_IMUinWorld(3);
    R_IMUinWorld = H_IMUinWorld.topLeftCorner(3, 3);
    T_IMUinWorld = H_IMUinWorld.topRightCorner(3, 1);
    
    MatrixXd Zt(6,1);
    Zt(0) = T_IMUinWorld(0);
    Zt(1) = T_IMUinWorld(1);
    Zt(2) = T_IMUinWorld(2);

    /*double yaw = atan(-R_IMUinWorld(1,1)/R_IMUinWorld(0,1));
    double roll = asin(R_IMUinWorld(2,1));
    double pitch = atan(-R_IMUinWorld(2,2)/R_IMUinWorld(2,0));*/
        Zt(3) = asin(R_IMUinWorld(2,1));
    //x(4) = atan(-R_IMUinWorld(2,0)/R_IMUinWorld(2,2));
    //x(5) = atan(-R_IMUinWorld(0,1)/R_IMUinWorld(1,1));
    //x(4) = acos(R_IMUinWorld(2,2)/cos(x(3)));
    Zt(4) = asin(-R_IMUinWorld(0,2)/cos(Zt(3)));
    if ((R_IMUinWorld(0,2)/cos(Zt(3)))>1)
    {
        Zt(4) = asin(1);
    }
    if ((R_IMUinWorld(0,2)/cos(Zt(3)))<-1)
    {
        Zt(4) = asin(-1);
    }
    Zt(5) = acos(R_IMUinWorld(1,1)/cos(Zt(3)));
    /*Zt(3) = roll;
    Zt(4) = pitch;
    Zt(5) = yaw;
/*****************************************************************************************************/
    if(Zt_1(0)==1) 
{ 
    Zt_1 = Zt;
}
   else 
{ 
if(Zt(3)-Zt_1(3)>1.57 )
{Zt(3)=Zt(3)-3.1416;}
else if (Zt(3)-Zt_1(3)<-1.57)
{Zt(3)=Zt(3)+3.1416;}

if(Zt(4)-Zt_1(4)>1.57 )
{Zt(4)=Zt(4)-3.1416;}
else if (Zt(4)-Zt_1(4)<-1.57)
{Zt(4)=Zt(4)+3.1416;}

if(Zt(5)-Zt_1(5)>1.57 )
{Zt(5)=Zt(5)-3.1416;}
else if (Zt(5)-Zt_1(5)<-1.57)
{Zt(5)=Zt(5)+3.1416;}
}
    Zt_1 = Zt;
    //cout <<"Zt"<<endl<<Zt<<endl;
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
        MidMatrix = Ct * Sigma_t * Ct.transpose() + Wt * Rt * Wt.transpose();
    Kt = Sigma_t * Ct.transpose() *(MidMatrix.inverse()); 
       
    ut = u_t + Kt * (Zt - g);
    Sigmat = Sigma_t - Kt * Ct * Sigma_t;

    //Mu   =  Mu_bar + Kt * (Ct*x - Ct*Mu_bar);
    //Sigma=  Sigma_bar - Kt*Ct*Sigma_bar;


        cout <<"ut"<<endl<<ut<<endl; 
        cout <<"Kt"<<endl<<Kt<<endl;

 
    
    AngleAxisd rollAngle(ut(3), Vector3d::UnitX());
    AngleAxisd pitchAngle(ut(4), Vector3d::UnitY());
    AngleAxisd yawAngle(ut(5), Vector3d::UnitZ());    

    Quaternion<double> Q_ekfwork = yawAngle * rollAngle * pitchAngle;
    //***********************************************************************************
    MatrixXd Rmw(3,3);
    Matrix3d real_pos;   //convert the world frame to the motion capture frame
    Rmw << 0  ,  1  ,  0  ,//the rotation matrix that from world frame to motion capture frame
           1  ,  0  ,  0  ,
           0  ,  0  , -1  ;

    /*Quaternion<double> Q_ekf = yawAngle * rollAngle * pitchAngle;
    
    
    real_pos << ut(0), ut(1), ut(2);

    real_pos = Rmw * real_pos;

    ut(0) = real_pos(0);
    ut(1) = real_pos(1);
    ut(2) = real_pos(2); 


    Matrix3d ekf_world;
    
    ekf_world = Quaterniond(Q_ekf).toRotationMatrix();
    //ekf_world << 
 
    ekf_world = Rmw * ekf_world;
    
    Quaterniond Q_ekfwork;
    Q_ekfwork = ekf_world;
    //Quaterniond Q_ekfwork(ekf_world);
    
    //cout <<"yawAngle"<<endl<< yawAngle <<endl;
*/
    /**/
    //************************************************************************************
    nav_msgs::Odometry odom_ekfwork;
    odom_ekfwork.header.stamp = msg->header.stamp;
    odom_ekfwork.header.frame_id = "world";
    odom_ekfwork.pose.pose.position.x = ut(0);
    odom_ekfwork.pose.pose.position.y = ut(1);
    odom_ekfwork.pose.pose.position.z = ut(2);
    odom_ekfwork.pose.pose.orientation.w = Q_ekfwork.w();
    odom_ekfwork.pose.pose.orientation.x = Q_ekfwork.x();
    odom_ekfwork.pose.pose.orientation.y = Q_ekfwork.y();
    odom_ekfwork.pose.pose.orientation.z = Q_ekfwork.z();
    odom_ekfwork.twist.twist.linear.x = ut(6);
    odom_ekfwork.twist.twist.linear.y = ut(7);
    odom_ekfwork.twist.twist.linear.z = ut(8);
    odom_pub.publish(odom_ekfwork);
    
    AngleAxisd rollAng(Zt(3), Vector3d::UnitX());
    AngleAxisd pitchAng(Zt(4), Vector3d::UnitY());
    AngleAxisd yawAng(Zt(5), Vector3d::UnitZ());
    Quaternion<double> Q_ekf_obs_o = yawAng * rollAng * pitchAng;    
    
    Matrix3d R_ekf_obs_o;
    Vector3d T_ekf_obs;
    T_ekf_obs << Zt(0), Zt(1),Zt(2);
    T_ekf_obs = Rmw * T_ekf_obs;
    R_ekf_obs = Rmw * Q_ekf_obs_o.toRotationMatrix();
    Quaterniond Q_ekf_obs(R_ekf_obs);
    nav_msgs::Odometry odom_ekf_obs;
    odom_ekf_obs.header.stamp = msg->header.stamp;
    odom_ekf_obs.header.frame_id = "world";
    odom_ekf_obs.pose.pose.position.x = T_ekf_obs(0);
    odom_ekf_obs.pose.pose.position.y = T_ekf_obs(1);
    odom_ekf_obs.pose.pose.position.z = T_ekf_obs(2);
    odom_ekf_obs.pose.pose.orientation.w = Q_ekf_obs.w();
    odom_ekf_obs.pose.pose.orientation.x = Q_ekf_obs.x();
    odom_ekf_obs.pose.pose.orientation.y = Q_ekf_obs.y();
    odom_ekf_obs.pose.pose.orientation.z = Q_ekf_obs.z();
    pub_odom_ekf_obs.publish(odom_ekf_obs);  
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ekf");
    ros::NodeHandle n("~");
    ros::Subscriber s1 = n.subscribe("imu", 1000, imu_callback);
    ros::Subscriber s2 = n.subscribe("tag_odom", 1000, odom_callback);
    odom_pub = n.advertise<nav_msgs::Odometry>("ekf_odom", 100);
    pub_odom_ekf_obs = n.advertise<nav_msgs::Odometry>("odom_ekf_obs",10);
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
