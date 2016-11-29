#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Eigen>
#include <Eigen/Geometry> 
#include <math.h> 

using namespace std;
using namespace Eigen;
ros::Publisher odom_pub;
MatrixXd Q = MatrixXd::Identity(12, 12);
MatrixXd Rt = MatrixXd::Identity(6,6);
//Define some para
int img_checktemp=0;
int temp_ff=0;
Eigen::Vector3d p,q,p_dot,b_g,b_a;
Eigen::Vector3d u_a,u_g;
Eigen::VectorXd ut(6);
Eigen::VectorXd x_dot_prev(15),x_prev(15);
Eigen::VectorXd x_dot(15)     ,x(15);
Eigen::VectorXd Mu(15) ,Mu_bar(15), Mu_prev(15);
Eigen::MatrixXd Sigma(15,15) ,Sigma_bar(15,15), Sigma_prev(15,15);
Eigen::VectorXd process_model(15);
Eigen::Matrix3d R_WorldinCAM;
Eigen::Vector3d T_WorldinCAM;
Eigen::VectorXd Q_WorldinCAM(4);
Eigen::Matrix3d R_IMUinWorld;
Eigen::Vector3d T_IMUinWorld;
Eigen::VectorXd Q_IMUinWorld(4);
double dt, time_old,grav;
//Define some Matrix
Eigen::MatrixXd At(15,15);
Eigen::MatrixXd Ut(15,12);
Eigen::MatrixXd Ft(15,15);
Eigen::MatrixXd Vt(15,12);
Eigen::MatrixXd Ct(6,15);
Eigen::MatrixXd Kt(15,6);
Eigen::MatrixXd H_CAMinIMU(4,4);
Eigen::MatrixXd H_WorldinCAM(4,4);
Eigen::MatrixXd H_IMUinWorld(4,4);
void imu_callback(const sensor_msgs::Imu::ConstPtr &msg)
{
    //your code for propagation
	//Get process model
/*Get Row Data as u*/
	u_a(0) = msg->linear_acceleration.x;
	u_a(1) = msg->linear_acceleration.y;
	u_a(2) = msg->linear_acceleration.z;
	//cout << "u_a(0):" << u_a(0) << "u_a(1):" << u_a(1) << "u_a(2):" << u_a(2) << endl;
	u_g(0) = msg->angular_velocity.x;//as roll
	u_g(1) = msg->angular_velocity.y;//as pitch
	u_g(2) = msg->angular_velocity.z;//as yaw
/*Get Row Data as u*/
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
/*Get the dt from stamp*/
{
	if (!img_checktemp) 
    {
	time_old = msg->header.stamp.toSec()-0.03;
	Mu.setZero();
	Sigma.setIdentity();
	//img_checktemp = 1;
	//cout << "Mu_prev" << endl << Mu_prev << endl;
    }
    dt=msg->header.stamp.toSec() - time_old;
    time_old = msg->header.stamp.toSec();
    //cout << "dt" << endl << sin(1.5) << endl;
}
/*Get the dt from stamp*/
/*Prediction Step*/
{
	//move forward
	x_prev     = x;
	x_dot_prev = x_dot;
	Mu_prev    = Mu;
	Sigma_prev = Sigma;
	//get ut from IMU
	ut        << u_a , u_g;
	//cout << "u_a(0):" << u_a(0) << "u_a(1):" << u_a(1) << "u_a(2):" << u_a(2) << endl;
	//cout << "Rcam:"   << Rcam   << endl;
	{//three matrix
	//get matrix At
	At            <<0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,
					0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,
                    0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,
  0, 0, 0,0,Mu_prev(9)*sin(Mu_prev(4)) - Mu_prev(11)*cos(Mu_prev(4)) + ut(5)*cos(Mu_prev(4)) - ut(3)*sin(Mu_prev(4)),                                                                                                                                                                                                                                                                                                                                                                                         0, 0, 0, 0,                                               -cos(Mu_prev(4)),  0,                                              -sin(Mu_prev(4)),                                                                                                                 0,                                            0,                                                                                                                 0,
  0, 0, 0,(Mu_prev(11)*cos(Mu_prev(4)) - Mu_prev(9)*sin(Mu_prev(4)) - ut(5)*cos(Mu_prev(4)) + ut(3)*sin(Mu_prev(4)))/pow(cos(Mu_prev(3)),2),                                                                                                 -(sin(Mu_prev(3))*(Mu_prev(9)*cos(Mu_prev(4)) + Mu_prev(11)*sin(Mu_prev(4)) - ut(3)*cos(Mu_prev(4)) - ut(5)*sin(Mu_prev(4))))/cos(Mu_prev(3)),                                                                                                                                                                                                                                                                                                                                                                                         0, 0, 0, 0, -(sin(Mu_prev(3))*sin(Mu_prev(4)))/cos(Mu_prev(3)), -1, (cos(Mu_prev(4))*sin(Mu_prev(3)))/cos(Mu_prev(3)),                                                                                                                 0,                                            0,                                                                                                                 0,
  0, 0, 0,-(sin(Mu_prev(3))*(Mu_prev(11)*cos(Mu_prev(4)) - Mu_prev(9)*sin(Mu_prev(4)) - ut(5)*cos(Mu_prev(4)) + ut(3)*sin(Mu_prev(4))))/pow(cos(Mu_prev(3)),2),                                                                                                                          (Mu_prev(9)*cos(Mu_prev(4)) + Mu_prev(11)*sin(Mu_prev(4)) - ut(3)*cos(Mu_prev(4)) - ut(5)*sin(Mu_prev(4)))/cos(Mu_prev(3)),                                                                                                                                                                                                                                                                                                                                                                                         0, 0, 0, 0,                          sin(Mu_prev(4))/cos(Mu_prev(3)),  0,                        -cos(Mu_prev(4))/cos(Mu_prev(3)),                                                                                                                 0,                                            0,                                                                                                                 0,
  0, 0, 0, cos(Mu_prev(3))*sin(Mu_prev(4))*sin(Mu_prev(5))*(Mu_prev(12) - ut(0)) - cos(Mu_prev(3))*cos(Mu_prev(4))*sin(Mu_prev(5))*(Mu_prev(14) - ut(2)) - sin(Mu_prev(3))*sin(Mu_prev(5))*(Mu_prev(13) - ut(1)), (Mu_prev(12) - ut(0))*(cos(Mu_prev(5))*sin(Mu_prev(4)) + cos(Mu_prev(4))*sin(Mu_prev(3))*sin(Mu_prev(5))) - (Mu_prev(14) - ut(2))*(cos(Mu_prev(4))*cos(Mu_prev(5)) - sin(Mu_prev(3))*sin(Mu_prev(4))*sin(Mu_prev(5))), (Mu_prev(12) - ut(0))*(cos(Mu_prev(4))*sin(Mu_prev(5)) + cos(Mu_prev(5))*sin(Mu_prev(3))*sin(Mu_prev(4))) + (Mu_prev(14) - ut(2))*(sin(Mu_prev(4))*sin(Mu_prev(5)) - cos(Mu_prev(4))*cos(Mu_prev(5))*sin(Mu_prev(3))) + cos(Mu_prev(3))*cos(Mu_prev(5))*(Mu_prev(13) - ut(1)), 0, 0, 0,                                                                    0,  0,                                                                   0,   sin(Mu_prev(3))*sin(Mu_prev(4))*sin(Mu_prev(5)) - cos(Mu_prev(4))*cos(Mu_prev(5)),  cos(Mu_prev(3))*sin(Mu_prev(5)), - cos(Mu_prev(5))*sin(Mu_prev(4)) - cos(Mu_prev(4))*sin(Mu_prev(3))*sin(Mu_prev(5)),
  0, 0, 0, cos(Mu_prev(5))*sin(Mu_prev(3))*(Mu_prev(13) - ut(1)) - cos(Mu_prev(3))*cos(Mu_prev(5))*sin(Mu_prev(4))*(Mu_prev(12) - ut(0)) + cos(Mu_prev(3))*cos(Mu_prev(4))*cos(Mu_prev(5))*(Mu_prev(14) - ut(2)), (Mu_prev(12) - ut(0))*(sin(Mu_prev(4))*sin(Mu_prev(5)) - cos(Mu_prev(4))*cos(Mu_prev(5))*sin(Mu_prev(3))) - (Mu_prev(14) - ut(2))*(cos(Mu_prev(4))*sin(Mu_prev(5)) + cos(Mu_prev(5))*sin(Mu_prev(3))*sin(Mu_prev(4))), cos(Mu_prev(3))*sin(Mu_prev(5))*(Mu_prev(13) - ut(1)) - (Mu_prev(14) - ut(2))*(cos(Mu_prev(5))*sin(Mu_prev(4)) + cos(Mu_prev(4))*sin(Mu_prev(3))*sin(Mu_prev(5))) - (Mu_prev(12) - ut(0))*(cos(Mu_prev(4))*cos(Mu_prev(5)) - sin(Mu_prev(3))*sin(Mu_prev(4))*sin(Mu_prev(5))), 0, 0, 0,                                                                    0,  0,                                                                   0, - cos(Mu_prev(4))*sin(Mu_prev(5)) - cos(Mu_prev(5))*sin(Mu_prev(3))*sin(Mu_prev(4)), -cos(Mu_prev(3))*cos(Mu_prev(5)),   cos(Mu_prev(4))*cos(Mu_prev(5))*sin(Mu_prev(3)) - sin(Mu_prev(4))*sin(Mu_prev(5)),
  0, 0, 0,cos(Mu_prev(4))*sin(Mu_prev(3))*(Mu_prev(14) - ut(2)) - cos(Mu_prev(3))*(Mu_prev(13) - ut(1)) - sin(Mu_prev(3))*sin(Mu_prev(4))*(Mu_prev(12) - ut(0)),                                                                                                                                             cos(Mu_prev(3))*cos(Mu_prev(4))*(Mu_prev(12) - ut(0)) + cos(Mu_prev(3))*sin(Mu_prev(4))*(Mu_prev(14) - ut(2)),                                                                                                                                                                                                                                                                                                                                                                                         0, 0, 0, 0,                                                                    0,  0,                                                                   0,                                                                       cos(Mu_prev(3))*sin(Mu_prev(4)),                       -sin(Mu_prev(3)),                                                                      -cos(Mu_prev(3))*cos(Mu_prev(4)),
                    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
                    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
                    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
                    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
					0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
					0,0,0,0,0,0,0,0,0,0,0,0,0,0,0;
	//get matrix Ut
	Ut            <<0,                                            0,                                                                                                                 0,                                                                    0,  0,                                                                   0, 0, 0, 0, 0, 0, 0,
                                                                                                                  0,                                            0,                                                                                                                 0,                                                                    0,  0,                                                                   0, 0, 0, 0, 0, 0, 0,
                                                                                                                  0,                                            0,                                                                                                                 0,                                                                    0,  0,                                                                   0, 0, 0, 0, 0, 0, 0,
                                                                                                                  0,                                            0,                                                                                                                 0,                                               -cos(Mu_prev(4)),  0,                                              -sin(Mu_prev(4)), 0, 0, 0, 0, 0, 0,
                                                                                                                  0,                                            0,                                                                                                                 0, -(sin(Mu_prev(3))*sin(Mu_prev(4)))/cos(Mu_prev(3)), -1, (cos(Mu_prev(4))*sin(Mu_prev(3)))/cos(Mu_prev(3)), 0, 0, 0, 0, 0, 0,
                                                                                                                  0,                                            0,                                                                                                                 0,                          sin(Mu_prev(4))/cos(Mu_prev(3)),  0,                        -cos(Mu_prev(4))/cos(Mu_prev(3)), 0, 0, 0, 0, 0, 0,
    sin(Mu_prev(3))*sin(Mu_prev(4))*sin(Mu_prev(5)) - cos(Mu_prev(4))*cos(Mu_prev(5)),  cos(Mu_prev(3))*sin(Mu_prev(5)), - cos(Mu_prev(5))*sin(Mu_prev(4)) - cos(Mu_prev(4))*sin(Mu_prev(3))*sin(Mu_prev(5)),                                                                    0,  0,                                                                   0, 0, 0, 0, 0, 0, 0,
  - cos(Mu_prev(4))*sin(Mu_prev(5)) - cos(Mu_prev(5))*sin(Mu_prev(3))*sin(Mu_prev(4)), -cos(Mu_prev(3))*cos(Mu_prev(5)),   cos(Mu_prev(4))*cos(Mu_prev(5))*sin(Mu_prev(3)) - sin(Mu_prev(4))*sin(Mu_prev(5)),                                                                    0,  0,                                                                   0, 0, 0, 0, 0, 0, 0,
                                                                        cos(Mu_prev(3))*sin(Mu_prev(4)),                       -sin(Mu_prev(3)),                                                                      -cos(Mu_prev(3))*cos(Mu_prev(4)),                                                                    0,  0,                                                                   0, 0, 0, 0, 0, 0, 0,
                                                                                                                  0,                                            0,                                                                                                                 0,                                                                    0,  0,                                                                   0, 0, 0, 0, 1, 0, 0,
                                                                                                                  0,                                            0,                                                                                                                 0,                                                                    0,  0,                                                                   0, 0, 0, 0, 0, 1, 0,
                                                                                                                  0,                                            0,                                                                                                                 0,                                                                    0,  0,                                                                   0, 0, 0, 0, 0, 0, 1,
                                                                                                                  0,                                            0,                                                                                                                 0,                                                                    0,  0,                                                                   0, 1, 0, 0, 0, 0, 0,
                                                                                                                  0,                                            0,                                                                                                                 0,                                                                    0,  0,                                                                   0, 0, 1, 0, 0, 0, 0,
                                                                                                                  0,                                            0,                                                                                                                 0,                                                                    0,  0,                                                                   0, 0, 0, 1, 0, 0, 0;
 
	//get Mu_bar
	grav = 9.8;
	process_model << Mu_prev(6),
                     Mu_prev(7),
                     Mu_prev(8),
  ut(3)*cos(Mu_prev(4)) - Mu_prev(11)*sin(Mu_prev(4)) - Mu_prev(9)*cos(Mu_prev(4)) + ut(5)*sin(Mu_prev(4)),
-(Mu_prev(10)*cos(Mu_prev(3)) - ut(4)*cos(Mu_prev(3)) - Mu_prev(11)*cos(Mu_prev(4))*sin(Mu_prev(3)) + Mu_prev(9)*sin(Mu_prev(3))*sin(Mu_prev(4)) + ut(5)*cos(Mu_prev(4))*sin(Mu_prev(3)) - ut(3)*sin(Mu_prev(3))*sin(Mu_prev(4)))/cos(Mu_prev(3)),
-(Mu_prev(11)*cos(Mu_prev(4)) - Mu_prev(9)*sin(Mu_prev(4)) - ut(5)*cos(Mu_prev(4)) + ut(3)*sin(Mu_prev(4)))/cos(Mu_prev(3)),
  - (Mu_prev(12) - ut(0))*(cos(Mu_prev(4))*cos(Mu_prev(5)) - sin(Mu_prev(3))*sin(Mu_prev(4))*sin(Mu_prev(5))) - (Mu_prev(14) - ut(2))*(cos(Mu_prev(5))*sin(Mu_prev(4)) + cos(Mu_prev(4))*sin(Mu_prev(3))*sin(Mu_prev(5))) + cos(Mu_prev(3))*sin(Mu_prev(5))*(Mu_prev(13) - ut(1)),
  - (Mu_prev(12) - ut(0))*(cos(Mu_prev(4))*sin(Mu_prev(5)) + cos(Mu_prev(5))*sin(Mu_prev(3))*sin(Mu_prev(4))) - (Mu_prev(14) - ut(2))*(sin(Mu_prev(4))*sin(Mu_prev(5)) - cos(Mu_prev(4))*cos(Mu_prev(5))*sin(Mu_prev(3))) - cos(Mu_prev(3))*cos(Mu_prev(5))*(Mu_prev(13) - ut(1)),
grav - sin(Mu_prev(3))*(Mu_prev(13) - ut(1)) - cos(Mu_prev(3))*cos(Mu_prev(4))*(Mu_prev(14) - ut(2)) + cos(Mu_prev(3))*sin(Mu_prev(4))*(Mu_prev(12) - ut(0)),
                     0,
                     0,
                     0,
					 0,
					 0,
					 0;
	}
	Mu_bar     = Mu_prev + dt * process_model;
	//get Sigma_bar
	Ft         = MatrixXd::Identity(15, 15) + dt * At;
	Vt         = dt * Ut;
	Sigma_bar  = Ft * Sigma_prev * Ft.transpose() + Vt * Q * Vt.transpose();

	//cout << "process_model" << endl << process_model << endl;
}
/*Prediction Step*/
/*Updata Step*/
{
	//transform imu to world using camera
	/*H_CAMinIMU	<<
	-1,0,0 ,0    ,
	0 ,1,0 ,-0.04,
	0 ,0,-1,-0.02,
	0 ,0,0 ,1    ;*/
	H_CAMinIMU.col(0) << Rcam.col(0)  , 0;
	H_CAMinIMU.col(1) << Rcam.col(1)  , 0;
	H_CAMinIMU.col(2) << Rcam.col(2)  , 0;
	H_CAMinIMU.col(3) << 0,-0.04,-0.02, 1;
	T_WorldinCAM(0) = msg->pose.pose.position.x;
	T_WorldinCAM(1) = msg->pose.pose.position.y;
	T_WorldinCAM(2) = msg->pose.pose.position.z;
	Q_WorldinCAM(0) = msg->pose.pose.orientation.w;
	Q_WorldinCAM(1) = msg->pose.pose.orientation.x;
	Q_WorldinCAM(2) = msg->pose.pose.orientation.y;
	Q_WorldinCAM(3) = msg->pose.pose.orientation.z;
    R_WorldinCAM = Quaterniond(Q_WorldinCAM(0),Q_WorldinCAM(1),Q_WorldinCAM(2),Q_WorldinCAM(3)).toRotationMatrix();
	
	H_WorldinCAM.col(0) << R_WorldinCAM.col(0), 0;
	H_WorldinCAM.col(1) << R_WorldinCAM.col(1), 0;
	H_WorldinCAM.col(2) << R_WorldinCAM.col(2), 0;
	H_WorldinCAM.col(3) << T_WorldinCAM       , 1;
	
	H_IMUinWorld = H_WorldinCAM.inverse() * H_CAMinIMU.inverse()  ;//????????????????
	R_IMUinWorld = H_IMUinWorld.topLeftCorner(3, 3);
	T_IMUinWorld = H_IMUinWorld.topRightCorner(3, 1);
	//cout << "H_IMUinWorld" << endl << H_IMUinWorld << endl;
	//get matrix Ct
	Ct   << MatrixXd::Identity(6, 15);
	//get matrix Kt
	Kt   =  Sigma_bar * Ct.transpose() * ((Ct * Sigma_bar * Ct.transpose() + MatrixXd::Identity(6, 6) * Rt * MatrixXd::Identity(6, 6)).inverse());
	//get Mu & Sigma
	x.head(3) = T_IMUinWorld;
	x(3) = asin(R_IMUinWorld(2,1));
	//x(4) = atan(-R_IMUinWorld(2,0)/R_IMUinWorld(2,2));
	//x(5) = atan(-R_IMUinWorld(0,1)/R_IMUinWorld(1,1));
	//x(4) = acos(R_IMUinWorld(2,2)/cos(x(3)));
	x(4) = asin(-R_IMUinWorld(0,2)/cos(x(3)));
	if ((R_IMUinWorld(0,2)/cos(x(3)))>1)
	{
		x(4) = asin(1);
	}
	if ((R_IMUinWorld(0,2)/cos(x(3)))<-1)
	{
		x(4) = asin(-1);
	}
	x(5) = acos(R_IMUinWorld(1,1)/cos(x(3)));
	//x(5) = asin(-R_IMUinWorld(0,1)/cos(x(3)));
	//x.segment(3,3) = R_IMUinWorld.eulerAngles(1, 2, 0);
	
	
	Mu   =  Mu_bar + Kt * (Ct*x - Ct*Mu_bar);
	Sigma=  Sigma_bar - Kt*Ct*Sigma_bar;

	
	if(std::isnan(Mu(0))&&temp_ff==0)
	{
		cout << "asin(-R_IMUinWorld(0,2)/cos(x(3)))" << endl << asin(-R_IMUinWorld(0,2)/cos(x(3))) << endl;
		cout << "R_IMUinWorld(0,2)" << endl << R_IMUinWorld(0,2) << endl;
		cout << "cos(x(3))" << endl << cos(x(3)) << endl;
		cout << "x" << endl << x << endl;
		cout << "Kt" << endl << Kt << endl;
		cout << "Mu_bar" << endl << Mu_prev << endl;
		/*cout << "Mu_prev" << endl << Mu_prev << endl;
		cout << "Ft" << endl << Ft << endl;
		cout << "Sigma_prev" << endl << Sigma_prev << endl;
		cout << "Sigma_bar" << endl << Sigma_bar << endl;*/
		cout << "dt" << endl << dt << endl;
		cout << "img_checktemp" << endl << img_checktemp << endl;
		temp_ff = 1;
	}
	//cout << "Mu" << endl << Mu << endl;
	if (img_checktemp>=0&&img_checktemp<=1) 
    {
	//cout << "Sigma_bar" << endl << Sigma_bar << endl;
	//cout << "Mu_bar" << endl << Mu_bar << endl;
	//cout << "Ut" << endl << Mu_prev << endl;
   // cout << "At" << endl << At << endl;
    //cout << "H_WorldinCAM" << endl << H_WorldinCAM << endl;
	cout << "H_CAMinIMU * H_WorldinCAM" << endl << H_CAMinIMU * H_WorldinCAM << endl;
	cout << "H_WorldinCAM * H_CAMinIMU" << endl << H_WorldinCAM * H_CAMinIMU << endl;
    cout << "H_IMUinWorld" << endl << H_IMUinWorld << endl;
	//cout << "H_IMUinWorld.topRightCorner(3, 1)" << endl << H_IMUinWorld.topRightCorner(3, 1) << endl;
	//cout << "H_IMUinWorld.topLeftCorner(3, 3)"  << endl << H_IMUinWorld.topLeftCorner(3, 3)  << endl;
	//cout << "(Ct * Sigma_bar * Ct.transpose())" << endl << (Ct * Sigma_bar * Ct.transpose()) << endl;
	//cout << "Kt" << endl << Kt << endl;
	cout <<endl<< "***********************************"<<endl;	
    }
	img_checktemp ++;
	//cout << "Kt" << endl << Kt << endl;
	Matrix3d m_temp;//??????????????
	m_temp << 
	cos(Mu(4))*cos(Mu(5))-sin(Mu(3))*sin(Mu(4))*sin(Mu(5)), -cos(Mu(3))*sin(Mu(5)), sin(Mu(4))*cos(Mu(5))+sin(Mu(3))*cos(Mu(4))*sin(Mu(5)),
    cos(Mu(4))*sin(Mu(5))+sin(Mu(3))*sin(Mu(4))*cos(Mu(5)), cos(Mu(3))*cos(Mu(5)) , sin(Mu(4))*sin(Mu(5))-sin(Mu(3))*cos(Mu(4))*cos(Mu(5)),
    -cos(Mu(3))*sin(Mu(4))                                , sin(Mu(3))            , cos(Mu(3))*cos(Mu(4))                                 ;
	Quaterniond Q_ekf;
    Q_ekf = m_temp;
    nav_msgs::Odometry odom_ekf;
    odom_ekf.header.stamp = msg->header.stamp;
    odom_ekf.header.frame_id = "world";
    odom_ekf.pose.pose.position.x = Mu(0);
    odom_ekf.pose.pose.position.y = Mu(1);
    odom_ekf.pose.pose.position.z = Mu(2);
    odom_ekf.pose.pose.orientation.w = Q_ekf.w();
    odom_ekf.pose.pose.orientation.x = Q_ekf.x();
    odom_ekf.pose.pose.orientation.y = Q_ekf.y();
    odom_ekf.pose.pose.orientation.z = Q_ekf.z();
	odom_ekf.twist.twist.linear.x    = Mu(6);
	odom_ekf.twist.twist.linear.y    = Mu(7);
	odom_ekf.twist.twist.linear.z    = Mu(8);
    odom_pub.publish(odom_ekf);
}
/*Updata Step*/	
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