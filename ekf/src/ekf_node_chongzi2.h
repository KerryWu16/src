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
MatrixXd Ut = MatrixXd::Identity(15, 1);
MatrixXd U_t(15,1);
MatrixXd Sigmat = MatrixXd::Identity(15, 15);
MatrixXd Sigma_t(15,15);
void imu_callback(const sensor_msgs::Imu::ConstPtr &msg)
{
    //your code for propagation
	
	//MatrixXd //define p q q_ bg ba x=[p q q_ bg ba];
	         //get x_4=[0,0,0]; 
			     //x_5=[0,0,0]; 
				 //g=[0,0,9.81];
				 //R_body2world=[ ]******
				 //x_3=g+R_body2world*[am-x_5];
				 //x_1=x3;
				 //Gt_1=[cos(pitch) 0 -cos(row)*sin(pitch);*****
				 //      0          1 sin(row)          ;*******
				 //      sin(pitch) 0 cos(row)*cos(pitch)];*****
				 //x_2=Gt_1.inverse*(msg.angular_velocity-x_4)
	             //yet we get the f(x,u,n) function
				 //f=[x_1 x_2 x_3 x_4 x_5];
	dt=
	MatrixXd Sigmat_1(15,15);
	Sigmat_1 = Sigmat;
	MatrixXd Ut_1(15,1);
	Ut_1 = Ut;
	double x11= Ut_1[0],x12= Ut_1[1],x13= Ut_1[2];
	double x21= Ut_1[3],x22= Ut_1[4],x23= Ut_1[0];
	double x31= Ut_1[0],x32= Ut_1[0],x33= Ut_1[0];
	double x41= Ut_1[0],x42= Ut_1[0],x43= Ut_1[0];
	double x51= Ut_1[0],x52= Ut_1[0],x53= Ut_1[0];
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
	MatrixXd f(15,1);
	f <<                                                                                                                                                                             x31,
                                                                                                                                                                                     x32,
                                                                                                                                                                                     x33,
                                                                       - (cos(x22)*(ng1 - wm1 + x41))/(cos(x22)^2 + sin(x22)^2) - (sin(x22)*(ng3 - wm3 + x43))/(cos(x22)^2 + sin(x22)^2),
 wm2 - ng2 - x42 - (sin(x21)*sin(x22)*(ng1 - wm1 + x41))/(cos(x21)*cos(x22)^2 + cos(x21)*sin(x22)^2) + (cos(x22)*sin(x21)*(ng3 - wm3 + x43))/(cos(x21)*cos(x22)^2 + cos(x21)*sin(x22)^2),
                                     (sin(x22)*(ng1 - wm1 + x41))/(cos(x21)*cos(x22)^2 + cos(x21)*sin(x22)^2) - (cos(x22)*(ng3 - wm3 + x43))/(cos(x21)*cos(x22)^2 + cos(x21)*sin(x22)^2),
           cos(x21)*sin(x22)*(na3 - am3 + x53) - (cos(x22)*sin(x23) + cos(x23)*sin(x21)*sin(x22))*(na2 - am2 + x52) - (cos(x22)*cos(x23) - sin(x21)*sin(x22)*sin(x23))*(na1 - am1 + x51),
                                                                                  cos(x21)*sin(x23)*(na1 - am1 + x51) - cos(x21)*cos(x23)*(na2 - am2 + x52) - sin(x21)*(na3 - am3 + x53),
 981/100 - (sin(x22)*sin(x23) - cos(x22)*cos(x23)*sin(x21))*(na2 - am2 + x52) - cos(x21)*cos(x22)*(na3 - am3 + x53) - (cos(x23)*sin(x22) + cos(x22)*sin(x21)*sin(x23))*(na1 - am1 + x51),
                                                                                                                                                                                    nbg1,
                                                                                                                                                                                    nbg2,
                                                                                                                                                                                    nbg3,
                                                                                                                                                                                    nba1,
                                                                                                                                                                                    nba2,
                                                                                                                                                                                    nba3;
				 //At=derivative(f)/x
	MatrixXd At(15,15);
	At <<  0, 0, 0,                                                                                                                                       0,                                                                                                                                 0,                                                                                                                                                                             0, 1, 0, 0,                             0,  0,                            0,                  0,                                                0,                                                0,
           0, 0, 0,                                                                                                                                       0,                                                                                                                                 0,                                                                                                                                                                             0, 0, 1, 0,                             0,  0,                            0,                  0,                                                0,                                                0,
           0, 0, 0,                                                                                                                                       0,                                                                                                                                 0,                                                                                                                                                                             0, 0, 0, 1,                             0,  0,                            0,                  0,                                                0,                                                0,
           0, 0, 0,                                                                                                                                       0,                                           wm3*cos(x22) - ng3*cos(x22) - x43*cos(x22) + ng1*sin(x22) - wm1*sin(x22) + x41*sin(x22),                                                                                                                                                                             0, 0, 0, 0,                     -cos(x22),  0,                    -sin(x22),                  0,                                                0,                                                0,
           0, 0, 0,                                    (ng3*cos(x22) - wm3*cos(x22) + x43*cos(x22) - ng1*sin(x22) + wm1*sin(x22) - x41*sin(x22))/cos(x21)^2,                    -(sin(x21)*(ng1*cos(x22) - wm1*cos(x22) + x41*cos(x22) + ng3*sin(x22) - wm3*sin(x22) + x43*sin(x22)))/cos(x21),                                                                                                                                                                             0, 0, 0, 0, -(sin(x21)*sin(x22))/cos(x21), -1, (cos(x22)*sin(x21))/cos(x21),                  0,                                                0,                                                0,
           0, 0, 0,                        -(sin(x21)*(ng3*cos(x22) - wm3*cos(x22) + x43*cos(x22) - ng1*sin(x22) + wm1*sin(x22) - x41*sin(x22)))/cos(x21)^2,                                (ng1*cos(x22) - wm1*cos(x22) + x41*cos(x22) + ng3*sin(x22) - wm3*sin(x22) + x43*sin(x22))/cos(x21),                                                                                                                                                                             0, 0, 0, 0,             sin(x22)/cos(x21),  0,           -cos(x22)/cos(x21),                  0,                                                0,                                                0,
           0, 0, 0, (sin(x21)*sin(x23) - cos(x21)*cos(x23)*sin(x22))*(na2 - am2 + x52) - (cos(x21)*sin(x23) + cos(x23)*sin(x21)*sin(x22))*(na3 - am3 + x53), cos(x23)*sin(x22)*(na1 - am1 + x51) - cos(x22)*cos(x23)*sin(x21)*(na2 - am2 + x52) + cos(x21)*cos(x22)*cos(x23)*(na3 - am3 + x53), cos(x22)*sin(x23)*(na1 - am1 + x51) - (cos(x23)*sin(x21) + cos(x21)*sin(x22)*sin(x23))*(na3 - am3 + x53) - (cos(x21)*cos(x23) - sin(x21)*sin(x22)*sin(x23))*(na2 - am2 + x52), 0, 0, 0,                             0,  0,                            0, -cos(x22)*cos(x23), - cos(x21)*sin(x23) - cos(x23)*sin(x21)*sin(x22),   cos(x21)*cos(x23)*sin(x22) - sin(x21)*sin(x23),
           0, 0, 0, (cos(x23)*sin(x21) + cos(x21)*sin(x22)*sin(x23))*(na2 - am2 + x52) - (cos(x21)*cos(x23) - sin(x21)*sin(x22)*sin(x23))*(na3 - am3 + x53), cos(x22)*sin(x21)*sin(x23)*(na2 - am2 + x52) - cos(x21)*cos(x22)*sin(x23)*(na3 - am3 + x53) - sin(x22)*sin(x23)*(na1 - am1 + x51), (cos(x21)*sin(x23) + cos(x23)*sin(x21)*sin(x22))*(na2 - am2 + x52) + (sin(x21)*sin(x23) - cos(x21)*cos(x23)*sin(x22))*(na3 - am3 + x53) + cos(x22)*cos(x23)*(na1 - am1 + x51), 0, 0, 0,                             0,  0,                            0,  cos(x22)*sin(x23),   sin(x21)*sin(x22)*sin(x23) - cos(x21)*cos(x23), - cos(x23)*sin(x21) - cos(x21)*sin(x22)*sin(x23),
           0, 0, 0,                                                               cos(x21)*cos(x22)*(na2 - am2 + x52) + cos(x22)*sin(x21)*(na3 - am3 + x53),                            cos(x21)*sin(x22)*(na3 - am3 + x53) - cos(x22)*(na1 - am1 + x51) - sin(x21)*sin(x22)*(na2 - am2 + x52),                                                                                                                                                                             0, 0, 0, 0,                             0,  0,                            0,          -sin(x22),                                cos(x22)*sin(x21),                               -cos(x21)*cos(x22),
           0, 0, 0,                                                                                                                                       0,                                                                                                                                 0,                                                                                                                                                                             0, 0, 0, 0,                             0,  0,                            0,                  0,                                                0,                                                0,
           0, 0, 0,                                                                                                                                       0,                                                                                                                                 0,                                                                                                                                                                             0, 0, 0, 0,                             0,  0,                            0,                  0,                                                0,                                                0,
           0, 0, 0,                                                                                                                                       0,                                                                                                                                 0,                                                                                                                                                                             0, 0, 0, 0,                             0,  0,                            0,                  0,                                                0,                                                0,
           0, 0, 0,                                                                                                                                       0,                                                                                                                                 0,                                                                                                                                                                             0, 0, 0, 0,                             0,  0,                            0,                  0,                                                0,                                                0,
           0, 0, 0,                                                                                                                                       0,                                                                                                                                 0,                                                                                                                                                                             0, 0, 0, 0,                             0,  0,                            0,                  0,                                                0,                                                0,
           0, 0, 0,                                                                                                                                       0,                                                                                                                                 0,                                                                                                                                                                             0, 0, 0, 0,                             0,  0,                            0,                  0,                                                0,                                                0;
 
				 //Ut=
	MatrixXd Ut(15,15);
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
 
				 //Ft=I+dt*At
	MatrixXd Ft(15,15);
	MatrixXd I15 = MatrixXd::Identity(15, 15);
	Ft = I15 + dt * At；
					 //Vt=dt*Ut
	MatrixXd Vt(15,12);
	Vt = dt*Ut；
				 //prediction step
				 //ut_1=ut original ut->ut_1
				 //u_t=ut_1+dt*f    ********former ones
    u_t = ut_1 + dt * f;
	             //Sigma_t=Ft*Sigmat_1*Ft' + Vt*Q*Vt'
    Sigma_t = Ft * Sigmat_1 * Ft.adjoint() + Vt * Q * Vt.adjoint();
	             
	             //
				 //
	
				  
}

//Rotation from the camera frame to the IMU frame
Eigen::Matrix3d Rcam;
void odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    //your code for update
    //camera position in the IMU frame = (0, -0.04, -0.02)
    //camera orientaion in the IMU frame = Quaternion(0, 0, 1, 0); w x y z, respectively
    //					   RotationMatrix << -1, 0,  0,
    //							              0, 1,  0,
    //                                        0, 0, -1;	
	
	//***********************************************my own notes
	//Gt=[p q p_dot bg ba]
    MatrixXd Zt(6,1);
	Zt << 
	MatrixXd g(6,1);
	g << u_t[0] , u_t[1], u_t[2], u_t[3], u_t[4], u_t[5];
	//Ct=[I 0 0 0 0; 
	//    0 I 0 0 0]
	MatrixXd Ct(6,15);
	Ct <<  1  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,
	       0  ,  1  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,
		   0  ,  0  ,  1  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,
		   0  ,  0  ,  0  ,  1  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,
		   0  ,  0  ,  0  ,  0  ,  1  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,
		   0  ,  0  ,  0  ,  0  ,  0  ,  1  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ;
    
	//Wt=
	MatrixXd Wt = MatrixXd::Identity(6,6);
	//Kt=Sigma_t * Ct' * (Ct * Sigma_t * Ct' + Wt * R *Wt')
	Kt = Sigma_t * Ct.adjoint() *( Ct * Sigma_t * Ct.adjoint() + Wt * R * Wt.adjoint() );
	//ut=u_t + 
	ut = u_t + Kt * (Zt - g);
	Sigmat = Sigma_t - Kt * Ct * Sigma_t;
	
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
