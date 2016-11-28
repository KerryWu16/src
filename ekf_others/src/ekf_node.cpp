#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Eigen>
#include <cmath>

using namespace std;
using namespace Eigen;
ros::Publisher odom_pub;
MatrixXd Q = MatrixXd::Identity(12, 12);
MatrixXd Rt = MatrixXd::Identity(6,6);
MatrixXd A = MatrixXd::Zero(15,15);
MatrixXd B = MatrixXd::Zero(15,6);
MatrixXd U = MatrixXd::Zero(15,12);
MatrixXd X = MatrixXd::Zero(15,1);
MatrixXd dX = MatrixXd::Zero(15,1);
MatrixXd C = MatrixXd::Identity(6,15);
MatrixXd W = MatrixXd::Identity(6,6);
MatrixXd Z = MatrixXd::Zero(6,1);
MatrixXd covar = MatrixXd::Identity(15,15);
MatrixXd covariance = MatrixXd::Identity(15,15);
MatrixXd mut = MatrixXd::Zero(15,1);
double imu_time=-1.0;
double odo_time=-1.0;
double pi = acos(-1);

void imu_callback(const sensor_msgs::Imu::ConstPtr &msg)
{
    //your code for propagation
    
    MatrixXd F;
    MatrixXd V;
    
    
    Matrix3d G = MatrixXd::Zero(3,3);
    
    double time=msg->header.stamp.sec;
    if (imu_time<0) return;
    double dt=time-imu_time;
    double p=msg->angular_velocity.x;
    double q=msg->angular_velocity.y;
    double r=msg->angular_velocity.z;
    double ax=msg->linear_acceleration.x;
    double ay=msg->linear_acceleration.y;
    double az=msg->linear_acceleration.z;
    double x=X(0);
    double y=X(1);
    double z=X(2);
    double phi=X(3);
    double theta=X(4);
    double psi=X(5);
    double vx=X(6);
    double vy=X(7);
    double vz=X(8);
    double bphi=X(9);
    double btheta=X(10);
    double bpsi=X(11);
    double bax=X(12);
    double bay=X(13);
    double baz=X(14);
    double g=-9.81;
    MatrixXd pqr= MatrixXd::Zero(3,1);
    pqr<<p,q,r;
    Vector3d am(ax,ay,az);
    
    
    
    G<<cos(theta),0,-cos(phi)*sin(theta),0,1,sin(phi),sin(theta),0,cos(phi)*cos(theta);
    MatrixXd dptp=(G.inverse())*pqr;
    double phip=dptp(0);
    double thetaq=dptp(1);
    double psir=dptp(2);
    
    
  /*  dX<<vx,vy,vz,(cos(theta)*(phip*cos(theta) - psir*cos(phi)*sin(theta))) + (sin(theta)*(phip*sin(theta) + psir*cos(phi)*cos(theta))),thetaq + psir*sin(phi) - (cos(theta)*sin(phi)*(phip*sin(theta) + psir*cos(phi)*cos(theta)))/(cos(phi)) + (sin(phi)*sin(theta)*(phip*cos(theta) - psir*cos(phi)*sin(theta)))/(cos(phi)),(cos(theta)*(phip*sin(theta) + psir*cos(phi)*cos(theta)))/(cos(phi)) - (sin(theta)*(phip*cos(theta) - psir*cos(phi)*sin(theta)))/(cos(phi)),(cos(theta)*sin(psi) + cos(psi)*sin(phi)*sin(theta))*(ax*(cos(theta)*sin(psi) + cos(psi)*sin(phi)*sin(theta)) + (sin(psi)*sin(theta) - cos(psi)*cos(theta)*sin(phi))*(az - g) + ay*cos(phi)*cos(psi)) + (cos(psi)*cos(theta) - sin(phi)*sin(psi)*sin(theta))*(ax*(cos(psi)*cos(theta) - sin(phi)*sin(psi)*sin(theta)) + (cos(psi)*sin(theta) + cos(theta)*sin(phi)*sin(psi))*(az - g) - ay*cos(phi)*sin(psi)) - cos(phi)*sin(theta)*(ay*sin(phi) - ax*cos(phi)*sin(theta) + cos(phi)*cos(theta)*(az - g)),sin(phi)*(ay*sin(phi) - ax*cos(phi)*sin(theta) + cos(phi)*cos(theta)*(az - g)) + cos(phi)*cos(psi)*(ax*(cos(theta)*sin(psi) + cos(psi)*sin(phi)*sin(theta)) + (sin(psi)*sin(theta) - cos(psi)*cos(theta)*sin(phi))*(az - g) + ay*cos(phi)*cos(psi)) - cos(phi)*sin(psi)*(ax*(cos(psi)*cos(theta) - sin(phi)*sin(psi)*sin(theta)) + (cos(psi)*sin(theta) + cos(theta)*sin(phi)*sin(psi))*(az - g) - ay*cos(phi)*sin(psi)),g + (sin(psi)*sin(theta) - cos(psi)*cos(theta)*sin(phi))*(ax*(cos(theta)*sin(psi) + cos(psi)*sin(phi)*sin(theta)) + (sin(psi)*sin(theta) - cos(psi)*cos(theta)*sin(phi))*(az - g) + ay*cos(phi)*cos(psi)) + (cos(psi)*sin(theta) + cos(theta)*sin(phi)*sin(psi))*(ax*(cos(psi)*cos(theta) - sin(phi)*sin(psi)*sin(theta)) + (cos(psi)*sin(theta) + cos(theta)*sin(phi)*sin(psi))*(az - g) - ay*cos(phi)*sin(psi)) + cos(phi)*cos(theta)*(ay*sin(phi) - ax*cos(phi)*sin(theta) + cos(phi)*cos(theta)*(az - g)),0,0,0,0,0,0;
    
    A<< 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
     0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
     0, 0, 0, psir*cos(phi) - (cos(phi)*cos(theta)*(phip*sin(theta) + psir*cos(phi)*cos(theta)))/(cos(phi)) + (cos(phi)*sin(theta)*(phip*cos(theta) - psir*cos(phi)*sin(theta)))/(cos(phi)) + (psir*cos(theta)*cos(theta)*sin(phi)*sin(phi))/(cos(phi)) + (psir*sin(phi)*sin(phi)*sin(theta)*sin(theta))/(cos(phi)) - (cos(theta)*sin(phi)*(sin(phi))*(phip*sin(theta) + psir*cos(phi)*cos(theta)))/(cos(phi))/(cos(phi)) + (sin(phi)*sin(theta)*(sin(phi))*(phip*cos(theta) - psir*cos(phi)*sin(theta)))/(cos(phi))/(cos(phi)), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
     0, 0, 0, (cos(theta)*(sin(phi))*(phip*sin(theta) + psir*cos(phi)*cos(theta)))/(cos(phi))/(cos(phi)) - (psir*sin(phi)*sin(theta)*sin(theta))/(cos(phi)) - (psir*cos(theta)*cos(theta)*sin(phi))/(cos(phi)) - (sin(theta)*(sin(phi))*(phip*cos(theta) - psir*cos(phi)*sin(theta)))/(cos(phi))/cos(phi), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
     0, 0, 0, (cos(psi)*cos(theta) - sin(phi)*sin(psi)*sin(theta))*(ay*sin(phi)*sin(psi) - ax*cos(phi)*sin(psi)*sin(theta) + cos(phi)*cos(theta)*sin(psi)*(az - g)) - (cos(theta)*sin(psi) + cos(psi)*sin(phi)*sin(theta))*(ay*cos(psi)*sin(phi) - ax*cos(phi)*cos(psi)*sin(theta) + cos(phi)*cos(psi)*cos(theta)*(az - g)) - cos(phi)*sin(theta)*(ay*cos(phi) + ax*sin(phi)*sin(theta) - cos(theta)*sin(phi)*(az - g)) + sin(phi)*sin(theta)*(ay*sin(phi) - ax*cos(phi)*sin(theta) + cos(phi)*cos(theta)*(az - g)) - cos(phi)*sin(psi)*sin(theta)*(ax*(cos(psi)*cos(theta) - sin(phi)*sin(psi)*sin(theta)) + (cos(psi)*sin(theta) + cos(theta)*sin(phi)*sin(psi))*(az - g) - ay*cos(phi)*sin(psi)) + cos(phi)*cos(psi)*sin(theta)*(ax*(cos(theta)*sin(psi) + cos(psi)*sin(phi)*sin(theta)) + (sin(psi)*sin(theta) - cos(psi)*cos(theta)*sin(phi))*(az - g) + ay*cos(phi)*cos(psi)), cos(phi)*sin(theta)*(ax*cos(phi)*cos(theta) + cos(phi)*sin(theta)*(az - g)) - (cos(psi)*cos(theta) - sin(phi)*sin(psi)*sin(theta))*(ax*(cos(psi)*sin(theta) + cos(theta)*sin(phi)*sin(psi)) - (cos(psi)*cos(theta) - sin(phi)*sin(psi)*sin(theta))*(az - g)) - (sin(psi)*sin(theta) - cos(psi)*cos(theta)*sin(phi))*(ax*(cos(theta)*sin(psi) + cos(psi)*sin(phi)*sin(theta)) + (sin(psi)*sin(theta) - cos(psi)*cos(theta)*sin(phi))*(az - g) + ay*cos(phi)*cos(psi)) - (cos(psi)*sin(theta) + cos(theta)*sin(phi)*sin(psi))*(ax*(cos(psi)*cos(theta) - sin(phi)*sin(psi)*sin(theta)) + (cos(psi)*sin(theta) + cos(theta)*sin(phi)*sin(psi))*(az - g) - ay*cos(phi)*sin(psi)) - cos(phi)*cos(theta)*(ay*sin(phi) - ax*cos(phi)*sin(theta) + cos(phi)*cos(theta)*(az - g)) - (cos(theta)*sin(psi) + cos(psi)*sin(phi)*sin(theta))*(ax*(sin(psi)*sin(theta) - cos(psi)*cos(theta)*sin(phi)) - (cos(theta)*sin(psi) + cos(psi)*sin(phi)*sin(theta))*(az - g)), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, cos(phi)*(ay*sin(phi) - ax*cos(phi)*sin(theta) + cos(phi)*cos(theta)*(az - g)) + sin(phi)*(ay*cos(phi) + ax*sin(phi)*sin(theta) - cos(theta)*sin(phi)*(az - g)) - cos(psi)*sin(phi)*(ax*(cos(theta)*sin(psi) + cos(psi)*sin(phi)*sin(theta)) + (sin(psi)*sin(theta) - cos(psi)*cos(theta)*sin(phi))*(az - g) + ay*cos(phi)*cos(psi)) + sin(phi)*sin(psi)*(ax*(cos(psi)*cos(theta) - sin(phi)*sin(psi)*sin(theta)) + (cos(psi)*sin(theta) + cos(theta)*sin(phi)*sin(psi))*(az - g) - ay*cos(phi)*sin(psi)) - cos(phi)*cos(psi)*(ay*cos(psi)*sin(phi) - ax*cos(phi)*cos(psi)*sin(theta) + cos(phi)*cos(psi)*cos(theta)*(az - g)) - cos(phi)*sin(psi)*(ay*sin(phi)*sin(psi) - ax*cos(phi)*sin(psi)*sin(theta) + cos(phi)*cos(theta)*sin(psi)*(az - g)), cos(phi)*sin(psi)*(ax*(cos(psi)*sin(theta) + cos(theta)*sin(phi)*sin(psi)) - (cos(psi)*cos(theta) - sin(phi)*sin(psi)*sin(theta))*(az - g)) - cos(phi)*cos(psi)*(ax*(sin(psi)*sin(theta) - cos(psi)*cos(theta)*sin(phi)) - (cos(theta)*sin(psi) + cos(psi)*sin(phi)*sin(theta))*(az - g)) - sin(phi)*(ax*cos(phi)*cos(theta) + cos(phi)*sin(theta)*(az - g)), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
     0, 0, 0, (cos(psi)*sin(theta) + cos(theta)*sin(phi)*sin(psi))*(ay*sin(phi)*sin(psi) - ax*cos(phi)*sin(psi)*sin(theta) + cos(phi)*cos(theta)*sin(psi)*(az - g)) - (sin(psi)*sin(theta) - cos(psi)*cos(theta)*sin(phi))*(ay*cos(psi)*sin(phi) - ax*cos(phi)*cos(psi)*sin(theta) + cos(phi)*cos(psi)*cos(theta)*(az - g)) + cos(phi)*cos(theta)*(ay*cos(phi) + ax*sin(phi)*sin(theta) - cos(theta)*sin(phi)*(az - g)) - cos(theta)*sin(phi)*(ay*sin(phi) - ax*cos(phi)*sin(theta) + cos(phi)*cos(theta)*(az - g)) + cos(phi)*cos(theta)*sin(psi)*(ax*(cos(psi)*cos(theta) - sin(phi)*sin(psi)*sin(theta)) + (cos(psi)*sin(theta) + cos(theta)*sin(phi)*sin(psi))*(az - g) - ay*cos(phi)*sin(psi)) - cos(phi)*cos(psi)*cos(theta)*(ax*(cos(theta)*sin(psi) + cos(psi)*sin(phi)*sin(theta)) + (sin(psi)*sin(theta) - cos(psi)*cos(theta)*sin(phi))*(az - g) + ay*cos(phi)*cos(psi)), (cos(theta)*sin(psi) + cos(psi)*sin(phi)*sin(theta))*(ax*(cos(theta)*sin(psi) + cos(psi)*sin(phi)*sin(theta)) + (sin(psi)*sin(theta) - cos(psi)*cos(theta)*sin(phi))*(az - g) + ay*cos(phi)*cos(psi)) - (sin(psi)*sin(theta) - cos(psi)*cos(theta)*sin(phi))*(ax*(sin(psi)*sin(theta) - cos(psi)*cos(theta)*sin(phi)) - (cos(theta)*sin(psi) + cos(psi)*sin(phi)*sin(theta))*(az - g)) - (cos(psi)*sin(theta) + cos(theta)*sin(phi)*sin(psi))*(ax*(cos(psi)*sin(theta) + cos(theta)*sin(phi)*sin(psi)) - (cos(psi)*cos(theta) - sin(phi)*sin(psi)*sin(theta))*(az - g)) + (cos(psi)*cos(theta) - sin(phi)*sin(psi)*sin(theta))*(ax*(cos(psi)*cos(theta) - sin(phi)*sin(psi)*sin(theta)) + (cos(psi)*sin(theta) + cos(theta)*sin(phi)*sin(psi))*(az - g) - ay*cos(phi)*sin(psi)) - cos(phi)*cos(theta)*(ax*cos(phi)*cos(theta) + cos(phi)*sin(theta)*(az - g)) - cos(phi)*sin(theta)*(ay*sin(phi) - ax*cos(phi)*sin(theta) + cos(phi)*cos(theta)*(az - g)), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
     0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
     0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
     0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
     0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
     0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
     0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
     
     B<< 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0,
      0, 0, 0, 1, 0, 0,
      0, 0, 0, 0, 1, 0,
      0, 0, 0, 0, 0, 1,
      (cos(theta)*sin(psi) + cos(psi)*sin(phi)*sin(theta))*(cos(theta)*sin(psi) + cos(psi)*sin(phi)*sin(theta)) + (cos(psi)*cos(theta) - sin(phi)*sin(psi)*sin(theta))*(cos(psi)*cos(theta) - sin(phi)*sin(psi)*sin(theta)) + cos(phi)*sin(theta)*cos(phi)*sin(theta), cos(phi)*cos(psi)*(cos(theta)*sin(psi) + cos(psi)*sin(phi)*sin(theta)) - cos(phi)*sin(psi)*(cos(psi)*cos(theta) - sin(phi)*sin(psi)*sin(theta)) - sin(phi)*cos(phi)*sin(theta), (cos(psi)*sin(theta) + cos(theta)*sin(phi)*sin(psi))*(cos(psi)*cos(theta) - sin(phi)*sin(psi)*sin(theta)) + (sin(psi)*sin(theta) - cos(psi)*cos(theta)*sin(phi))*(cos(theta)*sin(psi) + cos(psi)*sin(phi)*sin(theta)) - cos(phi)*cos(theta)*cos(phi)*sin(theta), 0, 0, 0,
      cos(phi)*cos(psi)*(cos(theta)*sin(psi) + cos(psi)*sin(phi)*sin(theta)) - cos(phi)*sin(psi)*(cos(psi)*cos(theta) - sin(phi)*sin(psi)*sin(theta)) - cos(phi)*sin(theta)*sin(phi), sin(phi)*sin(phi) + cos(phi)*cos(psi)*cos(phi)*cos(psi) + cos(phi)*sin(psi)*cos(phi)*sin(psi), cos(phi)*cos(psi)*(sin(psi)*sin(theta) - cos(psi)*cos(theta)*sin(phi)) - cos(phi)*sin(psi)*(cos(psi)*sin(theta) + cos(theta)*sin(phi)*sin(psi)) + cos(phi)*cos(theta)*sin(phi), 0, 0, 0,
       (cos(theta)*sin(psi) + cos(psi)*sin(phi)*sin(theta))*(sin(psi)*sin(theta) - cos(psi)*cos(theta)*sin(phi)) + (cos(psi)*cos(theta) - sin(phi)*sin(psi)*sin(theta))*(cos(psi)*sin(theta) + cos(theta)*sin(phi)*sin(psi)) - cos(phi)*sin(theta)*cos(phi)*cos(theta), cos(phi)*cos(psi)*(sin(psi)*sin(theta) - cos(psi)*cos(theta)*sin(phi)) + sin(phi)*cos(phi)*cos(theta) - cos(phi)*sin(psi)*(cos(psi)*sin(theta) + cos(theta)*sin(phi)*sin(psi)), (cos(psi)*sin(theta) + cos(theta)*sin(phi)*sin(psi))*(cos(psi)*sin(theta) + cos(theta)*sin(phi)*sin(psi)) + (sin(psi)*sin(theta) - cos(psi)*cos(theta)*sin(phi))*(sin(psi)*sin(theta) - cos(psi)*cos(theta)*sin(phi)) + cos(phi)*cos(theta)*cos(phi)*cos(theta), 0, 0, 0,
      0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0;*/
    
    U<<   0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 ,
          0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 ,
          0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 ,
          0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 ,
          0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 ,
          0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 ,
          0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 ,
          0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 ,
          0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 ,
          0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0 ,
          0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0 ,
          0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1 ,
          0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0 ,
          0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0 ,
          0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0 ;

    U.block(6,0,3,3)<<cos(psi)*cos(theta)-sin(phi)*sin(psi)*sin(theta), cos(theta)*sin(psi)+cos(psi)*sin(phi)*sin(theta), -cos(phi)*sin(theta),-cos(phi)*sin(psi),cos(phi)*cos(psi),sin(phi),cos(psi)*sin(theta)+cos(theta)*sin(phi)*sin(psi), sin(psi)*sin(theta)-cos(psi)*cos(theta)*sin(phi),cos(phi)*cos(theta);

	cout<<1<<endl;

    U.block(6,0,3,3)=(-1)*U.block(6,0,3,3);
    U.block(3,3,3,3)=-(G.inverse());


    
    cout<<2<<endl;

    Matrix3d Rphi;
    Matrix3d Rtheta;
    Matrix3d Rpsi;
    Rphi<<-cos(phi)*sin(psi)*sin(theta),  cos(phi)*cos(psi)*sin(theta),  sin(phi)*sin(theta),
                 sin(phi)*sin(psi),            -cos(psi)*sin(phi),             cos(phi),
        cos(phi)*cos(theta)*sin(psi), -cos(phi)*cos(psi)*cos(theta), -cos(theta)*sin(phi);
    Rtheta<<- cos(psi)*sin(theta) - cos(theta)*sin(phi)*sin(psi), cos(psi)*cos(theta)*sin(phi) - sin(psi)*sin(theta), -cos(phi)*cos(theta),
                    0,                                   0,                    0,
        cos(psi)*cos(theta) - sin(phi)*sin(psi)*sin(theta), cos(theta)*sin(psi) + cos(psi)*sin(phi)*sin(theta), -cos(phi)*sin(theta);
    Rpsi<<- cos(theta)*sin(psi) - cos(psi)*sin(phi)*sin(theta), cos(psi)*cos(theta) - sin(phi)*sin(psi)*sin(theta), 0,
            -cos(phi)*cos(psi),          -cos(phi)*sin(psi), 0,
            cos(psi)*cos(theta)*sin(phi) - sin(psi)*sin(theta), cos(psi)*sin(theta) + cos(theta)*sin(phi)*sin(psi), 0;
    
    A.block(0,6,3,3)=Matrix3d::Identity();
    A.block(3,3,3,3)<<  0, r*cos(theta) - p*sin(theta), 0,
    p*sin(theta)-r*cos(theta) - r*cos(theta)*sin(phi)*sin(phi)/cos(phi)/cos(phi) + p*sin(phi)*sin(theta)*sin(phi)/cos(phi)/cos(phi),p*cos(theta)*sin(phi)/(cos(phi)) + r*sin(phi)*sin(theta)/cos(phi), 0, 
    r*cos(theta)*sin(phi)/cos(phi)/cos(phi) - p*sin(theta)*sin(phi)/cos(phi)/cos(phi),                 - p*cos(theta)/cos(phi) - r*sin(theta)/cos(phi),0;
                                                                                                                                                                                                                                                                                                                                                                                                     
                                                                                                                                                                                                                                                                                                                                                                                                     
    A.block(6,3,3,1)=Rphi*am;
    A.block(6,4,3,1)=Rtheta*am;
    A.block(6,5,3,1)=Rpsi*am;

    B.block(3,3,3,3)=G.inverse();
    B.block(6,0,3,3)<<cos(psi)*cos(theta)-sin(phi)*sin(psi)*sin(theta), cos(theta)*sin(psi)+cos(psi)*sin(phi)*sin(theta), -cos(phi)*sin(theta),-cos(phi)*sin(psi),cos(phi)*cos(psi),sin(phi),cos(psi)*sin(theta)+cos(theta)*sin(phi)*sin(psi), sin(psi)*sin(theta)-cos(psi)*cos(theta)*sin(phi),cos(phi)*cos(theta);
    
    mut=X+dt*dX;
    F=MatrixXd::Identity(15,15)+dt*A;
    V=dt*U;
    covariance=F*covar*(F.transpose())+V*Q*(V.transpose());
    
    //cout<<mut<<"\n\n\n\n\n\n\n";
    cout<<3<<endl;
    imu_time=time;
    
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
    //Rcam*Z + (0 -0.04 -0.02) =IMU frame
    
    MatrixXd R;
    MatrixXd K;
    MatrixXd offset=MatrixXd::Zero(3,1);
    offset<<0,-0.04,-0.02;
    odo_time=msg->header.stamp.sec;
    if (imu_time==-1.0) imu_time=odo_time;
    double x=msg->pose.pose.position.x;
    double y=msg->pose.pose.position.y;
    double z=msg->pose.pose.position.z;
    double qw=msg->pose.pose.orientation.w;
    double qx=msg->pose.pose.orientation.x;
    double qy=msg->pose.pose.orientation.y;
    double qz=msg->pose.pose.orientation.z;
    Quaterniond m=Quaterniond(qw,qx,qy,qz);
    R=m.toRotationMatrix();
    
    
    double phi_imu=mut(3,0);
    double theta_imu=mut(4,0);
    double psi_imu=mut(5,0);
    Matrix3d Rot;
    Rot<<cos(psi_imu)*cos(theta_imu)-sin(phi_imu)*sin(psi_imu)*sin(theta_imu), cos(theta_imu)*sin(psi_imu)+cos(psi_imu)*sin(phi_imu)*sin(theta_imu), -cos(phi_imu)*sin(theta_imu),-cos(phi_imu)*sin(psi_imu),cos(phi_imu)*cos(psi_imu),sin(phi_imu),cos(psi_imu)*sin(theta_imu)+cos(theta_imu)*sin(phi_imu)*sin(psi_imu), sin(psi_imu)*sin(theta_imu)-cos(psi_imu)*cos(theta_imu)*sin(phi_imu),cos(phi_imu)*cos(theta_imu);
    
    
    //cout<<imu_euler<<phi_imu<<theta_imu<<psi_imu<<"\n\n\n\n\n";
    Z<<x,y,z,0,0,0;
    Z.block(0,0,3,1)=R.transpose()*(Rcam.transpose()*(-offset)-Z.block(0,0,3,1));
    Matrix3d R0=(Rcam*R).transpose();
    double phi=asin(R0(1,2));
    double psi = atan2(-R0(1,0)/cos(phi),R0(1,1)/cos(phi));
    double theta = atan2(-R0(0,2)/cos(phi),R0(2,2)/cos(phi));
    Z(3,0)=phi;
    Z(4,0)=theta;
    Z(5,0)=psi;
    
    if (phi_imu>phi && phi_imu-phi>2*pi-phi_imu+phi) Z(3,0)=phi+2*pi;
    if (phi_imu<phi && phi-phi_imu > 2*pi-phi+phi_imu) Z(3,0)=Z(3,0)-2*pi;
    if (theta_imu>theta && theta_imu-theta>2*pi-theta_imu+theta) Z(4,0)=theta+2*pi;
    if (theta_imu<theta && theta-theta_imu > 2*pi-theta+theta_imu) Z(4,0)=Z(4,0)-2*pi;
    if (psi_imu>psi && psi_imu-psi>2*pi-psi_imu+psi) Z(5,0)=psi+2*pi;
    if (psi_imu<psi && psi-psi_imu > 2*pi-psi+psi_imu) Z(5,0)=Z(5,0)-2*pi;
    
    
    
    cout<<"a"<<endl;
    K=covariance*(C.transpose())*((C*covariance*(C.transpose())+W*Rt*(W.transpose())).inverse());
    //K=MatrixXd::Zero(6,6);
	cout<<"aa"<<endl;
    X=mut+K*(Z-(mut.block(0,0,6,1)));
    cout<<"b"<<endl;
    cout<<(Z-(mut.block(0,0,6,1)))<<"\n";
    cout<<X(4)<<"\n\n\n\n\n";
    
    covar=covariance-K*C*covariance;
    
    nav_msgs::Odometry odom;
    odom.header.stamp = msg->header.stamp;
    odom.header.frame_id = "ekf_odom";
    odom.pose.pose.position.x=X(0);
    odom.pose.pose.position.y=X(1);
    odom.pose.pose.position.z=X(2);
    phi=X(3);
    theta=X(4);
    psi=X(5);
    Rot<<cos(psi)*cos(theta)-sin(phi)*sin(psi)*sin(theta), cos(theta)*sin(psi)+cos(psi)*sin(phi)*sin(theta), -cos(phi)*sin(theta),-cos(phi)*sin(psi),cos(phi)*cos(psi),sin(phi),cos(psi)*sin(theta)+cos(theta)*sin(phi)*sin(psi), sin(psi)*sin(theta)-cos(psi)*cos(theta)*sin(phi),cos(phi)*cos(theta);
    m=Quaterniond(Rot);
    odom.pose.pose.orientation.w=m.w();
    odom.pose.pose.orientation.x=m.x();
    odom.pose.pose.orientation.y=m.y();
    odom.pose.pose.orientation.z=m.z();
    odom_pub.publish(odom);
    
}

int main(int argc, char **argv)
{
    double var_na=0.01; double var_nba=0.01; double var_ng=0.01; double var_nbg=0.01;
    double var_p=0.5; double var_q=0.5;
    Q.block(0,0,3,3)=Matrix3d::Identity()*var_na;
    Q.block(3,3,3,3)=Matrix3d::Identity()*var_ng;
    Q.block(6,6,3,3)=Matrix3d::Identity()*var_nba;
    Q.block(9,9,3,3)=Matrix3d::Identity()*var_nbg;
    Rt.block(0,0,3,3)=Matrix3d::Identity()*var_p;
    Rt.block(3,3,3,3)=Matrix3d::Identity()*var_q;
    Rt(5,5)=0.05;
    
    ros::init(argc, argv, "ekf");
    ros::NodeHandle n("~");
    ros::Subscriber s1 = n.subscribe("imu", 1000, imu_callback);
    ros::Subscriber s2 = n.subscribe("tag_odom", 1000, odom_callback);
    odom_pub = n.advertise<nav_msgs::Odometry>("ekf_odom", 100);
    Rcam = Quaterniond(0, 0, -1, 0).toRotationMatrix();
    cout << "R_cam" << endl << Rcam << endl;
    // Q imu covariance matrix; Rt visual odomtry covariance matrix
    //Q.topLeftCorner(6, 6) = 0.01 * Q.topLeftCorner(6, 6);
    //Q.bottomRightCorner(6, 6) = 0.01 * Q.bottomRightCorner(6, 6);
    //Rt.topLeftCorner(3, 3) = 0.5 * Rt.topLeftCorner(3, 3);
    //Rt.bottomRightCorner(3, 3) = 0.5 * Rt.bottomRightCorner(3, 3);
    //Rt.bottomRightCorner(1, 1) = 0.1 * Rt.bottomRightCorner(1, 1);

    ros::spin();
}
