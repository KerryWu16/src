#include "ekf.h"

EKF::EKF()  
{
	init = false;

	gravity = 9.7;
  	g << 0,
  	     0,
  	     gravity;

  	I3 = MatrixXd::Identity(3,3);
	I15 = MatrixXd::Identity(15,15);
}

      	
EKF::~EKF() { }

bool EKF::isInit()
{
		return init;
}


VectorXd EKF::GetState()
{
	return StateHist[StateHist.size()-1].mean;
}
ros::Time EKF::GetTime()
{
	return StateHist[StateHist.size()-1].stamp;
}

void EKF::SetParam(double var_g, double var_a, double var_bg, double var_ba, double var_p, double var_q)
{
	Q.setZero(12,12);
	R.setZero(6, 6);
	W.setZero(6, 6);
	C.setZero(6, 15); 
	A.setZero(15,15);
    F.setZero(15, 15);
    U.setZero(15,12);
	V.setZero(15,12);

	for(int i=0;i<3; i++){
        Q(i,i) = var_g;}
    for(int i=3;i<6;i++){
    	Q(i,i) = var_a;}
    for(int i=6;i<9;i++){
        Q(i,i) = var_bg;}
    for(int i=9;i<12;i++){
        Q(i,i) = var_ba;}

    for(int i=0;i<3; i++){
        R(i,i) = var_p;}
    for(int i=3;i<6;i++){
        R(i,i) = var_q;}

    W.block(0, 0, 3, 3) =  I3;
    W.block(3, 3, 3, 3) =  I3;
    C.block(0, 0, 3, 3) =  I3;
    C.block(3, 3, 3, 3) =  I3;
}

void EKF::SetInit(VectorXd Z, ros::Time stamp)
{
    StateHist.clear();
    VectorXd mean_init = Eigen::VectorXd::Zero(15);
    MatrixXd var_init  = Eigen::MatrixXd::Zero(15,15);

    mean_init.segment<6>(0) = Z;
    State state_init;
    state_init.mean = mean_init;
    state_init.var  = var_init;
    state_init.stamp = stamp;
    StateHist.push_back(state_init);
    init = true;

}

void EKF::ImuPropagation(VectorXd u, ros::Time stamp)
{	
	Matrix3d G, Ginv, Rot, R_acc_dot, Ginv_w_dot;
    State state_last = StateHist[StateHist.size()-1];
    VectorXd mean_last = state_last.mean;
    MatrixXd var_last  = state_last.var;

    Vector3d X2, X3, X4, X5;
    X2 = mean_last.segment<3>(3); 
    X3 = mean_last.segment<3>(6); 
    X4 = mean_last.segment<3>(9); 
    X5 = mean_last.segment<3>(12); 
    Vector3d rpy = X2;
    double fai = rpy(0), theta = rpy(1), sai = rpy(2);
    Vector3d w   = u.segment<3>(0) - X4;
    Vector3d acc = u.segment<3>(3) - X5;
	
    G <<    cos(theta),    0,     -cos(fai)*sin(theta),
            0,             1,           sin(fai),
            sin(theta),    0,      cos(fai)*cos(theta);

    Ginv<<   cos(theta),                       0,               sin(theta),
            (sin(fai)*sin(theta))/cos(fai),    1,              -(cos(theta)*sin(fai))/cos(fai),
            -sin(theta)/cos(fai),              0,               cos(theta)/cos(fai);

    Rot = rpy_to_R(rpy);
   
    Ginv_w_dot << 0,                                                                w(2)*cos(theta) - w(0)*sin(theta),                      0,
                -(w(2)*cos(theta) - w(0)*sin(theta))/pow(cos(fai),2),              (sin(fai)*(w(0)*cos(theta) + w(2)*sin(theta)))/cos(fai), 0,
                 (sin(fai)*(w(2)*cos(theta) - w(0)*sin(theta)))/pow(cos(fai),2),  -(w(0)*cos(theta) + w(2)*sin(theta))/cos(fai),            0;

    R_acc_dot  << sin(sai)*(acc(1)*sin(fai) + acc(2)*cos(fai)*cos(theta) - acc(0)*cos(fai)*sin(theta)), acc(2)*(cos(sai)*cos(theta) - sin(fai)*sin(sai)*sin(theta)) - acc(0)*(cos(sai)*sin(theta) + cos(theta)*sin(fai)*sin(sai)),  -acc(0)*(cos(theta)*sin(sai) + cos(sai)*sin(fai)*sin(theta)) - acc(2)*(sin(sai)*sin(theta) - cos(sai)*cos(theta)*sin(fai)) - acc(1)*cos(fai)*cos(sai),
                 -cos(sai)*(acc(1)*sin(fai) + acc(2)*cos(fai)*cos(theta) - acc(0)*cos(fai)*sin(theta)), acc(2)*(cos(theta)*sin(sai) + cos(sai)*sin(fai)*sin(theta)) - acc(0)*(sin(sai)*sin(theta) - cos(sai)*cos(theta)*sin(fai)),   acc(0)*(cos(sai)*cos(theta) - sin(fai)*sin(sai)*sin(theta)) + acc(2)*(cos(sai)*sin(theta) + cos(theta)*sin(fai)*sin(sai)) - acc(1)*cos(fai)*sin(sai),
                  acc(1)*cos(fai) - acc(2)*cos(theta)*sin(fai) + acc(0)*sin(fai)*sin(theta),           -cos(fai)*(acc(0)*cos(theta) + acc(2)*sin(theta)),                                                                            0;

    A.block(0, 6, 3, 3) =  I3;
    A.block(3, 3, 3, 3) = Ginv_w_dot;
    A.block(3, 9, 3, 3) = -G.inverse();
   	A.block(6, 3, 3, 3) = R_acc_dot;
   	A.block(6,12, 3, 3) = -Rot;

    U.block(3, 0, 3, 3) = -G.inverse();
    U.block(6, 3, 3, 3) = -Rot;
    U.block(9, 6, 3, 3) = I3;
    U.block(12,9, 3, 3) = I3;

    double dT = (stamp - state_last.stamp).toSec();

    F = I15 + dT * A;
    V = dT * U;

    VectorXd xdot = Eigen::VectorXd::Zero(15);
    xdot.segment(0, 3) = X3;
    xdot.segment(3, 3) = Ginv * w;
    xdot.segment(6, 3) = Rot * acc + g;

    State state_new;
    state_new.mean  = mean_last + dT * xdot;
    state_new.var   = F * var_last * F.transpose() + V * Q * V.transpose();
    state_new.stamp = stamp;
    state_new.u     = u;
    StateHist.push_back(state_new);
}

Eigen::Vector3d lst_rpy;
void EKF::OdomUpdate(VectorXd z, ros::Time stamp)
{
    State state_last;
    vector< State >::iterator itr;
    vector< State >::iterator itr2;
    vector< State > state3;

    state_last = StateHist[StateHist.size()-1];
    for(itr = StateHist.begin(); itr != StateHist.end(); itr ++){   
        if(itr->stamp > stamp || itr->stamp == stamp) {	
            state_last = *(itr-1);
            state3.assign(itr,StateHist.end());
            break;
        }
    }

    StateHist.clear();
    Eigen::MatrixXd K = Eigen::MatrixXd::Zero(15,6);
    Eigen::MatrixXd K1 = Eigen::MatrixXd::Zero(15,6);
    Eigen::MatrixXd K2 = Eigen::MatrixXd::Zero(6,6);
    Eigen::MatrixXd K3 = Eigen::MatrixXd::Zero(6,6);

    K1 = state_last.var * (C.transpose());
    K2 = C*state_last.var*(C.transpose())+W*R*(W.transpose()) ;
    K3 = K2.inverse();
    K = K1*K3;

    for(int i=3;i<6;i++){
        if(abs(lst_rpy(i-3)-z(i))>M_PI){
            if(z(i)<0) z(i) = 2* M_PI + z(i);
            else z(i) = -2* M_PI + z(i);   
        }         
    }

    lst_rpy(0) = z(3);
    lst_rpy(1) = z(4);
    lst_rpy(2) = z(5);

    State state_new;
    state_new.mean = state_last.mean + K*(z-C*state_last.mean);
    state_new.var  = state_last.var - K*C*state_last.var;
    state_new.stamp = state_last.stamp;

    StateHist.push_back(state_new);
    for(itr2 = state3.begin(); itr2 != state3.end(); itr2 ++){   
        VectorXd u_again = itr2->u;
        ros::Time stamp_again = itr2->stamp;
        EKF::ImuPropagation(u_again, stamp_again);
    }
}