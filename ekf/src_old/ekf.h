#ifndef EKF_H
#define EKF_H
// Reconstructed by Beck Pang, Dec. 9th, 2016
// Credit to the source code from ELEC6910P TA's 2015 code
#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <Eigen/Eigen>
#include <math.h>
#include <fstream>
#include "pose.h"

using namespace std;
using namespace Eigen;

typedef struct State
{
		ros::Time stamp;
		VectorXd  u; // IMU input u : acceleration, augular_velocity
		VectorXd  mean;
		MatrixXd  var;
} State;

class EKF
{
	private:
		// A time sequencied states stored in a stack
		vector<State> StateStack;

		// boolean for the initialization status
		bool init;

		double gravity;

		Vector3d g;

		// Store present state roll pitch yaw
		Vector3d rpy_ps;

		// Q: x^ = f(x, u, n), 	n~N(0, Q), process model Gaussian white noise covariance;
		// Rt: z = g(x, v), 	v~N(0, Rt),observation Gaussian white noise covariance;
		MatrixXd Q,	 R, I3, I15;

		// Matrixes involved in IMU propogation
		// For continous time, 	x^ = Ax + Bu + Un
		// To discrete time, 	x^ = Fx + Gu + Vn
		// At: 15 x 15, continous state space model, Jacobian of the process model over state x
		// Ut: 15 x 12, continous state space model, Jacobian over linear Gaussian white noise
		// Ft: 15 x 15, discrete time state space model
		// Vt: 15 x 12, discrete time state space model
		MatrixXd At, Ut, Ft, Vt;

		// Matrixes involved in camera Update
		// Ct: observation model Jacobian over state x
		// Wt: observation model Jacobain over observation v
		MatrixXd Ct, Wt;

	public:
		EKF();
		~EKF();

		bool isInit();
		VectorXd Get_State();
		ros::Time Get_Time();

		void SetParam(double var_g, double var_a, double var_bg, double var_ba, double var_p, double var_q);
		void SetInit( VectorXd zt, ros::Time stamp);
		void IMU_Propagation( VectorXd u, ros::Time stamp);
		void Odom_Update( VectorXd zt, ros::Time stamp);
};

#endif
