#ifndef EKF_H
#define EKF_H

#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <Eigen/Eigen>
#include <fstream>
#include <math.h>
#include "pose.h"

#define PI M_PI

using namespace std;
using namespace Eigen;
struct State
{
		ros::Time stamp;
		VectorXd  u;
		VectorXd  mean;
		MatrixXd  var;
};
class EKF
{

	private:
		vector<State> StateHist;

		bool init;
		double gravity;
		Vector3d g;
		MatrixXd Q, R, W, C, I3, I15; 
		MatrixXd A, F, U, V;

 	public:
		EKF();
		~EKF();
		
		bool isInit();
		VectorXd GetState();
		ros::Time GetTime();
		void SetParam(double var_g, double var_a, double var_bg, double var_ba, double var_p, double var_q);
		void SetInit(VectorXd Z, ros::Time stamp);
		void ImuPropagation(VectorXd u, ros::Time stamp);
		void OdomUpdate(VectorXd z, ros::Time time);

};

#endif