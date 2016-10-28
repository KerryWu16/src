#ifndef trajectory_h
#define trajectory_h
#include <stdio.h>
#include <math.h>

using namespace Eigen;

/*
 * this function is to get desired states for specific trajectory, just generated, at time dt.
 * input:
 * dT   -> the time
 * hover_pos -> the desired position where you want quadrotor to hover
 * now_vel -> maybe useless
 *
 * output:
 * desired_pos -> desired position at dT
 * desired_vel -> desired velocity at dT
 * desired_acc -> desired acceleration at dT
 * return:
 * true  -> you have alread configured desired states
 * false -> no desired state
 */
bool trajectory_control(const double dT,
        const Vector3d hover_pos,
        const Vector3d now_vel,
        Vector3d & desired_pos,
        Vector3d & desired_vel,
        Vector3d & desired_acc
        )
{
    // //if you don't want to use Eigen, then you can use these arrays
    // //or you can delete them and use Eigen
    double hover_p[3], now_v[3], desired_p[3], desired_v[3], desired_a[3];
    hover_p[0] = hover_pos.x();
    hover_p[1] = hover_pos.y();
    hover_p[2] = hover_pos.z();
    now_v[0] = now_vel.x();
    now_v[1] = now_vel.y();
    now_v[2] = now_vel.z();
    const int R = 8; // Using R - 1 th order polynomial for the trajectory
    const int v_limit = 0.5;  // maximum velocity
    const int M = 4; // number of waypoints
    //your code // please use coefficients from matlab to get desired states
    /***********************************************************************/
    const double path1[3*M] = { 0.0, 0.0, 0.0, \
                                1.0, 0.0, 0.0, \
                                0.0, 0.0, 0.0, \
                                -1.0, 0.0, 0.0};
    const double T[M] = {0.0, 3.3333, 6.6667, 10};
    const double Px[R*(M-1)] = {  0,       0,      0,  0.0660,    0,  -0.0066,  0.0011,   0,\
                                  1,  0.3313,-0.2172, -0.0045,    0,   0.0019,       0,   0,\
                                  0, -0.6379, 0.0428,  0.0299,    0,  -0.0012,       0,   0};
    const double Py[R*(M-1)] = {0};
    const double Pz[R*(M-1)] = {0};

    // pre-process, using multi-segment first,
    // with trajectory generated offline


    // calculate the output
    int m = 0;
    int t = 0;
    if (dT > T[M-1]) {
      	t = T[M-1];
    } else {
        t = dT;
        for (int j = 0; j < M; j++) {
            if (t >= T[j] && t < T[j+1]) {
                m = j; break;
            }
        }
    	std::cout << "period: " << m << "\n";
        desired_p[0] = hover_p[0];
        desired_p[1] = hover_p[1];
        desired_p[2] = hover_p[2];
        for (int i = 0; i < R; i++) {
            desired_p[0] = desired_p[0] + Px[i+m*R] * pow((t-T[m]), i);
            desired_p[1] = desired_p[1] + Py[i+m*R] * pow((t-T[m]), i);
            desired_p[2] = desired_p[2] + Pz[i+m*R] * pow((t-T[m]), i);
            if (i == 0) {
                desired_v[0]=0;
                desired_v[1]=0;
                desired_v[2]=0;
            }
            if (i > 0) {
                desired_v[0] = desired_v[0] + i * Px[i+m*R] * pow((t-T[m]), (i-1));
                desired_v[1] = desired_v[1] + i * Py[i+m*R] * pow((t-T[m]), (i-1));
                desired_v[2] = desired_v[2] + i * Pz[i+m*R] * pow((t-T[m]), (i-1));
            }
        }
    }
    desired_a[0] = 0;
    desired_a[1] = 0;
    desired_a[2] = 0;

    std::cout << "Time: " << dT << ".\n";//print in C++
    std::cout << "x: " << desired_p[0] << "\n";
    std::cout << "y: " << desired_p[1] << "\n";
    std::cout << "z: " << desired_p[2] << "\n";
    std::cout << "pos_x: " << hover_p[0] << "\n";
    std::cout << "pos_y: " << hover_p[1] << "\n";
    std::cout << "pos_z: " << hover_p[2] << "\n";
    std::cout << "x_v: " << desired_v[0] << "\n";
    std::cout << "y_v: " << desired_v[1] << "\n";
    std::cout << "z_v: " << desired_v[2] << "\n";
    /*************************************************************************/
    // //output
    desired_pos.x() = desired_p[0];
    desired_pos.y() = desired_p[1];
    desired_pos.z() = desired_p[2];
    desired_vel.x() = desired_v[0];
    desired_vel.y() = desired_v[1];
    desired_vel.z() = desired_v[2];
    desired_acc.x() = desired_a[0];
    desired_acc.y() = desired_a[1];
    desired_acc.z() = desired_a[2];

    return true; // if you have got desired states, true.
}
#endif
