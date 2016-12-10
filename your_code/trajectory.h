#ifndef trajectory_h
#define trajectory_h
#include <stdio.h>
#include <Eigen/Eigen>
#include <math.h>

#define DEBUG false

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
    const int M = 9; // number of waypoints
    //your code // please use coefficients from matlab to get desired states
    /***********************************************************************/
    const double path1[3*M] = { 0.0, 0.0, 0.0, \
                                0.5, 0.0, 0.0, \
                                1.0, 0.0, 0.0, \
                                1.5, 0.0, 0.0, \
                                1.5, 0.5, 0.0, \
                                1.0, 0.5, 0.0, \
                                0.5, 0.5, 0.0, \
                                0.0, 0.5, 0.0, \
                                0.0, 0.0, 0.0};

    const double T[M] = {0.0, 3.125, 6.25, 9.375, 12.5, 15.625, 18.75, 21.875, 25};

    const double Px[R*(M-1)] = {  0.0,       0,       0,  0.0331,    0, -0.0032,  0.0006,   0,\
                                  0.5,  0.2675, -0.0400, -0.0027,    0,  0.0009, -0.0002,   0,\
                                  1.0,  0.1272,  0.0288, -0.0047,    0, -0.0002,       0,   0,\
                                  1.5,  0.1202, -0.0401,  0.0002,    0,       0,       0,   0,\
                                  1.5, -0.1106, -0.0311,  0.0050,    0,       0,       0,   0,\
                                  1.0, -0.1636,  0.0129, -0.0040,    0,       0,       0,   0,\
                                  0.5, -0.1892, -0.0185,  0.0096,    0,       0,       0,   0,\
                                  0.0, -0.0568,  0.0518, -0.0135,    0,  0.0002,       0,   0};










    const double Py[R*(M-1)] = {  0.0,       0,       0, -0.0004,    0,       0,       0,   0,\
                                  0.0,  0.0049,  0.0045, -0.0018,    0,       0,       0,   0,\
                                  0.0, -0.0253, -0.0150,  0.0073,    0,       0,       0,   0,\
                                  0.0,  0.0982,  0.0550, -0.0112,    0,       0,       0,   0,\
                                  0.5,  0.1106, -0.0519,  0.0050,    0,       0,       0,   0,\
                                  0.5, -0.0548,  0.0020,  0.0059,    0,       0,       0,   0,\
                                  0.5,  0.0873,  0.0323, -0.0226,    0,  0.0003,       0,   0,\
                                  0.5, -0.2156, -0.0873,  0.0462,    0, -0.0011, -0.0002,   0};

    const double Pz[R*(M-1)] = {  0, 0, 0, 0, 0, 0, 0, 0,\
                                  0, 0, 0, 0, 0, 0, 0, 0,\
                                  0, 0, 0, 0, 0, 0, 0, 0,\
                                  0, 0, 0, 0, 0, 0, 0, 0,\
                                  0, 0, 0, 0, 0, 0, 0, 0,\
                                  0, 0, 0, 0, 0, 0, 0, 0,\
                                  0, 0, 0, 0, 0, 0, 0, 0,\
                                  0, 0, 0, 0, 0, 0, 0, 0};


    // pre-process, using multi-segment first,
    // with trajectory generated offline

    // calculate the output
    int m = 0;
    double t = 0;
    if (dT > T[M-1]) {
        t = T[M-1];
        m = M-1;
        desired_p[0] = hover_p[0] + path1[3*M-3];
        desired_p[1] = hover_p[1] + path1[3*M-2];
        desired_p[2] = hover_p[2] + path1[3*M-1];
        desired_v[0] = 0;
        desired_v[1] = 0;
        desired_v[2] = 0;
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
    if (DEBUG) {
        std::cout << "Time used: " << t << ".\n";//print in C++
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
    }
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
