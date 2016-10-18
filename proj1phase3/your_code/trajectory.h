#ifndef trajectory_h
#define trajectory_h
#include <stdio.h>

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
    //if you don't want to use Eigen, then you can use these arrays
    //or you can delete them and use Eigen
    double hover_p[3], now_v[3], desired_p[3], desired_v[3], desired_a[3];
    hover_p[0] = hover_pos.x();
    hover_p[1] = hover_pos.y();
    hover_p[2] = hover_pos.z();
    now_v[0] = now_vel.x();
    now_v[1] = now_vel.y();
    now_v[2] = now_vel.z();
    //your code // please use coefficients from matlab to get desired states

    //output
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
