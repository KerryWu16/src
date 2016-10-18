#ifndef     _FLY_CYCLE_H_
#define     _FLY_CYCLE_H_

#include    "ros/ros.h"
#include    "mavlink_message/PositionCommand.h"
#include    <iostream>
#include    <cmath>
#include    "Eigen/Eigen"

void    fly_cycle(  Eigen::Vector3d     init_pos, 
                    Eigen::Vector3d     now_position,
                    double  delta_t,
                    double  CycleR,
                    double  CycleV,
                    mavlink_message::PositionCommand *set_pos
                    )
{
    double      X, Y, Z, VX, VY, VZ, AX, AY, AZ;
    double      theta = CycleV * delta_t / CycleR - M_PI/2;
    X   = CycleR    + CycleR * sin(theta)   + init_pos.x();
    Y   = CycleR * cos(theta)   + init_pos.y();
    Z   = init_pos.z();
    VX  = CycleV * cos(theta);
    VY  = -CycleV * sin(theta);
    VZ  = 0;
    AX  = -( CycleV * CycleV / CycleR) * sin(theta);
    AY  = -( CycleV * CycleV / CycleR) * cos(theta);
    AZ  = 0;
    
    set_pos->position.x  = X;
    set_pos->position.y  = Y;
    set_pos->position.z  = Z;
    set_pos->velocity.x  = VX;
    set_pos->velocity.y  = VY;
    set_pos->velocity.z  = VZ;
    set_pos->accelerate.x    = AX;
    set_pos->accelerate.y    = AY;
    set_pos->accelerate.z    = AZ;
}
#endif
