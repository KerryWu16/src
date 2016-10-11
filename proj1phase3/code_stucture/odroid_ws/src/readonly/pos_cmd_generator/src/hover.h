#ifndef     _HOVER_H_
#define     _HOVER_H_

#include    "ros/ros.h"
#include    "mavlink_message/PositionCommand.h"
#include    "mavlink_message/rc_chans.h"
#include    <iostream>
#include    <cmath>
#include    "Eigen/Eigen"
#include    "include/pose_.h"

void    hover(  Eigen::Vector3d     init_pos,
                Eigen::Vector3d     now_position,
                Eigen::Quaterniond  att_b2w,
                mavlink_message::rc_chans   rc,
                Eigen::Vector3d     &target_from_init,
                bool    work_sonar,
                bool    &static_hover,
                double  delta_t,
                double  RCVel,
                double  YawVel,
                double  &yaw_tar,
                double  hover_init_thrust,
                mavlink_message::PositionCommand    &set_pos
             )
{
    Eigen::Vector3d     target;
    double  chan1, chan2, chan3, chan4;
    double  yaw_rate;
    Eigen::Vector3d     rc_ratio;
    static_hover = true;

    if ((rc.chan3_raw < hover_init_thrust + 50) && (rc.chan3_raw > hover_init_thrust - 50))
        chan3 = hover_init_thrust;
    else if (rc.chan3_raw >= hover_init_thrust + 50)
    {
        chan3  = rc.chan3_raw - 50;
        static_hover = false;
    }
    else if (rc.chan3_raw <= hover_init_thrust - 50)
    {
        chan3 = rc.chan3_raw + 50;
        static_hover = false;
    }
    else
        chan3 = 1500;

    if ( rc.chan1_raw   > 1900 )
        chan1   = 1900.0;
    else if ( rc.chan1_raw < 1100 )
        chan1   = 1100.0;
    else if ((rc.chan1_raw < 1520) && (rc.chan1_raw > 1480))
        chan1 = 1500;
    else
    {
        chan1  = rc.chan1_raw;
        static_hover = false;
    }

    if ( rc.chan2_raw   > 1900 )
        chan2   = 1900.0;
    else if ( rc.chan2_raw < 1100 )
        chan2   = 1100.0;
    else if ((rc.chan2_raw < 1520) && (rc.chan2_raw > 1480))
        chan2 = 1500;
    else
    {
        chan2  = rc.chan2_raw;
        static_hover = false;
    }

    if ( rc.chan4_raw   > 1900 )
        chan4   = 1900.0;
    else if ( rc.chan4_raw < 1100 )
        chan4   = 1100.0;
    else if ((rc.chan4_raw < 1520) && (rc.chan4_raw > 1480))
        chan4 = 1500;
    else
    {
        chan4  = rc.chan4_raw;
        static_hover = false;
    }

    Eigen::Vector3d     rc_vel;
    yaw_rate           = -YawVel   * ((float)(chan4 - 1500.0)) / 400.0;
    rc_vel(0)          = RCVel     * ((float)(chan2 - 1500.0)) / 400.0;
    rc_vel(1)          = -RCVel     * ((float)(chan1 - 1500.0)) / 400.0;
    //Thrust controller works in the SO3 control as the gain of mass
    rc_vel(2)          = RCVel     * ((float)(chan3 - hover_init_thrust)) / 400.0;

    // get the pos xyz under control( in the body frame )
    Eigen::Matrix3d     R_mid;
    Eigen::Vector3d     rpy;
    rpy     = Qbw2RPY(att_b2w);
    R_mid   =   Eigen::AngleAxisd(rpy.z(), Eigen::Vector3d::UnitZ());
    target_from_init = target_from_init + R_mid * (rc_vel * delta_t);
    target  =   init_pos + target_from_init;
    Eigen::Vector3d     Vel_tar;
    Vel_tar     = R_mid * rc_vel;
    //get the yaw under control
    yaw_tar     = yaw_tar   + yaw_rate * delta_t;

    if ( !work_sonar )
    {
        target(0)  = now_position.x();
        target(1)  = now_position.y();
    }

    set_pos.position.x  = target.x();
    set_pos.position.y  = target.y();
    set_pos.position.z  = target.z();
    set_pos.velocity.x  = 0;//Vel_tar(0);
    set_pos.velocity.y  = 0;//Vel_tar(1);
    set_pos.velocity.z  = 0;//Vel_tar(2);
    set_pos.accelerate.x    = 0;
    set_pos.accelerate.y    = 0;
    set_pos.accelerate.z    = 0;
}

#endif
