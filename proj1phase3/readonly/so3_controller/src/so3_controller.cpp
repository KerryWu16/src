#include "ros/ros.h"
#include "mavlink_message/att_onboard.h"
#include "mavlink_message/set_att_offboard.h"
#include "mavlink_message/local_position.h"
#include "mavlink_message/imu_raw.h"
#include "mavlink_message/quad_state.h"
#include "mavlink_message/PositionCommand.h"
#include "mavlink_message/rc_chans.h"
#include "mavlink_message/rpy.h"
#include "nav_msgs/Odometry.h"
#include "Eigen/Geometry"
#include "rotation.h"
#include <iostream>
#include <cmath>
#include <string.h>
#include <time.h>

#define     G               9.89
#define     MAX_ACC         9
#define     YAW_OFF_MAX     120*(M_PI/180)
using std::string;
using namespace std;

//ros topic
ros::Publisher      set_att_pub;
ros::Subscriber     local_pos_sub;
ros::Subscriber     rc_test_sub;
ros::Subscriber     des_pos_sub;

double      thrust;
uint32_t    pos_time;
bool        Debug   = false;

Eigen::Vector3d     now_position, now_velocity, now_rpy_onboard;
Eigen::Quaterniond  orientation;
double      now_yaw;
double      thrust_gain;
double      M, ThrustRC;

Eigen::Quaterniond      Q_b2w_onboard, Q_b2w_offboard;
Eigen::Matrix3d  R_w2b;
Eigen::Vector3d  des_force;
Eigen::Vector3d  acc_body;
bool acc_ready = false;
bool is_hover = false;
double comp = 0;
void SO3Control_function(   const Eigen::Vector3d des_pos,
                            const Eigen::Vector3d des_vel,
                            const Eigen::Vector3d des_acc,
                            double now_yaw_,
                            double yaw_offset,
                            const Eigen::Vector3d now_pos,
                            const Eigen::Vector3d now_vel,
                            const Eigen::Vector3d kx,
                            const Eigen::Vector3d kv )
{
    //desired force in W frame
    des_force.noalias() =   kx.asDiagonal() * (des_pos - now_pos) +
                            kv.asDiagonal() * (des_vel - now_vel) +
                            M * des_acc;
    //Limite the max RP_angle
    double des_force_norm = des_force.norm();
    if ( des_force_norm  > MAX_ACC * M)
    {
        des_force      = MAX_ACC * des_force.normalized();
    }
    
    des_force.noalias()   = des_force + M * G * Eigen::Vector3d( 0, 0, 1);
    mavlink_message::set_att_offboard  att_offboard;
    //transfer the desforce_w to desforce_b
    des_force = R_w2b * des_force;
    //complemet of the batter lose
    if (is_hover)
    {
        double acc_z_est = des_force.z() / M;
        comp -= (acc_z_est - acc_body.z()) * 2e-4;
    }
    //cout << "comp: " << 1 + comp << endl;
    double new_thrust_gain = (1 + comp) * thrust_gain;
    //double new_thrust_gain = thrust_gain;

    att_offboard.body_roll_rate  = des_force.x();
    att_offboard.body_pitch_rate = des_force.y();
    att_offboard.body_yaw_rate   = des_force.z();
    att_offboard.q1 = now_yaw_;
    att_offboard.q2 = new_thrust_gain;
    att_offboard.q3 = yaw_offset;
    att_offboard.thrust = ThrustRC;
    set_att_pub.publish(att_offboard);
}

Eigen::Vector3d     des_position, des_velocity, des_acceleration, kx_, kv_;
double  des_yaw, yaw_offset;
bool    setp_state;
void des_pos_callback(const mavlink_message::PositionCommand &cmd)
{
    //if (cmd.header.frame_id == string("setp"))
    if (cmd.header.frame_id != string("null"))// || cmd.header.frame_id == string("hover"))
        setp_state  = true;
    else
        setp_state = false;
    if (cmd.header.frame_id == string("hover"))
        is_hover = true;
    else
        is_hover = false;

    des_position(0) = cmd.position.x;
    des_position(1) = cmd.position.y;
    des_position(2) = cmd.position.z;

    des_velocity(0) = cmd.velocity.x;
    des_velocity(1) = cmd.velocity.y;
    des_velocity(2) = cmd.velocity.z;

    des_acceleration(0) = cmd.accelerate.x;
    des_acceleration(1) = cmd.accelerate.y;
    des_acceleration(2) = cmd.accelerate.z;

    kx_(0)   = cmd.kx[0];
    kx_(1)   = cmd.kx[1];
    kx_(2)   = cmd.kx[2];

    kv_(0)   = cmd.kv[0];
    kv_(1)   = cmd.kv[1];
    kv_(2)   = cmd.kv[2];

    yaw_offset = cmd.yaw_offset;
    now_yaw = cmd.yaw;
    M       =  cmd.Mass;
    thrust_gain     = cmd.Thrust_Gain;
}
void local_pos_callback(const nav_msgs::Odometry &pos)
{
    Q_b2w_offboard.w()   = pos.pose.pose.orientation.w;
    Q_b2w_offboard.x()   = pos.pose.pose.orientation.x;
    Q_b2w_offboard.y()   = pos.pose.pose.orientation.y;
    Q_b2w_offboard.z()   = pos.pose.pose.orientation.z;
    R_w2b = Q_b2w_offboard.normalized().toRotationMatrix().transpose();
    Eigen::Vector3d     rpy;
    rpy = Qbw2RPY(Q_b2w_offboard);
    now_position(0)     =   pos.pose.pose.position.x;
    now_position(1)     =   pos.pose.pose.position.y;
    now_position(2)     =   pos.pose.pose.position.z;
    now_velocity(0)     =   pos.twist.twist.linear.x;
    now_velocity(1)     =   pos.twist.twist.linear.y;
    now_velocity(2)     =   pos.twist.twist.linear.z;
    if ( setp_state )
        SO3Control_function(    des_position,
                                des_velocity,
                                des_acceleration,
                                now_yaw,
                                yaw_offset,
                                now_position,
                                now_velocity,
                                kx_,
                                kv_     );
}

uint32_t     rc_time, rc_time_last;
double       dt;
void rc_test_callback(const mavlink_message::rc_chans &rc)
{
    uint32_t    chan3 = 1500;
    chan3   = rc.chan3_raw;
    if (chan3 > 1900)
        chan3 = 1900.0;
    else if (chan3 < 1100)
        chan3 = 1100.0;
    double t;
    t = (chan3 - 1100.0) / 800.0;
    if (t <= 0.5)
        ThrustRC = 2 * t;
    else
        ThrustRC = 1;
}
void imu_callback(const mavlink_message::imu_raw::ConstPtr &msg)
{
    acc_ready = true;
    acc_body.x() = msg->acceleration.x;
    acc_body.y() = msg->acceleration.y;
    acc_body.z() = msg->acceleration.z;
}

int main(int argc, char**argv)
{
    ros::init(argc, argv, "so3_control");
    ros::NodeHandle pos2att("~");
    set_att_pub     = pos2att.advertise<mavlink_message::set_att_offboard>("/mavlink/set_att", 1000);
    local_pos_sub   = pos2att.subscribe("odom",   1000,   local_pos_callback);
    des_pos_sub     = pos2att.subscribe("des_pos", 1000, des_pos_callback);
    rc_test_sub     = pos2att.subscribe("/mavlink/rc_chans", 1000, rc_test_callback);
    ros::Subscriber sub_imu = pos2att.subscribe("/mavlink/imu_raw", 100, imu_callback);
    rc_time     = 0;
    rc_time_last = 0;
    int     i;
    for (i = 1; i < argc; i++)
    {
        if (strcmp(argv[i], "-d") == 0)
            Debug       = true;
    }
    ros::spin();
    return 0;
}
