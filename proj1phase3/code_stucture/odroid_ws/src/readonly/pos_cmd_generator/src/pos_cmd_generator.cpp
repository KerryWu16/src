#include    "ros/ros.h"
#include    "ros/time.h"
#include    <dynamic_reconfigure/server.h>
#include    <pos_cmd_generator/pos_cmd_generator_paramsConfig.h>
#include    <iostream>
#include    "mavlink_message/PositionCommand.h"
#include    "quadrotor_msgs/PositionCommand.h"
#include    "mavlink_message/quad_state.h"
#include    "mavlink_message/att_onboard.h"
#include    "nav_msgs/Odometry.h"
#include    "geometry_msgs/Pose.h"
#include    "mavlink_message/rc_chans.h"
#include    "std_msgs/Float64.h"
#include    "Eigen/Eigen"
#include    "sensor_msgs/Range.h"

#include    <cmath>
#include    <sys/time.h>
#include    <time.h>

#include    <cstdlib>
#include    <string.h>
#include    "fly_cycle.h"
#include    "hover.h"
#include    "include/pose_.h"
#include    "trajectory.h"
using namespace     std;

ros::Publisher      poscmd_pub;
ros::Publisher      t4p_pub;
ros::Publisher      initp_pub;
ros::Subscriber     local_pos_sub;
ros::Subscriber     state_sub;
ros::Subscriber     auto_pos_cmd;
ros::Subscriber     att_sub;
ros::Subscriber     rc_sub;
Eigen::Vector3d     now_position, now_velocity;
Eigen::Quaterniond  ukf_att_b2w;
Eigen::Vector3d  rpy;
double      now_t;
void local_pos_callback( const nav_msgs::Odometry &pos )
{
    now_t   = pos.header.stamp.toSec();
    now_position(0)     = pos.pose.pose.position.x;
    now_position(1)     = pos.pose.pose.position.y;
    now_position(2)     = pos.pose.pose.position.z;

    ukf_att_b2w.w()     = pos.pose.pose.orientation.w;
    ukf_att_b2w.x()     = pos.pose.pose.orientation.x;
    ukf_att_b2w.y()     = pos.pose.pose.orientation.y;
    ukf_att_b2w.z()     = pos.pose.pose.orientation.z;

    now_velocity(0)     = pos.twist.twist.linear.x;
    now_velocity(1)     = pos.twist.twist.linear.y;
    now_velocity(2)     = pos.twist.twist.linear.z;

    rpy = Qbw2RPY(ukf_att_b2w);
}

double  now_yaw;
void att_callback(const mavlink_message::att_onboard &att)
{
    now_yaw     = att.yaw;
}

mavlink_message::rc_chans   rc_c;
void    rc_callback(const mavlink_message::rc_chans &rc)
{	
 //   ROS_INFO("rcv sth...");
    rc_c    = rc;
}

uint8_t    now_state = 0xff;
mavlink_message::PositionCommand    set_p;

Eigen::Vector3d     init_pos;
double      init_t;
bool        poscmd_init;
bool        auto_init;

//values using in hovering
Eigen::Vector3d target_from_init;
double  yaw_tar, yaw_offset, yaw_test;
double thrust_g = 0;
double hover_init_thrust = 1500;

nav_msgs::Odometry  PosInit;
void quad_state_callback(const mavlink_message::quad_state &state)
{
    if (state.offboard_state & 0x01)
        now_state   |= 0x01;
    else if (state.offboard_state & 0x02 )
        now_state   |= 0x02;
    if (((now_state & 0x0f ) == 0x01) || ((now_state & 0x0f) == 0x09)) // 0x01 from manal to hover; 0x09 from auto to hover
    {
        cout << "hover" << endl;
        //get the init position and yaw as the init information
        init_pos(0)     = now_position.x();
        init_pos(1)     = now_position.y();
        init_pos(2)     = now_position.z();
        target_from_init    = Eigen::VectorXd::Zero(3);
        hover_init_thrust = rc_c.chan3_raw;
        cout << "now position: " << now_position.transpose()  << " \t target_from_init: " << target_from_init.transpose() << " \t yaw: " << now_yaw << endl;
        yaw_tar     = rpy.z();//now_yaw;
        yaw_test    = now_yaw;
        yaw_offset  = 0;//rpy.z();
        /****************************************************/
        /*        for the trajectory generator use          */
        PosInit.header.stamp.fromSec(now_t);
        PosInit.pose.pose.position.x  = now_position.x();
        PosInit.pose.pose.position.y  = now_position.y();
        PosInit.pose.pose.position.z  = now_position.z();
        initp_pub.publish(PosInit);
        poscmd_init = true;
        auto_init   = false;
    }
    else if ( (now_state & 0x0f) == 0x06) // 0x06 from hover to auto
    {
        cout << "auto" << endl;
        //init_pos(0) = now_position.x();
        //init_pos(1) = now_position.y();
        //init_pos(2) = now_position.z();
        init_t      = now_t;
        auto_init   = true;
    }
    else if ( (now_state & 0x0f) == 0x04) //from hover to manual
    {
        cout << "manual" << endl;
        poscmd_init = false;
    }
    now_state <<= 2;
}

bool    work_sonar;
void    tera_callback(const sensor_msgs::Range & sonar)
{
    if ( sonar.header.frame_id == string("null") )
        work_sonar  = false;
    else //if (sonar.header.frame_id == string("null"))
        work_sonar  = true; //this is just testing
    //cout << "sonar state: " << work_sonar << endl;
}

void auto_cmd_callback(const quadrotor_msgs::PositionCommand & pos_cmd)
{
    if (pos_cmd.header.frame_id == "null")
        return;
    set_p.position.x    = pos_cmd.position.x;
    set_p.position.y    = pos_cmd.position.y;
    set_p.position.z    = pos_cmd.position.z;
    set_p.velocity.x    = pos_cmd.velocity.x;
    set_p.velocity.y    = pos_cmd.velocity.y;
    set_p.velocity.z    = pos_cmd.velocity.z;
    set_p.accelerate.x  = pos_cmd.acceleration.x;
    set_p.accelerate.y  = pos_cmd.acceleration.y;
    set_p.accelerate.z  = pos_cmd.acceleration.z;
}

void param_callback(pos_cmd_generator::pos_cmd_generator_paramsConfig &config, uint32_t level)
{
    set_p.kx[0] = config.KPx;
    set_p.kx[1] = config.KPy;
    set_p.kx[2] = config.KPz;
    set_p.kv[0] = config.KVx;
    set_p.kv[1] = config.KVy;
    set_p.kv[2] = config.KVz;
}


int main(int argc, char**argv)
{
    ros::init(argc, argv, "pos_cmd");
    ros::NodeHandle     set_pos("~");
    poscmd_pub          =  set_pos.advertise<mavlink_message::PositionCommand>("des_pos", 1000);
    t4p_pub             =  set_pos.advertise<std_msgs::Float64>("deltaT", 100);
    initp_pub           =  set_pos.advertise<nav_msgs::Odometry>("InitPosition", 100);
    local_pos_sub       =  set_pos.subscribe("odom", 1000, local_pos_callback);
    state_sub           =  set_pos.subscribe("/mavlink/quad_state", 1000, quad_state_callback);
    att_sub             =  set_pos.subscribe("/mavlink/att_onboard", 100, att_callback);
    rc_sub              =  set_pos.subscribe("/mavlink/rc_chans", 100, rc_callback);
    auto_pos_cmd        =  set_pos.subscribe("auto_cmd", 100, auto_cmd_callback);
    ros::Subscriber tera_sub = set_pos.subscribe("tera", 100, tera_callback);
    poscmd_init    = false;
    auto_init      = false;
    std_msgs::Float64   T4P;
    double   zero  = 0;
    double   ThrustGain;
    ros::NodeHandle  nh;
    dynamic_reconfigure::Server<pos_cmd_generator::pos_cmd_generator_paramsConfig> dr_srv;
    //cb = boost::bind<&Pos>
    dr_srv.setCallback(param_callback);
    ros::NodeHandle     n("~");
    double   Mass;
    n.param("ThrustGain", ThrustGain, zero);
    n.param("Mass", Mass, zero);
    thrust_g = ThrustGain;
    bool     CycleTest;
    double   CycleR;
    double   CycleV;
    double   YawVel;
    double   RCVel;
    n.param("CycleTest",    CycleTest, false);
    n.param("CycleR",       CycleR,     1.0);
    n.param("CycleV",       CycleV,     0.5);
    n.param("YawVel",       YawVel,     3.0);
    n.param("RCVel",        RCVel,      3.0);

    set_p.kx[0]     =  10.0; //Kx[0];
    set_p.kx[1]     =  10.0;//Kx[1];
    set_p.kx[2]     =  15.0;// Kx[2];
    set_p.kv[0]     =  4.0;//Kv[0];
    set_p.kv[1]     =  4.0;//Kv[1];
    set_p.kv[2]     =  12.0;//Kv[2];
    set_p.Thrust_Gain = ThrustGain;
    set_p.Mass      = Mass;

    printf("start setting position\n");
    ros::Rate loop_rate(100);

    double  delta_t;
    ros::Time   now_call, last_call;
    while (ros::ok())
    {
        if ( poscmd_init )
        {
            if ( !auto_init )
            {
                last_call   = now_call;
                now_call    = ros::Time::now();
                delta_t     = now_call.toSec() - last_call.toSec();
                if (delta_t > 0.1)
                    delta_t = 0;
                bool static_hover = true;
                hover(  init_pos,
                        now_position,
                        ukf_att_b2w,
                        rc_c,
                        target_from_init,
                        //work_sonar,
                        true,
                        static_hover,
                        delta_t,
                        RCVel,
                        YawVel,
                        yaw_tar,
                        hover_init_thrust,
                        set_p
                     );
                if (static_hover)
                    set_p.header.frame_id = (string)"hover";
                else
                    set_p.header.frame_id = (string)"hover_move";
            }
            else if (CycleTest && auto_init)
            {
                fly_cycle(  init_pos,
                        now_position,
                        (now_t - init_t),
                        CycleR,
                        CycleV,
                        &set_p
                        );
                set_p.header.frame_id = (string)"auto";
            }
            else if (!CycleTest && auto_init)
            //else if (auto_init)
            {
                T4P.data    = now_t - init_t;
                t4p_pub.publish(T4P);
                set_p.header.frame_id = (string)"auto";

                Eigen::Vector3d d_p, d_v, d_a, hover_point;
                hover_point = init_pos + target_from_init;
                bool get_traj = trajectory_control((now_t - init_t), hover_point, Eigen::Vector3d::Zero(),  d_p, d_v, d_a);
                if(get_traj)
                {
                    set_p.position.x    = d_p.x();
                    set_p.position.y    = d_p.y();
                    set_p.position.z    = d_p.z();
                    set_p.velocity.x    = d_v.x();
                    set_p.velocity.y    = d_v.y();
                    set_p.velocity.z    = d_v.z();
                    set_p.accelerate.x  = d_a.x();
                    set_p.accelerate.y  = d_a.y();
                    set_p.accelerate.z  = d_a.z();
                }
                else
                {
                    set_p.position.x    = hover_point.x();
                    set_p.position.y    = hover_point.y();
                    set_p.position.z    = hover_point.z();
                    set_p.velocity.x    = 0; 
                    set_p.velocity.y    = 0; 
                    set_p.velocity.z    = 0; 
                    set_p.accelerate.x  = 0;
                    set_p.accelerate.y  = 0;
                    set_p.accelerate.z  = 0;
                }
            }
            //set_p.header.frame_id = (string)"setp";
        }
        else
        {
            set_p.header.frame_id = (string)"null";
        }
        set_p.yaw_offset = yaw_tar - rpy.z();
        set_p.yaw = yaw_tar;
        //test
        poscmd_pub.publish(set_p);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

