#include "ros/ros.h"
#include <iostream>
#include "mavlink_message/set_att_offboard.h"
#include "mavlink_message/PositionCommand.h"
#include "mavlink_message/att_onboard.h"
#include "geometry_msgs/QuaternionStamped.h"
#include "nav_msgs/Odometry.h"
#include "Eigen/Geometry"
#include "rotation.h"
#include <iostream>
#include <cmath>
#include <string.h>
#include <time.h>
#include "control_function.h"

using namespace std;
using namespace std;
using namespace Eigen;


//ros topic
ros::Publisher      set_att_pub;


Eigen::Vector3d     des_pos, des_vel, des_acc;
double  des_yaw, Mass, ThrustGain, yaw_offset;
bool    setp_state;
void des_pos_callback(const mavlink_message::PositionCommand &cmd)
{
    if (cmd.header.frame_id != string("null"))// || cmd.header.frame_id == string("hover"))
        setp_state  = true;
    else
        setp_state = false;

    des_pos(0) = cmd.position.x;
    des_pos(1) = cmd.position.y;
    des_pos(2) = cmd.position.z;

    des_vel(0) = cmd.velocity.x;
    des_vel(1) = cmd.velocity.y;
    des_vel(2) = cmd.velocity.z;

    des_acc(0) = cmd.accelerate.x;
    des_acc(1) = cmd.accelerate.y;
    des_acc(2) = cmd.accelerate.z;

    des_yaw = cmd.yaw;
    yaw_offset = cmd.yaw_offset;
}

Quaterniond q_px4;
Matrix3d R_px4;
double yaw_px4;
void att_px4_callback(const mavlink_message::att_onboard::ConstPtr att_px4)
{
    yaw_px4 = att_px4->yaw;
    q_px4.w() = att_px4->Q_b2w.w;
    q_px4.x() = att_px4->Q_b2w.x;
    q_px4.y() = att_px4->Q_b2w.y;
    q_px4.z() = att_px4->Q_b2w.z;
    R_px4 = q_px4.normalized().toRotationMatrix();
}
Eigen::Vector3d Kp, Kd;
void local_pos_callback(const nav_msgs::Odometry &pos)
{
    Matrix3d R_ukf;
    Quaterniond Q_ukf;
    Vector3d now_pos, now_vel, rpy_ukf;
    double yaw_ukf;
    Q_ukf.w() = pos.pose.pose.orientation.w;
    Q_ukf.x() = pos.pose.pose.orientation.x;
    Q_ukf.y() = pos.pose.pose.orientation.y;
    Q_ukf.z() = pos.pose.pose.orientation.z;
    R_ukf = Q_ukf.normalized().toRotationMatrix();
    rpy_ukf = Qbw2RPY(Q_ukf);
   // std::cout<<"ekf rpy: "<<rpy_ukf<<std::endl;
    yaw_ukf = rpy_ukf.z();
    now_pos(0) = pos.pose.pose.position.x;
    now_pos(1) = pos.pose.pose.position.y;
    now_pos(2) = pos.pose.pose.position.z;
    now_vel(0) = pos.twist.twist.linear.x;
    now_vel(1) = pos.twist.twist.linear.y;
    now_vel(2) = pos.twist.twist.linear.z;
    mavlink_message::set_att_offboard att_offboard;
    att_offboard.header.stamp = pos.header.stamp;
    att_offboard.type_mask = 7;
    att_offboard.target_system = 0;
    att_offboard.target_component = 0;
    Quaterniond  target_q;
    Vector3d des_rpy, des_rpy_px4;
    double thrust = 0;
    if (setp_state)
    {
        double Gravity = 9.8;
        double des_p[3], des_v[3], des_a[3], now_p[3], now_v[3], kp[3], kd[3], rpy_d[3];
        des_p[0] = des_pos.x();
        des_p[1] = des_pos.y();
        des_p[2] = des_pos.z();
        des_v[0] = des_vel.x();
        des_v[1] = des_vel.y();
        des_v[2] = des_vel.z();
        des_a[0] = des_acc.x();
        des_a[1] = des_acc.y();
        des_a[2] = des_acc.z();
        now_p[0] = now_pos.x();
        now_p[1] = now_pos.y();
        now_p[2] = now_pos.z();
        now_v[0] = now_vel.x();
        now_v[1] = now_vel.y();
        now_v[2] = now_vel.z();
        kp[0] = Kp.x();
        kp[1] = Kp.y();
        kp[2] = Kp.z();
        kd[0] = Kd.x();
        kd[1] = Kd.y();
        kd[2] = Kd.z();
        SO3Control_function( des_p,
                des_v,
                des_a,
                des_yaw,
                now_p,
                now_v,
                yaw_ukf,
                kp,
                kd,
                Mass,
                Gravity,
                rpy_d,
                thrust
                );
        des_rpy.x() = rpy_d[0];
        des_rpy.y() = rpy_d[1];
        des_rpy.z() = rpy_d[2];
        Vector2d rp, acc_xy;
        Matrix2d R_inv;
        R_inv(0, 0) = sin(yaw_ukf);
        R_inv(0, 1) = -cos(yaw_ukf);
        R_inv(1, 0) = cos(yaw_ukf);
        R_inv(1, 1) = sin(yaw_ukf);
        rp(0) = des_rpy.x();
        rp(1) = des_rpy.y();

        Vector3d acc_xyz, acc_body, force_body;
        acc_xyz.segment(0, 2) = Gravity * R_inv.inverse() * rp;
        acc_xyz.z() = thrust / Mass;
        acc_body = R_ukf.inverse() * acc_xyz;
        force_body = Mass * acc_body;

        att_offboard.body_roll_rate     = force_body.x();
        att_offboard.body_pitch_rate    = force_body.y();
        att_offboard.body_yaw_rate      = force_body.z();
        att_offboard.q1 = 0;
        att_offboard.q2 = ThrustGain;
        att_offboard.q3 = yaw_offset;
        att_offboard.q4 = 0;
    }
    att_offboard.thrust = 40;
    set_att_pub.publish(att_offboard);
}

int main(int argc, char**argv)
{
    ros::init(argc, argv, "so3_control");
    ros::NodeHandle pos2att("~");
    set_att_pub     = pos2att.advertise<mavlink_message::set_att_offboard>("/mavlink/set_att", 1000);
    ros::Subscriber local_pos_sub   = pos2att.subscribe("odom",   1000,   local_pos_callback);
    ros::Subscriber des_pos_sub     = pos2att.subscribe("des_pos", 1000, des_pos_callback);
    ros::Subscriber att_px4_sub     = pos2att.subscribe("q_pixhawk", 100, att_px4_callback);
    double Px, Py, Pz, Dx, Dy, Dz;
    pos2att.param("Px", Px,  5.0);
    pos2att.param("Py", Py,  5.0);
    pos2att.param("Pz", Pz,  5.0);
    pos2att.param("Dx", Dx,  3.0);
    pos2att.param("Dy", Dy,  3.0);
    pos2att.param("Dz", Dz,  3.0);
    pos2att.param("Mass", Mass, 1.2);
    pos2att.param("ThrustGain", ThrustGain, 30.0);
    Kp << Px, Py, Pz;
    Kd << Dx, Dy, Dz;
    ros::spin();
    return 0;
}
