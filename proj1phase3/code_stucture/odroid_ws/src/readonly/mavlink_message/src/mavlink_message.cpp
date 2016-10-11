#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include "mavlink1.0/common/mavlink.h"
#include "include/usbserial.h"
#include "mavlink_message/imu_raw.h"
#include "mavlink_message/att_onboard.h"
#include "mavlink_message/rc_chans.h"
#include "mavlink_message/set_att_offboard.h"
#include "mavlink_message/local_position.h"
#include "mavlink_message/quad_state.h"
#include "mavlink_message/gps_raw.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"
#include "Eigen/Eigen"
// Standard includes
#include <glib.h>
#include <iostream>
#include <cstdlib>
#include <unistd.h>
#include <cmath>
#include <string.h>
#include <inttypes.h>
#include <fstream>
// Serial includes
#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#ifdef __linux
#include <sys/ioctl.h>
#endif

// Latency Benchmarking
#include <sys/time.h>
#include <time.h>

using std::string;
using namespace std;

struct timeval tv;        ///< System time

// Settings
int     sysid = 0;             ///< The unique system id of this MAV, 0-127. Has to be consistent across the system
int     compid = 0;
bool    silent = false;              ///< Wether console output should be enabled
bool    verbose = false;             ///< Enable verbose output
bool    debug = false;               ///< Enable debug functions and output
int     fd;
uint64_t    onboard_time;


ros::Publisher      imu_pub;
ros::Publisher      imu_msgs_pub;
ros::Publisher      att_onboard_pub;
ros::Publisher      rc_chans_raw_pub;
ros::Publisher      gps_raw_pub;
ros::Subscriber     set_att_sub;
ros::Publisher      local_position_pub;
ros::Publisher      quad_state_pub;
ros::Publisher      odom_pub;
ros::Publisher      pos_pub;

void serial_wait( void* serial_ptr)
{
    int fd = *((int*)serial_ptr);
    ros::Time  nt, lt;
    //unsigned long   TimeMS;
    unsigned long   last_imu_sec = 1;
    struct timeval tv;
    // Blocking wait for new data
    gettimeofday(&tv, NULL);
    //TimeMS = (unsigned long)(tv.tv_sec) * 1000 * 1000 + (unsigned long)(tv.tv_usec);
    ros::Rate listen_rate(1);
    while (1)
    {
        uint8_t cp;
        mavlink_message_t message;
        mavlink_status_t status;
        uint8_t msgReceived = false;
        while (read(fd, &cp, 1) > 0)
        {
            // Check if a message could be decoded, return the message in case yes
            msgReceived = mavlink_parse_char(MAVLINK_COMM_0, cp, &message, &status);
            // If a message could be decoded, handle it
            if (msgReceived)
            {
 //               cout << "receive mavlink msg, ID: " << (int) message.msgid << endl;
                switch (message.msgid)
                {
                    case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
                        {
                            static mavlink_local_position_ned_t pos;
                            mavlink_msg_local_position_ned_decode(&message, &pos);
                            mavlink_message::local_position local_position;

                            local_position.header.stamp     = ros::Time::now();
                            local_position.x    = pos.x;
                            local_position.y    = pos.y;
                            local_position.z    = pos.z;
                            local_position.vx   = pos.vx;
                            local_position.vy   = pos.vy;
                            local_position.vz   = pos.vz;

                            local_position.time_boot_ms = pos.time_boot_ms;
                            local_position_pub.publish(local_position);
                        }
                    case MAVLINK_MSG_ID_HIGHRES_IMU:
                        {
                            static mavlink_highres_imu_t imu;
                            mavlink_msg_highres_imu_decode(&message, &imu);
                            sensor_msgs::Imu imu_stan;
                            int     a, b;
                            b       = imu.time_usec / 1000000;
                            a       = b - last_imu_sec;
                            if (((imu.xacc != 0) || ( imu.zacc != 0 )) &&  ((last_imu_sec != 0) || (abs(a) <= 1)))
                            {
                                imu_stan.header.stamp = ros::Time::now();
                                imu_stan.linear_acceleration.x = imu.xacc;
                                imu_stan.linear_acceleration.y = -imu.yacc;
                                imu_stan.linear_acceleration.z = -imu.zacc;
                                imu_stan.angular_velocity.x = imu.xgyro;
                                imu_stan.angular_velocity.y = -imu.ygyro;
                                imu_stan.angular_velocity.z = -imu.zgyro;
                                imu_msgs_pub.publish(imu_stan);
                            }

                            mavlink_message::imu_raw    imu_raw_msg;
                            imu_raw_msg.time_usec   = imu.time_usec;
                            if (((imu.xacc != 0) || ( imu.zacc != 0 )) &&  ((last_imu_sec != 0) || (abs(a) <= 1)))
                            {
                                imu_raw_msg.header.stamp        = ros::Time::now();
                                nt = imu_raw_msg.header.stamp;
                                //cout << "delta T: " <<  (nt - lt).toSec() << " \t:" << (nt.toSec() - (int)(nt.toSec() / 1000) * 1000) - (imu.time_usec / 1000000.0 - (int)(imu.time_usec * 1e-9) * 1000) << endl;
                                lt = nt;
                                imu_raw_msg.acceleration.x      = imu.xacc;
                                imu_raw_msg.acceleration.y      = -imu.yacc;
                                imu_raw_msg.acceleration.z      = -imu.zacc;

                                imu_raw_msg.angular_velocity.x  = imu.xgyro;
                                imu_raw_msg.angular_velocity.y  = -imu.ygyro;
                                imu_raw_msg.angular_velocity.z  = -imu.zgyro;

                                imu_raw_msg.compass.x    = imu.xmag;
                                imu_raw_msg.compass.y    = -imu.ymag;
                                imu_raw_msg.compass.z    = -imu.zmag;
                                imu_raw_msg.abs_pressure    = imu.abs_pressure;
                                imu_raw_msg.pressure_alt    = imu.pressure_alt;
                                imu_raw_msg.diff_pressure   = imu.diff_pressure;
                                imu_raw_msg.temperature     = imu.temperature;

                                imu_pub.publish(imu_raw_msg);

                                last_imu_sec        = b;
                            }
                            break;
                        }
                    case MAVLINK_MSG_ID_RC_CHANNELS:
                        {
                            mavlink_rc_channels_t       rc_raw;
                            mavlink_message::rc_chans   rc;
                            mavlink_message::quad_state     state;
                            mavlink_msg_rc_channels_decode(&message, &rc_raw);
                            rc.header.stamp     = ros::Time::now();
                            rc.time_boot_ms     = rc_raw.time_boot_ms;
                            onboard_time        = rc.time_boot_ms;
                            rc.chan1_raw        = rc_raw.chan1_raw;
                            rc.chan2_raw        = rc_raw.chan2_raw;
                            rc.chan3_raw        = rc_raw.chan3_raw;
                            rc.chan4_raw        = rc_raw.chan4_raw;
                            rc.chan5_raw        = rc_raw.chan5_raw;
                            rc.chan6_raw        = rc_raw.chan6_raw;
                            rc.chan7_raw        = rc_raw.chan7_raw;
                            rc.chan8_raw        = rc_raw.chan8_raw;
                            rc_chans_raw_pub.publish(rc);

                            if (rc.chan7_raw < 1400)
                                state.offboard_state = 0;
                            else if ((rc.chan7_raw > 1400) && (rc.chan7_raw < 1600))
                                state.offboard_state = 0x01;
                            else if (rc.chan7_raw >= 1600)
                                state.offboard_state = 0x02;
                            quad_state_pub.publish(state);
                            break;
                        }
                    case MAVLINK_MSG_ID_ATTITUDE:
                        {
                            mavlink_message::att_onboard att;
                            mavlink_attitude_t  att_t;
                            mavlink_msg_attitude_decode(&message, &att_t);
                            att.header.stamp    = ros::Time::now();
                            att.time_boot_ms    = att_t.time_boot_ms;

                            att.roll            = att_t.roll;
                            att.pitch           = -att_t.pitch;
                            att.yaw             = -att_t.yaw;
                            att.rollspeed       = att_t.rollspeed;
                            att.pitchspeed      = -att_t.pitchspeed;
                            att.yawspeed        = -att_t.yawspeed;

                            Eigen::Matrix3d     Rx, Ry, Rz, R_b2w, R_w2b;
                            Rx  = Eigen::AngleAxisd( att.roll,     Eigen::Vector3d::UnitX() );
                            Ry  = Eigen::AngleAxisd( att.pitch,    Eigen::Vector3d::UnitY() );
                            Rz  = Eigen::AngleAxisd( att.yaw,      Eigen::Vector3d::UnitZ() );

                            R_b2w   =  (Rz * Ry * Rx);
                            Eigen::Quaterniond   Q_b2w;
                            Q_b2w   = R_b2w;

                            att.Q_b2w.w     = Q_b2w.w();
                            att.Q_b2w.x     = Q_b2w.x();
                            att.Q_b2w.y     = Q_b2w.y();
                            att.Q_b2w.z     = Q_b2w.z();

                            att_onboard_pub.publish(att);
                            break;
                        }
                    case MAVLINK_MSG_ID_GPS_RAW_INT:
                        {
                            mavlink_message::gps_raw    gps_r;
                            mavlink_gps_raw_int_t       gps;
                            mavlink_msg_gps_raw_int_decode(&message, &gps);
                            gps_r.time_usec     = gps.time_usec;
                            gps_r.lat           = gps.lat;
                            gps_r.lon           = gps.lon;
                            gps_r.alt           = gps.alt;
                            gps_r.eph           = gps.eph;
                            gps_r.epv           = gps.epv;
                            gps_r.vel           = gps.vel;
                            gps_r.cog           = gps.cog;
                            gps_r.fix_type      = gps.fix_type;
                            gps_r.satellites_visible        = gps.satellites_visible;
                            gps_raw_pub.publish( gps_r );
                            break;
                        }
                    case MAVLINK_MSG_ID_SET_ATTITUDE_TARGET:
                        {
                            mavlink_set_attitude_target_t att_received;
                            mavlink_msg_set_attitude_target_decode(&message, &att_received);
                            printf("received sucessfully! time: %d\n", att_received.time_boot_ms);
                            cout << "att q0: " << att_received.q[0] << endl;
                            cout << "att q1: " << att_received.q[1] << endl;
                            cout << "att q2: " << att_received.q[2] << endl;
                            cout << "att q3: " << att_received.q[3] << endl;
                        }
                        break;
                }
            }
        }
        usleep(50);
    }
}


void set_att_Callback(const mavlink_message::set_att_offboard &set_att)
{
    //printf("start writing\n");
    mavlink_set_attitude_target_t  mavlink_set_att;
    mavlink_message_t message;
    char buf[100];
    mavlink_set_att.time_boot_ms = 0;
    // q1   -->  offboard yaw
    // q3   -->  yaw offset
    // q2   -->  Thrust Gain
    mavlink_set_att.q[0] = -set_att.q1;
    mavlink_set_att.q[1] = set_att.q2;
    mavlink_set_att.q[2] = -set_att.q3;
    mavlink_set_att.q[3] = 0;
    //body_roll_rate    --> Fx_body   in the offboard frame
    //body_pitch_rate   --> Fy_body
    //body_yaw_rate     --> Fz_body
    mavlink_set_att.body_roll_rate  = set_att.body_roll_rate;
    mavlink_set_att.body_pitch_rate = -set_att.body_pitch_rate;
    mavlink_set_att.body_yaw_rate   = -set_att.body_yaw_rate;
    //thrust    -->  thrust rc
    mavlink_set_att.thrust = set_att.thrust;
    mavlink_set_att.type_mask = 7;
    mavlink_msg_set_attitude_target_encode_chan(0, 0, MAVLINK_COMM_1, &message, &mavlink_set_att);
    unsigned len = mavlink_msg_to_send_buffer((uint8_t*)buf, &message);
    /* write packet via serial link */
    write(fd, buf, len);
    /* wait until all data has been written */
    tcdrain(fd);
    //printf("write set_att_offboard\n");
}


int main(int argc, char **argv)
{

    ros::init(argc, argv, "mavlink_message");
    ros::NodeHandle mavhandle("/mavlink");
    imu_pub             = mavhandle.advertise<mavlink_message::imu_raw>( "imu_raw",         100);
    imu_msgs_pub        = mavhandle.advertise<sensor_msgs::Imu>( "imu_standard",         100);
    gps_raw_pub         = mavhandle.advertise<mavlink_message::gps_raw>( "gps_raw",         100);
    quad_state_pub      = mavhandle.advertise<mavlink_message::quad_state>( "quad_state",   100);

    att_onboard_pub     = mavhandle.advertise<mavlink_message::att_onboard>(    "att_onboard",      1000);
    rc_chans_raw_pub    = mavhandle.advertise<mavlink_message::rc_chans>(   "rc_chans",         1000);
    local_position_pub  = mavhandle.advertise<mavlink_message::local_position>( "local_position",   1000);
    set_att_sub         = mavhandle.subscribe("set_att", 1000, set_att_Callback);

    /* default values for arguments */
    char *uart_name = (char*)"/dev/mavlink";
    int baudrate = 115200;
    const char *commandline_usage = "\tusage: %s -d <devicename> -b <baudrate> [-v/--verbose] [--debug]\n\t\tdefault: -d %s -b %i\n";
    /* read program arguments */
    int i;
    for (i = 1; i < argc; i++)   /* argv[0] is "mavlink" */
    {
        if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0)
        {
            printf(commandline_usage, argv[0], uart_name, baudrate);
            return 0;
        }
        /* UART device ID */
        if (strcmp(argv[i], "-d") == 0 || strcmp(argv[i], "--device") == 0)
        {
            if (argc > i + 1)
            {
                uart_name = argv[i + 1];

            }
            else
            {
                printf(commandline_usage, argv[0], uart_name, baudrate);
                return 0;
            }
        }
        /* baud rate */
        if (strcmp(argv[i], "-b") == 0 || strcmp(argv[i], "--baud") == 0)
        {
            if (argc > i + 1)
            {
                baudrate = atoi(argv[i + 1]);

            }
            else
            {
                printf(commandline_usage, argv[0], uart_name, baudrate);
                return 0;
            }
        }
        /* terminating MAVLink is allowed - yes/no */
        if (strcmp(argv[i], "-v") == 0 || strcmp(argv[i], "--verbose") == 0)
        {
            verbose = true;
        }
        if (strcmp(argv[i], "--debug") == 0)
        {
            debug = true;
        }
    }

    // SETUP SERIAL PORT
    // Exit if opening port failed
    // Open the serial port.
    if (!silent) printf("Trying to connect to %s.. ", uart_name);
    fflush(stdout);
    fd = open_port(uart_name);
    if (fd == -1)
    {
        if (!silent) printf("failure, could not open port.\n");
        exit(EXIT_FAILURE);
    }
    else
    {
        if (!silent) printf("success.\n");
    }
    if (!silent) printf("Trying to configure %s.. ", uart_name);
    bool setup = setup_port(fd, baudrate );//, 8, 1, false, false);
    if (!setup)
    {
        if (!silent) printf("failure, could not configure port.\n");
        exit(EXIT_FAILURE);
    }
    else
    {
        if (!silent) printf("success.\n");
    }

    int noErrors = 0;
    if (fd == -1 || fd == 0)
    {
        if (!silent) fprintf(stderr, "Connection attempt to port %s with %d baud, 8N1 failed, exiting.\n", uart_name, baudrate);
        exit(EXIT_FAILURE);
    }
    else
    {
        if (!silent) fprintf(stderr, "\nConnected to %s with %d baud, 8 data bits, no parity, 1 stop bit (8N1)\n", uart_name, baudrate);
    }
    if (fd < 0)
    {
        exit(noErrors);
    }


    // Run indefinitely while the serial loop handles data
    if (!silent) printf("\nREADY, waiting for serial data.\n");

    int* fd_ptr = &fd;

    if (fd < 0)
    {
        exit(noErrors);
    }

    /**
     * Now pump callbacks (execute mavlinkCallback) until CTRL-c is pressed
     */
    GThread* serial_thread;
    GError* err = NULL;

    // Run indefinitely while the ROS and serial threads handle the data
    if (!silent)
        printf("\nREADY, waiting for serial/ROS data.\n");

    if ((serial_thread = g_thread_new( "mav_serial", (GThreadFunc)serial_wait, (void *)fd_ptr)) == NULL)
    {
        printf("Failed to create serial handling thread: %s!!\n", err->message);
        g_error_free(err);
    }

    // Ready to roll
    printf("\nMAVLINK SERIAL TO ROS BRIDGE STARTED ON MAV %d (COMPONENT ID:%d) - RUNNING..\n\n", sysid, compid);
    ros::spin();
    close_port(fd);

    return 0;
}
