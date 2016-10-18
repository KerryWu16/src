#ifndef control_funtion_h
#define control_funtion_h

#include <math.h>

/*please fill this controller function
 * input:
 * des_pos -> desired position
 * des_vel -> desired velocity
 * des_acc -> desired acceleration
 * des_yaw -> desired yaw angle
 * now_pos -> now psition
 * now_vel -> body velocity
 * Kp      -> P gain for position loop
 * Kd      -> P gain for velocity loop
 * Mass    -> quality of the quadrotor
 *
 
 * output:
 * rpy               ->    target attitude for autopilot
 * target_thrust     ->    target thrust of the quadrotor
 * */
void SO3Control_function( const double des_pos[3],
                          const double des_vel[3],
                          const double des_acc[3],
                          const double des_yaw,
                          const double now_pos[3],
                          const double now_vel[3],
                          const double now_yaw,
                          const double Kp[3],
                          const double Kd[3],
                          const double Mass,
                          const double Gravity,
                          double rpy[3],
                          double &target_thrust
                        )
{
	double K_px = 6;
	double K_dx = 4.5;
	double K_py = 6;
	double K_dy = 4.5;
	double K_pz = 50.0;
	double K_dz = 80.0;

	double r_1_acc = des_acc[1] + K_dx * (des_vel[1] - now_vel[1]) + K_px * (des_pos[1] - now_pos[1]);
	double r_2_acc = des_acc[2] + K_dy * (des_vel[2] - now_vel[2]) + K_py * (des_pos[2] - now_pos[2]); 
	double r_3_acc = des_acc[3] + K_dz * (des_vel[3] - now_vel[3]) + K_pz * (des_pos[3] - now_pos[3]);
	
	rpy[0] = (r_1_acc * sin(now_yaw) - r_2_acc * cos(now_yaw)) / Gravity;	
	rpy[1] = (r_1_acc * cos(now_yaw) + r_2_acc * sin(now_yaw)) / Gravity;
	rpy[2] = 0.0;
	target_thrust = Mass*(Gravity + r_3_acc);       
}
#endif
