#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>
#include <main.h>

#include <motor_controller.h>
#include <motors.h>

#define DEDTORAD(n)	n * M_PI / 180
#define RADTODEG(n)	n * 180 / M_PI

#define NSTEP_ONE_TURN      1000 		// number of step for 1 turn of the motor
#define WHEEL_PERIMETER     12.8 		// [cm]
#define	WHEEL_TO_WHEEL_DIST	5.31		// [cm]

#define Kp		2	//Proportional term (error to e_puck rotation speed)
#define Ki		0.2	//Integral term (regulate lateral drift from initial straight line to direction)
#define Kspeed	10	//Proportional term (regulate front speed depending on direction error)

#define MAX_REFERENCE_ANGLE M_PI/4 		// [rad] Maximal reference to follow to compensate lateral drift

//static systime_t dt = 0; 				// [us] time elapsed between each loop - Not used anymore.

static float alpha_error = 0; 			// [rad] direction error
static float last_alpha_error = 0; 		// [rad]
static float initial_alpha_error = 0; 	// [rad]
static float lateral_error = 0; 		// [cm] lateral position error
static float front_speed = 0; 			// [cm/s] speed toward the front of the e_puck
static float w_z = 0; 					// [rad/s] rotational speed of the e_puck

static float last_dist_left = 0; 		// [cm] distance travelled by a wheel during the last loop
static float last_dist_right = 0; 		// [cm]

////// PRIVATE FUNCTIONS ///////
static void compute_controls(void);
static void actuate(void);
static float front_speed_calc(void);
static float rotation_calc(void);
//static void compute_dt(void); // Not used anymore.
////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////// PUBLIC FUNCTIONS ///////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////

/*
*	Wrapper to call the inits during startup.
*/
void motor_controller_setUp(void)
{
	//inits the motors
	motors_init();
}

/*
*	Motor control loop :
*		-> Orientation and position determination
*		-> Control computation
*		-> Motors actuation
*
*	Params:
*		- direction : angle [deg]  describing where to go (useful only when new direction to go is computed)
*		- flag_new : 1 if the direction is new, 0 if not.
		- lateral_distance : distance [mm], where the e-puck has to go laterally
*/
void goTo(float direction, bool flag_new, int8_t lateral_distance)
{
	//compute_dt(); - Not used anymore.

	if(flag_new) //new direction to follow
	{
		initial_alpha_error = DEDTORAD(direction);
		alpha_error = DEDTORAD(direction);
		lateral_error = (float)lateral_distance/10; //[mm] to [cm]

		left_motor_set_pos(0); //reset motor step counter for orientation and position determination
		right_motor_set_pos(0);

		last_dist_left = 0;
		last_dist_right = 0;
	}
	else //no new direction to go
	{
		alpha_error = initial_alpha_error - rotation_calc();

		lateral_error += sinf((alpha_error + last_alpha_error)/2) * front_speed_calc();
	}

	compute_controls();

	actuate();

	last_alpha_error = alpha_error;
}

void stop(void)
{
	right_motor_set_speed(0);
	left_motor_set_speed(0);
}

float get_actual_error(void)
{
	return RADTODEG(alpha_error); //[deg]
}

float get_actual_w_z(void)
{
	return RADTODEG(w_z); //[deg/s]
}

///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////// PRIVATE FUNCTIONS //////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////

/*
*	Computes how much the e_puck should rotate on its vertical axis and the desired front speed.
*	We use reference_angle to compensate for lateral motion to the line going from e_puck to its destination.
*/
void compute_controls(void)
{
	float reference_angle = -Ki * lateral_error;	//[rad], new reference to follow

	if(fabsf(reference_angle) > MAX_REFERENCE_ANGLE) // Limit to the reference
		reference_angle = MAX_REFERENCE_ANGLE * reference_angle / fabsf(reference_angle);

	w_z = -Kp * (reference_angle - alpha_error); //[rad/s], proportional compensation

	front_speed = Kspeed * cosf(alpha_error); //[cm/s], goes faster as it points more toward the target
}

/*
*	Computes the wheels speed from the e_puck desired rotation speed and front speed.
*	Modify front_speed if the computed wheel speed is over or under its limits.
*/
void actuate(void)
{
	/////// Wheel speed for e-puck rotation ///////
	float rotation_speed = w_z * WHEEL_TO_WHEEL_DIST/2 * NSTEP_ONE_TURN / WHEEL_PERIMETER; //[step/s], initial rotational speed for each wheel

	if(fabsf(rotation_speed) > MOTOR_SPEED_LIMIT) // Limit to the wheel rotation speed
		rotation_speed = MOTOR_SPEED_LIMIT * rotation_speed / fabsf(rotation_speed);

	/////// Wheel speed for e-puck rotation + forward motion ///////
	float speed_left_wheel = front_speed * NSTEP_ONE_TURN / WHEEL_PERIMETER + rotation_speed; //[step/s]
	float speed_right_wheel = front_speed * NSTEP_ONE_TURN / WHEEL_PERIMETER - rotation_speed; //[step/s]

	// Limit and recomputation if needed to avoid saturation and cancellation of the rotation speed of the e-puck
	if (fabsf(speed_left_wheel) > MOTOR_SPEED_LIMIT || fabsf(speed_left_wheel) > MOTOR_SPEED_LIMIT) {
		if(front_speed >= 0)
		{
			if(rotation_speed>=0)
			{
				speed_left_wheel = MOTOR_SPEED_LIMIT;
				speed_right_wheel = MOTOR_SPEED_LIMIT - 2 * rotation_speed;
				front_speed = (MOTOR_SPEED_LIMIT - rotation_speed) * WHEEL_PERIMETER / NSTEP_ONE_TURN;
			}
			else
			{
				speed_left_wheel = MOTOR_SPEED_LIMIT + 2 * rotation_speed;
				speed_right_wheel = MOTOR_SPEED_LIMIT;
				front_speed = (MOTOR_SPEED_LIMIT + rotation_speed) * WHEEL_PERIMETER / NSTEP_ONE_TURN;
			}
		}
		else
		{
			if(rotation_speed>=0)
			{
				speed_left_wheel = - MOTOR_SPEED_LIMIT + 2 * rotation_speed;
				speed_right_wheel = - MOTOR_SPEED_LIMIT;
				front_speed = (- MOTOR_SPEED_LIMIT + rotation_speed) * WHEEL_PERIMETER / NSTEP_ONE_TURN;
			}
			else
			{
				speed_left_wheel = - MOTOR_SPEED_LIMIT;
				speed_right_wheel = - MOTOR_SPEED_LIMIT - 2 * rotation_speed;
				front_speed = (- MOTOR_SPEED_LIMIT - rotation_speed) * WHEEL_PERIMETER / NSTEP_ONE_TURN;
			}
		}
	}

	right_motor_set_speed(speed_right_wheel);
	left_motor_set_speed(speed_left_wheel);
}

/*
*	Computes the effective front speed without using dt, the time between each loop.
*/
float front_speed_calc(void)
{
	float dist_left = (float)left_motor_get_pos() * WHEEL_PERIMETER/NSTEP_ONE_TURN;		//[cm], compute distance travelled by a wheel since the begining
	float dist_right = (float)right_motor_get_pos() * WHEEL_PERIMETER/NSTEP_ONE_TURN;	//[cm]

	float short_dist_left = dist_left - last_dist_left; 	// [cm], distance travelled by a wheel in a loop
	float short_dist_right = dist_right - last_dist_right;	// [cm]

	last_dist_right = dist_right;
	last_dist_left = dist_left;

	return (short_dist_left+short_dist_right)/2; // tiny distance travelled in [cm]
}

/*
*	Computes how much the e_puck has rotated since the last direction given.
*/
float rotation_calc(void)
{
	float dist_left = (float)left_motor_get_pos() * WHEEL_PERIMETER/NSTEP_ONE_TURN;		//[cm], compute distance travelled by a wheel since the begining
	float dist_right = (float)right_motor_get_pos() * WHEEL_PERIMETER/NSTEP_ONE_TURN;	//[cm]

	return (dist_left-dist_right)/(WHEEL_TO_WHEEL_DIST);	//angle travelled in [rad]
}

/*
*	Used to compute the elapsed time between each loop.
*/
/* Not used anymore
void compute_dt(void)
{	
	static systime_t last_time = 0; //last measured time init in system ticks

	systime_t time = chVTGetSystemTime(); //time in system ticks
	dt = ST2US(time - last_time);

	last_time = time;
}
*/
