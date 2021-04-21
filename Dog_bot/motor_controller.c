#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>
#include <main.h>

#include <motor_controller.h>
#include <motors.h>

#define DEDTORAD(n)	n * M_PI / 180
#define USTOS(n)	n/1000000.

#define NSTEP_ONE_TURN      1000 // number of step for 1 turn of the motor
#define WHEEL_PERIMETER     12.83333 // [cm]
#define	WHEEL_TO_WHEEL_DIST	5.35	// [cm]

#define P	1.5	//Proportional term (error to e_puck rotation speed)
#define I	0.15	//Integral term (regulate perpendicular drift from initial straight line to direction)
#define Kv	10	//Proportional term (regulate front speed depending on direction error)

static systime_t dt = 0; //[us] time elapsed between each loop

static float initial_error = 0;
static float alpha_error = 0; //direction error in [rad]
static float last_alpha_error = 0; //last direction error in [rad]
static float perpendicular_error = 0; //perpendicular position error in [cm]
static float front_speed = 0; //speed toward the front of the e_puck in [cm/s]
static float w_z = 0; //rotational speed of the e_puck

void motor_controller_setUp(void)
{
	//inits the motors
	motors_init();
}

void goTo(double direction, bool flag_new)
{
	compute_dt();

	if(flag_new) //new direction to follow
	{
		initial_error = DEDTORAD(direction);
		alpha_error = DEDTORAD(direction);
		perpendicular_error = 0;

		left_motor_set_pos(0); //reset motor step counter for orientation determination
		right_motor_set_pos(0);
	}
	else //no new direction to go
	{
		alpha_error = initial_error - rotation_calc();

		perpendicular_error += sinf((alpha_error + last_alpha_error)/2) * front_speed * (float)dt/1000000.;
	}

	//chprintf((BaseSequentialStream *) &SD3, "alpha_error = %3f \n", alpha_error);

	compute_controls();

	actuate();

	last_alpha_error = alpha_error;
}

void compute_controls(void)
{
	float reference_angle = -I * perpendicular_error;	//[rad], new reference to follow

	w_z = -P * (reference_angle - alpha_error); //[rad/s], proportional compensation

	front_speed = Kv * cosf(alpha_error); //[cm/s], goes faster as it points more toward the target

}

void actuate(void)
{
	float rotation_speed = w_z * WHEEL_TO_WHEEL_DIST/2 * NSTEP_ONE_TURN / WHEEL_PERIMETER; //[step/s], initial rotational speed for each wheel

	if(fabsf(rotation_speed) > MOTOR_SPEED_LIMIT)
	{
		rotation_speed = MOTOR_SPEED_LIMIT * rotation_speed / fabsf(rotation_speed);
	}

	float speed_left_wheel = front_speed * NSTEP_ONE_TURN / WHEEL_PERIMETER + rotation_speed; //[step/s]
	float speed_right_wheel = front_speed * NSTEP_ONE_TURN / WHEEL_PERIMETER - rotation_speed; //[step/s]

	//chprintf((BaseSequentialStream *) &SD3, "speed_left_wheel = %3f ; speed_right_wheel = %3f \n", speed_left_wheel, speed_right_wheel);

	if (fabsf(speed_left_wheel) > MOTOR_SPEED_LIMIT || fabsf(speed_left_wheel) > MOTOR_SPEED_LIMIT) {
		if(front_speed >= 0)
		{
			if(rotation_speed>=0)
			{
				speed_left_wheel = MOTOR_SPEED_LIMIT;
				speed_right_wheel = MOTOR_SPEED_LIMIT - rotation_speed;
				front_speed = speed_right_wheel;
			}
			else
			{
				speed_left_wheel = MOTOR_SPEED_LIMIT + rotation_speed;
				speed_right_wheel = MOTOR_SPEED_LIMIT;
				front_speed = speed_left_wheel;
			}
		}
		else
		{
			if(rotation_speed>=0)
			{
				speed_left_wheel = - MOTOR_SPEED_LIMIT + rotation_speed;
				speed_right_wheel = - MOTOR_SPEED_LIMIT;
				front_speed = speed_left_wheel;
			}
			else
			{
				speed_left_wheel = - MOTOR_SPEED_LIMIT;
				speed_right_wheel = - MOTOR_SPEED_LIMIT - rotation_speed;
				front_speed = speed_right_wheel;
			}
		}
	}

	right_motor_set_speed(speed_right_wheel);
	left_motor_set_speed(speed_left_wheel);
}

float rotation_calc(void)
{
	float dist_left = (float)left_motor_get_pos() * WHEEL_PERIMETER/NSTEP_ONE_TURN;	//[cm], compute distance travelled by a wheel
	float dist_right = (float)right_motor_get_pos() * WHEEL_PERIMETER/NSTEP_ONE_TURN;//[cm]

	//chprintf((BaseSequentialStream *) &SD3, "left_motor_get_pos = %3f; right_motor_get_pos = %3f \n", (float)left_motor_get_pos(), (float)right_motor_get_pos());

	return (dist_left-dist_right)/(WHEEL_TO_WHEEL_DIST);	//angle travelled in [rad]
}

void compute_dt(void)
{	
	static systime_t last_time = 0; //last measured time init in system ticks

	systime_t time = chVTGetSystemTime(); //time in system ticks
	dt = ST2US(time - last_time);

	//chprintf((BaseSequentialStream *) &SD3, "dt = %d ; ", chVTGetSystemTime());

	last_time = time;
}
