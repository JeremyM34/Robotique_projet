#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

/*
*	Function to be called for motor initialization.
*/
void motor_controller_setUp(void);

/*
*   Motor control loop : make the e-puck move toward a direction without having to call for a new direction at each loop.
*	Params:
*		- direction : angle [deg]  describing where to go (useful only when new direction to go is computed)
*		- flag_new : 1 if the direction is new, 0 if not.
		- lateral_distance : distance [cm], where the e-puck has to go laterally
*/
void goTo(float direction, bool flag_new, int lateral_distance);

/*
*	Function to stop the motors.
*/
void stop(void);

float get_actual_error(void);

#endif /* MOTOR_CONTROLLER_H */
