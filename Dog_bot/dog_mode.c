#include "ch.h"
#include "hal.h"
#include <usbcfg.h>
#include <chprintf.h>
#include <main.h>
#include <math.h>

#include <dog_mode.h>
//#include <sound_direction.h>
#include <mapper.h>
#include <motor_controller.h>

static double direction_error = -135;

void dog_mode_setUp(void)
{
	//sound_direction_setUp();
	motor_controller_setUp();
	mapper_setUp();
	goTo(direction_error, 1);
}

void playTheDog(void)
{	
	follow_trajectory();
	/*
	if(directionAge>DIRECTION_TIMEOUT)
	{
		led_showDirection();
		led_showMood();
		compute_trajectory();
		follow_trajectory();
	}
	else
	{
		led_standBy();
	}
	*/
}

///////////// MOVE ////////////////

void compute_trajectory(void)
{

}

void follow_trajectory(void)
{
	goTo(direction_error, 0);
}

///////////// LEDS ////////////////
void led_showDirection(void)
{

}

void led_showMood(void)
{

}

void led_standBy(void)
{

}

