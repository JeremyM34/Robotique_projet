#include "ch.h"
#include "hal.h"
#include <usbcfg.h>
#include <chprintf.h>
#include <math.h>

#include <dog_mode.h>
#include <arm_math.h>
#include <arm_const_structs.h>
#include <sound_direction.h>

#define DIRECTION_TIMEOUT 15 // [sec]

void dog_mode_setUp()
{

}

void playTheDog()
{
	/*
	if(directionAge>DIRECTION_TIMEOUT)
	{
		led_showDirection();
		led_showMood();
		compute_trajectory();
		follow_trajectory()
	}
	else
	{
		led_standBy();
	}
	*/
}

///////////// MOVE ////////////////

void compute_trajectory()
{

}

void follow_trajectory()
{
	//goTo();
}

///////////// LEDS ////////////////
void led_showDirection()
{

}

void led_showMood()
{

}

void led_standBy()
{

}
