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
#include <leds.h>

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
	led_showDirection();

	/*
	if(directionAge<DIRECTION_TIMEOUT)
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
	float actual_error = get_actual_error(); //[deg] get precise error form motor_controller.c
	
	int led_number = round((actual_error+180)*8/360) + 1;

	clear_leds();

	switch (led_number)
	{
	case 1:
		set_led(LED1, 1);
		break;
	case 2:
		set_rgb_led(LED2, 255, 0, 0)
		break;
	case 3:
		set_led(LED3, 1);
		break;
	case 4:
		set_rgb_led(LED4, 255, 0, 0)
		break;
	case 5:
		set_led(LED5, 1);
		break;
	case 6:
		set_rgb_led(LED6, 255, 0, 0)
		break;
	case 7:
		set_led(LED7, 1);
		break;
	case 8:
		set_rgb_led(LED8, 255, 0, 0)
		break;
	case 9:
		set_led(LED1, 1);
		break;
	}
}

void led_showMood(void)
{

}

void led_standBy(void)
{
	static bool 
}

