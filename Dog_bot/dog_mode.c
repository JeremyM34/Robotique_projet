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
#include <spi_comm.h>

static float direction_error = -180;

static int body_led_flag = 1;

static void body_led_callback(PWMDriver *gptp)
{
	(void) gptp;
	body_led_flag?set_body_led(2):set_body_led(0); //2 for toogle
}

static const PWMConfig pwm_body_led = {
       .frequency = 0, //variable
		.period = 0xFFFF,
		.callback = body_led_callback
};

void dog_mode_setUp(void)
{
	//sound_direction_setUp();
	motor_controller_setUp();
	mapper_setUp();
	spi_comm_start(); //For the RGB leds

	pwmStart(&PWMD5, &pwm_body_led);
	pwmEnablePeriodicNotification(&PWMD5); // PWM general interrupt at the beginning of the period to handle motor steps.

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
	static map_data map_info;

	map_info = get_map();
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
	set_front_led(0);

	switch (led_number)
	{
	case 1:
		set_led(LED5, 1);
		break;
	case 2:
		set_rgb_led(LED6, 255, 0, 0);
		break;
	case 3:
		set_led(LED7, 1);
		break;
	case 4:
		set_rgb_led(LED8, 255, 0, 0);
		break;
	case 5:
		set_led(LED1, 1);
		set_front_led(1);
		break;
	case 6:
		set_rgb_led(LED2, 255, 0, 0);
		break;
	case 7:
		set_led(LED3, 1);
		break;
	case 8:
		set_rgb_led(LED4, 255, 0, 0);
		break;
	case 9:
		set_led(LED5, 1);
		break;
	}
}

void led_showMood(void)
{

}

void led_standBy(void)
{
	!body_led_flag?body_led_flag=1:0;

	//pwm_body_led.frequency = new_frequency;
}

