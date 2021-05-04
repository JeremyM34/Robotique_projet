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

#define STANDARD_DIRECTION_TIMEOUT	7 //[s]
#define AVOIDANCE_TIMEOUT	5 //[s]
#define TIME_BETWEEN_OBSTACLE 1.5 //[s]

#define STOMS(n)	n*1000

static float direction_error = -90;

static int new_direction_flag = 1;
static systime_t last_direction_time; //[ms]

static int avoidance_manoeuver_flag = 0;

static int lateral_distance = 0; //[cm]

/*
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
*/

void dog_mode_setUp(void)
{
	//sound_direction_setUp();
	motor_controller_setUp();
	mapper_setUp();
	spi_comm_start(); //For the RGB leds

	//pwmStart(&PWMD5, &pwm_body_led);
	//pwmEnablePeriodicNotification(&PWMD5); // PWM general interrupt at the beginning of the period to handle motor steps.

	//goTo(direction_error, 1);
	last_direction_time = ST2MS(chVTGetSystemTime());
}

void playTheDog(void)
{	
	//compute_trajectory();
	//follow_trajectory();
	//led_showDirection();

	compute_trajectory();



	/*
	if(get_sound_angle(&sound_direction))
	{

	}
	*/
	int direction_timeout;
	int directionAge = ST2MS(chVTGetSystemTime()) - last_direction_time;

	//chprintf((BaseSequentialStream *) &SD3, "flag = %d \n", avoidance_manoeuver_flag);

	if(avoidance_manoeuver_flag)
		direction_timeout = STOMS(AVOIDANCE_TIMEOUT);
	else
		direction_timeout = STOMS(STANDARD_DIRECTION_TIMEOUT);

	if(directionAge<direction_timeout)
	{
		follow_trajectory();
		led_showDirection();
	}
	else
	{
		stop();
		led_standBy();
	}

}

///////////// MOVE ////////////////

void compute_trajectory(void)
{
	static systime_t last_obstacle_time = 0;

	static map_data map_info;

	compute_map();

	map_info = get_map();

	if(map_info.obstacle_flag && ((ST2MS(chVTGetSystemTime())- last_obstacle_time) > STOMS(TIME_BETWEEN_OBSTACLE)))
	{
		map_info.obstacle_flag = 0;
		last_obstacle_time = ST2MS(chVTGetSystemTime());

		if(fabsf(map_info.obstacle_direction) > 50)
		{
			direction_error = 90 * map_info.obstacle_direction/fabsf(map_info.obstacle_direction) - map_info.obstacle_direction;
			lateral_distance = 4 * map_info.obstacle_direction/fabsf(map_info.obstacle_direction);
		}
		else
		{
			direction_error = 180 - map_info.obstacle_direction;
			direction_error>180?direction_error-=360:0;
			last_direction_time = ST2MS(chVTGetSystemTime());
			avoidance_manoeuver_flag = 1;
			lateral_distance = 0;
		}
		new_direction_flag = 1;
	}

}

void follow_trajectory(void)
{
	goTo(direction_error, new_direction_flag, lateral_distance);

	if(new_direction_flag)
	{
		new_direction_flag = 0;
		lateral_distance = 0;
	}
}

///////////// LEDS ////////////////
void led_showDirection(void)
{
	float actual_error = get_actual_error(); //[deg] get precise error form motor_controller.c
	
	int led_number = round((actual_error+180)*8/360) + 1;

	clear_leds();
	set_front_led(0);
	set_body_led(0);

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
	static int body_led_counter = 0;

	clear_leds();
	set_front_led(0);

	if(body_led_counter%20==1)
	{
		set_body_led(1); //2 for toggle
	}
	else
	{
		set_body_led(0);
	}

	body_led_counter++;
}

