#include "ch.h"
#include "hal.h"
#include <usbcfg.h>
#include <chprintf.h>
#include <main.h>
#include <math.h>

#include <dog_mode.h>
#include <sound_direction.h>
#include <mapper.h>
#include <motor_controller.h>
#include <leds.h>
#include <spi_comm.h>

#define AVOIDANCE_TIMEOUT		1.5		//[s]
#define TIME_BETWEEN_OBSTACLE	0.5		//[s]
#define THRESHOLD_FOLLOW_ANGLE	50		//[deg]
#define WALL_FOLLOW_DISTANCE	4		//[cm]
#define THRESHOLD_FOLLOW_W_W	5		//[deg/s] The e-puck avoids following a new trajectory if it rotational speed is above this limit.

#define BODY_LED_PWM_FREQ		10000	//[Hz]
#define BODY_LED_PWM_PERIOD		50		//[in ticks]
#define BODY_LED_PULSE_SPEED_FACTOR	600.//[/ms] body_led glows from off to on in 1000/speed_factor seconds
#define DUTY_CYCLE_MAX			10000
#define MINIMUM_DUTY_CYCLE		200 	//[0 - DUTY_CYCLE_MAX]

#define UPSET_LED_TOGGLE_TIME	400 	//[ms]

#define STOMS(n)	n*1000


enum STATES state = STAND_BY;

static bool state_change = TRUE;

static float direction_error = -90;
static bool new_direction_flag = FALSE;

static systime_t last_direction_time; //[ms]
static int lateral_distance = 0; //[cm]

///////////////////////////
////// BODY LED PWM ///////
///////////////////////////
static void body_led_callback_on(PWMDriver *pwmp)
{
	(void)pwmp;
	set_body_led(1);
}

static void body_led_callback_off(PWMDriver *pwmp)
{
	(void)pwmp;
	set_body_led(0);
}

static PWMConfig pwm_body_led_cfg = {
	.frequency = BODY_LED_PWM_FREQ,
	.period = BODY_LED_PWM_PERIOD,
	.cr2 = 0,
	.callback = body_led_callback_on,
	{
		{PWM_OUTPUT_DISABLED, body_led_callback_off},
		{PWM_OUTPUT_DISABLED, NULL},
		{PWM_OUTPUT_DISABLED, NULL},
		{PWM_OUTPUT_DISABLED, NULL}
	}
};
///////////////////////////
///////////////////////////

void dog_mode_setUp(void)
{
	sound_direction_setUp();
	motor_controller_setUp();
	mapper_setUp();
	spi_comm_start(); //For the RGB leds

	pwmStart(&PWMD5, &pwm_body_led_cfg);
	pwmEnableChannel(&PWMD5, 0, 0);

	last_direction_time = ST2MS(chVTGetSystemTime());
}

void playTheDog(void)
{
	if(state == STAND_BY)
	{
		if(state_change)
		{
			clear_leds();
			set_front_led(0);
			pwmEnablePeriodicNotification(&PWMD5); // Activate PWM for the body LED
			pwmEnableChannelNotification(&PWMD5, 0);
			stop();

			state_change = FALSE;
		}

		led_standBy();
	}
	else if(state == UPSET)
	{
		if(state_change)
		{
			set_front_led(0);
			stop();
			led_showUpset(1);

			state_change = FALSE;
		}

		led_showUpset(0);
	}
	else if(state == FOLLOWING || state == AVOIDING)
	{
		if(state_change)
		{
			pwmDisablePeriodicNotification(&PWMD5); // Activate PWM for the body LED
			pwmDisableChannelNotification(&PWMD5, 0);
			state_change = FALSE;
		}

		systime_t direction_timeout = STOMS(FOLLOWING_TIMEOUT);
		systime_t directionAge = ST2MS(chVTGetSystemTime()) - last_direction_time;

		if(state == AVOIDING)
			direction_timeout = STOMS(AVOIDANCE_TIMEOUT);

		if(directionAge > direction_timeout)
		{
			state = STAND_BY;
			state_change = TRUE;
		}

		compute_trajectory();
		follow_trajectory();
		led_showDirection();
	}

	if(get_sound_angle(&direction_error))
	{
		if(state == STAND_BY || state == UPSET || (get_actual_w_z()<5))
		{
			new_direction_flag = 1;
			last_direction_time = ST2MS(chVTGetSystemTime());

			state = FOLLOWING;
			state_change = TRUE;	
		}
	}
}

///////////// MOVE ////////////////

void compute_trajectory(void)
{
	static systime_t last_obstacle_time = 0;
	static map_data map_info;

	if((ST2MS(chVTGetSystemTime())- last_obstacle_time) > STOMS(TIME_BETWEEN_OBSTACLE))
	{
		if(compute_map())
		{
			map_info = get_map();
			last_obstacle_time = ST2MS(chVTGetSystemTime());

			if(fabsf(map_info.obstacle_direction) > THRESHOLD_FOLLOW_ANGLE)
			{
				direction_error = 90 * map_info.obstacle_direction/fabsf(map_info.obstacle_direction) - map_info.obstacle_direction;
				lateral_distance = WALL_FOLLOW_DISTANCE * map_info.obstacle_direction/fabsf(map_info.obstacle_direction);
			}
			else if(state == AVOIDING) //Dog_bot gets angry if there is too many obstacles (it was already was avoiding an obstacle)
			{
				state = UPSET;
				state_change = TRUE;
			}
			else
			{
				direction_error = 180 - map_info.obstacle_direction;
				direction_error>180?direction_error-=360:0;

				last_direction_time = ST2MS(chVTGetSystemTime());
				
				state = AVOIDING;
			}
			new_direction_flag = TRUE;
		}
	}
}

void follow_trajectory(void)
{
	goTo(direction_error, new_direction_flag, lateral_distance);

	if(new_direction_flag)
	{
		new_direction_flag = FALSE;
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

void led_showUpset(bool new_call)
{
	static systime_t last_led_toggle = 0;
	systime_t now = ST2MS(chVTGetSystemTime());

	if(new_call)
	{
		toggle_rgb_led(LED2, RED_LED, 255);
		toggle_rgb_led(LED4, RED_LED, 255);
		set_led(LED3, 2); //2 for toggle
	}

	if((now - last_led_toggle) > UPSET_LED_TOGGLE_TIME)
	{
		set_led(LED5, 1);
		set_led(LED1, 1);
		toggle_rgb_led(LED2, RED_LED, 255);
		toggle_rgb_led(LED4, RED_LED, 255);
		toggle_rgb_led(LED6, RED_LED, 255);
		toggle_rgb_led(LED8, RED_LED, 255);
		set_led(LED3, 2);
		set_led(LED7, 2);

		last_led_toggle = now;
	}

}

void led_standBy(void)
{
	int duty_cycle = 0;

	duty_cycle = (sinf(M_PI * (float)ST2MS(chVTGetSystemTime())/BODY_LED_PULSE_SPEED_FACTOR) + 1) * (DUTY_CYCLE_MAX - MINIMUM_DUTY_CYCLE)/2 + MINIMUM_DUTY_CYCLE;

	pwmEnableChannel(&PWMD5, 0, PWM_PERCENTAGE_TO_WIDTH(&PWMD5, duty_cycle));
}

