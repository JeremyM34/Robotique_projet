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

#define STOMS(n)	n*1000

#define MAIN_THREAD_FREQUENCY		100		// [Hz]

#define AVOIDANCE_TIMEOUT			1500	// [ms], during a avoidance manoeuver, time after which the e-puck goes in STAND_BY
#define MIN_TIME_OBSTACLE			500		// [ms], minimum time between obstacle detection
#define THRESHOLD_FOLLOW_ANGLE		60		// [deg], threshold angle (e-puck FRONT-BACK axis to obstacle perpendicular axis) over which the dog_bot prefers to follow an obstacle 
#define WALL_FOLLOW_DISTANCE		20		// [mm]
#define THRESHOLD_FOLLOW_W_Z		2		// [deg/s] The e-puck avoids following a new trajectory if it rotational speed is above this limit.
#define MIN_TIME_NEW_DIR			500		// [ms] minimum time between each new direction call

#define NUM_LEDS 					(NUM_RGB_LED + NUM_LED) //Total number of top leds
#define BODY_LED_PWM_FREQ			10000	// [Hz]
#define BODY_LED_PWM_PERIOD			50		// [in ticks]
#define BODY_LED_PULSE_SPEED_FACTOR	1000	// [/ms] body_led glows from off to on in 1000/speed_factor seconds
#define DUTY_CYCLE_MAX				10000	// PWM duty cycle
#define MINIMUM_DUTY_CYCLE			200 	// [0 - DUTY_CYCLE_MAX] 
#define UPSET_LED_TOGGLE_TIME		400 	// [ms]

enum STATES state = STAND_BY;
static bool state_change_flag = TRUE;

static float direction_error = 0;			// [deg], angle corresponding to rotation the e-puck has to do to be in the right direction
static bool new_direction_flag = FALSE;
static systime_t last_direction_time; 		// [ms], last time a new direction to follow has been set
static int8_t lateral_distance = 0; 		// [mm], distance where the e-puck has to go laterally

static THD_WORKING_AREA(waPlayTheDog, 1024);
static THD_FUNCTION(PlayTheDog, arg);

////// PRIVATE FUNCTIONS ///////
static void compute_trajectory(void);
static void follow_trajectory(void);
static void led_showDirection(void);
static void led_showUpset(bool new_call_flag);
static void led_standBy(void);
////////////////////////////////


////////////////////////////////
///////// BODY LED PWM /////////
////////////////////////////////
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
////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////// PUBLIC FUNCTIONS ///////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////

/*
*	Wrapper to setup and start dog_bot
*/
void dog_mode_setUp(void)
{
	sound_direction_setUp();
	motor_controller_setUp();
	mapper_setUp();
	spi_comm_start(); //SPI communication with ESP32 for the RGB leds

	pwmStart(&PWMD5, &pwm_body_led_cfg);
	pwmEnableChannel(&PWMD5, 0, 0);

	chThdCreateStatic(waPlayTheDog, sizeof(waPlayTheDog), NORMALPRIO, PlayTheDog, NULL);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////// PRIVATE FUNCTIONS //////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////

/*
*	Main thread, contains the state machine and controls the e-puck behaviour
*/
static THD_FUNCTION(PlayTheDog, arg)
{
	chRegSetThreadName(__FUNCTION__);
	(void)arg;

	systime_t time;

	while(1){
		time = chVTGetSystemTime();

		///// STATE MACHINE /////
		if(state == STAND_BY)
		{
			if(state_change_flag)
			{
				clear_leds();
				set_front_led(0);
				pwmEnablePeriodicNotification(&PWMD5); // Activate PWM for the body LED
				pwmEnableChannelNotification(&PWMD5, 0);
				stop(); // Stop the motors

				state_change_flag = FALSE;
			}

			led_standBy();
		}
		else if(state == UPSET)
		{
			if(state_change_flag)
			{
				set_front_led(0);
				clear_leds();
				stop();	// Stop the motors
				led_showUpset(TRUE); //toggle half of the leds for left-right alternance

				state_change_flag = FALSE;
			}

			led_showUpset(FALSE);
		}
		else if(state == FOLLOWING || state == AVOIDING)
		{
			if(state_change_flag)
			{
				clear_leds();
				set_front_led(0);
				pwmDisablePeriodicNotification(&PWMD5);	// Desactivate PWM for the body LED
				pwmDisableChannelNotification(&PWMD5, 0);
				set_body_led(0); //Switch of the body led if the PWM ended on led_callback_on 
				state_change_flag = FALSE;
			}

			systime_t motion_timeout = FOLLOWING_TIMEOUT;

			if(state == AVOIDING)
				motion_timeout = AVOIDANCE_TIMEOUT;

			if((ST2MS(chVTGetSystemTime()) - last_direction_time) > motion_timeout)
			{
				state = STAND_BY;
				state_change_flag = TRUE;
			}

			compute_trajectory();
			follow_trajectory();
			led_showDirection();
		}
		/////////////////////////

		//// NEW DIRECTION CHECK ////
		bool rotational_speed_cond = get_actual_w_z() < THRESHOLD_FOLLOW_W_Z; // The e-puck cannot read a new direction correctly if it rotates to fast
		static bool already_called = FALSE;	// We need to wait the e-puck to make a new reading when its w_z is under THRESHOLD_FOLLOW_W_Z

		if(get_sound_angle(&direction_error)  && ((ST2MS(time) - last_direction_time) > MIN_TIME_NEW_DIR)) //avoids to frequent direction changes
		{
			if((state == STAND_BY || state == UPSET || rotational_speed_cond))
			{
				if(already_called)
				{
					new_direction_flag = TRUE;
					last_direction_time = ST2MS(chVTGetSystemTime());

					state = FOLLOWING;
					state_change_flag = TRUE;
				}
				else
				{
					already_called = TRUE;
				}
			}
			else
			{
				already_called = FALSE;
			}
		}
		/////////////////////////////
	
		chThdSleepUntilWindowed(time, time + MS2ST(1000/MAIN_THREAD_FREQUENCY));
	}
}

/////////////////////////////////////////////////////
////////////////////// MOVE /////////////////////////
/////////////////////////////////////////////////////

/*
*	Trajectory computing function, watch for obstacles and choose an action if needed.
*/
static void compute_trajectory(void)
{
	static systime_t last_obstacle_time = 0;
	static map_data map_info;

	if((ST2MS(chVTGetSystemTime())- last_obstacle_time) > MIN_TIME_OBSTACLE)
	{
		if(compute_map()) // returns TRUE is there is an obstacle
		{
			map_info = get_map();
			last_obstacle_time = ST2MS(chVTGetSystemTime());

			if(fabsf(map_info.obstacle_direction) > THRESHOLD_FOLLOW_ANGLE) //Follow the obstacle
			{
				direction_error = 90 * map_info.obstacle_direction/fabsf(map_info.obstacle_direction) - map_info.obstacle_direction;
				lateral_distance = WALL_FOLLOW_DISTANCE * map_info.obstacle_direction/fabsf(map_info.obstacle_direction);
			}
			else if(state == AVOIDING) //Dog_bot gets angry if there is too many obstacles (it was already was avoiding an obstacle)
			{
				state = UPSET;
				state_change_flag = TRUE;
			}
			else //Go to the opposite of the obstacle
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

/*
*	Wrapper to call the motor controller.
*/
static void follow_trajectory(void)
{
	goTo(direction_error, new_direction_flag, lateral_distance);

	if(new_direction_flag)
	{
		new_direction_flag = FALSE;
		lateral_distance = 0;
	}
}

/////////////////////////////////////////////////////
/////////////////////////////////////////////////////

/////////////////////////////////////////////////////
////////////////////// LEDS /////////////////////////
/////////////////////////////////////////////////////

static void led_showDirection(void)
{
	float actual_error = get_actual_error(); //[deg] get angular error to sound source (form motor_controller.c)
	
	uint8_t led_number = (uint8_t)roundf((actual_error + 180) * NUM_LEDS/360) % NUM_LEDS; //not the real led number, but faster than using multiple if

	clear_leds();
	set_front_led(0);

	switch (led_number)
	{
	case 0:
		set_led(LED5, 1);
		break;
	case 1:
		set_rgb_led(LED6, 255, 0, 0);
		break;
	case 2:
		set_led(LED7, 1);
		break;
	case 3:
		set_rgb_led(LED8, 255, 0, 0);
		break;
	case 4:
		set_led(LED1, 1);
		set_front_led(1);
		break;
	case 5:
		set_rgb_led(LED2, 255, 0, 0);
		break;
	case 6:
		set_led(LED3, 1);
		break;
	case 7:
		set_rgb_led(LED4, 255, 0, 0);
		break;
	}
}

static void led_showUpset(bool new_call_flag)
{
	static systime_t last_led_toggle = 0;
	systime_t now = ST2MS(chVTGetSystemTime());

	if(new_call_flag)
	{
		toggle_rgb_led(LED2, RED_LED, 255);
		toggle_rgb_led(LED4, RED_LED, 255);
		set_led(LED3, 2); //2 for toggle

		set_led(LED5, 1);
		set_led(LED1, 1);
	}

	if((now - last_led_toggle) > UPSET_LED_TOGGLE_TIME)
	{
		toggle_rgb_led(LED2, RED_LED, 255);
		toggle_rgb_led(LED4, RED_LED, 255);
		toggle_rgb_led(LED6, RED_LED, 255);
		toggle_rgb_led(LED8, RED_LED, 255);
		set_led(LED3, 2);
		set_led(LED7, 2);

		last_led_toggle = now;
	}

}

/*
*	Wrapper to control the PWM duty cycle.
*/
static void led_standBy(void)
{
	uint16_t duty_cycle = (sinf(M_PI * (float)ST2MS(chVTGetSystemTime())/BODY_LED_PULSE_SPEED_FACTOR) + 1)
							 * (DUTY_CYCLE_MAX - MINIMUM_DUTY_CYCLE)/2 + MINIMUM_DUTY_CYCLE;

	pwmEnableChannel(&PWMD5, 0, PWM_PERCENTAGE_TO_WIDTH(&PWMD5, duty_cycle));
}

/////////////////////////////////////////////////////
/////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////
