#include "ch.h"
#include "hal.h"
#include <usbcfg.h>
#include <chprintf.h>
#include <main.h>

#include <motor_controller.h>
#include <motors.h>

void motor_controller_setUp(void)
{
	//inits the motors
	motors_init();
}

void goTo(double direction_error)
{

}
