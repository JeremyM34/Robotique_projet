#include "ch.h"
#include "hal.h"
#include <usbcfg.h>
#include <chprintf.h>
#include <main.h>

#include <mapper.h>
#include <sensors/proximity.h>
#include <sensors/VL53L0X/VL53L0X.h>

void mapper_setUp(void)
{
	//proximity_start();
	//calibrate_ir();
	VL53L0X_start();
}

void compute_map(void)
{

}
