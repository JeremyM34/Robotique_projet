#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <usbcfg.h>
#include <main.h>
#include <chprintf.h>

#include <dog_mode.h>

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

static void serial_start(void)  //For DEBUG
{
	static SerialConfig ser_cfg = {
        115200,
        0,
        0,
        0,
	};

	sdStart(&SD3, &ser_cfg); // UART3.
}

int main(void)
{
    halInit();
    chSysInit();
    mpu_init();

    messagebus_init(&bus, &bus_lock, &bus_condvar);

    serial_start(); //For DEBUG
    usb_start();    //For DEBUG

//////////////////////////
    dog_mode_setUp(); //Initialize everything and starts the main thread
//////////////////////////

    while (1) {
		chThdSleepMilliseconds(1000);
	}

}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
