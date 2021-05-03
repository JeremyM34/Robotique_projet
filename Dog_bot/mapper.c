#include "ch.h"
#include "hal.h"
#include <usbcfg.h>
#include <chprintf.h>
#include <main.h>
#include <math.h>

#include <mapper.h>
#include <sensors/proximity.h>
//#include <sensors/VL53L0X/VL53L0X.h>

#define RADTODEG(n)	n * 180 / M_PI
#define DEGTORAD(n)	n * M_PI / 180

#define DISTANCE_THRESHOLD 100
#define PROX_VALUE_TO_DIST(n) n/10 //[cm]
#define IR_SENSOR_DISTANCE 2.5 //[cm], distance from the center of the e-puck to the IR sensor
#define IR8_TO_IR1_ANGLE 34 //[deg]
#define IR1_TO_IR2_ANGLE 32 //[deg]
#define IR2_TO_IR3_ANGLE 41 //[deg]

static float dist_to_alpha(float min_distance, float secondary_min_distance, float ir_to_ir_angle)
{
	return asinf(((IR_SENSOR_DISTANCE + secondary_min_distance)/(IR_SENSOR_DISTANCE + min_distance) - cosf(ir_to_ir_angle))/sinf(ir_to_ir_angle));
}

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

static messagebus_topic_t *proximity_topic;
static proximity_msg_t prox_values;

static map_data map_info;

void mapper_setUp(void)
{
	proximity_start();
	calibrate_ir();

	messagebus_init(&bus, &bus_lock, &bus_condvar);
	proximity_topic = messagebus_find_topic_blocking(&bus, "/proximity");

	//VL53L0X_start();
}

void compute_map(void)
{
	int max_value_number = 0;
	messagebus_topic_read(proximity_topic, &prox_values, sizeof(prox_values));

	map_info.obstacle_flag = 0;

	for(int i = 0; i < PROXIMITY_NB_CHANNELS; i++)
	{
		i==3?i=5:0; //Skip IR4 and IR5 as we go forward

		if(prox_values.delta[i] > DISTANCE_THRESHOLD)
		{
			map_info.obstacle_flag = 1;
			if (prox_values.delta[i]>prox_values.delta[max_value_number])
			{
				max_value_number = i;
			}
		}
	}

	if(map_info.obstacle_flag)
	{
		float min_distance = PROX_VALUE_TO_DIST(prox_values.delta[max_value_number]);

		int secondary_max_value_number;

		if (prox_values.delta[(max_value_number+1)%PROXIMITY_NB_CHANNELS] > prox_values.delta[(max_value_number-1)%PROXIMITY_NB_CHANNELS])
		{
			secondary_max_value_number = (max_value_number+1)%PROXIMITY_NB_CHANNELS;
		}
		else
		{
			secondary_max_value_number = (max_value_number-1)%PROXIMITY_NB_CHANNELS;
		}

		max_value_number==5?secondary_max_value_number=6:0; //Avoid bug if the e-puck is almost 90deg to the obstacle
		max_value_number==2?secondary_max_value_number=1:0;

		float secondary_min_distance = PROX_VALUE_TO_DIST(prox_values.delta[secondary_max_value_number]);

		switch (max_value_number)
		{
		case 2:
			map_info.obstacle_direction = 90 - RADTODEG(dist_to_alpha(min_distance, secondary_min_distance, DEGTORAD(IR2_TO_IR3_ANGLE)));
			break;

		case 5:
			map_info.obstacle_direction = - 90 + RADTODEG(dist_to_alpha(min_distance, secondary_min_distance, DEGTORAD(IR2_TO_IR3_ANGLE)));
			break;
		
		case 1:
			if(secondary_max_value_number == 2)
				map_info.obstacle_direction = IR8_TO_IR1_ANGLE/2 + IR1_TO_IR2_ANGLE + RADTODEG(dist_to_alpha(min_distance, secondary_min_distance, DEGTORAD(IR2_TO_IR3_ANGLE)));
			else
				map_info.obstacle_direction = IR8_TO_IR1_ANGLE/2 + IR1_TO_IR2_ANGLE - RADTODEG(dist_to_alpha(min_distance, secondary_min_distance, DEGTORAD(IR1_TO_IR2_ANGLE)));
			break;

		case 6:
			if(secondary_max_value_number == 5)
				map_info.obstacle_direction = - IR8_TO_IR1_ANGLE/2 - IR1_TO_IR2_ANGLE - RADTODEG(dist_to_alpha(min_distance, secondary_min_distance, DEGTORAD(IR2_TO_IR3_ANGLE)));
			else
				map_info.obstacle_direction = - IR8_TO_IR1_ANGLE/2 - IR1_TO_IR2_ANGLE + RADTODEG(dist_to_alpha(min_distance, secondary_min_distance, DEGTORAD(IR1_TO_IR2_ANGLE)));
			break;

		case 0:
			if(secondary_max_value_number == 1)
				map_info.obstacle_direction = IR8_TO_IR1_ANGLE/2 + RADTODEG(dist_to_alpha(min_distance, secondary_min_distance, DEGTORAD(IR1_TO_IR2_ANGLE)));
			else
				map_info.obstacle_direction = IR8_TO_IR1_ANGLE/2 - RADTODEG(dist_to_alpha(min_distance, secondary_min_distance, DEGTORAD(IR8_TO_IR1_ANGLE)));
			break;

		case 7:
			if(secondary_max_value_number == 6)
				map_info.obstacle_direction = - IR8_TO_IR1_ANGLE/2 - RADTODEG(dist_to_alpha(min_distance, secondary_min_distance, DEGTORAD(IR1_TO_IR2_ANGLE)));
			else
				map_info.obstacle_direction = - IR8_TO_IR1_ANGLE/2 + RADTODEG(dist_to_alpha(min_distance, secondary_min_distance, DEGTORAD(IR8_TO_IR1_ANGLE)));
			break;
		}
	}
}

map_data get_map(void)
{
	return map_info;
}