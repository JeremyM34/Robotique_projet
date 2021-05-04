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

#define DISTANCE_THRESHOLD 500 // in IR reading units
#define PROX_VALUE_TO_DIST(n) 63.605*powf(n, -0.564) //[cm]
#define FRONT_IR_SENSOR_DISTANCE 3.43 //[cm], distance from the center of the e-puck to the Front IR sensors
#define LATERAL_IR_SENSOR_DISTANCE 3.29 //[cm], distance from the center of the e-puck to the lateral IR sensors
#define IR8_TO_IR1_ANGLE 34 //[deg]
#define IR1_TO_IR2_ANGLE 32 //[deg]
#define IR2_TO_IR3_ANGLE 41 //[deg]

static float dist_to_alpha(float min_distance, float secondary_min_distance, float ir_to_ir_angle, float primary_sensor_distance, float secondary_sensor_distance)
{
	return asinf(((secondary_sensor_distance + secondary_min_distance)/(primary_sensor_distance + min_distance) - cosf(ir_to_ir_angle))/sinf(ir_to_ir_angle));
}

static messagebus_topic_t *proximity_topic;
static proximity_msg_t prox_values;

static map_data map_info;

void mapper_setUp(void)
{
	proximity_start();
	calibrate_ir();

	proximity_topic = messagebus_find_topic_blocking(&bus, "/proximity");

	//VL53L0X_start();
}

void compute_map(void)
{

	if(messagebus_topic_read(proximity_topic, &prox_values, sizeof(prox_values)))
	{
		int max_value_number = 0;
		map_info.obstacle_flag = 0;

		int real_IR_dist[PROXIMITY_NB_CHANNELS]; //not unsigned to avoid issue with negative value when the offset is substracted

		for(int i = 0; i < PROXIMITY_NB_CHANNELS; i++)
		{
			i==3?i=5:0; //Skip IR4 and IR5 as we go forward

			real_IR_dist[i] = prox_values.delta[i]-prox_values.initValue[i];

			if(real_IR_dist[i] > DISTANCE_THRESHOLD)
			{
				map_info.obstacle_flag = 1;
				if (real_IR_dist[i]>real_IR_dist[max_value_number])
				{
					max_value_number = i;
				}
			}
		}

		if(map_info.obstacle_flag)
		{
			float min_distance = PROX_VALUE_TO_DIST(real_IR_dist[max_value_number]);

			int secondary_max_value_number;

			if (real_IR_dist[(max_value_number+1)%PROXIMITY_NB_CHANNELS] > real_IR_dist[(max_value_number-1)%PROXIMITY_NB_CHANNELS])
			{
				secondary_max_value_number = (max_value_number+1)%PROXIMITY_NB_CHANNELS;
			}
			else if(max_value_number==0)
			{
				secondary_max_value_number=7;
			}
			else
			{
				secondary_max_value_number=(max_value_number-1);
			}

			max_value_number==5?secondary_max_value_number=6:0; //Avoid bug if the e-puck is almost 90deg to the obstacle
			max_value_number==2?secondary_max_value_number=1:0;

			float secondary_min_distance = PROX_VALUE_TO_DIST(real_IR_dist[secondary_max_value_number]);

			switch (max_value_number)
			{
			case 2:
				map_info.obstacle_direction = - 90 + RADTODEG(dist_to_alpha(secondary_min_distance, min_distance, DEGTORAD(IR2_TO_IR3_ANGLE),FRONT_IR_SENSOR_DISTANCE,LATERAL_IR_SENSOR_DISTANCE));
				break;

			case 5:
				map_info.obstacle_direction = 90 - RADTODEG(dist_to_alpha(secondary_min_distance, min_distance, DEGTORAD(IR2_TO_IR3_ANGLE),FRONT_IR_SENSOR_DISTANCE,LATERAL_IR_SENSOR_DISTANCE));
				break;

			case 1:
				if(secondary_max_value_number == 2)
					map_info.obstacle_direction = - IR8_TO_IR1_ANGLE/2 - IR1_TO_IR2_ANGLE - RADTODEG(dist_to_alpha(secondary_min_distance, min_distance, DEGTORAD(IR2_TO_IR3_ANGLE),LATERAL_IR_SENSOR_DISTANCE,FRONT_IR_SENSOR_DISTANCE));
				else
					map_info.obstacle_direction = - IR8_TO_IR1_ANGLE/2 - IR1_TO_IR2_ANGLE + RADTODEG(dist_to_alpha(secondary_min_distance, min_distance, DEGTORAD(IR1_TO_IR2_ANGLE),FRONT_IR_SENSOR_DISTANCE,FRONT_IR_SENSOR_DISTANCE));
				break;

			case 6:
				if(secondary_max_value_number == 5)
					map_info.obstacle_direction = IR8_TO_IR1_ANGLE/2 + IR1_TO_IR2_ANGLE + RADTODEG(dist_to_alpha(secondary_min_distance, min_distance, DEGTORAD(IR2_TO_IR3_ANGLE),LATERAL_IR_SENSOR_DISTANCE,FRONT_IR_SENSOR_DISTANCE));
				else
					map_info.obstacle_direction = IR8_TO_IR1_ANGLE/2 + IR1_TO_IR2_ANGLE - RADTODEG(dist_to_alpha(secondary_min_distance, min_distance, DEGTORAD(IR1_TO_IR2_ANGLE),FRONT_IR_SENSOR_DISTANCE,FRONT_IR_SENSOR_DISTANCE));
				break;

			case 0:
				if(secondary_max_value_number == 1)
					map_info.obstacle_direction = -IR8_TO_IR1_ANGLE/2 - RADTODEG(dist_to_alpha(secondary_min_distance, min_distance, DEGTORAD(IR1_TO_IR2_ANGLE),FRONT_IR_SENSOR_DISTANCE,FRONT_IR_SENSOR_DISTANCE));
				else
					map_info.obstacle_direction = -IR8_TO_IR1_ANGLE/2 + RADTODEG(dist_to_alpha(secondary_min_distance, min_distance, DEGTORAD(IR8_TO_IR1_ANGLE),FRONT_IR_SENSOR_DISTANCE,FRONT_IR_SENSOR_DISTANCE));
				break;

			case 7:
				if(secondary_max_value_number == 6)
					map_info.obstacle_direction = IR8_TO_IR1_ANGLE/2 + RADTODEG(dist_to_alpha(secondary_min_distance, min_distance, DEGTORAD(IR1_TO_IR2_ANGLE),FRONT_IR_SENSOR_DISTANCE,FRONT_IR_SENSOR_DISTANCE));
				else
					map_info.obstacle_direction = IR8_TO_IR1_ANGLE/2 - RADTODEG(dist_to_alpha(secondary_min_distance, min_distance, DEGTORAD(IR8_TO_IR1_ANGLE),FRONT_IR_SENSOR_DISTANCE,FRONT_IR_SENSOR_DISTANCE));
				break;
			}
		}
	}
}

map_data get_map(void)
{
	return map_info;
}
