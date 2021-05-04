#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

void motor_controller_setUp(void);

void goTo(float direction, bool flag_new, int lateral_distance);
void stop(void);

void compute_controls(void);
void actuate(void);

float front_speed_calc(void);
float rotation_calc(void);
void compute_dt(void);

float get_actual_error(void);

#endif /* MOTOR_CONTROLLER_H */
