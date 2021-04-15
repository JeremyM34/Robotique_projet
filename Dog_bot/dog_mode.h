#ifndef DOG_MODE_H
#define DOG_MODE_H

#define DIRECTION_TIMEOUT 15 // [sec]

void dog_mode_setUp(void);

void playTheDog(void);

void compute_trajectory(void);
void follow_trajectory(void);

void led_showDirection(void);
void led_showMood(void);
void led_standBy(void);

#endif /* DOG_MODE_H */
