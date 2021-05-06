#ifndef DOG_MODE_H
#define DOG_MODE_H

#define FOLLOWING_TIMEOUT	7 //[s]

enum STATES{STAND_BY, FOLLOWING, AVOIDING, UPSET};

void dog_mode_setUp(void);

void playTheDog(void);

void compute_trajectory(void);
void follow_trajectory(void);

void led_showDirection(void);
void led_showUpset(bool new_call);
void led_standBy(void);

#endif /* DOG_MODE_H */
