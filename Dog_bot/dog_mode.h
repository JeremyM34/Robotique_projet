#ifndef DOG_MODE_H
#define DOG_MODE_H

#define FOLLOWING_TIMEOUT	10000 //[ms], Time after which the dog bot stop if there's not new direction or obstacle.

enum STATES{STAND_BY, FOLLOWING, AVOIDING, UPSET};

/*
*	Wrapper to setup and start dog_bot, launches the main thread and the various sensors/actuators
*/
void dog_mode_setUp(void);

#endif /* DOG_MODE_H */
