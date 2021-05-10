#ifndef MAPPER_H
#define MAPPER_H

typedef struct {
    bool obstacle_flag; //TRUE if there is an obstacle under the threshold distance
    float obstacle_direction; //[deg]
} map_data;

/*
*	Function to be called for sensors initialization.
*/
void mapper_setUp(void);

/*
*	Mapping function :
*	    return TRUE is there's an obstacle under the threshold distance
*/
bool compute_map(void);

map_data get_map(void);

#endif /* MAPPER_H */
