#ifndef MAPPER_H
#define MAPPER_H

typedef struct {
    int obstacle_flag; //true if there is an obstacle under the threshold distance
    float obstacle_direction; //[deg]
} map_data;

void mapper_setUp(void);

void compute_map(void);

map_data get_map(void);

#endif /* MAPPER_H */
