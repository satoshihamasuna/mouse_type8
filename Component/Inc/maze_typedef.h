/*
 * maze_typedef.h
 *
 *  Created on: 2024/03/05
 *      Author: sato1
 */

#ifndef CPP_PHERIPHERAL_INCLUDE_MAZE_TYPEDEF_H_
#define CPP_PHERIPHERAL_INCLUDE_MAZE_TYPEDEF_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <cstdint>

typedef enum{
	UNKNOWN	= 2,
	NOWALL	= 0,
	WALL	= 1,
	VWALL   = 3,
}t_wall_state;



typedef enum{
	Front = 0,
	Right = 1,
	Rear  = 2,
	Left  = 3,
	Right_Front = 4,
	Right_Rear 	= 5,
	Left_Rear	= 6,
	Left_Front	= 7,
	None	= 8,
}t_local_dir;

typedef enum{
	wall_n = 0,
	wall_e = 1,
	wall_s = 2,
	wall_w = 3,
}t_wall_pos_dir;

typedef enum{
	North 		= 0,
	East		= 1,
	South		= 2,
	West 		= 3,
	NorthEast 	= 4,
	SouthEast	= 5,
	SouthWest   = 6,
	NorthWest 	= 7,
	Dir_None	= 8,
}t_direction;


typedef struct{
	uint8_t x;
	uint8_t y;
	t_direction dir;
}t_position;

typedef struct{
	unsigned char north : 2;
	unsigned char east  : 2;
	unsigned char south : 2;
	unsigned char west  : 2;
}t_wall;


#ifdef __cplusplus
}
#endif

#endif /* CPP_PHERIPHERAL_INCLUDE_MAZE_TYPEDEF_H_ */
