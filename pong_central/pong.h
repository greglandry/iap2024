/******************************************************************************
* File Name: pong.h
*
* Description: This file defines the structs used to represent game objects
*
******************************************************************************/

#ifndef PONG_H_
#define PONG_H_

typedef struct ball {
	int dim;
	int16_t posX, posY, prev_posX, prev_posY;
	int16_t speedX, speedY;
	int16_t numBounces;
} ball;

typedef struct paddle {
	int dimX, dimY;
	volatile int posX, posY, prev_posX, prev_posY;
} paddle;

#endif /* PONG_H_ */
