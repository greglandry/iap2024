/******************************************************************************
* File Name: displayTask.h
*
* Description: This file is the public interface of displayTask.c
*
******************************************************************************/

#ifndef DISPLAY_TASK_H_
#define DISPLAY_TASK_H_

// Middleware Headers
#include "FreeRTOS.h"
#include "task.h"

// Application Headers
#include "pong.h"

/*******************************************************************************
* Macros
********************************************************************************/
#define DISPLAY_TASK_PRIORITY   (1)
#define DISPLAY_TASK_STACK_SIZE (1024 * 2)

#define SCREEN_MAX_X (320 - 1)
#define SCREEN_MAX_Y (240 - 1)

#define PADDLE_WIDTH  (40)
#define PADDLE_HEIGHT (10)

#define PADDLE_START_X (((SCREEN_MAX_X/2) - (PADDLE_WIDTH/2)))

#define BALL_SIZE     (10)

#define BALL_START_X (((SCREEN_MAX_X/2) - (BALL_SIZE/2)))

/*******************************************************************************
* Extern Variables
********************************************************************************/
// Defined in displayTask.c
extern TaskHandle_t display_task_handle;
extern ball gameBall;
extern paddle gamePaddle;

/*******************************************************************************
* Function Prototypes
********************************************************************************/
void displayTask(void *arg);

/* Structure to hold handles of discovered characteristics: */
typedef struct {
	uint16_t startHandle;
	uint16_t endHandle;
	uint16_t valHandle;
	uint16_t cccdHandle;
} charHandle_t;

#endif /* DISPLAY_TASK_H_ */
