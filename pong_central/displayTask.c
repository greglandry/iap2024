/******************************************************************************
* File Name: displayTask.c
*
* Description: This task updates the TFT display
*
******************************************************************************/

// MCU Headers
#include "cy_pdl.h"
#include "cyhal.h"

// BT Stack
#include "wiced_bt_stack.h"

// BT app utils
#include "app_bt_utils.h"

// BT configurator generated headers
#include "cycfg_bt_settings.h"
#include "cycfg_gap.h"

// Display headers
#include "GUI.h"

// srand(), rand()
#include "stdlib.h"

// Display task header file
#include "displayTask.h"

/*******************************************************************
 * External Function Prototypes
 ******************************************************************/
extern void writeAttribute(uint16_t conn_id, uint16_t handle, uint16 offset, wiced_bt_gatt_auth_req_t auth_req, uint16_t len, uint8_t *val);

/*******************************************************************************
* Global Variables
********************************************************************************/
/* Task handle for this task. */
TaskHandle_t display_task_handle;

// The ball object the game will be played with - the position is the top left corner
ball gameBall = {.dim = BALL_SIZE, .posX = BALL_START_X, .posY = 0, .prev_posX = BALL_START_X, .prev_posY = 0, .speedX = 0, .speedY = 1, .numBounces = 0};

// The paddle object the game will be played with - the position is the top left corner
paddle gamePaddle = {.dimX = PADDLE_WIDTH, .dimY = PADDLE_HEIGHT, .posX = PADDLE_START_X, .posY = SCREEN_MAX_Y - PADDLE_HEIGHT, .prev_posX = PADDLE_START_X, .prev_posY = SCREEN_MAX_Y - PADDLE_HEIGHT};

// bool to record where the ball is
bool hasBall = false;

// Connection id declared in main.c
extern uint16_t connection_id;

// Declared in main.c
extern uint16_t ballHandle;

/*******************************************************************************
* Function Name: resetGamePositions
********************************************************************************
* Summary:
*  Resets the game objects to their default positions
*
* Return:
*  void
*
*******************************************************************************/
void resetGamePositions() {
	gameBall.posX = BALL_START_X;
	gameBall.posY = 0;
	gameBall.speedY = 1;
	gameBall.speedX = 0;
	gameBall.numBounces = 0;

	/* Erase the  ball location. It will be redrawn in the next pass through the loop at the start position */
	taskENTER_CRITICAL();
	GUI_ClearRect(gameBall.prev_posX, gameBall.prev_posY, gameBall.prev_posX + gameBall.dim, gameBall.prev_posY + gameBall.dim);
	taskEXIT_CRITICAL();
}

/*******************************************************************************
* Function Name: displayTask
********************************************************************************
* Summary:
* Calculates new position for the ball
* Updates the display with current ball and paddle locations
* Wires to GATT database when ball is sent to peripheral
*
* Return:
*  void
*
*******************************************************************************/
void displayTask(void *arg) {
	(void)arg;

	// Generate true random numbers for ball's vertical speed upon bouncing off the paddle
	cyhal_trng_t trng_obj;
	uint32_t rdmNum;

	// Initialize the true random number generator block
	cyhal_trng_init(&trng_obj);
	// Initialize rand() with a true random number
	rdmNum = cyhal_trng_generate(&trng_obj);
	// Free true random number generator
	cyhal_trng_free(&trng_obj);
	// Seed rand with true random number
	srand(rdmNum);

	// Clear the display, reset paddle position, and draw the paddle
	GUI_Clear();
	gamePaddle.posX = PADDLE_START_X;
	GUI_FillRect(gamePaddle.posX, gamePaddle.posY, gamePaddle.posX + gamePaddle.dimX, gamePaddle.posY + gamePaddle.dimY);

	for(;;)
	{
		/* Erase old paddle and draw new one if it has moved */
		if(gamePaddle.prev_posX != gamePaddle.posX) {
			// Don't allow interrupts while redrawing the paddle
			taskENTER_CRITICAL();
			GUI_ClearRect(gamePaddle.prev_posX, gamePaddle.prev_posY, gamePaddle.prev_posX + gamePaddle.dimX, gamePaddle.prev_posY + gamePaddle.dimY);
			GUI_FillRect(gamePaddle.posX, gamePaddle.posY, gamePaddle.posX + gamePaddle.dimX, gamePaddle.posY + gamePaddle.dimY);
			gamePaddle.prev_posX = gamePaddle.posX;
			taskEXIT_CRITICAL();
		}

		/* Handle ball motion and send it to the central when it reaches the edge */
		if(hasBall)
		{
			GUI_FillRect(gameBall.posX, gameBall.posY, gameBall.posX + gameBall.dim, gameBall.posY + gameBall.dim);
			// Update the ball's previous position
			gameBall.prev_posX = gameBall.posX;
			gameBall.prev_posY = gameBall.posY;

			// Limit refresh rate
			vTaskDelay(10);

			// Write to GATT database when ball goes off far wall
			// Invert the speeds and mirror the X coordinate since the other screen is rotated 180 degrees relative to this one
			if(gameBall.posY <= 0 && gameBall.speedY < 0) {
				//Populate write buffer
				uint16_t writeData[5] = {(SCREEN_MAX_X - gameBall.posX),
										 (0), /* Always send  Y position of 0 even if negative */
										 -(gameBall.speedX),
										 -(gameBall.speedY),
										 gameBall.numBounces};
				writeAttribute(connection_id, ballHandle, 0, GATT_AUTH_REQ_NONE, sizeof(writeData), (uint8_t *)writeData);
				printf("Sending Game Ball Data to Peripheral via Write to GATT Database:\nposX: %d\nposY: %d\nspeedX: %d\nspeedY: %d\nNumber of Bounces: %d\n",
						(SCREEN_MAX_X - gameBall.posX), gameBall.posY, -(gameBall.speedX), -(gameBall.speedY), gameBall.numBounces);
				hasBall = false;
			}

			// If the ball goes past the paddle, wait 1 second, then restart the game
			if(gameBall.posY + BALL_SIZE >= SCREEN_MAX_Y )
			{
				resetGamePositions();
				vTaskDelay(1000);
			}

			// Calculate the ball's new position
			gameBall.posX += gameBall.speedX;
			gameBall.posY += gameBall.speedY;

			// Bounce off sides of screen
			if(gameBall.posX <= 0 || gameBall.posX >= SCREEN_MAX_X - gameBall.dim) {
				gameBall.speedX *= -1;
			}

			/* Erase the old ball location. It will be redrawn in the next pass through the loop */
			taskENTER_CRITICAL();
			GUI_ClearRect(gameBall.prev_posX, gameBall.prev_posY, gameBall.prev_posX + gameBall.dim, gameBall.prev_posY + gameBall.dim);
			taskEXIT_CRITICAL();

			// If the ball is intersecting with an area around the paddle that is one pixel larger than the paddle, and it is moving in the positive Y direction, bounce it
			if((gameBall.posY + gameBall.dim + 1 >= gamePaddle.posY) && (gameBall.posX + gameBall.dim + 1 >= gamePaddle.posX) &&
			   (gameBall.posX - 1 <= gamePaddle.posX + PADDLE_WIDTH) && (gameBall.speedY > 0))
			{
				/* Redraw the paddle since the ball erased above may erase part of the paddle */
				taskENTER_CRITICAL();
				GUI_FillRect(gamePaddle.posX, gamePaddle.posY, gamePaddle.posX + gamePaddle.dimX, gamePaddle.posY + gamePaddle.dimY);
				taskEXIT_CRITICAL();

				// When ball is near the left edge of the paddle when it bounces, horizontal speed is a random number between -2 and -1
				if(gameBall.posX <= gamePaddle.posX)
				{
					gameBall.speedX = rand() % 1 - 2;
				}
				// When ball is near the right edge of the paddle when it bounces, horizontal speed is a random number between 1 and 2
				else  if( gameBall.posX + gameBall.dim >= gamePaddle.posX + gamePaddle.dimX )
				{
					gameBall.speedX = rand() % 1 + 1;
				}
				// When completely above the paddle when it bounces, horizontal speed is a random number between -2 and 2
				else
				{
					gameBall.speedX = rand() % 4 - 2;
				}
				(gameBall.numBounces)++;
				gameBall.speedY = -((gameBall.numBounces)>>1);
				if(gameBall.speedY == 0 )
				{
					gameBall.speedY = -1;
				}
			} // end of else ball is in play
		} /* End of if this side has the ball */
	} // End of infinite loop
}
