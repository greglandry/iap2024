/******************************************************************************
* File Name: displayTask.c
*
* Description: This task updates the display
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
bool hasBall = true;

// Connection id declared in main.c
extern uint16_t connection_id;

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
* Sends notification when ball is sent to central
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

			// Send BT notification when ball goes off far wall
			// Invert the speeds and mirror the X coordinate since the other screen is rotated 180 degrees relative to this one
			if(gameBall.posY <= 0 && gameBall.speedY < 0) {
				if(connection_id) /* Check if we have an active connection */
				{
					/* Check to see if the client has asked for notifications */
					if(app_pong_ball_client_char_config[0] & GATT_CLIENT_CONFIG_NOTIFICATION) {
						// Update characteristic values with the ball values
						app_pong_ball[0] = (SCREEN_MAX_X - gameBall.posX);
						app_pong_ball[1] = (SCREEN_MAX_X - gameBall.posX) >> 8;
						app_pong_ball[2] = 0; /* Always send Y position of 0 even if negative */
						app_pong_ball[3] = 0;
						app_pong_ball[4] = -(gameBall.speedX);
						app_pong_ball[5] = -(gameBall.speedX) >> 8;
						app_pong_ball[6] = -(gameBall.speedY);
						app_pong_ball[7] = -(gameBall.speedY) >> 8;
						app_pong_ball[8] = (gameBall.numBounces);
						app_pong_ball[9] = (gameBall.numBounces) >>8;

						// Send notification
						wiced_bt_gatt_server_send_notification(connection_id, HDLC_PONG_BALL_VALUE, app_pong_ball_len, app_pong_ball, NULL);
						printf("Sending Game Ball Data to Central via Notification:\nposX: %d\nposY: %d\nspeedX: %d\nspeedY: %d\nNumber of Bounces: %d\n",
								(SCREEN_MAX_X - gameBall.posX), gameBall.posY, -(gameBall.speedX), -(gameBall.speedY), gameBall.numBounces);
						hasBall = false;
					} else {
						printf("Notifications not enabled! Stopping execution!\n");
						printf("app_pong_ball_client_char_config: 0x%x, 0x%x\n", app_pong_ball_client_char_config[0],
							   app_pong_ball_client_char_config[1]);
						CY_ASSERT(0);
					}
				} else {
					printf("Connection error! Stopping execution!\n");
					CY_ASSERT(0);
				}
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
				if(gameBall.posX <= (gamePaddle.posX + gameBall.dim))
				{
					gameBall.speedX = rand() % 2 - 2;
				}
				// When ball is near the right edge of the paddle when it bounces, horizontal speed is a random number between 1 and 2
				else  if( gameBall.posX >= gamePaddle.posX + gamePaddle.dimX - gameBall.dim )
				{
					gameBall.speedX = rand() % 2 + 1;
				}
				// When completely above the paddle when it bounces, horizontal speed is a random number between -1 and 1
				else
				{
					gameBall.speedX = rand() % 3 - 1;
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
