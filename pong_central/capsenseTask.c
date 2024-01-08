/*******************************************************************************
* File Name: capsense_task.c
*
* Description: This file contains the task that handles touch sensing.
*
* Related Document: README.md
*
*******************************************************************************
* Copyright 2019-2023, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/


/******************************************************************************
* Header files includes
******************************************************************************/
#include "capsenseTask.h"

#include "cybsp.h"
#include "cyhal.h"
#include "cycfg.h"
#include "cycfg_capsense.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "displayTask.h"

/*******************************************************************************
* Global constants
*******************************************************************************/
#define CAPSENSE_INTERRUPT_PRIORITY    (7u)
#define CAPSENSE_SCAN_INTERVAL_MS      (10u)   /* in milliseconds*/

/*******************************************************************************
* Function Prototypes
*******************************************************************************/
static uint32_t capsense_init(void);
static void process_touch(void);
static void capsense_isr(void);

/******************************************************************************
* Global variables
******************************************************************************/
QueueHandle_t capsense_command_q;
TimerHandle_t scan_timer_handle;

/* SysPm callback params */
cy_stc_syspm_callback_params_t callback_params =
{
    .base       = CYBSP_CSD_HW,
    .context    = &cy_capsense_context
};

cy_stc_syspm_callback_t capsense_deep_sleep_cb =
{
    Cy_CapSense_DeepSleepCallback,
    CY_SYSPM_DEEPSLEEP,
    (CY_SYSPM_SKIP_CHECK_FAIL | CY_SYSPM_SKIP_BEFORE_TRANSITION | CY_SYSPM_SKIP_AFTER_TRANSITION),
    &callback_params,
    NULL,
    NULL
};


/*******************************************************************************
* Function Name: task_capsense
********************************************************************************
* Summary:
*  Task that initializes the CapSense block and processes the touch input.
*
* Parameters:
*  void *param : Task parameter defined during task creation (unused)
*
*******************************************************************************/
void capsenseTask(void* param)
{
    cy_status status;

    /* Remove warning for unused parameter */
    (void)param;

    /* Initialize CapSense block */
    status = capsense_init();
    if(CY_RET_SUCCESS != status)
    {
        CY_ASSERT(0u);
    }

    /* Start First Scan */
	Cy_CapSense_ScanAllWidgets(&cy_capsense_context);

    /* Repeatedly running part of the task */
    for(;;)
    {
    	/* Scan every 10ms */
    	vTaskDelay(10);
		/* Check if CapSense is busy with a previous scan */
		if(CY_CAPSENSE_NOT_BUSY == Cy_CapSense_IsBusy(&cy_capsense_context))
		{
			Cy_CapSense_ProcessAllWidgets(&cy_capsense_context);
			process_touch();
			Cy_CapSense_ScanAllWidgets(&cy_capsense_context);
		}
    }
}


/*******************************************************************************
* Function Name: process_touch
********************************************************************************
* Summary:
*  This function processes the touch input and sends command to LED task.
*
*******************************************************************************/
static void process_touch(void)
{
    /* Variables used to store touch information */
    uint16_t slider_pos = 0;
    uint8_t slider_touched = 0;
    cy_stc_capsense_touch_t *slider_touch;

    /* Variables used to store previous touch information */
    static uint16_t slider_pos_perv = 0;

    /* Get slider status */
    slider_touch = Cy_CapSense_GetTouchInfo(
        CY_CAPSENSE_LINEARSLIDER0_WDGT_ID,
        &cy_capsense_context);
    slider_pos = slider_touch->ptrPosition->x;
    slider_touched = slider_touch->numPosition;

    /* The slider resolution is 1/4 the size of the screen to reduce jitter in the paddle */
    uint16_t gamePaddlePos = (slider_pos * 4);

    /* Detect new touch on slider */
    if((0u != slider_touched) && (slider_pos_perv != slider_pos ))
    {
    	/* Move paddle to new location */
    	if(gamePaddlePos < SCREEN_MAX_X - gamePaddle.dimX) {
    		// Update paddle position
    		gamePaddle.posX = gamePaddlePos;
    	}
    	else
    	{
    		/* Paddle is at max location */
    		gamePaddle.posX = (SCREEN_MAX_X - gamePaddle.dimX);
    	}
    }

    slider_pos_perv = slider_pos;
}


/*******************************************************************************
* Function Name: capsense_init
********************************************************************************
* Summary:
*  This function initializes the CSD HW block, and configures the CapSense
*  interrupt.
*
*******************************************************************************/
static uint32_t capsense_init(void)
{
    uint32_t status = CYRET_SUCCESS;

    /* CapSense interrupt configuration parameters */
    static const cy_stc_sysint_t capSense_intr_config =
    {
        .intrSrc = csd_interrupt_IRQn,
        .intrPriority = CAPSENSE_INTERRUPT_PRIORITY,
    };

    /*Initialize CapSense Data structures */
    status = Cy_CapSense_Init(&cy_capsense_context);
    if (CYRET_SUCCESS != status)
    {
        return status;
    }

    /* Initialize CapSense interrupt */
    cyhal_system_set_isr(csd_interrupt_IRQn, csd_interrupt_IRQn, CAPSENSE_INTERRUPT_PRIORITY, &capsense_isr);
    NVIC_ClearPendingIRQ(capSense_intr_config.intrSrc);
    NVIC_EnableIRQ(capSense_intr_config.intrSrc);

    /* Initialize the CapSense deep sleep callback functions. */
    Cy_CapSense_Enable(&cy_capsense_context);
    Cy_SysPm_RegisterCallback(&capsense_deep_sleep_cb);

    if (CYRET_SUCCESS != status)
    {
        return status;
    }
    /* Initialize the CapSense firmware modules. */
    status = Cy_CapSense_Enable(&cy_capsense_context);
    if (CYRET_SUCCESS != status)
    {
        return status;
    }
    
    return status;
}

/*******************************************************************************
* Function Name: capsense_isr
********************************************************************************
* Summary:
*  Wrapper function for handling interrupts from CSD block.
*
*******************************************************************************/
static void capsense_isr(void)
{
    Cy_CapSense_InterruptHandler(CYBSP_CSD_HW, &cy_capsense_context);
}



/* END OF FILE [] */
