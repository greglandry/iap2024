/******************************************************************************
* File Name:   main.c
*
* Description: This file contains the main for the central BLE pong game
*
* Related Document: See README.md
*
*
*******************************************************************************
* (c) 2019-2021, Cypress Semiconductor Corporation. All rights reserved.
*******************************************************************************
* This software, including source code, documentation and related materials
* ("Software"), is owned by Cypress Semiconductor Corporation or one of its
* subsidiaries ("Cypress") and is protected by and subject to worldwide patent
* protection (United States and foreign), United States copyright laws and
* international treaty provisions. Therefore, you may use this Software only
* as provided in the license agreement accompanying the software package from
* which you obtained this Software ("EULA").
*
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software source
* code solely for use in connection with Cypress's integrated circuit products.
* Any reproduction, modification, translation, compilation, or representation
* of this Software except as specified above is prohibited without the express
* written permission of Cypress.
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
* including Cypress's product in a High Risk Product, the manufacturer of such
* system or application assumes all risk of such use and in doing so agrees to
* indemnify Cypress against all liability.
*******************************************************************************/

// MCU headers
#include "cy_pdl.h"
#include "cy_retarget_io.h"
#include "cybsp.h"
#include "cyhal.h"

// RTOS headers
#include "FreeRTOS.h"
#include "task.h"

// malloc and free
#include "stdlib.h"

// BT Stack
#include "wiced_bt_stack.h"

// BT app utils
#include "app_bt_utils.h"

// BT configurator generated headers
#include "cycfg_bt_settings.h"
#include "cycfg_gap.h"

// Display header files
#include "GUI.h"
#include "cy8ckit_028_tft_pins.h"
#include "mtb_st7789v.h"

// Task header files
#include "displayTask.h"
#include "capsenseTask.h"

/*******************************************************************************
* Macros
********************************************************************************/
#define GPIO_INTERRUPT_PRIORITY (1u)

#define LED_ON_DUTY    (0.1)
#define LED_OFF_DUTY   (100)
#define LED_BLINK_DUTY (50)

// Typdef for function used to free allocated buffer to stack
typedef void (*pfn_free_buffer_t)(uint8_t *);

/*******************************************************************
 * Function Prototypes
 ******************************************************************/
/* Callback function for Bluetooth stack management type events */
static wiced_bt_dev_status_t app_bt_management_callback(wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data);

/* GATT Event Callback and Handler Functions */
static wiced_bt_gatt_status_t app_bt_gatt_event_callback(wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_event_data);

static wiced_bt_gatt_status_t app_bt_connect_event_handler(wiced_bt_gatt_connection_status_t *p_conn_status);

/* Declaration for scan callback function */
void scanCallback(wiced_bt_ble_scan_results_t *p_scan_result, uint8_t *p_adv_data);

/* Function to write to an attribute */
void writeAttribute(uint16_t conn_id, uint16_t handle, uint16 offset, wiced_bt_gatt_auth_req_t auth_req, uint16_t len, uint8_t *val);

/* Helper functions to allocate/free buffers for GATT operations */
static uint8_t *app_bt_alloc_buffer(uint16_t len);
static void app_bt_free_buffer(uint8_t *p_data);

/*******************************************************************************
* Global Vars
********************************************************************************/
/* Enable RTOS aware debugging in OpenOCD */
volatile int uxTopUsedPriority;

// Pins for the TFT display
const mtb_st7789v_pins_t tft_pins = {.db08 = CY8CKIT_028_TFT_PIN_DISPLAY_DB8,
									 .db09 = CY8CKIT_028_TFT_PIN_DISPLAY_DB9,
									 .db10 = CY8CKIT_028_TFT_PIN_DISPLAY_DB10,
									 .db11 = CY8CKIT_028_TFT_PIN_DISPLAY_DB11,
									 .db12 = CY8CKIT_028_TFT_PIN_DISPLAY_DB12,
									 .db13 = CY8CKIT_028_TFT_PIN_DISPLAY_DB13,
									 .db14 = CY8CKIT_028_TFT_PIN_DISPLAY_DB14,
									 .db15 = CY8CKIT_028_TFT_PIN_DISPLAY_DB15,
									 .nrd = CY8CKIT_028_TFT_PIN_DISPLAY_NRD,
									 .nwr = CY8CKIT_028_TFT_PIN_DISPLAY_NWR,
									 .dc = CY8CKIT_028_TFT_PIN_DISPLAY_DC,
									 .rst = CY8CKIT_028_TFT_PIN_DISPLAY_RST};

// PWM for connection status LED
cyhal_pwm_t status_pwm_obj;

// Hard coded handles for GATT database characteristics and descriprots
uint16_t ballHandle = 0x09u;
uint16_t cccdHandle = 0x0Au;
// BLE connection ID
uint16_t connection_id = 0;

// Defined in displayTasak.c
extern bool hasBall;

/******************************************************************************
 * Function Name: main
 ******************************************************************************
 * Summary:
 *  System entrance point. This function initializes HW, creates tasks,
 *  then starts the scheduler.
 *
 * Parameters:
 *  void
 *
 * Return:
 *  int
 *
 ******************************************************************************/
int main(void) {
	/*******************************************************************************
    * HW Initialization
    ********************************************************************************/
	cy_rslt_t result;

	/* Initialize the device and board peripherals */
	result = cybsp_init();
	if(result != CY_RSLT_SUCCESS) {
		CY_ASSERT(0);
	}

	__enable_irq();

	/* Initialize PWM for connection status LED */
	cyhal_pwm_init(&status_pwm_obj, CYBSP_USER_LED2, NULL);
	cyhal_pwm_set_duty_cycle(&status_pwm_obj, LED_OFF_DUTY, 2);
	cyhal_pwm_start(&status_pwm_obj);

	/* Initialize retarget-io to use the debug UART port. */
	cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX, CY_RETARGET_IO_BAUDRATE);

	/* \x1b[2J\x1b[;H - ANSI ESC sequence to clear UART screen. */
	printf("\x1b[2J\x1b[;H");
	printf("BLE Pong Central\n");

	/*******************************************************************************
	* RTOS Initialization
	********************************************************************************/

	/* Configure platform specific settings for the BT device */
	cybt_platform_config_init(&cybsp_bt_platform_cfg);

	/* Initialize stack and register the callback function */
	wiced_bt_stack_init(app_bt_management_callback, &wiced_bt_cfg_settings);

	// Start the scheduler
	vTaskStartScheduler();

	/* Should never get here. */
	CY_ASSERT(0);
}

/*******************************************************************************
* Function Name: wiced_bt_dev_status_t app_bt_management_callback(
* 					wiced_bt_management_evt_t event,
* 					wiced_bt_management_evt_data_t *p_event_data )
********************************************************************************/
static wiced_result_t app_bt_management_callback(wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data) {
	/* Start in error state so that any unimplemented states will return error */
	wiced_result_t result = WICED_BT_ERROR;
	wiced_bt_device_address_t bda = {0};

	printf("Bluetooth Management Event: 0x%x %s\n", event, get_bt_event_name(event));

	switch(event) {
		case BTM_ENABLED_EVT:    // Bluetooth Controller and Host Stack Enabled
			if(WICED_BT_SUCCESS == p_event_data->enabled.status) {
				printf("Bluetooth Enabled\n");

				// Initialize the display
				mtb_st7789v_init8(&tft_pins);

				// Initialize the emwin library
				GUI_Init();

				// Clear the display
				GUI_Clear();

				/* Set the local BDA from the value in the configurator and print it */
				wiced_bt_set_local_bdaddr((uint8_t *)cy_bt_device_address, BLE_ADDR_PUBLIC);
				wiced_bt_dev_read_local_addr(bda);
				printf("Local Bluetooth Device Address: ");
				print_bd_address(bda);

				/* Register GATT callback */
				wiced_bt_gatt_register(app_bt_gatt_event_callback);

				// Start scanning
				wiced_bt_ble_scan(BTM_BLE_SCAN_TYPE_HIGH_DUTY, WICED_TRUE, scanCallback);

				result = WICED_BT_SUCCESS;
			} else {
				printf("Failed to initialize Bluetooth controller and stack\n");
			}
			break;

		case BTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT:
			p_event_data->pairing_io_capabilities_ble_request.auth_req = BTM_LE_AUTH_REQ_NO_BOND;
			p_event_data->pairing_io_capabilities_ble_request.init_keys = BTM_LE_KEY_PENC | BTM_LE_KEY_PID;
			p_event_data->pairing_io_capabilities_ble_request.local_io_cap = BTM_IO_CAPABILITIES_NONE;
			p_event_data->pairing_io_capabilities_ble_request.max_key_size = 0x10;
			p_event_data->pairing_io_capabilities_ble_request.resp_keys = BTM_LE_KEY_PENC | BTM_LE_KEY_PID;
			p_event_data->pairing_io_capabilities_ble_request.oob_data = BTM_OOB_NONE;
			result = WICED_BT_SUCCESS;
			break;

		case BTM_PAIRING_COMPLETE_EVT:
			printf("Pairing complete: %d\n", p_event_data->pairing_complete.pairing_complete_info.ble.status);
			result = WICED_BT_SUCCESS;
			break;

		case BTM_ENCRYPTION_STATUS_EVT:
			printf("Encrypt status: %d\n", p_event_data->encryption_status.result);
			result = WICED_BT_SUCCESS;
			break;

		case BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT: result = WICED_BT_SUCCESS; break;

		case BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT:
			result = WICED_BT_ERROR;    // Return error since keys are not stored in EEPROM
			break;

		case BTM_LOCAL_IDENTITY_KEYS_REQUEST_EVT:    // Read keys from EEPROM
													 /* This should return WICED_BT_SUCCESS if not using privacy. If RPA is enabled but keys are not
               stored in EEPROM, this must return WICED_BT_ERROR so that the stack will generate new privacy keys */
			result = WICED_BT_SUCCESS;
			break;

		case BTM_BLE_SCAN_STATE_CHANGED_EVT:    // Scan State Change
			switch(p_event_data->ble_scan_state_changed) {
				case BTM_BLE_SCAN_TYPE_NONE:
					printf("Scanning stopped\r\n");
					cyhal_pwm_set_duty_cycle(&status_pwm_obj, LED_OFF_DUTY, 2);    // LED off when not advertising
					GUI_Clear();                                                   // Clear the display
					GUI_SetFont(&GUI_Font32B_ASCII);                               // Font size
					GUI_DispString("Scanning stopped");                            // Display advertising message
					break;

				case BTM_BLE_SCAN_TYPE_HIGH_DUTY:
					printf("High duty scanning\r\n");
					cyhal_pwm_set_duty_cycle(&status_pwm_obj, LED_BLINK_DUTY, 2);    // LED blinks when advertising
					GUI_Clear();                                                     // Clear the display
					GUI_SetFont(&GUI_Font32B_ASCII);                                 // Font size
					GUI_DispString("High duty scanning");                            // Display advertising message
					break;

				case BTM_BLE_SCAN_TYPE_LOW_DUTY:
					printf("Low duty scanning\r\n");
					cyhal_pwm_set_duty_cycle(&status_pwm_obj, LED_BLINK_DUTY, 2);    // LED blinks when advertising
					GUI_Clear();                                                     // Clear the display
					GUI_SetFont(&GUI_Font32B_ASCII);                                 // Font size
					GUI_DispString("Low duty scanning");                             // Display advertising message
					break;
			}
			result = WICED_BT_SUCCESS;
			break;

		case BTM_BLE_CONNECTION_PARAM_UPDATE: result = WICED_BT_SUCCESS; break;

		default: break;
	}

	return result;
}

/*******************************************************************************
* Function Name: wiced_bt_gatt_status_t app_bt_gatt_event_callback(
* 					wiced_bt_gatt_evt_t event,
* 					wiced_bt_gatt_event_data_t *p_data )
********************************************************************************/
static wiced_bt_gatt_status_t app_bt_gatt_event_callback(wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_event_data) {
	/* Start in error state so that any unimplemented states will return error */
	wiced_bt_gatt_status_t status = WICED_BT_GATT_ERROR;

	/* Call the appropriate callback function based on the GATT event type, and pass the relevant event
     * parameters to the callback function */
	switch(event) {
		case GATT_CONNECTION_STATUS_EVT: status = app_bt_connect_event_handler(&p_event_data->connection_status); break;

		case GATT_OPERATION_CPLT_EVT:
			/* Look for any type of successful GATT completion */
			if(p_event_data->operation_complete.status == WICED_BT_GATT_SUCCESS ||
			   p_event_data->operation_complete.status == WICED_BT_GATT_ENCRYPTED_MITM ||
			   p_event_data->operation_complete.status == WICED_BT_GATT_ENCRYPTED_NO_MITM ||
			   p_event_data->operation_complete.status == WICED_BT_GATT_NOT_ENCRYPTED) {
				printf("GATT operation completed successfully\n");
				if(p_event_data->operation_complete.op == GATTC_OPTYPE_READ_HANDLE) {
				} else if(p_event_data->operation_complete.op == GATTC_OPTYPE_NOTIFICATION) {
					if(p_event_data->operation_complete.response_data.handle == ballHandle)    // Notification received
					{
						gameBall.posX = p_event_data->operation_complete.response_data.att_value.p_data[1];
						gameBall.posX = gameBall.posX << 8;
						gameBall.posX |= p_event_data->operation_complete.response_data.att_value.p_data[0];
						gameBall.posY = p_event_data->operation_complete.response_data.att_value.p_data[3];
						gameBall.posY = gameBall.posY << 8;
						gameBall.posY |= p_event_data->operation_complete.response_data.att_value.p_data[2];
						gameBall.speedX = p_event_data->operation_complete.response_data.att_value.p_data[5];
						gameBall.speedX = gameBall.speedX << 8;
						gameBall.speedX |= p_event_data->operation_complete.response_data.att_value.p_data[4];
						gameBall.speedY = p_event_data->operation_complete.response_data.att_value.p_data[7];
						gameBall.speedY = gameBall.speedY << 8;
						gameBall.speedY |= p_event_data->operation_complete.response_data.att_value.p_data[6];
						gameBall.numBounces = p_event_data->operation_complete.response_data.att_value.p_data[9];
						gameBall.numBounces = gameBall.numBounces << 8;
						gameBall.numBounces |= p_event_data->operation_complete.response_data.att_value.p_data[8];
						printf("Game Ball Data Received via Notification:\nposX: %d\nposY: %d\nspeedX: %d\nspeedY: %d\nNumber of Bounces: %d\n", gameBall.posX,
							   gameBall.posY, gameBall.speedX, gameBall.speedY, gameBall.numBounces);
						hasBall = true;
					}
				}
			} else {
				printf("GATT operation failed with status: %d\n", p_event_data->operation_complete.status);
			}
			break;

		case GATT_GET_RESPONSE_BUFFER_EVT: /* GATT buffer request, typically sized to max of bearer mtu - 1 */
			p_event_data->buffer_request.buffer.p_app_rsp_buffer = app_bt_alloc_buffer(p_event_data->buffer_request.len_requested);
			p_event_data->buffer_request.buffer.p_app_ctxt = (void *)app_bt_free_buffer;
			status = WICED_BT_GATT_SUCCESS;
			break;

		case GATT_APP_BUFFER_TRANSMITTED_EVT: /* GATT buffer transmitted event,  check \ref wiced_bt_gatt_buffer_transmitted_t*/
		{
			pfn_free_buffer_t pfn_free = (pfn_free_buffer_t)p_event_data->buffer_xmitted.p_app_ctxt;
			/* If the buffer is dynamic, the context will point to a function to free it. */
			if(pfn_free) {
				pfn_free(p_event_data->buffer_xmitted.p_app_data);
			}
			status = WICED_BT_GATT_SUCCESS;
		} break;

		default:
			printf("Unhandled GATT Event: 0x%x (%d)\n", event, event);
			status = WICED_BT_GATT_SUCCESS;
			break;
	}
	return status;
}

/*******************************************************************************
 * Function Name: app_bt_connect_event_handler
 *
 * Handles GATT connection status changes.
 *
 * Param:	p_conn_status  Pointer to data that has connection details
 * Return:	wiced_bt_gatt_status_t
 * See possible status codes in wiced_bt_gatt_status_e in wiced_bt_gatt.h
*******************************************************************************/
static wiced_bt_gatt_status_t app_bt_connect_event_handler(wiced_bt_gatt_connection_status_t *p_conn_status) {
	wiced_bt_gatt_status_t status = WICED_BT_GATT_ERROR;

	if(NULL != p_conn_status) {
		if(p_conn_status->connected) {
			/* Handle the connection */
			printf("GATT_CONNECTION_STATUS_EVT: Connect BDA ");
			print_bd_address(p_conn_status->bd_addr);
			printf("Connection ID %d\n", p_conn_status->conn_id);

			connection_id = p_conn_status->conn_id;

			cyhal_pwm_set_duty_cycle(&status_pwm_obj, LED_ON_DUTY, 2);    // LED Solid when connected

			// Enable notifications
			uint8_t writeData[2] = {0};
			writeData[0] = GATT_CLIENT_CONFIG_NOTIFICATION; /* Values are sent little endian */
			writeAttribute(connection_id, cccdHandle, 0, GATT_AUTH_REQ_NONE, sizeof(uint16_t), writeData);

			// Start the game
			/* Create the display and CAPSENSE tasks. */
			xTaskCreate(capsenseTask, "CapSense Task", CAPSENSE_TASK_STACK_SIZE, NULL, CAPSENSE_TASK_PRIORITY, NULL);
			xTaskCreate(displayTask, "Display task", DISPLAY_TASK_STACK_SIZE, NULL, DISPLAY_TASK_PRIORITY, &display_task_handle);
		} else {
			/* Handle the disconnection */
			printf("Disconnected : BDA ");
			print_bd_address(p_conn_status->bd_addr);
			printf("Connection ID '%d', Reason '%s'\n", p_conn_status->conn_id, get_bt_gatt_disconn_reason_name(p_conn_status->reason));
			connection_id = 0;

			cyhal_pwm_set_duty_cycle(&status_pwm_obj, LED_OFF_DUTY, 2);    // LED off when connected
			GUI_Clear();                                                   // Clear the display
			GUI_SetFont(&GUI_Font32B_ASCII);                               // Font size
			GUI_DispString("Disconnected");                                // Display disconnected message
		}

		status = WICED_BT_GATT_SUCCESS;
	}

	return status;
}

/*******************************************************************************
* Function Name: void scanCallback(
* 					wiced_bt_ble_scan_results_t *p_scan_result,
* 					uint8_t *p_adv_data )
********************************************************************************/
void scanCallback(wiced_bt_ble_scan_results_t *p_scan_result, uint8_t *p_adv_data) {
#define MAX_ADV_NAME_LEN   (28)              /* Maximum possible name length since flags take 3 bytes and max packet is 31. */
#define SEARCH_DEVICE_NAME "pong_peripheral" /* Name of device to search for */

	uint8_t len;
	uint8_t *p_name = NULL;

	uint8_t dev_name[MAX_ADV_NAME_LEN];

	p_name = wiced_bt_ble_check_advertising_data(p_adv_data, BTM_BLE_ADVERT_TYPE_NAME_COMPLETE, &len);

	if(p_name && (len == strlen(SEARCH_DEVICE_NAME)) && (memcmp(SEARCH_DEVICE_NAME, p_name, len) == 0)) {
		memcpy(dev_name, p_name, len);
		dev_name[len] = 0x00; /* Null terminate the string */

		printf("Found Device \"%s\" with BD Address: ", dev_name);
		print_bd_address(p_scan_result->remote_bd_addr);

		/* Connect and turn off scanning */
		wiced_bt_gatt_le_connect(p_scan_result->remote_bd_addr, p_scan_result->ble_addr_type, BLE_CONN_MODE_HIGH_DUTY, WICED_TRUE);
		wiced_bt_ble_scan(BTM_BLE_SCAN_TYPE_NONE, WICED_TRUE, scanCallback);
	}
}

/*******************************************************************************
* Function Name: void writeAttribute
********************************************************************************/
void writeAttribute(uint16_t conn_id, uint16_t handle, uint16 offset, wiced_bt_gatt_auth_req_t auth_req, uint16_t len, uint8_t *val) {
	if(conn_id && handle) /* Only write if we have a connection and the handle is defined */
	{
		/* Set up write parameters */
		wiced_bt_gatt_write_hdr_t write_params;
		write_params.handle = handle;
		write_params.offset = offset;
		write_params.len = len;
		write_params.auth_req = auth_req;

		/* Send the write command */
		wiced_bt_gatt_client_send_write(connection_id, GATT_REQ_WRITE, &write_params, val, NULL);
	}
}

/*******************************************************************************
* Function Name: app_bt_alloc_buffer
*
* This Function allocates the buffer of requested length
*
* Param:  len			Length of buffer
* Return: uint8_t*      Pointer to allocated buffer
********************************************************************************/
static uint8_t *app_bt_alloc_buffer(uint16_t len) {
	uint8_t *p = (uint8_t *)malloc(len);
	return p;
}

/*******************************************************************************
* Function Name: app_bt_free_buffer
*
* This Function frees the buffer requested
*
* Param:  p_data		Pointer to buffer to be freed
********************************************************************************/
static void app_bt_free_buffer(uint8_t *p_data) {
	if(p_data != NULL) {
		free(p_data);
	}
}

/* [] END OF FILE */
