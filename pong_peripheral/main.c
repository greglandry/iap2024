/******************************************************************************
* File Name:   main.c
*
* Description: This file contains the main for the peripheral BLE pong game
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
#include "cycfg_gatt_db.h"

// Display headers
#include "GUI.h"
#if defined(COMPONENT_SHIELD_TFT)
	#include "cy8ckit_028_tft_pins.h"
	#include "mtb_st7789v.h"
#elif defined(COMPONENT_SHIELD_SENSE)
	#include "cy8ckit_028_sense_pins.h"
	#include "mtb_ssd1306.h"
#endif

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
static wiced_bt_gatt_status_t app_bt_server_event_handler(wiced_bt_gatt_event_data_t *p_data);

static wiced_bt_gatt_status_t app_bt_write_handler(wiced_bt_gatt_event_data_t *p_data);
static wiced_bt_gatt_status_t app_bt_gatt_req_read_handler(uint16_t conn_id, wiced_bt_gatt_opcode_t opcode, wiced_bt_gatt_read_t *p_read_req,
														   uint16_t len_requested);
static wiced_bt_gatt_status_t app_bt_gatt_req_read_multi_handler(uint16_t conn_id, wiced_bt_gatt_opcode_t opcode,
																 wiced_bt_gatt_read_multiple_req_t *p_read_req, uint16_t len_requested);
static wiced_bt_gatt_status_t app_bt_gatt_req_read_by_type_handler(uint16_t conn_id, wiced_bt_gatt_opcode_t opcode,
																   wiced_bt_gatt_read_by_type_t *p_read_req, uint16_t len_requested);
/* Helper functions to find GATT database handles and allocate/free buffers for GATT operations */
static gatt_db_lookup_table_t *app_bt_find_by_handle(uint16_t handle);
static uint8_t *app_bt_alloc_buffer(uint16_t len);
static void app_bt_free_buffer(uint8_t *p_data);

/*******************************************************************************
* Global Vars
********************************************************************************/
/* Enable RTOS aware debugging in OpenOCD */
volatile int uxTopUsedPriority;

#if defined(COMPONENT_SHIELD_TFT)
#define GUI_FONT (GUI_Font32B_ASCII)

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
#elif defined(COMPONENT_SHIELD_SENSE)
#define GUI_FONT (GUI_Font13B_ASCII)

cyhal_i2c_t i2c_obj;
#endif

// PWM for connection status LED
cyhal_pwm_t status_pwm_obj;

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
	printf("BLE Pong Peripheral\n");

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

#if defined(COMPONENT_SHIELD_TFT)
				// Initialize the display
				mtb_st7789v_init8(&tft_pins);
#elif defined(COMPONENT_SHIELD_SENSE)
				/* Initialize the I2C to use with the OLED display */
				cyhal_i2c_init(&i2c_obj, CYBSP_I2C_SDA, CYBSP_I2C_SCL, NULL);
				/* Initialize the OLED display */
				mtb_ssd1306_init_i2c(&i2c_obj);
#endif

				// Initialize the emwin library
				GUI_Init();

				// Clear the display
				GUI_Clear();

				/* Set the local BDA from the value in the configurator and print it */
				wiced_bt_set_local_bdaddr((uint8_t *)cy_bt_device_address, BLE_ADDR_PUBLIC);
				wiced_bt_dev_read_local_addr(bda);
				printf("Local Bluetooth Device Address: ");
				print_bd_address(bda);

				/* Register GATT callback and initialize database*/
				wiced_bt_gatt_register(app_bt_gatt_event_callback);
				wiced_bt_gatt_db_init(gatt_database, gatt_database_len, NULL);

				/* Disable pairing */
				wiced_bt_set_pairable_mode(WICED_FALSE, WICED_FALSE);

				/* Set advertisement and scan response packets and begin advertising */
				wiced_bt_ble_set_raw_advertisement_data(CY_BT_ADV_PACKET_DATA_SIZE, cy_bt_adv_packet_data);

				wiced_bt_ble_set_raw_scan_response_data(CY_BT_SCAN_RESP_PACKET_DATA_SIZE, cy_bt_scan_resp_packet_data);

				wiced_bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_HIGH, 0, NULL);

				result = WICED_BT_SUCCESS;
			} else {
				printf("Failed to initialize Bluetooth controller and stack\n");
			}
			break;

		case BTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT:    // IO capabilities request
			result = WICED_BT_SUCCESS;
			break;

		case BTM_PAIRING_COMPLETE_EVT:    // Pairing Complete event
			result = WICED_BT_SUCCESS;
			break;

		case BTM_ENCRYPTION_STATUS_EVT:    // Encryption Status Event
			result = WICED_BT_SUCCESS;
			break;

		case BTM_SECURITY_REQUEST_EVT:    // Security access
			result = WICED_BT_SUCCESS;
			break;

		case BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT:    // Save link keys with app
			result = WICED_BT_SUCCESS;
			break;

		case BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT:    // Retrieve saved link keys
														 /* This must return WICED_BT_ERROR if bonding information is not stored in EEPROM */
			result = WICED_BT_ERROR;
			break;

		case BTM_LOCAL_IDENTITY_KEYS_UPDATE_EVT:    // Save keys to NVRAM
			result = WICED_BT_SUCCESS;
			break;

		case BTM_LOCAL_IDENTITY_KEYS_REQUEST_EVT:    // Read keys from NVRAM
													 /* This should return WICED_BT_SUCCESS if not using privacy. If RPA is enabled but keys are not
               stored in EEPROM, this must return WICED_BT_ERROR so that the stack will generate new privacy keys */
			result = WICED_BT_SUCCESS;
			break;
		case BTM_BLE_SCAN_STATE_CHANGED_EVT:    // Scan State Change
			result = WICED_BT_SUCCESS;
			break;

		case BTM_BLE_ADVERT_STATE_CHANGED_EVT:    // Advertising State Change
			printf("Advertisement State Change: %s\n", get_bt_advert_mode_name(p_event_data->ble_advert_state_changed));
			if(p_event_data->ble_advert_state_changed == BTM_BLE_ADVERT_OFF) /* Advertising stopped */
			{
				if(0 == connection_id) /* not connected  - LED off */
				{
					cyhal_pwm_set_duty_cycle(&status_pwm_obj, LED_OFF_DUTY, 2);
					GUI_Clear();                        // Clear the display
					GUI_SetFont(&GUI_FONT);             // Font size
					GUI_DispString("Disconnected");     // Display advertising message

				} else /* connected - LED on */
				{
					cyhal_pwm_set_duty_cycle(&status_pwm_obj, LED_ON_DUTY, 2);
					GUI_Clear();                                       // Clear the display
					GUI_SetFont(&GUI_FONT);                            // Font size
					GUI_DispString("Connected!\nStarting Game...");    // Display advertising message
				}
			} else /* Advertising is on - LED blink */
			{
				cyhal_pwm_set_duty_cycle(&status_pwm_obj, LED_BLINK_DUTY, 2);    // LED blinks when advertising
				GUI_Clear();                                                     // Clear the display
				GUI_SetFont(&GUI_FONT);                                          // Font size
				GUI_DispString("Advertising");                                   // Display advertising message
			}

			result = WICED_BT_SUCCESS;
			break;

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

		case GATT_ATTRIBUTE_REQUEST_EVT: status = app_bt_server_event_handler(p_event_data); break;

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
			printf("GATT_CONNECTION_STATUS_EVT: Connect BDA ");
			print_bd_address(p_conn_status->bd_addr);
			printf("Connection ID %d\n", p_conn_status->conn_id);

			/* Handle the connection */
			connection_id = p_conn_status->conn_id;

			cyhal_pwm_set_duty_cycle(&status_pwm_obj, LED_ON_DUTY, 2);    // LED Solid when connected

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
			GUI_SetFont(&GUI_FONT);                                        // Font size
			GUI_DispString("Disconnected");                                // Display disconnected message
		}

		status = WICED_BT_GATT_SUCCESS;
	}

	return status;
}

/*******************************************************************************
 * Function Name: app_bt_server_event_handler
 *
 * Invoked when GATT_ATTRIBUTE_REQUEST_EVT occurs in GATT Event callback.
 *
 * Param:	p_data   				Pointer to BLE GATT request data
 * Return:	wiced_bt_gatt_status_t  BLE GATT status
*******************************************************************************/
static wiced_bt_gatt_status_t app_bt_server_event_handler(wiced_bt_gatt_event_data_t *p_data) {
	wiced_bt_gatt_status_t status = WICED_BT_GATT_ERROR;
	wiced_bt_gatt_attribute_request_t *p_att_req = &p_data->attribute_request;

	switch(p_att_req->opcode) {
		case GATT_REQ_READ: /* Attribute read notification (attribute value internally read from GATT database) */
		case GATT_REQ_READ_BLOB:
			status = app_bt_gatt_req_read_handler(p_att_req->conn_id, p_att_req->opcode, &p_att_req->data.read_req, p_att_req->len_requested);
			break;

		case GATT_REQ_READ_BY_TYPE:
			status =
			app_bt_gatt_req_read_by_type_handler(p_att_req->conn_id, p_att_req->opcode, &p_att_req->data.read_by_type, p_att_req->len_requested);
			break;

		case GATT_REQ_READ_MULTI:
		case GATT_REQ_READ_MULTI_VAR_LENGTH:
			status =
			app_bt_gatt_req_read_multi_handler(p_att_req->conn_id, p_att_req->opcode, &p_att_req->data.read_multiple_req, p_att_req->len_requested);
			break;

		case GATT_REQ_WRITE:
		case GATT_CMD_WRITE:
		case GATT_CMD_SIGNED_WRITE:
			status = app_bt_write_handler(p_data);
			if((p_att_req->opcode == GATT_REQ_WRITE) && (status == WICED_BT_GATT_SUCCESS)) {
				wiced_bt_gatt_write_req_t *p_write_request = &p_att_req->data.write_req;
				wiced_bt_gatt_server_send_write_rsp(p_att_req->conn_id, p_att_req->opcode, p_write_request->handle);
				printf("Write response sent!\n");
			}
			break;

		case GATT_REQ_PREPARE_WRITE: status = WICED_BT_GATT_SUCCESS; break;

		case GATT_REQ_EXECUTE_WRITE:
			wiced_bt_gatt_server_send_execute_write_rsp(p_att_req->conn_id, p_att_req->opcode);
			status = WICED_BT_GATT_SUCCESS;
			break;

		case GATT_REQ_MTU:
			/* Application calls wiced_bt_gatt_server_send_mtu_rsp() with the desired mtu */
			status =
			wiced_bt_gatt_server_send_mtu_rsp(p_att_req->conn_id, p_att_req->data.remote_mtu, wiced_bt_cfg_settings.p_ble_cfg->ble_max_rx_pdu_size);
			break;

		case GATT_HANDLE_VALUE_CONF: /* Value confirmation */ break;

		case GATT_HANDLE_VALUE_NOTIF: break;

		default: printf("Unhandled GATT Server Event: 0x%x (%d)\n", p_att_req->opcode, p_att_req->opcode); break;
	}

	return status;
}

/*******************************************************************************
 * Function Name: app_bt_write_handler
 *
 * Invoked when GATTS_REQ_TYPE_WRITE is received from the
 * client device. Handles "Write Requests" received from Client device.
 *
 * Param:	p_write_req   			Pointer to BLE GATT write request
 * Return:	wiced_bt_gatt_status_t  BLE GATT status
*******************************************************************************/
static wiced_bt_gatt_status_t app_bt_write_handler(wiced_bt_gatt_event_data_t *p_data) {
	wiced_bt_gatt_write_req_t *p_write_req = &p_data->attribute_request.data.write_req;
	;

	wiced_bt_gatt_status_t status = WICED_BT_GATT_INVALID_HANDLE;

	printf("WRITE REQUEST RECEIVED!\n");

	for(int i = 0; i < app_gatt_db_ext_attr_tbl_size; i++) {
		/* Check for a matching handle entry */
		if(app_gatt_db_ext_attr_tbl[i].handle == p_write_req->handle) {
			/* Detected a matching handle in the external lookup table */
			if(app_gatt_db_ext_attr_tbl[i].max_len >= p_write_req->val_len) {
				/* Value fits within the supplied buffer; copy over the value */
				app_gatt_db_ext_attr_tbl[i].cur_len = p_write_req->val_len;
				memset(app_gatt_db_ext_attr_tbl[i].p_data, 0x00, app_gatt_db_ext_attr_tbl[i].max_len);
				memcpy(app_gatt_db_ext_attr_tbl[i].p_data, p_write_req->p_val, app_gatt_db_ext_attr_tbl[i].cur_len);

				if(memcmp(app_gatt_db_ext_attr_tbl[i].p_data, p_write_req->p_val, app_gatt_db_ext_attr_tbl[i].cur_len) == 0) {
					status = WICED_BT_GATT_SUCCESS;
				}

				switch(p_write_req->handle) {
					case HDLC_PONG_BALL_VALUE:    // Write To ball value occurred
						gameBall.posX = app_pong_ball[1];
						gameBall.posX = gameBall.posX << 8;
						gameBall.posX |= app_pong_ball[0];
						gameBall.posY = app_pong_ball[3];
						gameBall.posY = gameBall.posY << 8;
						gameBall.posY |= app_pong_ball[2];
						gameBall.speedX = app_pong_ball[5];
						gameBall.speedX = gameBall.speedX << 8;
						gameBall.speedX |= app_pong_ball[4];
						gameBall.speedY = app_pong_ball[7];
						gameBall.speedY = gameBall.speedY << 8;
						gameBall.speedY |= app_pong_ball[6];
						gameBall.numBounces = app_pong_ball[9];
						gameBall.numBounces = gameBall.numBounces << 8;
						gameBall.numBounces |= app_pong_ball[8];
						printf("Game Ball Data Received via Write:\nposX: %d\nposY: %d\nspeedX: %d\nspeedY: %d\nNumber of Bounces: %d\n", gameBall.posX, gameBall.posY,
							   gameBall.speedX, gameBall.speedY, gameBall.numBounces);
						hasBall = true;
						break;
				}
			} else {
				/* Value to write will not fit within the table */
				status = WICED_BT_GATT_INVALID_ATTR_LEN;
				printf("Invalid attribute length during GATT write\n");
			}
			break;
		}
	}
	if(WICED_BT_GATT_SUCCESS != status) {
		printf("GATT write failed: %d\n", status);
	}

	return status;
}

/*******************************************************************************
 * Function Name: app_bt_gatt_req_read_handler
 *
 * This Function handles GATT read and read blob events
 *
 * Params: 	conn_id       			Connection ID
 * 			opcode        			BLE GATT request type opcode
 * 			p_read_req    			Pointer to read request containing the handle to read
 * 			len_requested 			Length of data requested
 * Return: 	wiced_bt_gatt_status_t  BLE GATT status
*******************************************************************************/
static wiced_bt_gatt_status_t app_bt_gatt_req_read_handler(uint16_t conn_id, wiced_bt_gatt_opcode_t opcode, wiced_bt_gatt_read_t *p_read_req,
														   uint16_t len_requested) {
	gatt_db_lookup_table_t *puAttribute;
	uint16_t attr_len_to_copy, to_send;
	uint8_t *from;

	if((puAttribute = app_bt_find_by_handle(p_read_req->handle)) == NULL) {
		wiced_bt_gatt_server_send_error_rsp(conn_id, opcode, p_read_req->handle, WICED_BT_GATT_INVALID_HANDLE);
		return WICED_BT_GATT_INVALID_HANDLE;
	}

	attr_len_to_copy = puAttribute->cur_len;

	if(p_read_req->offset >= puAttribute->cur_len) {
		wiced_bt_gatt_server_send_error_rsp(conn_id, opcode, p_read_req->handle, WICED_BT_GATT_INVALID_OFFSET);
		return WICED_BT_GATT_INVALID_OFFSET;
	}

	switch(p_read_req->handle) {
		case HDLC_PONG_BALL:
			// Read Action
			break;
	}

	to_send = MIN(len_requested, attr_len_to_copy - p_read_req->offset);
	from = puAttribute->p_data + p_read_req->offset;
	return wiced_bt_gatt_server_send_read_handle_rsp(conn_id, opcode, to_send, from, NULL); /* No need for context, as buff not allocated */
}

/*******************************************************************************
 * Function Name: app_bt_gatt_req_read_by_type_handler
 *
 * Process read-by-type request from peer device
 *
 * Params:	conn_id       			Connection ID
 * 			opcode        			BLE GATT request type opcode
 * 			p_read_req    			Pointer to read request containing the handle to read
 * 			len_requested 			Length of data requested
 * Return:	wiced_bt_gatt_status_t	BLE GATT status
*******************************************************************************/
static wiced_bt_gatt_status_t app_bt_gatt_req_read_by_type_handler(uint16_t conn_id, wiced_bt_gatt_opcode_t opcode,
																   wiced_bt_gatt_read_by_type_t *p_read_req, uint16_t len_requested) {
	gatt_db_lookup_table_t *puAttribute;
	uint16_t attr_handle = p_read_req->s_handle;
	uint8_t *p_rsp = app_bt_alloc_buffer(len_requested);
	uint8_t pair_len = 0;
	int used = 0;

	if(p_rsp == NULL) {
		wiced_bt_gatt_server_send_error_rsp(conn_id, opcode, attr_handle, WICED_BT_GATT_INSUF_RESOURCE);
		return WICED_BT_GATT_INSUF_RESOURCE;
	}

	/* Read by type returns all attributes of the specified type, between the start and end handles */
	while(WICED_TRUE) {
		attr_handle = wiced_bt_gatt_find_handle_by_type(attr_handle, p_read_req->e_handle, &p_read_req->uuid);

		if(attr_handle == 0)
			break;

		if((puAttribute = app_bt_find_by_handle(attr_handle)) == NULL) {
			wiced_bt_gatt_server_send_error_rsp(conn_id, opcode, p_read_req->s_handle, WICED_BT_GATT_ERR_UNLIKELY);
			app_bt_free_buffer(p_rsp);
			return WICED_BT_GATT_INVALID_HANDLE;
		}

		{
			int filled = wiced_bt_gatt_put_read_by_type_rsp_in_stream(p_rsp + used, len_requested - used, &pair_len, attr_handle,
																	  puAttribute->cur_len, puAttribute->p_data);
			if(filled == 0) {
				break;
			}
			used += filled;
		}

		/* Increment starting handle for next search to one past current */
		attr_handle++;
	}

	if(used == 0) {
		wiced_bt_gatt_server_send_error_rsp(conn_id, opcode, p_read_req->s_handle, WICED_BT_GATT_INVALID_HANDLE);
		app_bt_free_buffer(p_rsp);
		return WICED_BT_GATT_INVALID_HANDLE;
	}

	switch(p_read_req->s_handle) {
		case HDLC_PONG_BALL:
			// Read by type action
			break;
	}

	/* Send the response */
	wiced_bt_gatt_server_send_read_by_type_rsp(conn_id, opcode, pair_len, used, p_rsp, (void *)app_bt_free_buffer);

	return WICED_BT_GATT_SUCCESS;
}

/*******************************************************************************
 * Function Name: app_bt_gatt_req_read_multi_handler
 *
 * Process write read multi request from peer device
 *
 * Params:	conn_id       			Connection ID
 * 			opcode        			BLE GATT request type opcode
 * 			p_read_req    			Pointer to read request containing the handle to read
 * 			len_requested 			Length of data requested
 * Return:	wiced_bt_gatt_status_t  BLE GATT status
*******************************************************************************/
static wiced_bt_gatt_status_t app_bt_gatt_req_read_multi_handler(uint16_t conn_id, wiced_bt_gatt_opcode_t opcode,
																 wiced_bt_gatt_read_multiple_req_t *p_read_req, uint16_t len_requested) {
	gatt_db_lookup_table_t *puAttribute;
	uint8_t *p_rsp = app_bt_alloc_buffer(len_requested);
	int used = 0;
	int xx;
	uint16_t handle = wiced_bt_gatt_get_handle_from_stream(p_read_req->p_handle_stream, 0);

	if(p_rsp == NULL) {
		wiced_bt_gatt_server_send_error_rsp(conn_id, opcode, handle, WICED_BT_GATT_INSUF_RESOURCE);
		return WICED_BT_GATT_INVALID_HANDLE;
	}

	/* Read by type returns all attributes of the specified type, between the start and end handles */
	for(xx = 0; xx < p_read_req->num_handles; xx++) {
		handle = wiced_bt_gatt_get_handle_from_stream(p_read_req->p_handle_stream, xx);
		if((puAttribute = app_bt_find_by_handle(handle)) == NULL) {
			wiced_bt_gatt_server_send_error_rsp(conn_id, opcode, *p_read_req->p_handle_stream, WICED_BT_GATT_ERR_UNLIKELY);
			app_bt_free_buffer(p_rsp);
			return WICED_BT_GATT_ERR_UNLIKELY;
		}

		{
			int filled = wiced_bt_gatt_put_read_multi_rsp_in_stream(opcode, p_rsp + used, len_requested - used, puAttribute->handle,
																	puAttribute->cur_len, puAttribute->p_data);
			if(!filled) {
				break;
			}
			used += filled;
		}
	}

	if(used == 0) {
		wiced_bt_gatt_server_send_error_rsp(conn_id, opcode, *p_read_req->p_handle_stream, WICED_BT_GATT_INVALID_HANDLE);
		return WICED_BT_GATT_INVALID_HANDLE;
	}

	switch(*p_read_req->p_handle_stream) {
		case HDLC_PONG_BALL:
			// Multi read action
			break;
	}

	/* Send the response */
	wiced_bt_gatt_server_send_read_multiple_rsp(conn_id, opcode, used, p_rsp, (void *)app_bt_free_buffer);

	return WICED_BT_GATT_SUCCESS;
}

/*******************************************************************************
* Function Name: app_bt_find_by_handle
*
* Finds attribute location by handle
*
* Param:  handle    				handle to look up
* Return: gatt_db_lookup_table_t   	pointer to location containing handle data
********************************************************************************/
static gatt_db_lookup_table_t *app_bt_find_by_handle(uint16_t handle) {
	int i;
	for(i = 0; i < app_gatt_db_ext_attr_tbl_size; i++) {
		if(app_gatt_db_ext_attr_tbl[i].handle == handle) {
			return (&app_gatt_db_ext_attr_tbl[i]);
		}
	}
	return NULL;
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
