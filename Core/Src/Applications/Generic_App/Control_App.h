/*******************************************************************************
* @file           : Control_App.h
* @brief          : remote control task to get rc data from controller or pc
* @created time	  : Jul, 2023
* @author         : Haoran
******************************************************************************
* Copyright (c) 2023 UARM Artemis.
* All rights reserved.
*******************************************************************************/

/*
 * if you copy this file from other OPEN-SOURCE project, please do not
 * add this declaration above. Instead, you are supposed to add their
 * licenses or copyright declaration. Please check ramp.c for more details
 *
 * */

#ifndef __RC_APP_H__
#define __RC_APP_H__

#include "stdint.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "maths.h"
#include "public_defines.h"
#include "message_center.h"
#include "usart.h"


/* define general declarations for gimbal task here */
#define DBUS_BUFFER_LEN 18
#define DBUS_RAW_OFFSET 0
#define DBUS_INDEX(i) ((i + DBUS_RAW_OFFSET) % DBUS_BUFFER_LEN)

#define CHANNEL_CENTER  1024

#define MOUSE_MAX_SPEED_VALUE 15000
//#define MOUSE_MIN_SPEED_VALUE 10000

/* switch status defines */
#define SW_UP   1
#define SW_MID  3
#define SW_DOWN 2


/********************** Define PC End Keys Begin *************************/
#define KEY_BOARD_W 		((uint16_t)0x01<<0)
#define KEY_BOARD_S 		((uint16_t)0x01<<1)
#define KEY_BOARD_A 		((uint16_t)0x01<<2)
#define KEY_BOARD_D 		((uint16_t)0x01<<3)
#define KEY_BOARD_SHIFT 	((uint16_t)0x01<<4)
#define KEY_BOARD_CTRL 	    ((uint16_t)0x01<<5)
#define KEY_BOARD_Q 		((uint16_t)0x01<<6)
#define KEY_BOARD_E 		((uint16_t)0x01<<7)
#define KEY_BOARD_R 		((uint16_t)0x01<<8)
#define KEY_BOARD_F 		((uint16_t)0x01<<9)
#define KEY_BOARD_G 		((uint16_t)0x01<<10)
#define KEY_BOARD_Z 		((uint16_t)0x01<<11)
#define KEY_BOARD_X 		((uint16_t)0x01<<12)
#define KEY_BOARD_C 		((uint16_t)0x01<<13)
#define KEY_BOARD_V 		((uint16_t)0x01<<14)
#define KEY_BOARD_B 		((uint16_t)0x01<<15)
/********************** Define PC End Keys End *************************/

/* define user structure here */
/**
  * @brief  main struct of rc task
  */
/* remote controller mode */

typedef enum{
	RELEASED = 0,	    // key released
	RELEASED_TO_PRESS,  // key just pressed, rising edge
	PRESSED_TO_RELEASE, // key just released, falling edge
	PRESSED			    // key pressed
}KeyStatus_t;

typedef struct{
	int16_t ch0;
	int16_t ch1;
	int16_t ch2;
	int16_t ch3;
	uint8_t s1;
	uint8_t s2;
	int16_t wheel;
}Controller_t;

/* pc mode */
typedef struct{
	KeyStatus_t status;
	KeyStatus_t pre_status;
	uint8_t status_count;
}KeyObject_t;

typedef struct{
	KeyObject_t W;
	KeyObject_t A;
	KeyObject_t S;
	KeyObject_t D;
	KeyObject_t Q;
	KeyObject_t E;
	KeyObject_t R;
	KeyObject_t V;
	KeyObject_t Ctrl;
	KeyObject_t F;
	KeyObject_t Shift;
	KeyObject_t G;
	KeyObject_t C;
	KeyObject_t B;

	uint16_t key_buffer;// used to grab current key info
}Key_t;

typedef struct{
	int16_t x;
	int16_t y;
	int16_t z;
	uint8_t click_l;
	uint8_t click_r;
	KeyObject_t left_click;
	KeyObject_t right_click;

	/* low pass filter */
	first_order_low_pass_t x_folp;
	first_order_low_pass_t y_folp;
}Mouse_t;

typedef struct{
	Mouse_t mouse;
	Key_t   key;
}PC_t;


typedef struct{
	/* controll mode selection */
	Controller_t    ctrl;
	PC_t      		pc;
	CtrlMode_t      control_mode;

	/* status update */
	BoardMode_t     board_mode;
	BoardActMode_t  board_act_mode;
}RemoteControl_t;

/* extern global variables here */
extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;


/* functions declaration here */
void RC_Task_Func();
void rc_task_init(RemoteControl_t *rc_hdlr);
void rc_process_rx_data(RemoteControl_t *rc_hdlr, uint8_t *rc_rx_buffer);
void rc_reset(RemoteControl_t *rc_hdlr);

void rc_key_init(KeyObject_t *key);
void rc_key_scan(KeyObject_t *key_obj, uint16_t key_buffer, uint16_t compare_key);
KeyStatus_t rc_get_key_status(KeyObject_t *key);


void rc_on_uart_complete();
void rc_on_uart_error();

#endif /*__RC_APP_H__*/


