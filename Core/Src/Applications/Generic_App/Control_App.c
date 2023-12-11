/*******************************************************************************
* @file           : Control_App.c
* @brief          : Recv remote control signal via rc or pc
* @created time	  : Jul, 2023
* @author         : Haoran
******************************************************************************
* Copyright (c) 2023 UARM Artemis.
* All rights reserved.
*******************************************************************************/

/* @attention:
 * 	we used circular mode of UART1 DMA to receive remote data in real time
 *  thus we need this task to parse the data (bc the data received constantly, do not
 *  handle it with IT.)*/

//FIXME: We don't have any risk management for rc. Need to add a watch dog


#ifndef __RC_APP_C__
#define __RC_APP_C__

#include <Control_App.h>
#include "main.h"
#include "string.h"
#include "Chassis_App.h"
#include "comms.h"

/*********************************************************************************
 *  				  <   GENERAL CTRL OPERATION TABLE  >
 *    @note: <- left dialing /->right dialing /A front dialing /V back dialing
 *    ----------------------------------------------------------------------------
 *    | 						CONTROLLER MODE - Infantry 			     	     |
 *    ----------------------------------------------------------------------------
 *    |		Keys    |    Action   |   				Description  		         |
 *    ----------------------------------------------------------------------------
 *    |  Right Rod  |   <- & ->   | CH0, -&+, Yaw gimbal left&right spin         |
 *    |             |             |           INPDT_MODE: ground left/right spin |
 *    ----------------------------------------------------------------------------
 *    |  Right Rod  |    A & V    | CH1, +&-, Pitch motor's Head down/head up    |
 *    ----------------------------------------------------------------------------
 *    |  Left  Rod  |   <- & ->   | CH2, -&+, INDPT_MODE: ground left/right move |
 *    |             |             |           Gimbal_follow: left/right spin     |
 *    ----------------------------------------------------------------------------
 *    |  Left  Rod  |    A & V    | CH3, +&-, INDPT_MODE: Ground  for/backward   |
 *    |             |             |           other: Follow yaw's direction      |
 *    ----------------------------------------------------------------------------
 *    |  Right Sw   |     UP      | Shoot Continuously    				         |
 *    |    (S2)     |     MID     | Shoot cease fire     					     |
 *    |             |     DOWN    | Self-Gyro(Combo): spin chassis & follow yaw  |
 *    ----------------------------------------------------------------------------
 *    |  Left  Sw   |     UP      | Gimbal Center: yaw axis, chassis chase yaw   |
 *    |    (S1)     |     MID     | IDLE_mode: shut down all system              |
 *    |             |     DOWN    | Gimbal Follow: yaw axis, chassis no spin     |
 *    ----------------------------------------------------------------------------
 *    * Combo SWs:
 *    		S1 Down&S2 Down: Independent mode
 *    		S1 Up  &S2 Down: Self-Gyro mode
 **********************************************************************************
 *    ----------------------------------------------------------------------------
 *    | 							PC MODE - Infantry 		 		     	     |
 *    ----------------------------------------------------------------------------
 *    |	Keys&Mouse  |    Action      |   			Description  		         |
 *    ----------------------------------------------------------------------------
 *    |  	W	  	|  Go forward    |  All mode								 |
 *    ----------------------------------------------------------------------------
 *    |  	A	  	|  left panning  |  All mode								 |
 *    ----------------------------------------------------------------------------
 *    |	    S       |  go backward   |  All mode       				             |
 *    ----------------------------------------------------------------------------
 *    |  	D	  	|  right panning |  All mode								 |
 *    ----------------------------------------------------------------------------
 *    |  	Q	  	|  left spin 	 |  Only independent mode & follow mode      |
 *    ----------------------------------------------------------------------------
 *    |  	E	  	|  right spin  	 |  Only independent mode & follow mode      |
 *    ----------------------------------------------------------------------------
 *    |  	R	  	|  Reload  	     |  Open lids/Close lids. All mode   		 |
 *    ----------------------------------------------------------------------------
 *    |  	V	  	|  Gear  	     |  Chassis gear manually switch.All mode    |
 *    ----------------------------------------------------------------------------
 *    |  	Ctrl	|  Mode Switch   |  Gyro or Encoder mode  				     |
 *    ----------------------------------------------------------------------------
 *    |  	F		|  Sub Mode Swap |  Gyro mode: Gimbal Center or Self-Gyro    |
 *    |  	 		|  			  	 |  Encoder mode: Gimbal Follow or Ground	 |
 *    ----------------------------------------------------------------------------
 *    |  	Shift	|  Accelerate    |  Supercapacitor turned on(long press)/off |
 *    ----------------------------------------------------------------------------
 *    |  	G	  	|  Safety Key    |  IDLE MODE, shut down everything(hold) 	 |
 *    |				|				 |  (release to select the normal mode) 	 |
 *    ----------------------------------------------------------------------------
 *    |  	C		|  Auto aim 	 |  Swap to auto aiming mode    			 |
 *    ----------------------------------------------------------------------------
 *    | Left Click	|  Shoot 	 	 |  Tap to fire a single shot, press and 	 |
 *    |				|				 |	hold to continously shoot.   			 |
 *    ----------------------------------------------------------------------------
 *    | Right Click |  Swap Barrel 	 |  Swap barrels if have double barrel	     |
 *    ----------------------------------------------------------------------------
 *    |  	B	  	|  Shoot speed   |  shoot fric and mag speed manually switch |
 *    ----------------------------------------------------------------------------
 *
 *    Note: Shift need to be combined with any of WASD keys.
 *
 *********************************************************************************/

/* define rc global handler */
RemoteControl_t rc;
extern Chassis_t chassis;
extern CommMessage_t rc_message;
extern CommMessage_t pc_message;

/* extern glbal vars here */


/* define internal functions */
static void rc_update_comm_pack(RemoteControl_t *rc_hdlr, CommRemoteControl_t *comm_rc, CommPCControl_t *comm_pc);
/**
  * @brief     main remote control task
  * @param[in] None
  * @retval    None
  */
void RC_Task_Func(){

	/* set task exec period */
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = pdMS_TO_TICKS(10); // 100Hz, make sure this task slower than comm app

	/* init rc task */
	rc_task_init(&rc);

	/* reset rc test */
	rc_reset(&rc);

	/* init the task ticks */
	xLastWakeTime = xTaskGetTickCount();

	for(;;){

		rc_process_rx_data(&rc, rc_rx_buffer);
		rc_update_comm_pack(&rc, &(rc_message.message.comm_rc), &(pc_message.message.comm_pc));

		/* delay until wake time */
	    vTaskDelayUntil(&xLastWakeTime, xFrequency);
	}

}
/**
  * @brief     init all the struct before task begin
  * @param[in] main rc struct
  * @retval    None
  */
void rc_task_init(RemoteControl_t *rc_hdlr){
	/* controller init */
	rc_hdlr->ctrl.ch0 = 0;
	rc_hdlr->ctrl.ch1 = 0;
	rc_hdlr->ctrl.ch2 = 0;
	rc_hdlr->ctrl.ch3 = 0;
	rc_hdlr->ctrl.s1 = SW_MID; //idle mode
	rc_hdlr->ctrl.s2 = SW_MID; //cease fire

	/* pc init */
	rc_hdlr->pc.mouse.x = 0;
	rc_hdlr->pc.mouse.y = 0;
	rc_hdlr->pc.mouse.z = 0;
	rc_hdlr->pc.mouse.click_l = 0;
	rc_hdlr->pc.mouse.click_r = 0;

	rc_key_init(&rc_hdlr->pc.key.W);
	rc_key_init(&rc_hdlr->pc.key.A);
	rc_key_init(&rc_hdlr->pc.key.S);
	rc_key_init(&rc_hdlr->pc.key.D);
	rc_key_init(&rc_hdlr->pc.key.Q);
	rc_key_init(&rc_hdlr->pc.key.E);
	rc_key_init(&rc_hdlr->pc.key.R);
	rc_key_init(&rc_hdlr->pc.key.V);
	rc_key_init(&rc_hdlr->pc.key.Ctrl);
	rc_key_init(&rc_hdlr->pc.key.F);
	rc_key_init(&rc_hdlr->pc.key.Shift);
	rc_key_init(&rc_hdlr->pc.key.G);
	rc_key_init(&rc_hdlr->pc.key.C);
	rc_key_init(&rc_hdlr->pc.key.B);
	rc_key_init(&rc_hdlr->pc.mouse.left_click);
	rc_key_init(&rc_hdlr->pc.mouse.right_click);


	/* init low pass params */
	rc_hdlr->pc.mouse.x_folp.a = 0.95;
	rc_hdlr->pc.mouse.x_folp.cur_data = 0;
	rc_hdlr->pc.mouse.x_folp.last_output_data = 0;
	rc_hdlr->pc.mouse.x_folp.output_data = 0;

	rc_hdlr->pc.mouse.y_folp.a = 0.95;
	rc_hdlr->pc.mouse.y_folp.cur_data = 0;
	rc_hdlr->pc.mouse.y_folp.last_output_data = 0;
	rc_hdlr->pc.mouse.y_folp.output_data = 0;

	/* apply deflaut mode */
	rc_hdlr->control_mode = CTRLER_MODE;
}
/**
  * @brief     rc process recv data from dbus
  * @param[in] main rc struct
  * @retval    None
  */
void rc_process_rx_data(RemoteControl_t *rc_hdlr, uint8_t *rc_rx_buffer){
	/* no matter what mode, read switch data */
	rc_hdlr->ctrl.s1   = ((rc_rx_buffer[5] >> 4)& 0x000C) >> 2;
	rc_hdlr->ctrl.s2   = ((rc_rx_buffer[5] >> 4)& 0x0003);      //may use this as mode swap indicator

	/* currently hard coding */
	if(rc_hdlr->ctrl.s1 == SW_MID && rc_hdlr->ctrl.s2 == SW_DOWN)
		rc_hdlr->control_mode = PC_MODE;
	else
		rc_hdlr->control_mode = CTRLER_MODE;

	if(rc_hdlr->control_mode == CTRLER_MODE){
		/* remote controller parse process */
		rc_hdlr->ctrl.ch0  = ((rc_rx_buffer[0]| (rc_rx_buffer[1] << 8)) & 0x07ff) - CHANNEL_CENTER;
		rc_hdlr->ctrl.ch1  = (((rc_rx_buffer[1] >> 3) | (rc_rx_buffer[2] << 5)) & 0x07ff) - CHANNEL_CENTER;
		rc_hdlr->ctrl.ch2  = (((rc_rx_buffer[2] >> 6) | (rc_rx_buffer[3] << 2) | (rc_rx_buffer[4] << 10)) & 0x07ff) - CHANNEL_CENTER;
		rc_hdlr->ctrl.ch3  = (((rc_rx_buffer[4] >> 1) | (rc_rx_buffer[5] << 7)) & 0x07ff) - CHANNEL_CENTER;
		rc_hdlr->ctrl.wheel = ((rc_rx_buffer[16]|(rc_rx_buffer[17]<<8))&0x07FF) - CHANNEL_CENTER;

		/* calibration process to avoid some unexpected values */
		if ((abs(rc_hdlr->ctrl.ch0)  > CHANNEL_OFFSET_MAX_ABS_VAL) ||(abs(rc_hdlr->ctrl.ch1) > CHANNEL_OFFSET_MAX_ABS_VAL) || \
			 (abs(rc_hdlr->ctrl.ch2) > CHANNEL_OFFSET_MAX_ABS_VAL) ||(abs(rc_hdlr->ctrl.ch3) > CHANNEL_OFFSET_MAX_ABS_VAL))
		  {
			rc.ctrl.ch0 = 0;
			rc.ctrl.ch1 = 0;
			rc.ctrl.ch2 = 0;
			rc.ctrl.ch3 = 0;
		  }
	}
	else if(rc_hdlr->control_mode == PC_MODE){
		/* pc parse process */
		rc_hdlr->pc.mouse.x = rc_rx_buffer[6] | (rc_rx_buffer[7] << 8);
		rc_hdlr->pc.mouse.y = rc_rx_buffer[8] | (rc_rx_buffer[9] << 8);
		rc_hdlr->pc.mouse.z = rc_rx_buffer[10] | (rc_rx_buffer[11] << 8);//why the official parse process has z axis??
		rc_hdlr->pc.mouse.click_l = rc_rx_buffer[12];
		rc_hdlr->pc.mouse.click_r = rc_rx_buffer[13];
		rc_hdlr->pc.key.key_buffer = rc_rx_buffer[14] | (rc_rx_buffer[15] << 8);//multiple keys reading

		/* scan all the keys */
		rc_key_scan(&rc_hdlr->pc.key.W, rc_hdlr->pc.key.key_buffer, KEY_BOARD_W);
		rc_key_scan(&rc_hdlr->pc.key.S, rc_hdlr->pc.key.key_buffer, KEY_BOARD_S);
		rc_key_scan(&rc_hdlr->pc.key.A, rc_hdlr->pc.key.key_buffer, KEY_BOARD_A);
		rc_key_scan(&rc_hdlr->pc.key.D, rc_hdlr->pc.key.key_buffer, KEY_BOARD_D);
		rc_key_scan(&rc_hdlr->pc.key.Shift, rc_hdlr->pc.key.key_buffer, KEY_BOARD_SHIFT);
		rc_key_scan(&rc_hdlr->pc.key.Ctrl, rc_hdlr->pc.key.key_buffer, KEY_BOARD_CTRL);
		rc_key_scan(&rc_hdlr->pc.key.Q, rc_hdlr->pc.key.key_buffer, KEY_BOARD_Q);
		rc_key_scan(&rc_hdlr->pc.key.E, rc_hdlr->pc.key.key_buffer, KEY_BOARD_E);
		rc_key_scan(&rc_hdlr->pc.key.R, rc_hdlr->pc.key.key_buffer, KEY_BOARD_R);
		rc_key_scan(&rc_hdlr->pc.key.F, rc_hdlr->pc.key.key_buffer, KEY_BOARD_F);
		rc_key_scan(&rc_hdlr->pc.key.G, rc_hdlr->pc.key.key_buffer, KEY_BOARD_G);
		rc_key_scan(&rc_hdlr->pc.key.C, rc_hdlr->pc.key.key_buffer, KEY_BOARD_C);
		rc_key_scan(&rc_hdlr->pc.key.V, rc_hdlr->pc.key.key_buffer, KEY_BOARD_V);
		rc_key_scan(&rc_hdlr->pc.key.B, rc_hdlr->pc.key.key_buffer, KEY_BOARD_B);

		/* apply low pass filter for mouse axis to avoid noise */
		rc_hdlr->pc.mouse.x = first_order_low_pass_filter(&rc_hdlr->pc.mouse.x_folp, rc_hdlr->pc.mouse.x);
		rc_hdlr->pc.mouse.y = first_order_low_pass_filter(&rc_hdlr->pc.mouse.y_folp, rc_hdlr->pc.mouse.y);

		/* calibration process to avoid some unexpected values */
		if ((abs(rc_hdlr->pc.mouse.x)  > MOUSE_MAX_SPEED_VALUE) ||(abs(rc_hdlr->pc.mouse.x) > MOUSE_MAX_SPEED_VALUE)){
			rc_hdlr->pc.mouse.x = 0;
			rc_hdlr->pc.mouse.y = 0;
		}

		rc_key_scan(&rc_hdlr->pc.mouse.left_click, rc_hdlr->pc.mouse.click_l, 0x0001);
		rc_key_scan(&rc_hdlr->pc.mouse.right_click, rc_hdlr->pc.mouse.click_r, 0x0001);

	}
}
/**
  * @brief     update the remote comm pack
  * @param[in] main rc struct
  * @param[in] comm rc struct
  * @retval    None
  */
static void rc_update_comm_pack(RemoteControl_t *rc_hdlr, CommRemoteControl_t *comm_rc, CommPCControl_t *comm_pc){
	if(rc_hdlr->control_mode == CTRLER_MODE){
		comm_rc->rc_data[0] = rc_hdlr->ctrl.ch0;
		comm_rc->rc_data[1] = rc_hdlr->ctrl.ch1;
		comm_rc->rc_data[2]  = rc_hdlr->ctrl.s1;
		comm_rc->rc_data[3]  = rc_hdlr->ctrl.s2;
		comm_rc->send_flag = 1;

		/* not send pc data */
		comm_pc->send_flag = 0;
	}
	else if(rc_hdlr->control_mode == PC_MODE){
		/* still need to update switch info */
		comm_rc->rc_data[0] = chassis.chassis_mode;//board mode
		comm_rc->rc_data[1] = chassis.chassis_act_mode;//act_mode
		comm_rc->rc_data[2]  = rc_hdlr->ctrl.s1;
		comm_rc->rc_data[3]  = rc_hdlr->ctrl.s2;
		comm_rc->send_flag = 1;

		/* update mouse info */
		comm_pc->pc_data[0] = rc_hdlr->pc.mouse.x;
		comm_pc->pc_data[1] = rc_hdlr->pc.mouse.y;
		comm_pc->pc_data[2]  = rc_hdlr->pc.mouse.left_click.status;
		comm_pc->pc_data[3]  = rc_hdlr->pc.mouse.right_click.status;
		comm_pc->send_flag = 1;
	}
}

/**
  * @brief     reset everything when error occurs
  * @param[in] main rc struct
  * @retval    None
  */
void rc_reset(RemoteControl_t *rc_hdlr){
	/* stop DMA */
	HAL_UART_DMAStop(&huart3);
	/* try to reconnect to rc */
	HAL_UART_Receive_DMA(&huart3, rc_rx_buffer, DBUS_BUFFER_LEN);
}

/* Keyboard operation process */
/**
  * @brief	   key object initialization
  * @param[in] key object
  * @retval    None
  */
void rc_key_init(KeyObject_t *key){
	key->status = RELEASED;
	key->pre_status = RELEASED;
	key->status_count = 0;
}
/**
  * @brief	   key object initialization
  * @param[in] key object
  * @retval    None
  */
void rc_key_scan(KeyObject_t *key_obj, uint16_t key_buffer, uint16_t compare_key){
	if(key_buffer & compare_key)
		key_obj->status_count++;
	else
		key_obj->status_count = 0;
	rc_get_key_status(key_obj);
}
/**
  * @brief     get the current key status based on the count
  * @param[in] key object
  * @retval    current key status
  */
KeyStatus_t rc_get_key_status(KeyObject_t *key){
	key->pre_status = key->status;
	if(key->status_count > 1){// hold pressed
		switch(key->pre_status){
			/* in this case, we have 2 possible pre status */
			case RELEASED:
			case PRESSED_TO_RELEASE:
			case RELEASED_TO_PRESS: key->status = PRESSED;break;
			case PRESSED: key->status = PRESSED;break;
		}
		if(key->status_count > 100)
			key->status_count = 100; //avoid infinite addition
	}
	else if(key->status_count == 1){ // rising edge triggered
		switch(key->pre_status){
			/* in this case , we have 2 possible pre status */
			case RELEASED_TO_PRESS:
			case PRESSED:
			case RELEASED: key->status = RELEASED_TO_PRESS;break;
			case PRESSED_TO_RELEASE:key->status = RELEASED_TO_PRESS;break;// count not ++, indicate																	 // not pressed
		}
	}
	else if(key->status_count == 0){ // released
		switch(key->pre_status){
			/* in this case , we have 3 possible pre status */
			case RELEASED_TO_PRESS:
			case RELEASED: key->status = RELEASED;break;//release
			case PRESSED: key->status  =  PRESSED_TO_RELEASE;break;//just release, falling edge triggered
			case PRESSED_TO_RELEASE:key->status = RELEASED;break; // release
		}
	}
	return key->status;
}


#endif /*__RC_APP_C__*/
