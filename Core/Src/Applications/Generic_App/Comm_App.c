/*
******************************************************************************
* @file           : Comm_App.c
* @brief          : communication real time task between boards
* @created time	  : Jul, 2023
* @author         : Haoran
******************************************************************************
* Copyright (c) 2023 UARM Artemis.
* All rights reserved.
******************************************************************************
*/

#ifndef __COMM_APP_C__
#define __COMM_APP_C__

#include <Control_App.h>
#include "Comm_App.h"
#include "Chassis_App.h"
#include "tim.h"
#include "string.h"

#define USART_COMM 0


extern Comm_t comm_pack;
extern Chassis_t chassis;
extern Gimbal_t gimbal; // TODO if we have multiple gimbals, we need to change this
extern can_comm_rx_t can_comm_rx[TOTAL_COMM_ID];
extern RemoteControl_t rc;

/* user defined variable */
/* rx and tx buffer for ensure the transmission accuracy:
 * rel angle    1
   rcontroller  2
   pc info      3
   key		    4
   referee      5
 * */
float can_rx_scale_buffer[TOTAL_COMM_ID][4] = {0};//or use malloc. Since it has infinite live period, they are same.
/* rx and tx buffer for ensure the transmission accuracy:
 * rel angle    1
   rcontroller  2
   pc info      3
   key		    4
   referee      5
 * */
float can_tx_scale_buffer[TOTAL_COMM_ID][4] = {0};
int32_t capture_flag = 0;

/**
* @brief Function implementing the Comm_Task thread. Set for the comm task bw upper and lower boards
* @param argument: Not used
* @retval None
*/
void Comm_Task_Func(void const * argument)
{
  /* USER CODE BEGIN Comm_Task_Func */
	while(1){
		if(USART_COMM == 1){
			usart_comm_process();
		}
		else{
			if(board_status == CHASSIS_BOARD)
				can_comm_process(&chassis_comm);
			else if(board_status == GIMBAL_BOARD)
				can_comm_process(&gimbal_comm);
		}
	}
  /* USER CODE END Comm_Task_Func */
}
/***************************** CAN COMM BEGAIN ************************************/
/**
* @brief CAN commnication message subscription
* @param None
* @retval None
*/
void can_comm_subscribe_process(void){
	if(board_status == CHASSIS_BOARD){
		comm_subscribe(&chassis_comm.sub_list, COMM_REMOTE_CONTROL, Transmitter);
		comm_subscribe(&chassis_comm.sub_list, COMM_PC_CONTROL, Transmitter);
		comm_subscribe(&chassis_comm.sub_list, COMM_EXT_PC_CONTROL, Transmitter);
		comm_subscribe(&chassis_comm.sub_list, COMM_GIMBAL_ANGLE, Receiver);
	}
	else if(board_status == GIMBAL_BOARD){
		comm_subscribe(&gimbal_comm.sub_list, COMM_GIMBAL_ANGLE, Transmitter);
		comm_subscribe(&gimbal_comm.sub_list, COMM_REMOTE_CONTROL, Receiver);
		comm_subscribe(&gimbal_comm.sub_list, COMM_PC_CONTROL, Receiver);
		comm_subscribe(&gimbal_comm.sub_list, COMM_EXT_PC_CONTROL, Receiver);
	}
}
/**
* @brief CAN commnication struct initialization
* @param None
* @retval None
*/
void can_comm_reset_config(BoardComm_t *comm){
	comm->comm_mode = CAN_COMM_MODE;
	comm->can_comm.comm_id = IDLE_COMM_ID;
	/* init rx buffer */

	for(int i = 0; i < TOTAL_COMM_ID; i++) {
		if(i<4)
			comm->can_comm.tx_data[i] = 0;
		comm->can_comm.rx_data[i][0] = 0;
		comm->can_comm.rx_data[i][1] = 0;
		comm->can_comm.rx_data[i][2] = 0;
		comm->can_comm.rx_data[i][3] = 0;
	}
    comm->can_comm.can_send_comm_data = can_send_comm_data;
    comm->can_comm.can_recv_comm_data = can_recv_comm_data;

    /* init subscription list */
    memset(&comm->sub_list, 0, sizeof(CommMessageSublist_t));
	/* init fifo queue */
	queueM_init(&canqm);
}

/**
* @brief CAN communication process
* @param None
* @retval None
*/
/* Task exec time: 3ms */
void can_comm_process(BoardComm_t *comm){

	TickType_t xLastWakeTime;
	const TickType_t xFrequency = pdMS_TO_TICKS(3); // 200hz make sure this task quicker than rc app

	/* reset the comm struct configure */
	can_comm_reset_config(comm);
	/* subscribe the message before starting comms */
	can_comm_subscribe_process();

	/* init the task ticks */
    xLastWakeTime = xTaskGetTickCount();

	for(;;){

		/* recv data */
		comm->can_comm.can_recv_comm_data(&hcan2, 8, comm->can_comm.rx_data);
		/* process data */
		if(isSubscribed(&comm->sub_list, COMM_GIMBAL_ANGLE) == SUB_SUCCESS){
			switch(gimbal_angle_message.role){
			 	 /* need to scale and re-scale angle data in both side */
				case Transmitter:
					if(gimbal_angle_message.message.comm_ga.send_flag == 1){
						process_tx_data_ftoi16(gimbal_angle_message.message.comm_ga.angle_data, comm->can_comm.tx_data, 4, ANGLE_COMM_SCALE_FACTOR);
						comm->can_comm.can_send_comm_data(&hcan2, comm->can_comm.tx_data, ANGLE_COMM_ID);
						gimbal_angle_message.message.comm_ga.send_flag = 0;//reset flag to avoid message flooding
					}
					break;
				case Receiver:
					if(can_comm_rx[ANGLE_IDX].comm_id == ANGLE_COMM_ID){
						process_rx_data_i16tof(comm, gimbal_angle_message.message.comm_ga.angle_data, ANGLE_COMM_SCALE_FACTOR, ANGLE_IDX);
						chassis.gimbal_yaw_rel_angle = gimbal_angle_message.message.comm_ga.angle_data[0];//can_rx_scale_buffer[ANGLE_IDX][0];
						chassis.gimbal_yaw_abs_angle = gimbal_angle_message.message.comm_ga.angle_data[1];//can_rx_scale_buffer[ANGLE_IDX][1];
						can_comm_rx[ANGLE_IDX].comm_id = 0;//reset id to avoid message flooding
					}
					break;
			}
		}
		if(isSubscribed(&comm->sub_list, COMM_REMOTE_CONTROL) == SUB_SUCCESS){
			switch(rc_message.role){
				case Transmitter:
					if(rc_message.message.comm_rc.send_flag == 1){
						memcpy(comm->can_comm.tx_data, &(rc_message.message.comm_rc.rc_data), sizeof(rc_message.message.comm_rc.rc_data));
						comm->can_comm.can_send_comm_data(&hcan2, comm->can_comm.tx_data, RC_COMM_ID);
						rc_message.message.comm_rc.send_flag = 0;//reset flag to avoid message flooding
					}
					break;
				case Receiver:
					if(can_comm_rx[RC_IDX].comm_id == RC_COMM_ID){
						rc.ctrl.s1  = comm->can_comm.rx_data[RC_IDX][2];
						rc.ctrl.s2  = comm->can_comm.rx_data[RC_IDX][3];
						if(rc.ctrl.s1 == SW_MID && rc.ctrl.s2 == SW_DOWN){
							rc.control_mode = PC_MODE;
							/* update gimbal mode */
							//FIXME: if have multiple gimbals, we need to change this.
							gimbal.gimbal_mode = comm->can_comm.rx_data[RC_IDX][0];
							gimbal.gimbal_act_mode = comm->can_comm.rx_data[RC_IDX][1];
						}
						else{
							rc.control_mode = CTRLER_MODE;
							/* get normal controller data*/
							rc.ctrl.ch0 = comm->can_comm.rx_data[RC_IDX][0];
							rc.ctrl.ch1 = comm->can_comm.rx_data[RC_IDX][1];
						}
						can_comm_rx[RC_IDX].comm_id = 0;//reset id to avoid message flooding
					}

					break;
			 }
		}
		if(isSubscribed(&comm->sub_list, COMM_PC_CONTROL) == SUB_SUCCESS){
			switch(pc_message.role){
				case Transmitter:
					if(pc_message.message.comm_pc.send_flag == 1){
						memcpy(comm->can_comm.tx_data, &(pc_message.message.comm_pc.pc_data), sizeof(pc_message.message.comm_pc.pc_data));
						comm->can_comm.can_send_comm_data(&hcan2, comm->can_comm.tx_data, PC_COMM_ID);
						pc_message.message.comm_pc.send_flag = 0;//reset flag to avoid message flooding
					}
					break;
				case Receiver:
					if(can_comm_rx[PC_IDX].comm_id == PC_COMM_ID){
						rc.pc.mouse.x = comm->can_comm.rx_data[PC_IDX][0];
						rc.pc.mouse.y = comm->can_comm.rx_data[PC_IDX][1];
						rc.pc.mouse.left_click.status  = comm->can_comm.rx_data[PC_IDX][2];
						rc.pc.mouse.right_click.status  = comm->can_comm.rx_data[PC_IDX][3];
						can_comm_rx[PC_IDX].comm_id = 0;//reset id to avoid message flooding
					}
					break;
			 }
		}

		if(isSubscribed(&comm->sub_list, COMM_EXT_PC_CONTROL) == SUB_SUCCESS){
					switch(pc_ext_message.role){
						case Transmitter:
							if(pc_ext_message.message.comm_ext_pc.send_flag == 1){
								memcpy(comm->can_comm.tx_data, &(pc_ext_message.message.comm_ext_pc.pc_data), sizeof(pc_ext_message.message.comm_ext_pc.pc_data));
								comm->can_comm.can_send_comm_data(&hcan2, comm->can_comm.tx_data, PC_EXT_KEY_ID);
								pc_ext_message.message.comm_ext_pc.send_flag = 0;//reset flag to avoid message flooding
							}
							break;
						case Receiver:
							if(can_comm_rx[PC_EXT_KEY_IDX].comm_id == PC_EXT_KEY_ID){
								rc.pc.key.C.status = comm->can_comm.rx_data[PC_EXT_KEY_IDX][0];
								rc.pc.key.V.status = comm->can_comm.rx_data[PC_EXT_KEY_IDX][1];
								rc.pc.key.B.status  = comm->can_comm.rx_data[PC_EXT_KEY_IDX][2];
//								rc.pc.mouse.right_click.status  = comm->can_comm.rx_data[PC_EXT_KEY_IDX][3];
								can_comm_rx[PC_EXT_KEY_IDX].comm_id = 0;//reset id to avoid message flooding
							}
							break;
					 }
				}

		/* delay until wake time */
		vTaskDelayUntil(&xLastWakeTime, xFrequency);
	}
}

/**
* @brief CAN commnication send data float to int16
* @param comm: main comm app struct
* 		 scale_factor: corresponding scale_factor
*/
void process_tx_data_ftoi16(float* input_data, int16_t* output_data, int length, float scale_factor){
    for (int i = 0; i < length; i++) {
        output_data[i] = (int16_t)(input_data[i] * scale_factor);
    }
}

/**
* @brief CAN commnication recving data int16 to float
* @param comm: main comm app struct
* 		 scale_factor: corresponding scale_factor
*/
void process_rx_data_i16tof(BoardComm_t *comm, float *output_buffer, float scale_factor, uint32_t idx) {
	/*we cannot set the output buffer as a local variable in this fucntion
	 * since it may cause "dangling pointer" problem */
    for (int i = 0; i < 4; i++) {
//        comm->can_comm.rx_data[idx][i] = (comm->can_comm.rx_data[idx][i*2] << 8) | comm->can_comm.rx_data[1][i*2+1];
        output_buffer[i] = (float)comm->can_comm.rx_data[idx][i] / scale_factor;
    }
}

/**
* @brief CAN commnication sending function, activated for CAN2 comms
* @param CAN_HandleTypeDef object: hcan pointer refer to a HAL CAN structure
* 		 int32_t* send_data: The data is ready to be sent
* 		 uint32_t comm_id：
* @retval None
*/
void can_send_comm_data(CAN_HandleTypeDef* hcan, int16_t* send_data, uint32_t comm_id){
	uint8_t comm_can_send_data[8];
//	uint32_t send_mail_box;
	CAN_TxHeaderTypeDef  comm_tx_message;

	comm_tx_message.IDE = CAN_ID_STD;
	comm_tx_message.RTR = CAN_RTR_DATA;
	comm_tx_message.DLC = 0x08;
	comm_tx_message.StdId = comm_id;

	comm_can_send_data[0] = send_data[0] >> 8;
	comm_can_send_data[1] = send_data[0];
	comm_can_send_data[2] = send_data[1] >> 8;
	comm_can_send_data[3] = send_data[1];
	comm_can_send_data[4] = send_data[2] >> 8;
	comm_can_send_data[5] = send_data[2];
	comm_can_send_data[6] = send_data[3] >> 8;
	comm_can_send_data[7] = send_data[3];

	/* send to fifo queue*/
	enqueueCanMessage(&comm_tx_message, canQueue, &canqm, comm_can_send_data);

//	if(HAL_CAN_GetTxMailboxesFreeLevel(hcan)>0){
		sendNextCanMessage(hcan, canQueue, &canqm);
//		HAL_CAN_AddTxMessage(hcan, &comm_tx_message, comm_can_send_data, (uint32_t *)CAN_TX_MAILBOX0);
//	}
}

/**
* @brief CAN commnication receiving function, activated for CAN2 comms
* @param CAN_HandleTypeDef object: A can pointer refer to a CAN structure
* 		 int32_t* send_data: The data is ready to be sent
* @retval None
*/
void can_recv_comm_data(CAN_HandleTypeDef* hcan, uint32_t data_len, int16_t (*rx_buffer)[TOTAL_COMM_ID][4]){
	uint8_t comm_temp_rx_buffer[8];
	for(int i=0;i<TOTAL_COMM_ID;i++){
		memcpy(comm_temp_rx_buffer, can_comm_rx[i].comm_rx_buffer, data_len);
		(*rx_buffer)[i][0] = (int16_t)(comm_temp_rx_buffer[0] << 8 | comm_temp_rx_buffer[1]);
		(*rx_buffer)[i][1] = (int16_t)(comm_temp_rx_buffer[2] << 8 | comm_temp_rx_buffer[3]);
		(*rx_buffer)[i][2] = (int16_t)(comm_temp_rx_buffer[4] << 8 | comm_temp_rx_buffer[5]);
		(*rx_buffer)[i][3] = (int16_t)(comm_temp_rx_buffer[6] << 8 | comm_temp_rx_buffer[7]);
	}
}
/***************************** CAN COMM END ************************************/
/* Since we have multiple can comm works in the future , there is necessity that apply
 * FIFO Queue management of our CAN2 data pool. This function has been moved to algo*/
/***************************** UART COMM BEGAIN ************************************/
/**
* @brief uart commnication process
* @param None
* @retval None
*/
void usart_comm_process(void){
	for(;;){
		if(capture_flag == 0){
			comm_pack.vision.target_num = 0;
		}
		else{
			osDelay(3000);
			if(capture_flag == 1)
				capture_flag = 0;
		}
	}
}
#ifndef USE_UART_DMA
static uint8_t temp_buffer[PACKLEN+1];
/**
* @brief uart global interrupt function
* @param UART_HandleTypeDef object refered to a uart structure
* @retval None
*/
//FIXME: Use DMA to trigger UART6 and UART7
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	 // When enter this callback function, the variable pdata has been filled with the received data.
	 // Thus parse it directly.
	  HAL_GPIO_WritePin(LD_E_GPIO_Port,LD_E_Pin,RESET);
	  if(huart == &huart6 && board_status == CHASSIS_BOARD){
		  HAL_GPIO_WritePin(LD_G_GPIO_Port,LD_G_Pin,RESET);
//		  comm_pack=parse_all(pdata);
		  strncpy(temp_buffer, pdata, PACKLEN);
		  comm_pack.vision = parse_all(temp_buffer);
		  abs_yaw=angle_preprocess(&motor_data[5], comm_pack.vision.yaw_data, 1.8, YAW_MOTOR);
//		  abs_pitch=angle_preprocess(&motor_data[4], comm_pack.pitch_data, 1, PITCH_MOTOR);
		  temp_buffer[sizeof(temp_buffer) - 1] = '\0';
		  HAL_UART_Transmit(&huart7, (char*)temp_buffer, strlen(temp_buffer),1000);
		  HAL_UART_Receive_IT(&huart6, (char*)pdata, PACKLEN);
		  capture_flag = 1;
	  }
	  else if(huart == &huart6 && board_status == GIMBAL_BOARD){
		  comm_pack.vision=parse_all(pdata);
//		  abs_yaw=angle_preprocess(&motor_data[5], comm_pack.yaw_data, 1, YAW_MOTOR);
		  abs_pitch=angle_preprocess(&motor_data[4], comm_pack.vision.pitch_data, 1, PITCH_MOTOR);
		  HAL_GPIO_WritePin(LD_F_GPIO_Port,LD_F_Pin,RESET);
		  HAL_UART_Receive_IT(&huart6, (char*)pdata, PACKLEN);
		  capture_flag = 1;
	  }
	  else{
		  HAL_GPIO_WritePin(LD_F_GPIO_Port,LD_F_Pin,SET);
		  capture_flag = 0;
	  }
	  // Enable the uart interrupt again
}

/* no sub comm func for backing up */
void can_comm_process_nosub(void){
	Comm_t comm_pack;
	/* can comm tasks */
	if(board_status == CHASSIS_BOARD){
		//reset the comm struct configure
//		can_comm_reset_config(&chassis_comm, IDLE_COMM_ID);
		for(;;){
			/* send tasks */
			if(comm_pack.comm_rc.send_flag == 1){
				memcpy(&(chassis_comm.can_comm.tx_data),&(comm_pack.comm_rc.rc_data), sizeof(comm_pack.comm_rc.rc_data));
				chassis_comm.can_comm.can_send_comm_data(&hcan2, chassis_comm.can_comm.tx_data, RC_COMM_ID);
				comm_pack.comm_rc.send_flag = 0;//reset flag to avoid message flooding
			}

			/* recv tasks */
			chassis_comm.can_comm.can_recv_comm_data(&hcan2, 8, chassis_comm.can_comm.rx_data);
			/* relative angle for attitude breakdown */
			if(can_comm_rx[ANGLE_IDX].comm_id == ANGLE_COMM_ID){
				process_rx_data(&chassis_comm, comm_pack.comm_ga.angle_data, ANGLE_COMM_SCALE_FACTOR, ANGLE_IDX);
				chassis.gimbal_yaw_rel_angle = -comm_pack.comm_ga.angle_data[0];//can_rx_scale_buffer[ANGLE_IDX][0];
				chassis.gimbal_yaw_abs_angle = comm_pack.comm_ga.angle_data[1];//can_rx_scale_buffer[ANGLE_IDX][1];
				can_comm_rx[ANGLE_IDX].comm_id = 0;//reset id to avoid message flooding
			}
       		osDelay(1);
		}
	}
	else if(board_status == GIMBAL_BOARD){
//		can_comm_reset_config(&gimbal_comm, IDLE_COMM_ID);
		for(;;){
			/* send tasks */
			/*yaw relative angle can_tx_scale_buffer[ANGLE_IDX]*/
			if(comm_pack.comm_ga.send_flag == 1){
				process_tx_data(comm_pack.comm_ga.angle_data, gimbal_comm.can_comm.tx_data, 4, ANGLE_COMM_SCALE_FACTOR);
				gimbal_comm.can_comm.can_send_comm_data(&hcan2, gimbal_comm.can_comm.tx_data, ANGLE_COMM_ID);
				comm_pack.comm_ga.send_flag = 0;//reset flag to avoid message flooding
			}

			/* recv tasks */
			gimbal_comm.can_comm.can_recv_comm_data(&hcan2, 8, gimbal_comm.can_comm.rx_data);
			/* relative angle for attitude breakdown */
			if(can_comm_rx[RC_IDX].comm_id == RC_COMM_ID){
				rc.ctrl.ch0 = gimbal_comm.can_comm.rx_data[RC_IDX][0];
				rc.ctrl.ch1 = gimbal_comm.can_comm.rx_data[RC_IDX][1];
				rc.ctrl.s1 = gimbal_comm.can_comm.rx_data[RC_IDX][2];
				rc.ctrl.s2 = gimbal_comm.can_comm.rx_data[RC_IDX][3];
				can_comm_rx[RC_IDX].comm_id = 0;//reset id to avoid message flooding
			}
			osDelay(1);
		}
	}
}

#endif

/*****************************  UART COMM END  ************************************/
/* The UART comm between boards are being on hold since ethe CAN comm can be more
 * reliable and fast, and we will use UART DMA to save cpu's arithmetic power */
/********************************  COMM END  ************************************/



#endif /*__COMM_TASK_C__*/
