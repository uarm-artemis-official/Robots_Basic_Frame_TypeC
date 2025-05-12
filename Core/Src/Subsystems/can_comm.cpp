#include "can_comm.h"
#include "can.h"

#include "uarm_lib.h"
#include "uarm_os.h"

#include "queue_m.h"


static CanMessage_t canQueue[MAX_QUEUE_SIZE]; // TODO: Get rid of this for FreeRTOS queue or something.
static QueueManage_t canqm;

void init_can_comm() {
    memset(canQueue, 0, sizeof(CanMessage_t) * MAX_QUEUE_SIZE);
    memset(&canqm, 0, sizeof(QueueManage_t));
}


void can_transmit_comm_message(uint8_t *send_data, uint32_t comm_id) {
	CAN_TxHeaderTypeDef  comm_tx_message;

	comm_tx_message.IDE = CAN_ID_STD;
	comm_tx_message.RTR = CAN_RTR_DATA;
	comm_tx_message.DLC = 0x08;
	comm_tx_message.StdId = comm_id;

	enqueueCanMessage(&comm_tx_message, canQueue, &canqm, send_data);
	sendNextCanMessage(&hcan2, canQueue, &canqm);
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