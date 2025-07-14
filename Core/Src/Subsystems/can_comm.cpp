#include "can_comm.hpp"
#include "can.h"
#include "queue_m.h"
#include "subsystems_defines.h"
#include "uarm_lib.h"
#include "uarm_os.h"

void CanComm::init() {
    memset(canQueue, 0, sizeof(CanMessage_t) * CAN_COMM_QUEUE_SIZE);
    memset(&canqm, 0, sizeof(QueueManage_t));
    canqm.queue_size = CAN_COMM_QUEUE_SIZE;
}

void CanComm::can_transmit_comm_message(uint8_t send_data[8],
                                        uint32_t comm_id) {
    CAN_TxHeaderTypeDef comm_tx_message;

    comm_tx_message.IDE = CAN_ID_STD;
    comm_tx_message.RTR = CAN_RTR_DATA;
    comm_tx_message.DLC = 0x08;
    comm_tx_message.StdId = comm_id;

    enqueueCanMessage(&comm_tx_message, canQueue, &canqm, send_data);
    sendNextCanMessage(&hcan2, canQueue, &canqm);
}