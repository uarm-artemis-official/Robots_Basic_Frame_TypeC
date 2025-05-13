#ifndef __CAN_COMM_H
#define __CAN_COMM_H

#ifdef __cplusplus
extern "C" {
#endif

#include "uarm_types.h"

#define MAX_QUEUE_SIZE 5

void init_can_comm();
void can_transmit_comm_message(uint8_t* send_data, uint32_t comm_id);

#ifdef __cplusplus
}
#endif

#endif