#ifndef __CAN_COMM_H
#define __CAN_COMM_H

#include "queue_m.h"
#include "subsystems_interfaces.h"
#include "uarm_types.hpp"

class CanComm : public ICanComm {
   private:
    CanMessage_t canQueue
        [CAN_COMM_QUEUE_SIZE];  // TODO: Get rid of this for FreeRTOS queue or something.
    QueueManage_t canqm;

   public:
    void init() override;
    void can_transmit_comm_message(uint8_t send_data[8],
                                   uint32_t comm_id) override;
};

#endif