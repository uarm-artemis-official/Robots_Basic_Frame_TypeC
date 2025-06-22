#ifndef __PC_COMM_H
#define __PC_COMM_H

#include "uarm_types.h"

class PCComm {
   public:
    static uint8_t uc_check_pack_integrity(uint8_t* pack_bytes,
                                           uint8_t pack_size);
    static void send_bytes(uint8_t* bytes, uint32_t size);
    static uint8_t get_data_size(uint8_t header_id);
    static void start_receive(uint8_t* pack_buffer);
    static void restart_receive(uint8_t* pack_buffer);
};

#endif