#ifndef __PC_COMM_H
#define __PC_COMM_H

#include "subsystems_interfaces.h"
#include "uarm_types.h"

class PCComm : public IPCComm {
   public:
    uint8_t uc_check_pack_integrity(uint8_t* pack_bytes,
                                    uint8_t pack_size) override;
    uint8_t get_data_size(uint8_t header_id) override;
    void start_receive(uint8_t* pack_buffer) override;
    void restart_receive(uint8_t* pack_buffer) override;
    void send_bytes(uint8_t* bytes, uint32_t size) override;
    UC_Checksum_t calc_checksum(void* data, size_t size) override;
    uint8_t is_valid_header(uint8_t* input_buffer) override;
};

#endif