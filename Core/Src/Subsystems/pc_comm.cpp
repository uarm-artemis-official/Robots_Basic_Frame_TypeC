#include "pc_comm.h"

#include "pack_handler.h"

static uint32_t invalid_pack_count = 0;
static uint32_t valid_pack_count = 0;
uint8_t PCComm::uc_check_pack_integrity(uint8_t* pack_bytes,
                                        uint8_t pack_size) {
    // Pack checksum can never be exactly 0, so if any bits
    // in the checksum are set then a pack was just received.
    uint8_t checksum_bits =
        pack_bytes[pack_size - 1] | pack_bytes[pack_size - 2] |
        pack_bytes[pack_size - 3] | pack_bytes[pack_size - 4];
    UC_Checksum_t pack_checksum =
        calculate_checksum(pack_bytes, MAX_PACK_BUFFER_SIZE);
    uint32_t sent_checksum =
        *((uint32_t*) pack_bytes + (MAX_PACK_BUFFER_SIZE / 4) - 1);

    if (checksum_bits != 0 && pack_checksum.as_integer == sent_checksum &&
        is_valid_header(pack_bytes)) {
        valid_pack_count++;
        return 0;
    } else {
        invalid_pack_count++;
        return 1;
    }
}

void PCComm::send_bytes(uint8_t* bytes, uint32_t size) {
    uc_send_bytes(bytes, size);
}
