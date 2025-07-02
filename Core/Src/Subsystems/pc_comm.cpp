#include "pc_comm.h"

#include "pack_handler.h"

namespace {
    Pack_Metadata_t pack_metadata[] = {
        {UC_AUTO_AIM_HEADER, PACK_HEADER_SIZE, sizeof(UC_Auto_Aim_Pack_t),
         PACK_TRAILER_SIZE},
        {UC_BOARD_DATA_HEADER, PACK_HEADER_SIZE, sizeof(UC_Board_Data_Pack_t),
         PACK_TRAILER_SIZE},
        {UC_FLOW_CONTROL_HEADER, PACK_HEADER_SIZE,
         sizeof(UC_Flow_Control_Pack_t), PACK_TRAILER_SIZE},
    };
}

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
        calc_checksum(pack_bytes, MAX_PACK_BUFFER_SIZE);
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

uint8_t PCComm::get_data_size(uint8_t header_id) {
    for (size_t i = 0; i < sizeof(pack_metadata) / sizeof(Pack_Metadata_t);
         i++) {
        if (header_id == pack_metadata[i].header_id) {
            return pack_metadata[i].data_size;
        }
    }
    return -1;
}

void PCComm::start_receive(uint8_t* pack_buffer) {
    uc_start_receive(pack_buffer, MAX_PACK_BUFFER_SIZE);
}

void PCComm::restart_receive(uint8_t* pack_buffer) {
    uc_restart_receive(pack_buffer, MAX_PACK_BUFFER_SIZE);
}

UC_Checksum_t PCComm::calc_checksum(void* buffer_ptr, size_t buffer_size) {
    /*
	 * This function assumes the last 4 bytes represent the pack checksum sent from UC,
	 * therefore, they are ignored during checksum calculation (unless the checksum will
	 * always be different).
	 *
	 * ALGORITHM:
	 *  - Separates bytes of uc_pack_input_buffer into 32 bit integers.
	 *  - Add integers together.
	 *  - Mod sum by 1000000007 (arbitrary prime number).
	 *
	 */
    const uint32_t MOD = 1000000007;
    uint32_t* data = (uint32_t*) buffer_ptr;
    uint32_t running_checksum = 0;

    for (size_t i = 0; i < buffer_size / 4 - 1; i++) {
        running_checksum = (running_checksum + (data[i] % MOD)) % MOD;
    }

    UC_Checksum_t checksum_union;
    checksum_union.as_integer = running_checksum;
    return checksum_union;
}

uint8_t PCComm::is_valid_header(uint8_t* input_buffer) {
    // All valid packs will have the header as the first byte in the input buffer.
    // Check under PACK HEADER in auto_aim.h to see macros defining recognized headers.
    for (size_t i = 0; i < sizeof(pack_metadata) / sizeof(Pack_Metadata_t);
         i++) {
        if (input_buffer[0] == pack_metadata[i].header_id) {
            return 1;
        }
    }
    return 0;
}