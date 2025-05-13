#ifndef __PC_COMM_H
#define __PC_COMM_H

#ifdef __cplusplus
extern "C" {
#endif

#include "uarm_types.h"

class PCComm {
   public:
    static uint8_t uc_check_pack_integrity(uint8_t* pack_bytes,
                                           uint8_t pack_size);
};

#ifdef __cplusplus
}
#endif

#endif