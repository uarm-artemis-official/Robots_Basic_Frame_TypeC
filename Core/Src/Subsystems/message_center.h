#ifndef __MESSAGE_CENTER_H
#define __MESSAGE_CENTER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "uarm_lib.h"
#include "uarm_os.h"
#include "subsystems_types.h"


void message_center_init();
uint8_t get_message(Topic_Name_t topic, void *data_ptr, int ticks_to_wait);
uint8_t peek_message(Topic_Name_t topic, void *data_ptr, int ticks_to_wait);
uint8_t pub_message(Topic_Name_t topic, void *data_ptr);
uint8_t pub_message_from_isr(Topic_Name_t topic, void *data_ptr, uint8_t *will_context_switch);

#ifdef __cplusplus
}
#endif

#endif
