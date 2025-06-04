#ifndef __RC_COMM_H
#define __RC_COMM_H

#include <array>
#include "subsystems_types.h"

namespace rc_comm {
    void buffer_init(Buffer& buffer);

    void key_object_init(KeyObject& key);
    void keyboard_init(Keyboard& keyboard);
    void mouse_init(Mouse& mouse);
    void pc_init(PC& pc);

    void controller_init(Controller& controller);

    void parse_switches(Buffer& buffer, ESwitchState& s1, ESwitchState& s2);
    void parse_controller(Buffer& buffer, Controller& controller);
    void parse_pc(Buffer& buffer, PC& pc);

    void key_scan(KeyObject& key, uint16_t key_buffer,
                  EKeyBitIndex key_bit_index);
}  // namespace rc_comm

#endif