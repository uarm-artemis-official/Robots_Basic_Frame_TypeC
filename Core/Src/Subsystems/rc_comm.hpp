#ifndef __RC_COMM_H
#define __RC_COMM_H

#include <array>
#include "subsystems_interfaces.h"
#include "subsystems_types.h"

class RCComm : public IRCComm {
   public:
    void buffer_init(Buffer& buffer) override;
    void key_object_init(KeyObject& key) override;
    void keyboard_init(Keyboard& keyboard) override;
    void mouse_init(Mouse& mouse) override;
    void pc_init(PC& pc) override;
    void controller_init(Controller& controller) override;
    void parse_switches(Buffer& buffer, ESwitchState& s1,
                        ESwitchState& s2) override;
    void parse_controller(Buffer& buffer, Controller& controller) override;
    void parse_pc(Buffer& buffer, PC& pc) override;
    void key_scan(KeyObject& key, uint16_t key_buffer,
                  EKeyBitIndex key_bit_index) override;
};

#endif