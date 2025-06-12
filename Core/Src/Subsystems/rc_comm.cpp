#include "rc_comm.hpp"
#include <cmath>

void RCComm::buffer_init(Buffer& buffer) {
    for (size_t i = 0; i < buffer.size(); i++) {
        buffer[i] = 0;
    }
}

void RCComm::key_object_init(KeyObject& key) {
    key.status = EKeyStatus::RELEASED;
    key.pre_status = EKeyStatus::RELEASED;
    key.status_count = 0;
}

void RCComm::keyboard_init(Keyboard& keyboard) {
    key_object_init(keyboard.W);
    key_object_init(keyboard.A);
    key_object_init(keyboard.S);
    key_object_init(keyboard.D);
    key_object_init(keyboard.Q);
    key_object_init(keyboard.E);
    key_object_init(keyboard.R);
    key_object_init(keyboard.V);
    key_object_init(keyboard.Ctrl);
    key_object_init(keyboard.F);
    key_object_init(keyboard.Shift);
    key_object_init(keyboard.G);
    key_object_init(keyboard.C);
    key_object_init(keyboard.B);
    keyboard.key_buffer = 0;
}

void RCComm::mouse_init(Mouse& mouse) {
    mouse.x = 0;
    mouse.y = 0;
    mouse.z = 0;
    mouse.click_l = 0;
    mouse.click_r = 0;
    key_object_init(mouse.left_click);
    key_object_init(mouse.right_click);
}

void RCComm::pc_init(PC& pc) {
    mouse_init(pc.mouse);
    keyboard_init(pc.keyboard);
}

void RCComm::controller_init(Controller& controller) {
    controller.ch0 = 0;
    controller.ch1 = 0;
    controller.ch2 = 0;
    controller.ch3 = 0;
    controller.s1 = ESwitchState::UNKNOWN;
    controller.s2 = ESwitchState::UNKNOWN;
}

void RCComm::parse_switches(Buffer& buffer, ESwitchState& s1,
                            ESwitchState& s2) {
    uint8_t s1_val = ((std::get<5>(buffer) >> 4) & 0x000C) >> 2;
    uint8_t s2_val = (std::get<5>(buffer) >> 4) & 0x0003;

    if (1 <= s1_val && s1_val <= 3) {
        s1 = static_cast<ESwitchState>(s1_val);
    } else {
        s1 = ESwitchState::UNKNOWN;
    }

    if (1 <= s2_val && s2_val <= 3) {
        s2 = static_cast<ESwitchState>(s2_val);
    } else {
        s2 = ESwitchState::UNKNOWN;
    }
}

void RCComm::parse_controller(Buffer& buffer, Controller& controller) {
    parse_switches(buffer, controller.s1, controller.s2);

    controller.ch0 =
        ((std::get<0>(buffer) | (std::get<1>(buffer) << 8)) & 0x07ff) -
        CHANNEL_CENTER;

    controller.ch1 =
        (((std::get<1>(buffer) >> 3) | (std::get<2>(buffer) << 5)) & 0x07ff) -
        CHANNEL_CENTER;
    controller.ch2 = (((std::get<2>(buffer) >> 6) | (std::get<3>(buffer) << 2) |
                       (std::get<4>(buffer) << 10)) &
                      0x07ff) -
                     CHANNEL_CENTER;
    controller.ch3 =
        (((std::get<4>(buffer) >> 1) | (std::get<5>(buffer) << 7)) & 0x07ff) -
        CHANNEL_CENTER;
    controller.wheel =
        ((std::get<16>(buffer) | (std::get<17>(buffer) << 8)) & 0x07FF) -
        CHANNEL_CENTER;

    // TODO: Redo with asserts?
    if ((abs(controller.ch0) > MAX_CHANNEL_VALUE) ||
        (abs(controller.ch1) > MAX_CHANNEL_VALUE) ||
        (abs(controller.ch2) > MAX_CHANNEL_VALUE) ||
        (abs(controller.ch3) > MAX_CHANNEL_VALUE)) {
        controller.ch0 = 0;
        controller.ch1 = 0;
        controller.ch2 = 0;
        controller.ch3 = 0;
    }
}

void RCComm::parse_pc(Buffer& buffer, PC& pc) {
    pc.mouse.x = std::get<6>(buffer) | (std::get<7>(buffer) << 8);
    pc.mouse.y = std::get<8>(buffer) | (std::get<9>(buffer) << 8);
    pc.mouse.z = std::get<10>(buffer) |
                 (std::get<11>(buffer)
                  << 8);  //why the official parse process has z axis??
    pc.mouse.click_l = std::get<12>(buffer);
    pc.mouse.click_r = std::get<13>(buffer);
    pc.keyboard.key_buffer =
        std::get<14>(buffer) |
        (std::get<15>(buffer) << 8);  //multiple keys reading

    key_scan(pc.keyboard.W, pc.keyboard.key_buffer, EKeyBitIndex::W);
    key_scan(pc.keyboard.W, pc.keyboard.key_buffer, EKeyBitIndex::S);
    key_scan(pc.keyboard.W, pc.keyboard.key_buffer, EKeyBitIndex::A);
    key_scan(pc.keyboard.W, pc.keyboard.key_buffer, EKeyBitIndex::D);
    key_scan(pc.keyboard.W, pc.keyboard.key_buffer, EKeyBitIndex::SHIFT);
    key_scan(pc.keyboard.W, pc.keyboard.key_buffer, EKeyBitIndex::CTRL);
    key_scan(pc.keyboard.W, pc.keyboard.key_buffer, EKeyBitIndex::Q);
    key_scan(pc.keyboard.W, pc.keyboard.key_buffer, EKeyBitIndex::E);
    key_scan(pc.keyboard.W, pc.keyboard.key_buffer, EKeyBitIndex::R);
    key_scan(pc.keyboard.W, pc.keyboard.key_buffer, EKeyBitIndex::F);
    key_scan(pc.keyboard.W, pc.keyboard.key_buffer, EKeyBitIndex::G);
    key_scan(pc.keyboard.W, pc.keyboard.key_buffer, EKeyBitIndex::C);
    key_scan(pc.keyboard.W, pc.keyboard.key_buffer, EKeyBitIndex::V);
    key_scan(pc.keyboard.W, pc.keyboard.key_buffer, EKeyBitIndex::B);

    if ((abs(pc.mouse.x) > MOUSE_MAX_SPEED) ||
        (abs(pc.mouse.x) > MOUSE_MAX_SPEED)) {
        pc.mouse.x = 0;
        pc.mouse.y = 0;
    }

    key_scan(pc.mouse.left_click, pc.mouse.click_l, EKeyBitIndex::W);
    key_scan(pc.mouse.right_click, pc.mouse.click_r, EKeyBitIndex::W);
}

void RCComm::key_scan(KeyObject& key, uint16_t key_buffer,
                      EKeyBitIndex key_bit_index) {
    uint16_t key_index = static_cast<uint16_t>(key_bit_index);
    if (key_buffer & key_index) {
        key.status_count++;
    } else
        key.status_count = 0;

    // Update key's status
    key.pre_status = key.status;
    if (key.status_count > 1) {  // hold pressed
        switch (key.pre_status) {
            /* in this case, we have 2 possible pre status */
            case EKeyStatus::RELEASED:
            case EKeyStatus::PRESSED_TO_RELEASE:
            case EKeyStatus::RELEASED_TO_PRESS:
                key.status = EKeyStatus::PRESSED;
                break;
            case EKeyStatus::PRESSED:
                key.status = EKeyStatus::PRESSED;
                break;
        }
        if (key.status_count > 100)
            key.status_count = 100;      //avoid infinite addition
    } else if (key.status_count == 1) {  // rising edge triggered
        switch (key.pre_status) {
            /* in this case , we have 2 possible pre status */
            case EKeyStatus::RELEASED_TO_PRESS:
            case EKeyStatus::PRESSED:
            case EKeyStatus::RELEASED:
                key.status = EKeyStatus::RELEASED_TO_PRESS;
                break;
            case EKeyStatus::PRESSED_TO_RELEASE:
                key.status = EKeyStatus::RELEASED_TO_PRESS;
                break;  // count not ++, indicate																	 // not pressed
        }
    } else if (key.status_count == 0) {  // released
        switch (key.pre_status) {
            /* in this case , we have 3 possible pre status */
            case EKeyStatus::RELEASED_TO_PRESS:
            case EKeyStatus::RELEASED:
                key.status = EKeyStatus::RELEASED;
                break;  //release
            case EKeyStatus::PRESSED:
                key.status = EKeyStatus::PRESSED_TO_RELEASE;
                break;  //just release, falling edge triggered
            case EKeyStatus::PRESSED_TO_RELEASE:
                key.status = EKeyStatus::RELEASED;
                break;  // release
        }
    }
}