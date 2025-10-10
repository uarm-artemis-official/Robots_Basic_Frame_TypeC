#include "topics.hpp"
#include <cstring>
#include "message_center.hpp"

namespace mc2 {
    void GimbalCommand::encode(std::array<uint8_t, 200>& bytes) {
        ASSERT(-2 * PI < yaw && yaw < 2 * PI,
               "Outgoing yaw out of acceptable range (-2PI, 2PI).");
        ASSERT(-2 * PI < pitch && pitch < 2 * PI,
               "Outgoing pitch out of acceptable range (-2PI, 2PI).");
        ASSERT((command_bits & 0xffff0000) == 0,
               "Outgoing command_bits must not have 8 MSB set.");
        int16_t encoded_yaw = yaw * 5000;
        int16_t encoded_pitch = pitch * 5000;
        uint16_t encoded_command_bits = command_bits & 0xffff;
        memcpy(bytes.data(), &encoded_yaw, sizeof(int16_t));
        memcpy(bytes.data() + sizeof(int16_t), &encoded_pitch, sizeof(int16_t));
        memcpy(bytes.data() + sizeof(int16_t) + sizeof(int16_t),
               &encoded_command_bits, sizeof(uint16_t));
    }

    void GimbalCommand::decode(std::array<uint8_t, 200>& bytes) {
        int16_t encoded_yaw;
        int16_t encoded_pitch;
        uint16_t encoded_command_bits;
        memcpy(&encoded_yaw, bytes.data(), sizeof(int16_t));
        memcpy(&encoded_pitch, bytes.data() + sizeof(int16_t), sizeof(int16_t));
        memcpy(&encoded_command_bits,
               bytes.data() + sizeof(int16_t) + sizeof(int16_t),
               sizeof(uint16_t));
        yaw = static_cast<float>(encoded_yaw) / 5000;
        pitch = static_cast<float>(encoded_pitch) / 5000;
        command_bits = static_cast<uint32_t>(encoded_command_bits);
        ASSERT(-2 * PI < yaw && yaw < 2 * PI,
               "Incoming yaw out of acceptable range (-2PI, 2PI).");
        ASSERT(-2 * PI < pitch && pitch < 2 * PI,
               "Incoming pitch out of acceptable range (-2PI, 2PI).");
        ASSERT((command_bits & 0xffff0000) == 0,
               "Incoming command_bits must not have 8 MSB set.");
    }

    void ShootCommand::encode(std::array<uint8_t, 200>& bytes) {
        ASSERT((extra_bits & 0xffff0000) == 0,
               "Incoming extra_bits must not have 16 MSB set.");
        uint32_t encoded_command_bits = command_bits;
        uint16_t encoded_extra_bits = extra_bits && 0xffff;
        memcpy(bytes.data(), &encoded_command_bits, sizeof(uint32_t));
        memcpy(bytes.data() + sizeof(uint32_t), &encoded_extra_bits,
               sizeof(uint16_t));
    }

    void ShootCommand::decode(std::array<uint8_t, 200>& bytes) {
        uint32_t encoded_command_bits;
        uint16_t encoded_extra_bits;
        memcpy(&encoded_command_bits, bytes.data(), sizeof(uint32_t));
        memcpy(&encoded_extra_bits, bytes.data(), sizeof(uint16_t));
        command_bits = encoded_command_bits;
        extra_bits = encoded_extra_bits & 0xffff;
        ASSERT((extra_bits & 0xffff0000) == 0,
               "Incoming extra_bits must not have 16 MSB set.");
    }

    void GimbalRelativeAngles::encode(std::array<uint8_t, 200>& bytes) {
        ASSERT(-PI < yaw && yaw < PI,
               "Outgoing yaw out of acceptable range (-PI, PI).");
        ASSERT(-PI < pitch && pitch < PI,
               "Outgoing pitch out of acceptable range (-PI, PI).");
        int16_t encoded_yaw = yaw * 10000;
        int16_t encoded_pitch = pitch * 10000;
        memcpy(bytes.data(), &encoded_yaw, sizeof(int16_t));
        memcpy(bytes.data() + sizeof(int16_t), &encoded_pitch, sizeof(int16_t));
    }

    void GimbalRelativeAngles::decode(std::array<uint8_t, 200>& bytes) {
        int16_t encoded_yaw;
        int16_t encoded_pitch;
        memcpy(&encoded_yaw, bytes.data(), sizeof(int16_t));
        memcpy(&encoded_pitch, bytes.data() + sizeof(int16_t), sizeof(int16_t));
        yaw = static_cast<float>(encoded_yaw) / 10000;
        pitch = static_cast<float>(encoded_pitch) / 10000;
        ASSERT(-PI < yaw && yaw < PI,
               "Incoming yaw out of acceptable range (-PI, PI).");
        ASSERT(-PI < pitch && pitch < PI,
               "Incoming pitch out of acceptable range (-PI, PI).");
    }
}  // namespace mc2