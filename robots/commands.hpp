#include "simple_comm.hpp"
#include "simple_comm_utils.hpp"

namespace simple_comm {
    inline namespace v1 {

        enum class ACKStatus : uint8_t { OK = 0, ERROR = 1, INVALID = 2 };

        struct CommandACK {
            static const simple_comm::MessageType MESSAGE_TYPE =
                MessageType::COMMAND;
            static const size_t SERIALIZED_SIZE = 2;

            uint8_t command_id;
            ACKStatus status;

            static bool serialize_payload(
                const CommandACK& msg,
                std::span<std::byte, SERIALIZED_SIZE> dst) {
                dst[0] = std::byte {msg.command_id};
                dst[1] = std::byte {static_cast<uint8_t>(msg.status)};
                return true;
            }

            static bool deserialize_payload(
                std::span<const std::byte, SERIALIZED_SIZE> src,
                CommandACK& msg) {
                msg.command_id = static_cast<uint8_t>(src[0]);
                msg.status = static_cast<ACKStatus>(src[1]);
                return true;
            }
        };

        struct PingPongCommand {
            static const simple_comm::MessageType MESSAGE_TYPE =
                MessageType::COMMAND;
            static const size_t SERIALIZED_SIZE = 0;

            static bool serialize_payload(
                const PingPongCommand&, std::span<std::byte, SERIALIZED_SIZE>) {
                return true;  // no payload to serialize
            }

            static bool deserialize_payload(
                std::span<const std::byte, SERIALIZED_SIZE>, PingPongCommand&) {
                return true;  // no payload to deserialize
            }
        };
    }  // namespace v1
}  // namespace simple_comm
