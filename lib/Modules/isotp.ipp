#include <cstring>
#include "uarm_lib.hpp"

namespace isotp {

    template <typename FSend, typename FDelay>
    ISOTP<FSend, FDelay>::ISOTP(FSend& _send_function, FDelay& _delay_function,
                                ISOTPConfig _config)
        : delay_function(_delay_function),
          send_function(_send_function),
          config(_config) {
        send_machine.send_state = ISOTPSendState::Ready;
        send_machine.send_message_buffer.fill(0);
        send_machine.send_message_length = 0;
        send_machine.used_send_buffer_length = 0;

        receive_machine.receive_state = ISOTPReceiveState::Ready;
        receive_machine.receive_message_buffer.fill(0);
        receive_machine.receive_indices_buffer.fill(0);
        receive_machine.indice_buffer_index = 0;
        receive_machine.used_receive_buffer_length = 0;
        receive_machine.receive_message_length = 0;
    }

    template <typename FSend, typename FDelay>
    void ISOTP<FSend, FDelay>::process_receive_frame(uint8_t* message,
                                                     size_t length) {
        PCICode pci_type = static_cast<PCICode>((message[0] & 0xF0) >> 4);
        switch (pci_type) {
            case PCICode::SingleFrame: {
                size_t msg_len = message[0] & 0x0F;
                check(msg_len <= 7,
                      "Single frame length exceeds single frame limits");
                std::memcpy(receive_machine.receive_message_buffer.data(),
                            &message[1], msg_len);
                receive_machine.receive_message_length = msg_len;
                receive_machine.used_receive_buffer_length = msg_len;
                set_receive_state(ISOTPReceiveState::HaveFullMessage);
                break;
            }
            case PCICode::FirstFrame: {
                size_t msg_len = ((message[0] & 0x0F) << 8) | message[1];
                check(msg_len <= MAX_MESSAGE_LENGTH,
                      "First frame length exceeds max");
                std::memcpy(receive_machine.receive_message_buffer.data(),
                            &message[2], length - 2);
                receive_machine.receive_message_length = msg_len;
                receive_machine.used_receive_buffer_length = length - 2;
                set_receive_state(ISOTPReceiveState::AssemblingMessage);
                break;
            }
            case PCICode::ConsecutiveFrame: {
                if (receive_machine.receive_state !=
                    ISOTPReceiveState::AssemblingMessage) {
                    check(false, "Unexpected Consecutive Frame");
                    // TODO: Robust mode: ignore or reset state
                    return;
                }
                size_t data_len = length - 1;
                size_t remaining = receive_machine.receive_message_length -
                                   receive_machine.used_receive_buffer_length;
                size_t copy_len = (data_len < remaining) ? data_len : remaining;

                uint8_t index = message[0] & 0x0F;
                receive_machine.receive_indices_buffer
                    [receive_machine.indice_buffer_index++] = index;

                std::memcpy(receive_machine.receive_message_buffer.data() +
                                receive_machine.used_receive_buffer_length,
                            &message[1], copy_len);
                receive_machine.used_receive_buffer_length += copy_len;
                if (receive_machine.used_receive_buffer_length >=
                    receive_machine.receive_message_length) {
                    check(receive_machine.receive_message_length ==
                              receive_machine.used_receive_buffer_length,
                          "Declared message length doesn't match size of "
                          "received message "
                          "bytes.");

                    set_receive_state(ISOTPReceiveState::HaveFullMessage);
                }
                break;
            }
            case PCICode::FlowControl: {
                FlowControlFlag flow_status =
                    static_cast<FlowControlFlag>(message[0] & 0x0F);
                uint8_t block_size = message[1];
                uint8_t stmin_raw = message[2];

                send_machine.block_size = block_size;

                // Parse separation time according to ISO-TP spec
                uint32_t stmin_ms = 0;
                if (stmin_raw < 0xF0) {
                    stmin_ms = stmin_raw;
                } else if (stmin_raw >= 0xF1 && stmin_raw <= 0xF9) {
                    stmin_ms = (stmin_raw - 0xF0) * 100;
                } else {
                    check(false, "STmin value out of range");
                }
                send_machine.separation_time_min_ms = stmin_ms;

                // Change send_machine state according to flow status
                switch (flow_status) {
                    case FlowControlFlag::Continue:
                        set_send_state(ISOTPSendState::SendingConsecutive);
                        break;
                    case FlowControlFlag::Wait:
                        set_send_state(ISOTPSendState::WaitingForControl);
                        break;
                    case FlowControlFlag::Abort:
                        set_send_state(ISOTPSendState::Aborted);
                        break;
                    default:
                        check(false, "Unknown FlowControlFlag value");
                        break;
                }
                break;
            }
            default:
                check(false, "Unknown PCI type");
                // TODO: Robust mode: ignore or reset state
                break;
        }
    }

    template <typename FSend, typename FDelay>
    void ISOTP<FSend, FDelay>::tick_send_process() {
        switch (send_machine.send_state) {
            case ISOTPSendState::Ready:
                [[fallthrough]];
            case ISOTPSendState::WaitingForControl:
                [[fallthrough]];
            case ISOTPSendState::Aborted:
                // Nothing to send
                break;
            case ISOTPSendState::SendingSingleOrFirst: {
                if (send_machine.send_message_length <= 7) {
                    // Single Frame
                    uint8_t frame[8] = {0};
                    frame[0] = static_cast<uint8_t>(
                        send_machine.send_message_length & 0x0F);
                    std::memcpy(&frame[1],
                                send_machine.send_message_buffer.data(),
                                send_machine.send_message_length);
                    send_function(frame, send_machine.send_message_length + 1);
                    set_send_state(ISOTPSendState::Ready);
                } else {
                    // First Frame
                    uint8_t frame[8] = {0};
                    frame[0] = static_cast<uint8_t>(
                        0x10 |
                        ((send_machine.send_message_length >> 8) & 0x0F));
                    frame[1] = static_cast<uint8_t>(
                        send_machine.send_message_length & 0xFF);
                    size_t first_data_len = 6;
                    std::memcpy(&frame[2],
                                send_machine.send_message_buffer.data(),
                                first_data_len);
                    send_function(frame, 8);
                    send_machine.used_send_buffer_length = first_data_len;
                    send_machine.frames_sent_in_block = 0;
                    set_send_state(ISOTPSendState::WaitingForControl);
                }
                break;
            }
            case ISOTPSendState::SendingConsecutive: {
                size_t remaining = send_machine.send_message_length -
                                   send_machine.used_send_buffer_length;
                uint8_t block_size = send_machine.block_size;
                uint32_t stmin_ms = send_machine.separation_time_min_ms;
                size_t frame_idx =
                    1 + (send_machine.used_send_buffer_length > 0
                             ? (send_machine.used_send_buffer_length - 6) / 7
                             : 0);

                if (remaining > 0 &&
                    (block_size == 0 ||
                     send_machine.frames_sent_in_block < block_size)) {
                    uint8_t frame[8] = {0};
                    frame[0] = static_cast<uint8_t>(0x20 | (frame_idx & 0x0F));
                    size_t data_len = (remaining > 7) ? 7 : remaining;
                    std::memcpy(&frame[1],
                                send_machine.send_message_buffer.data() +
                                    send_machine.used_send_buffer_length,
                                data_len);
                    send_function(frame, data_len + 1);
                    send_machine.used_send_buffer_length += data_len;
                    remaining -= data_len;
                    send_machine.frames_sent_in_block++;
                }

                if (remaining == 0) {
                    set_send_state(ISOTPSendState::Ready);
                } else if (block_size != 0 &&
                           send_machine.frames_sent_in_block == block_size) {
                    set_send_state(ISOTPSendState::WaitingForControl);
                } else {
                    if (stmin_ms > 0 && remaining > 0) {
                        delay_function(stmin_ms);
                    }
                }
                break;
            }
            default:
                check(false, "Unsupported send state.");
        }
    }

    template <typename FSend, typename FDelay>
    void ISOTP<FSend, FDelay>::set_send_message(uint8_t* message,
                                                size_t length) {
        check(length <= MAX_MESSAGE_LENGTH, "Send message too long");
        if (send_machine.send_state != ISOTPSendState::Ready &&
            send_machine.send_state != ISOTPSendState::Aborted) {
            check(false, "Send message while previous not finished");
        }
        std::memcpy(send_machine.send_message_buffer.data(), message, length);
        send_machine.send_message_length = length;
        send_machine.used_send_buffer_length = 0;
        set_send_state(ISOTPSendState::SendingSingleOrFirst);
    }

    template <typename FSend, typename FDelay>
    void ISOTP<FSend, FDelay>::reset() {
        send_machine.send_message_buffer.fill(0);
        send_machine.send_message_length = 0;
        send_machine.used_send_buffer_length = 0;
        send_machine.send_state = ISOTPSendState::Ready;

        receive_machine.receive_message_buffer.fill(0);
        receive_machine.receive_indices_buffer.fill(0);
        receive_machine.indice_buffer_index = 0;
        receive_machine.used_receive_buffer_length = 0;
        receive_machine.receive_message_length = 0;
        receive_machine.receive_state = ISOTPReceiveState::Ready;
    }

    template <typename FSend, typename FDelay>
    bool ISOTP<FSend, FDelay>::get_receive_message(void* dst) {
        if (receive_machine.receive_state !=
            ISOTPReceiveState::HaveFullMessage) {
            return false;
        }

        std::memcpy(dst, receive_machine.receive_message_buffer.data(),
                    receive_machine.receive_message_length);
        set_receive_state(ISOTPReceiveState::Ready);
        return true;
    }

    template <typename FSend, typename FDelay>
    ISOTPSendState ISOTP<FSend, FDelay>::get_send_state() {
        return send_machine.send_state;
    }

    template <typename FSend, typename FDelay>
    ISOTPReceiveState ISOTP<FSend, FDelay>::get_receive_state() {
        return receive_machine.receive_state;
    }

    template <typename FSend, typename FDelay>
    void ISOTP<FSend, FDelay>::set_send_state(ISOTPSendState new_state) {
        switch (new_state) {
            case ISOTPSendState::Aborted:
                break;
            case ISOTPSendState::SendingSingleOrFirst:
                break;
            case ISOTPSendState::Ready:
                send_machine.send_message_length = 0;
                send_machine.used_send_buffer_length = 0;
                send_machine.send_message_buffer.fill(0);
                send_machine.separation_time_min_ms = 0;
                send_machine.block_size = 0;
                send_machine.frames_sent_in_block = 0;
                break;
            case ISOTPSendState::WaitingForControl:
                check(send_machine.send_state ==
                              ISOTPSendState::SendingSingleOrFirst ||
                          send_machine.send_state ==
                              ISOTPSendState::SendingConsecutive,
                      "WaitingForControl state can only be reached from "
                      "SendingSingleOrFirst or SendingConsecutive states.");
                break;
            case ISOTPSendState::SendingConsecutive:
                send_machine.frames_sent_in_block = 0;
                break;
        }
        send_machine.send_state = new_state;
    }

    template <typename FSend, typename FDelay>
    void ISOTP<FSend, FDelay>::set_receive_state(ISOTPReceiveState new_state) {
        check(new_state != ISOTPReceiveState::AssemblingMessage ||
                  (new_state == ISOTPReceiveState::AssemblingMessage &&
                   (receive_machine.receive_state == ISOTPReceiveState::Ready ||
                    receive_machine.receive_state ==
                        ISOTPReceiveState::HaveFullMessage)),
              "Attempt to assemble new message when previous message wasn't "
              "finished assembling");
        receive_machine.receive_state = new_state;
        switch (new_state) {
            case ISOTPReceiveState::Ready:
                receive_machine.receive_message_buffer.fill(0);
                receive_machine.used_receive_buffer_length = 0;
                receive_machine.receive_message_length = 0;
                receive_machine.receive_indices_buffer.fill(0);
                receive_machine.indice_buffer_index = 0;
                break;
            case ISOTPReceiveState::AssemblingMessage:
                break;
            case ISOTPReceiveState::HaveFullMessage:
                // TODO: Check indices if multi-frame message for errors.
                check(receive_machine.used_receive_buffer_length ==
                          receive_machine.receive_message_length,
                      "Size of received message bytes should have the same "
                      "length as declared length.");
                break;
            default:
                break;
        }
    }

    template <typename FSend, typename FDelay>
    void ISOTP<FSend, FDelay>::check(bool cond, const char* msg) {
        if (config == ISOTPConfig::FailFast) {
            ASSERT(cond, msg);
        } else {
            // TODO: Implement robust error handling.
        }
    }
}  // namespace isotp