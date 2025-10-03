#ifndef __ISOTP_HPP
#define __ISOTP_HPP

#include <array>
#include <cstdint>

namespace isotp {
    constexpr size_t MAX_MESSAGE_LENGTH = 4095;

    /**
     * @brief PCI (Protocol Control Information) codes for ISO-TP frames.
     */
    enum class PCICode : uint8_t {
        SingleFrame = 0x0,
        FirstFrame = 0x1,
        ConsecutiveFrame = 0x2,
        FlowControl = 0x3
    };

    enum class ISOTPSendState { Ready, Sending, WaitingForControl };

    enum class ISOTPReceiveState { Ready, AssemblingMessage, HaveFullMessage };

    /**
     * @brief Configure ISOTPFSM how to determine handle errors.
     * 
     * FailFast will use asserts and immediately hard fault on errors.
     * Robust will ignore/recover from errors.
     */
    enum class ISOTPConfig { FailFast, Robust };

    struct SendMachine {
        ISOTPSendState send_state;
        std::array<uint8_t, MAX_MESSAGE_LENGTH> send_message_buffer;
        size_t send_message_length;
        size_t used_send_buffer_length;
    };

    struct ReceiveMachine {
        ISOTPReceiveState receive_state;
        std::array<uint8_t, MAX_MESSAGE_LENGTH> receive_message_buffer;
        std::array<uint8_t, MAX_MESSAGE_LENGTH / 8 + 1> receive_indices_buffer;
        size_t indice_buffer_index;
        size_t used_receive_buffer_length;
        size_t receive_message_length;
    };

    /**
     * @brief Implementation class for ISO-TP communication.
     * 
     * The protocol is implemented according to its outline in the below
     * webpage.
     * https://ramn.readthedocs.io/en/latest/userguide/isotp_tutorial.html#iso-tp-basics
     * 
     * All frame types (single, first, consecutive, and flow control) are supported.
     */
    template <typename FSend, typename FDelay>
    class ISOTP {
       private:
        FDelay& delay_function;
        FSend& send_function;
        ISOTPConfig config;

        SendMachine send_machine;
        ReceiveMachine receive_machine;

       public:
        ISOTP(FSend& _send_function, FDelay& _delay_function,
              ISOTPConfig _config);

        /**
         * @brief Process received frame.
         * 
         * This method will process frames according to ISO-TP protocol, this 
         * includes control flow frames which is why this class requires a 
         * delay function. 
         * 
         * @note First and single frames will overwrite receive message buffers and 
         * related fields regardless of receive state. This causes a assert error
         * in FailFast mode.
         * 
         * @param message Pointer to the received frame data.
         * @param length Length of the received frame.
         */
        void process_receive_frame(uint8_t* message, size_t length);

        /**
         * @brief Execute one iteration of sending.
         * 
         * This is the main function for sending. Processing will be done to
         * according to the current send_state. At most, during one iteration,
         * a send frame will be sent.
         */
        void tick_send_process();

        /**
         * @brief Set send message for transmission.
         * 
         * This method only sets a new send message and does not send any frames.
         * For sending frames, use tick_send_process().
         * 
         * In FailFast mode, attempting to set a new send message before a previous
         * message is finished sending will trigger an assert error. In Robost mode,
         * this is not the case and the state will be set to send the new message
         * immediately on the next tick.
         * 
         * @param message Pointer to the message data to send.
         * @param length Length of the message to send.
         */
        void set_send_message(uint8_t* message, size_t length);

        /**
         * @brief Reset FSM to initial state.
         * 
         * This will zero all message buffers and related fields along
         * with reset states to initial states.
         */
        void reset();

        /**
         * @brief Tries to retrieve a full message otherwise errors.
         * 
         * This function will only copy message in the FSM's buffer to dst
         * if the message is fully reconstucted. This could be a single frame
         * message or the correct assembly of a multi-frame message.
         * 
         * @param dst Pointer to the destination buffer where the message will be copied.
         * @return true if message is intact and copied to destination, otherwise false.
         */
        [[nodiscard]] bool get_receive_message(void* dst);

        /**
         * @brief Get the current send state of the FSM.
         * 
         * @return The current ISOTPSendState.
         */
        ISOTPSendState get_send_state();

        /**
         * @brief Get the current receive state of the FSM.
         * 
         * @return The current ISOTPReceiveState.
         */
        ISOTPReceiveState get_receive_state();

        /**
         * @brief Set the current send state of the FSM.
         * 
         * This handles transitions between states and setting relevant fields to
         * the correct values for each state. Error checking is also done in this
         * function.
         * 
         * @param state The new ISOTPSendState to set.
         */
        void set_send_state(ISOTPSendState new_state);

        /**
         * @brief Set the current receive state of the FSM.
         * 
         * This handles transitions between states and setting relevant fields to
         * the correct values for each state. Error checking is also done in this
         * function.
         * 
         * @param state The new ISOTPReceiveState to set.
         */
        void set_receive_state(ISOTPReceiveState new_state);

        /**
         * @brief Internal check/assert helper for this class.
         * 
         * @param cond Condition to check.
         * @param msg Message to display on failure.
         */
        void check(bool cond, const char* msg);
    };
}  // namespace isotp

#include "isotp.ipp"

#endif