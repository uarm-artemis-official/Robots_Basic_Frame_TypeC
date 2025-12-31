// tests/fakes/fake_uart.hpp
#ifndef __FAKE_UART_HPP
#define __FAKE_UART_HPP

#include <cstdint>
#include <vector>
#include "../../lib/Middleware/middleware_interfaces.hpp"

// Minimal FakeUART for tests. receive_data always returns true.
class FakeUART : public MW_UART::IUART {
   public:
    struct Call {
        MW_UART::Peripheral p;
        uint32_t len;
    };

    std::vector<uint8_t> rx_queue;  // placeholder for future use
    std::vector<Call> calls;

    void enqueue_rx_bytes(const std::vector<uint8_t>& b) {
        rx_queue.insert(rx_queue.end(), b.begin(), b.end());
    }

    void send_data(MW_UART::Peripheral, const uint8_t*, uint32_t,
                   uint32_t) override {}

    bool receive_data(MW_UART::Peripheral p, uint8_t* data,
                      uint32_t length) override {
        calls.push_back({p, length});
        if (rx_queue.size() < length) {
            return false;
        }
        // copy next `length` bytes into provided buffer
        for (uint32_t i = 0; i < length; ++i) {
            data[i] = rx_queue[i];
        }
        // remove consumed bytes
        rx_queue.erase(rx_queue.begin(), rx_queue.begin() + static_cast<std::vector<uint8_t>::difference_type>(length));
        return true;
    }

    void abort_receive(MW_UART::Peripheral) override {}
    void abort_transmit(MW_UART::Peripheral) override {}
    void clear_flags(MW_UART::Peripheral, uint32_t) override {}
};

#endif  // __FAKE_UART_HPP