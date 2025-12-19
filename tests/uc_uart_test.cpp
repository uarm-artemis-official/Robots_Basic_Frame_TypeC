#include <gtest/gtest.h>
#include "uc_uart.hpp"

namespace {
    class UCUARTTest : public ::testing::Test {
       protected:
        static constexpr size_t max_size = 128;
        comm::uc_uart::UCUART<max_size> uart_comm {1};  // source = 1

        // Build a deterministic payload of length n (0..255 repeating)
        static std::vector<uint8_t> BuildPayload(size_t n) {
            std::vector<uint8_t> v(n);
            for (size_t i = 0; i < n; ++i) v[i] = static_cast<uint8_t>(i & 0xFF);
            return v;
        }
    };

    TEST_F(UCUARTTest, UC_UART_Dummy) {
        ASSERT_TRUE(true);
    }
}  // namespace