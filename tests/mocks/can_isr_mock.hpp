#ifndef __CAN_ISR_MOCK_HPP
#define __CAN_ISR_MOCK_HPP

#include <gmock/gmock.h>
#include "can_isr.hpp"

class MockCANISR : public isr::can::CAN_ISR {
   public:
    explicit MockCANISR(MW_CAN::ICAN& can_ref) : isr::can::CAN_ISR(can_ref) {}

    MOCK_METHOD(bool, init, (), (override));
    MOCK_METHOD(bool, on_register_init, (size_t init_func_idx), (override));
    MOCK_METHOD(bool, on_register_routine, (size_t routine_func_idx),
                (override));
    MOCK_METHOD(void, run_isr_routines,
                (isr::can::ECallbacks callback_running,
                 isr::can::TISRState callback_state),
                (override));
};

#endif