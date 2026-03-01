#ifndef __COMM_APP_FIXTURE_HPP
#define __COMM_APP_FIXTURE_HPP

#include <gtest/gtest.h>
#include "../fakes/fake_message_center.hpp"
#include "../mocks/middleware_mocks.hpp"
#include "apps_classes.hpp"

// WIP.
class CommAppFixture : public ::testing::Test {
   protected:
    MW_RTOS::IRTOS* rtos;
    mc2::RobotMC* mc;
    // comm::CANComm<>* can_comm;  // Removed - CANComm depends on can2_tp
    MW_CAN::ICAN* can;
    isr::can::CAN_ISR* can_isr;
    comm::UARTComm<>* uart_comm;
    isr::uart::UART_ISR* uart_isr;
    CommApp::v2::CommApp* comm_app;

    void SetUp() override {}

    void TearDown() override {}
};

#endif