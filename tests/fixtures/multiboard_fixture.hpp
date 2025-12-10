#ifndef __MULTIBOARD_FIXTURE_HPP
#define __MULTIBOARD_FIXTURE_HPP

#include <gtest/gtest.h>
#include "apps_classes.hpp"
#include "can_isr.hpp"
#include "communication.hpp"
#include "fake_message_center.hpp"
#include "middleware_interfaces.hpp"
#include "middleware_mocks.hpp"
#include "subsystems_interfaces.hpp"
#include "subsystems_mocks.hpp"
#include "topics.hpp"

class MultiboardCAN2Test : public testing::Test {
   protected:
    MockICAN *top_can, *bottom_can;
    MockIRTOS* rtos;
    FakeMessageCenter<mc2::RobotTopics>* message_center;
    IDebug* debug;
    isr::can::CAN_ISR *top_can_isr, *bottom_can_isr;
    CommApp::CommApp *top_comm_app, *bottom_comm_app;

    void SetUp() override {
        // Create mocks and fakes
        rtos = new MockIRTOS();
        top_can = new MockICAN();
        bottom_can = new MockICAN();
        debug = new MockDebug();
        top_can_isr = new isr::can::CAN_ISR(*top_can);
        bottom_can_isr = new isr::can::CAN_ISR(*bottom_can);
        message_center = new FakeMessageCenter<mc2::RobotTopics>(*rtos);

        // Create applications
        bottom_comm_app = new CommApp::CommApp(
            *rtos, *message_center, *debug, *bottom_can,
            CommApp::Config {CommApp::OperationMode::Normal}, *bottom_can_isr);
        top_comm_app = new CommApp::CommApp(
            *rtos, *message_center, *debug, *top_can,
            CommApp::Config {CommApp::OperationMode::Normal}, *top_can_isr);

        // Initialize applications
        // top_comm_app->init();
        // bottom_comm_app->init();
    }

    void TearDown() override {
        delete bottom_can;
        delete top_can;
        delete rtos;
        delete debug;
    }
};

#endif