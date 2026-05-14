#include <gtest/gtest.h>

#include "apps_classes.hpp"
#include "fake_chassis_drive.hpp"
#include "messages.hpp"
#include "middleware_mocks.hpp"

using namespace testing;

template class ChassisApp<FakeChassisDrive>;

class ChassisAppTest : public Test {
   protected:
    MockIRTOS rtos;
    MockICAN can;
    MockIUART uart;
    MockIGPIO gpio;

    mc2::RobotMC mc;
    simple_comm::SimpleCommCodec codec;
    comm::Communication<mc2::RobotMC, mc2::RobotMC::Topics> communication;

    modules::debug::Debug debug;
    FakeChassisDrive fake_drive;

    ChassisApp<FakeChassisDrive> chassis_app;

    ChassisAppTest()
        : mc(rtos),
          communication(mc, codec, can, uart),
          debug(gpio, uart),
          chassis_app(rtos, fake_drive, mc, communication, debug) {

        // Setup default board mode as Chassis
        EXPECT_CALL(gpio, read_pin(MW_GPIO::Port::PORT_F, MW_GPIO::Pin::PIN_1))
            .WillRepeatedly(Return(MW_GPIO::State::LOW));

        communication.set_node_id(simple_comm::NodeID::Chassis);
    }

    void SetUp() override {
        mc.init();
        communication.init();
        debug.init();
    }
};

TEST_F(ChassisAppTest, InitializationAndDefaultState) {
    chassis_app.init();

    Chassis_t state = chassis_app.get_chassis_state();
    EXPECT_EQ(state.chassis_mode, IDLE_MODE);
    EXPECT_EQ(state.chassis_act_mode, INDPET_MODE);
    EXPECT_EQ(state.chassis_gear_mode, AUTO_GEAR);

    EXPECT_FLOAT_EQ(state.vx, 0.0f);
    EXPECT_FLOAT_EQ(state.vy, 0.0f);
    EXPECT_FLOAT_EQ(state.wz, 0.0f);
}
