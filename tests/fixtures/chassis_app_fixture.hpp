#ifndef __CHASSIS_APP_FIXTURE_HPP
#define __CHASSIS_APP_FIXTURE_HPP

#include <gtest/gtest.h>
#include "Chassis_App.h"
#include "Omni_Drive.h"
#include "Swerve_Drive.h"
#include "subsystems_mocks.hpp"

class ChassisAppTest : public testing::Test {
   protected:
    MockMessageCenter message_center;
    MockMotors motors;
    MockDebug debug;
    OmniDrive* omni_drive;
    SwerveDrive* swerve_drive;
    ChassisApp<OmniDrive>* omni_chassis;
    ChassisApp<SwerveDrive>* swerve_chassis;

    virtual void SetUp() {
        omni_drive =
            new OmniDrive(message_center, motors, 1.0f, 1.0f, 50.0f, 0.005f);
        swerve_drive = new SwerveDrive(message_center, motors, 1.0f, 0.005f);
        omni_chassis =
            new ChassisApp<OmniDrive>(*omni_drive, message_center, debug);
        swerve_chassis =
            new ChassisApp<SwerveDrive>(*swerve_drive, message_center, debug);
    }
};

#endif