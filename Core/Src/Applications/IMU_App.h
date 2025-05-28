/*******************************************************************************
* @file           : IMU_App.h
* @brief          : imu temp control and mpu get data
* @created time	  : Jul, 2023
* @author         : Haoran
******************************************************************************
* Copyright (c) 2023 UARM Artemis.
* All rights reserved.
*******************************************************************************/

#ifndef __SRC_IMU_APP_H__
#define __SRC_IMU_APP_H__

#include "apps_defines.h"
#include "apps_interfaces.h"
#include "apps_types.h"

class IMUApp : public RTOSApp<IMUApp> {
   private:
    IMessageCenter& message_center;
    IEventCenter& event_center;
    IImu& imu;
    IDebug& debug;

    IMU_t imu_app_state;
    IMU_Heat_t imu_heating_control;
    Attitude_t attitude;
    float message_data[2];

   public:
    static constexpr uint32_t LOOP_PERIOD_MS = IMU_TASK_EXEC_TIME;
    static constexpr float TARGET_IMU_TEMP = 40.0f;
    static constexpr float NORMAL_TEMP_THRESHOLD = 1.0f;
    static constexpr float IMU_RESET_THRESHOLD = 7.0f;

    IMUApp(IMessageCenter& message_center_ref, IEventCenter& event_center_ref,
           IImu& imu_ref, IDebug& debug_ref);
    void init();
    void loop();

    void calibrate_imu();
    int32_t imu_temp_pid_control();

    void set_imu_temp_status(IMU_temp_status status);
};

#endif /*__SRC_IMU_APP_H__*/