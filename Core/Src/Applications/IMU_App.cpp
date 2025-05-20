/*******************************************************************************
* @file           : IMU_App.c
* @brief          : imu temp control and mpu get data
* @created time	  : Jul, 2023
* @author         : James
******************************************************************************
* Copyright (c) 2023 UARM Artemis.
* All rights reserved.
*******************************************************************************/
#ifndef __IMU_APP_C__
#define __IMU_APP_C__

#include "IMU_App.h"
#include "apps_defines.h"
#include "buzzer.h"
#include "debug.h"
#include "event_center.h"
#include "imu.h"
#include "message_center.h"
#include "pid.h"
#include "public_defines.h"
#include "string.h"

IMUApp::IMUApp(IMessageCenter& message_center_ref,
               IEventCenter& event_center_ref, IImu& imu_ref, IDebug& debug_ref)
    : message_center(message_center_ref),
      event_center(event_center_ref),
      imu(imu_ref),
      debug(debug_ref) {}

void IMUApp::init() {
    memset(&imu_app_state, 0, sizeof(IMU_t));
    memset(&imu_heating_control, 0, sizeof(IMU_Heat_t));
    memset(&attitude, 0, sizeof(Attitude_t));
    memset(message_data, 0, sizeof(float) * 2);

    imu.init();

    /* init sensor pid */
    //	pid_param_init(&(imu.tmp_pid), 4000, 1500, 25, 1000, 0.1, 1000);
    pid2_init(&(imu_heating_control.pid), 1200, 220, 0, 1, 1, 0, 4000);
    set_imu_temp_status(ABNORMAL);
    imu_app_state.imu_mode = GA_MODE;  // forbid ist8310
    if (imu_app_state.imu_mode == GA_MODE) {
        // no use ist8310
        imu_app_state.ahrs_sensor.mx = 0.0f;
        imu_app_state.ahrs_sensor.my = 0.0f;
        imu_app_state.ahrs_sensor.mz = 0.0f;
    }
    //	imu.sample_time = DWT_Get();
    imu_app_state.temp = 0.0;
    calibrate_imu();
}

void IMUApp::loop() {
    imu_temp_pid_control();
    /* read the mpu data */

    if (imu_app_state.temp_status == NORMAL) {
        imu.get_attitude(&attitude);

        message_data[0] = attitude.yaw;
        message_data[1] = attitude.pitch;
        message_center.pub_message(IMU_READINGS, message_data);
    }
}

void IMUApp::calibrate_imu() {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (imu_app_state.temp_status != NORMAL) {
        imu_app_state.temp = imu.get_temp();
        imu_temp_pid_control();
        vTaskDelayUntil(&xLastWakeTime, IMU_CALI_FREQ);
    }
    imu.set_offset();
}

/**
  * @brief     set IMU temp status
  * @param[in] pimu: main imu sturct
  * @param[in] status: IMU_temp_status enum variable
  * @retval    None
  */
void IMUApp::set_imu_temp_status(IMU_temp_status status) {
    imu_app_state.temp_status = status;

    if (imu_app_state.temp_status == NORMAL) {
        event_center.emit_events(IMU_READY);
    } else {
        event_center.clear_events(IMU_READY);
    }
}

/**
  * @brief  temperature of imu pid control
  * @param[in]: Not used
  * @retval 0
  */
int32_t IMUApp::imu_temp_pid_control() {
    imu_app_state.temp = imu.get_temp();
    float temp_threshold = 0.875f;

    pid2_single_loop_control(&(imu_heating_control.pid), DEFAULT_IMU_TEMP,
                             imu_app_state.temp,
                             IMU_TASK_EXEC_TIME * 0.001f * 100);  // pid control
    imu.set_heat_pwm(imu_heating_control.pid.total_out);

    if (DEFAULT_IMU_TEMP - temp_threshold <= imu_app_state.temp &&
        imu_app_state.temp <= DEFAULT_IMU_TEMP + temp_threshold) {
        debug.set_led_state(RED, OFF);
        set_imu_temp_status(NORMAL);
    } else {
        debug.set_led_state(RED, ON);
        set_imu_temp_status(ABNORMAL);
    }
    return 0;
}

#endif
