/*******************************************************************************
* @file           : IMU_App.c
* @brief          : imu temp control and mpu get data
* @created time	  : Jul, 2023
* @author         : James
******************************************************************************
* Copyright (c) 2023 UARM Artemis.
* All rights reserved.
*******************************************************************************/

#include "IMU_App.h"
#include "apps_defines.h"
#include "pid.h"
#include "string.h"
#include "uarm_lib.h"
#include "uarm_math.h"
#include "uarm_os.h"

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

    // TODO: Change prescalar to 1280 or higher and find new pid values.
    // Rationale: temperature data is only updated every 1.28 seconds.
    prescaled_pid2_init(&(imu_heating_control.pid), 500, 800, 80, 260, 1, 0, 0,
                        4000);
    set_imu_temp_status(ABNORMAL);
    imu_app_state.imu_mode = GA_MODE;  // forbid ist8310
    if (imu_app_state.imu_mode == GA_MODE) {
        imu_app_state.ahrs_sensor.mx = 0.0f;
        imu_app_state.ahrs_sensor.my = 0.0f;
        imu_app_state.ahrs_sensor.mz = 0.0f;
    }
    imu_app_state.temp = 0.0;
    imu_app_state.prev_temp = 0.0;
}

void IMUApp::calibrate() {
    imu_app_state.prev_temp = imu_app_state.temp;
    imu_temp_pid_control();
}

bool IMUApp::exit_calibrate_cond() {
    return 39.75 <= imu_app_state.prev_temp &&
           imu_app_state.prev_temp <= 40.25 && 39.75 <= imu_app_state.temp &&
           imu_app_state.temp <= 40.25;
}

void IMUApp::loop() {
    imu_temp_pid_control();

    if (imu_app_state.temp_status == NORMAL) {
        imu.get_attitude(attitude);

        message_data[0] = attitude.yaw;
        message_data[1] = attitude.roll;
        message_center.pub_message(IMU_READINGS, message_data);

        // Sending raw sensor data for calibration.
        // imu.get_sensor_data(sensor_data);
        // uint8_t uc_out_buffer[196];
        // memset(uc_out_buffer, 0, sizeof(uint8_t) * 196);
        // uc_out_buffer[0] = 143;
        // memcpy(uc_out_buffer + 4, &sensor_data, sizeof(AhrsSensor_t));
        // message_center.pub_message(UC_PACK_OUT, uc_out_buffer);
    }
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

    float temp_diff_magnitude =
        std::abs(IMUApp::TARGET_IMU_TEMP - imu_app_state.temp);
    if (temp_diff_magnitude >= IMU_RESET_THRESHOLD) {
        imu_heating_control.pid.pid.i_out = 0;
        imu_heating_control.pid.prescalar_count = 0;
        imu_heating_control.pid.cumsum_dt = 0;
        debug.set_led_state(RED, ON);
        set_imu_temp_status(ABNORMAL);

        if (imu_app_state.temp > IMUApp::TARGET_IMU_TEMP) {
            imu.set_heat_pwm(imu_heating_control.pid.pid.min_out);
        } else {
            imu.set_heat_pwm(imu_heating_control.pid.pid.max_out);
        }
    } else {
        prescaled_pid2_single_loop_control(
            &(imu_heating_control.pid), IMUApp::TARGET_IMU_TEMP,
            imu_app_state.temp,
            IMU_TASK_EXEC_TIME * 0.001f);  // pid control
        imu.set_heat_pwm(imu_heating_control.pid.pid.total_out);

        if (temp_diff_magnitude <= IMUApp::NORMAL_TEMP_THRESHOLD) {
            debug.set_led_state(RED, OFF);
            set_imu_temp_status(NORMAL);
        } else {
            debug.set_led_state(RED, ON);
            set_imu_temp_status(ABNORMAL);
        }
    }

    return 0;
}