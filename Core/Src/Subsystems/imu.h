#ifndef __IMU_H
#define __IMU_H

#include "subsystems_types.h"

void init_imu();
float get_imu_temp();
void get_attitude(Attitude_t *attitude);

void set_imu_offset();
void set_imu_pwm(uint16_t duty_cycle);

#endif