/****************************************************************************
 *  Copyright (C) 2019 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/

#ifndef __AHRS_H__
#define __AHRS_H__

#ifdef AHRS_H_GLOBAL
    #define AHRS_H_EXTERN 
#else
    #define AHRS_H_EXTERN extern
#endif

typedef struct
{
  float ax;
  float ay;
  float az;

  float wx;
  float wy;
  float wz;

  float mx;
  float my;
  float mz;

  float roll;
  float pitch;
  float yaw;

}AhrsSensor_t;

typedef struct
{
  float roll;
  float pitch;
  float yaw;

  uint32_t timestamp;
}Attitude_t;

float invSqrt(float x);

void madgwick_ahrs_update(AhrsSensor_t *sensor, Attitude_t *atti);
void madgwick_ahrs_updateIMU(AhrsSensor_t *sensor, Attitude_t *atti);

void mahony_ahrs_update(AhrsSensor_t *sensor, Attitude_t *atti);
void mahony_ahrs_updateIMU(AhrsSensor_t *sensor, Attitude_t *atti);

#endif // __AHRS_H__
