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


#ifdef __cplusplus
extern "C" {
#endif

#include "attitude_types.h"

#ifdef AHRS_H_GLOBAL
    #define AHRS_H_EXTERN 
#else
    #define AHRS_H_EXTERN extern
#endif

float invSqrt(float x);

void madgwick_ahrs_update(AhrsSensor_t *sensor, Attitude_t *atti);
void madgwick_ahrs_updateIMU(AhrsSensor_t *sensor, Attitude_t *atti);

void mahony_ahrs_update(AhrsSensor_t *sensor, Attitude_t *atti);
void mahony_ahrs_updateIMU(AhrsSensor_t *sensor, Attitude_t *atti);

#ifdef __cplusplus
}
#endif

#endif // __AHRS_H__
