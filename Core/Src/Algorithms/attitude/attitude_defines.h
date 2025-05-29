#ifndef __ATTITUDE_RESOLUTION_DEFINES_H
#define __ATTITUDE_RESOLUTION_DEFINES_H

/* =========================================================================
 * ANGLE PROCESS DEFINES 
 * ====================================================================== */
#define ACCEL_WEIGHT \
    0.01f  // Define the weight of the accelerometer data, adjust according to the actual situation
#define GYRO_WEIGHT \
    (1.0f -         \
     ACCEL_WEIGHT)  // Define the weight of the gyroscope data, adjust according to the actual situation

// TODO: Investigate this sampling dt
#define SAMPLE_DT \
    0.0001f  // Define the sampling period, adjust according to the actual situation

/* =========================================================================
 * MADGWICK DEFINES 
 * ====================================================================== */
#define sampleFreq 500.0f  // sample frequency in Hz
#define betaDef 0.1f       // 2 * proportional gain 0.1

/* =========================================================================
 * MAHONY DEFINES 
 * ====================================================================== */
// #define sampleFreq 1000.0f        // sample frequency in Hz
#define twoKpDef (2.0f * 0.5f)    // 2 * proportional gain
#define twoKiDef (2.0f * 0.001f)  // 2 * integral gain

#endif