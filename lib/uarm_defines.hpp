#ifndef __UARM_DEFINES_H
#define __UARM_DEFINES_H

#include <cstddef>

/* =========================================================================
 * UARM MATH DEFINES
 * ====================================================================== */
constexpr float pi = 3.14159265358979f;

constexpr float degrees_to_radians(float degrees) {
    return degrees * 0.01745329252f;
}

constexpr float radians_to_degrees(float radians) {
    return radians * 57.295779513f;
}

constexpr size_t max_window_size = 300;

#endif