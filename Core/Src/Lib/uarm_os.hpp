#ifndef __UARM_OS_HPP
#define __UARM_OS_HPP

#ifdef GTEST
#include "uarm_types.hpp"

typedef uint32_t TickType_t;
typedef long BaseType_t;

#define pdTRUE 1
#define pdFALSE 0

#define pdMS_TO_TICKS(x) ((x) * 1000)
#define xTaskGetTickCount() 0
#define vTaskDelayUntil(x, y)
#define vTaskDelay(x)

#else
#include "cmsis_os.h"
#endif

#endif