#ifndef __UARM_LIB_H
#define __UARM_LIB_H

#include "uarm_types.h"

// TODO: Make type safe memcpy/memset?
#ifdef GTEST
#include "assert.h"
#include "stdlib.h"

#define ASSERT(cond, msg) \
    if (!(cond))          \
    assert(0 && (msg))
#define MALLOC(size) malloc(size)
#define FREE(ptr) free(ptr)
#else
#include "FreeRTOS.h"
#include "error_handler.h"

#define ASSERT(cond, msg) \
    if (!(cond))          \
    error_handler((msg))
#define MALLOC(size) pvPortMalloc(size)
#define FREE(ptr) vPortFree(ptr)
#endif

#endif