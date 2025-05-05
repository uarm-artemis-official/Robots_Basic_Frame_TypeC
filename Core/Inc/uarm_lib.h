#ifndef __UARM_LIB_H
#define __UARM_LIB_H

#ifdef GTEST
    #include "stdint.h"
    #include "assert.h"
    #include "stdlib.h"

    #define ASSERT(cond, msg) if (!(cond)) assert(0 && (msg))
    #define MALLOC malloc
    #define FREE free
#else
    #include "stm32f407xx.h"
    #include "error_handler.h"
    #include "FreeRTOS.h"

    #define ASSERT(cond, msg) if (!(cond)) error_handler((msg))
    #define MALLOC pvPortMalloc
    #define FREE vPortFree
#endif


#endif