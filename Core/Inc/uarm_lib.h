#ifndef __UARM_LIB_H
#define __UARM_LIB_H

#ifdef GTEST
    #include "assert.h"
    #include "stdlib.h"

    #define ASSERT(cond, msg) if (!(cond)) assert(0 && (msg))
    #define MALLOC malloc
    #define FREE free
#else
    #include "error_handler.h"
    #include "FreeRTOS.h"

    #define ASSERT(cond, msg) if (!(cond)) error_handler((msg))
    #define MALLOC pvPortMalloc
    #define FREE vPortFree
#endif


#endif