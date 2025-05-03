#ifndef __UARM_LIB_H
#define __UARM_LIB_H

#ifdef GTEST
    #include "assert.h"

    #define ASSERT(cond, msg) if (!(cond)) assert(0 && (msg))
#else
    #include "error_handler.h"

    #define ASSERT(cond, msg) if (!(cond)) error_handler((msg))
#endif


#endif