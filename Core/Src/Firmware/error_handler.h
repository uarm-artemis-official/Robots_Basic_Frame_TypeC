/*
 * error_handler.h
 *
 *  Created on: Mar 19, 2025
 *      Author: johnm
 */

#ifndef SRC_ERROR_HANDLER_H_
#define SRC_ERROR_HANDLER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include "ssd1306_driver.h"
#include "dwt.h"


#define ASSERT(cond, msg) if (!(cond)) error_handler((msg))

void error_handler(const char *msg);

#ifdef __cplusplus
}
#endif

#endif /* SRC_ERROR_HANDLER_H_ */
