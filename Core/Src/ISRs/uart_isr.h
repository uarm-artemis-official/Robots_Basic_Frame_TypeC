#ifndef __UART_ISR_H
#define __UART_ISR_H

#ifdef __cplusplus
extern "C" {
#endif


#define DBUS_BUFFER_LEN 18

typedef enum {
    CHASSIS,
    GIMBAL,
    UART_NONE,
} UART_Config_t;


#ifdef __cplusplus
}
#endif

#endif