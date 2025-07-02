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

void init_uart_isr(UART_Config_t config);

#ifdef __cplusplus
}
#endif

#endif