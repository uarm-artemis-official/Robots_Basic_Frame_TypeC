#ifndef __CAN_ISR_H
#define __CAN_ISR_H

enum class CAN_ISR_Config { NORMAL, SWERVE };

void init_can_isr(CAN_ISR_Config config);

#endif