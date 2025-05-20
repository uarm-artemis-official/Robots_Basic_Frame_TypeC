/*******************************************************************************
* @file           : Timer_App.c
* @brief          : A software timer task to register different periodical task.
* @created time	  : Dec, 2020
* @creator        : AzureRin
*
* @restructed     : Jul, 2023
* @maintainer     : Haoran
******************************************************************************
* Copyright (c) 2023 UARM Artemis.
* All rights reserved.
*******************************************************************************/

#ifndef SRC_APPLICATIONS_TIMER_APP_H_
#define SRC_APPLICATIONS_TIMER_APP_H_

#include "apps_defines.h"
#include "apps_interfaces.h"
#include "apps_types.h"

class TimerApp : public RTOSApp<TimerApp> {
   private:
    IMotors& system_motors;
    IMessageCenter& message_center;
    IDebug& debug;
    MotorSetMessage_t motor_tx_message;
    BoardStatus_t board_status;

   public:
    static constexpr uint32_t LOOP_PERIOD_MS = TIMER_TASK_EXEC_TIME;

    TimerApp(IMotors& system_motors_ref, IMessageCenter& message_center_ref,
             IDebug& debug_ref);
    void init();
    void loop();
};

#endif /* SRC_APPLICATIONS_TIMER_APP_H_ */