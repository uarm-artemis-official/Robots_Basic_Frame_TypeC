/*******************************************************************************
* @file           : Comm_App.h
* @brief          : communication real time task between boards
* @created time	  : Jul, 2023
* @author         : Haoran
*
******************************************************************************
* Copyright (c) 2023 UARM Artemis.
* All rights reserved.
*******************************************************************************/

#ifndef __COMM_APP_H__
#define __COMM_APP_H__

#include "apps_defines.h"
#include "apps_interfaces.h"
#include "apps_types.hpp"

namespace CommApp {
    class CommApp : public RTOSApp<CommApp> {
       private:
        IMessageCenter& message_center;
        IDebug& debug;
        ICanComm& can_comm;
        BoardStatus_t board_status;
        Config config;

       public:
        static constexpr const uint32_t LOOP_PERIOD_MS = COMM_TASK_EXEC_TIME;

        CommApp(IMessageCenter& message_center, IDebug& debug,
                ICanComm& can_comm, Config config);
        void loop();
        void init();
    };
}  // namespace CommApp

#endif /*__COMM_APP_H__*/
