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

#ifdef __cplusplus
extern "C" {
#endif

void Timer_Task_Func(void const* argument);

#ifdef __cplusplus
}
#endif

#endif /* SRC_APPLICATIONS_TIMER_APP_H_ */