/* USER CODE BEGIN Header */
/*
 ************************************************************************************************
 * 	  	    		2023 UARM ELECTRICAL&CONTROL TEAM ROBOTS BASIC FRAME TTPE-C     		     *
 ************************************************************************************************
 * 							   Project Copyright (c) 2023 UARM Artemis.
 *
 *
 *                                         .]]]@@@@@@@@@@@@\]`.
 *                                   ,]@@@@[[..            .,[\@@@\].
 *                               ,/@@[`                           .[\@@].
 *                         ,/\]@@/.                                    ,\@\`
 *                       ,@@@@/.                                          ,\@\..`
 *                     /@@@/`           .@@^                                 ,@@`\`
 *                   ,@@@/.          \@/                                       ,@@`\`
 *                 ,@@@@.         @@/        ]]]]]@@@@@@@@@@@@@@.                ,@\.\
 *                  /@`        =@@.    ]]]]]]@@@@@@@@@@@/[[[[[[[                  .\@`=`
 *                .@@.       ]]...     @@       =@@@@.=@^                           ,@\/.
 *               .@/        .\@`       @@       ..... =@^                            ,@\
 *              .@/       .@@^         @@@@^          =@^                             ,@\
 *            =[@@.      ,].           [@@@@@@@@@.    =@\...,@@@@@@@@@@@@@@.           ,@^
 *           ,^=@`       \@^           .@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@.            =@`
 *           /.@^       /@^            .@@@@@@@@@@@@@@@                     .       .`  .@\
 *          .[\@.       ,[..]]]]]]`    .@@@@@@@@@@@@@@@                   ,@@        @.  =@^
 *            @/           =@\]].     @@@@@@@@@@@@@@@@@@\.=@@@./@@@@\`  ,@@@@        @^  .@^
 *           ,@^           ,[@@@^   =@/[[[[[[[@@@@@@@@@=@\/@@@.@@@@@@^,@@``@@       .@@   @@@^
 *           =@^            /@=@^ ,`=@^       @@\]]]]]]@@@@@@@.@@@@@@^O`/@^@@       /@@   \@@^
 *           =@^           /@.,[` =^=@^       @@@@@/@@@@@@@@^]]]]]]]]]]]]]`@@      /@@@   =@@^
 *           =@^          =@.@@@^ =\=@^       .]/@O.[\@@@@@@/[[[[[[[[[[[[[[[[    .@@@@@   @@@^
 *           .@^         .@^/@@@\]...=@@@@@@\@[]@\`\@=@.      .,]]].             =@@@@^  .@@@`
 *            \@.        .@@@[....,@\.@@/[\@@^@@@@@@=@@@@@@@@@[....,\@.          @@@@@`  =@@@
 *            ,@^        .@^,@`  ,@`,@\@` =@@^\@@@@^/@@@@@@@/.@[. ,\\.@`        =@@@@/   /@@^
 *             \@.       =@ @.=@@^ @.\^   =@^,@\]]/@`,[[[[[@`=^.@@^ =^=^       ,@@@@@.  =@^
 *            \.@\       .@^=\.  .//.@`   =@@@@@@@@@@      \^,@`   ,@.@^      ,@@@@@`  .@/
 *             \,@^       .@\.,[[`.]@`                      \\`,[[[.,@`      =@@@@@`  .@@
 *             .\,@\        .,\@@/[.                          ,[@@@[.      ./@@@@@`  .@@.
 *              .\,@\.   ..                                               /@@@@@/.  ,@/.
 *                [.@@`   ,\.                                          ./@@@@@@`   =@^
 *                   =@\.  .\@].                                     ,@.,@@@@/   .@@`
 *                    .\@`   .\@@@@@@@\`.                        .]@@@@\.=@`   ./@`
 *                      .\@\.   ,@@@@@@@@@@\`.             ..]/@@@@@@@@@/.   ./@[
 *                       ,@@@\`    ,\@@@@@@@@@@@@@@@@@@@@@@@@@  \@@@@[.    ]@@`=`
 *                         [@@@@\`     .[\@@@@@@@@@@@@@@@@@@@@@@/[      ]@@[./`
 *                            [@@@@@\`        .[[[[[[[[[[[`        .]/@@[\/`
 *                               ,\..[@@@\]`                 .]]@@@/`.
 *                                       .,[\@@@@@@@@@@@@@@@@@@@@@^
 *                                                .@@@@@@@[[[.
 *
 *           /@@@@@\     =@@@@@O@@@@@@@@@@@@@@@@@@^@@@@@@@@@@@@@@@@@].,/@@@@@@@@@@@@@@@@@@@]`.
 *           @@@@@@@     =@@@@@ @@@@@@@@@@@@@@@@@@^@@@@@@@@@@@@@@@@@ @@@@@@@@@@@@@@@@@@@@@@@@^
 *           @@@@@@@     =@@@@@ @@@@@@       @@@@@^@@@@*      =@@@@@ @@@@@@[[[[[\@@/[[[[\@@@@^
 *          @@@@@@@     =@@@@@@ @@@@@]]]]]]]@@@@@^@@@@\]]]]]]/@@@@@@ @@@@@     =@@^    =@@@@^
 *           @@@@@@@     =@@@@@ @@@@@@@@@@@@@@@@@@^@@@@@@@@@@@@@@@@@ @@@@@@     =@@^    =@@@@^
 *           @@@@@@@     =@@@@@ @@@@@@@@@@@@@@@@@@^@@@@@@@@@@@@@@@/@ @@@@@@     =@@^    =@@@@^
 *           @@@@@@@@@@@@@@@@@@ @@@@@@       @@@@@^@@@@^      =@@@@@@@@@@@@     =@@^    =@@@@^
 *          @@@@@@@@@@@@@@@@@@@ @@@@@       @@@@@^@@@@^      =@@@@@@@@@@@     =@@^    =@@@@^
 *
 *
 *
 *	@time 		Nov, 2023
 *	@version 	v1.0(Alpha test)
 *
 *
 * @attention
 * Code Frame Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 * @attention
 * if you regenerate the code using cubemx, make sure:
 * 	1. set #define INCLUDE_vTaskDelayUntil 1 in the FreeRTOSConfig.h
 * 	2. delete the generated SPI NSS GPIO part in spi.c
 * 	3. delete the generated MX_IWDG_Init().(if you want to deactivate IWDG, just
 * 	   go to main.h and comment #define USE_IWDG 1)
 *
 * 	ENJOY!
 *
 * */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "cmsis_os.h"
#include "dma.h"
#include "gpio.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"
#include "ammo_lid.hpp"
#include "can_comm.hpp"
#include "can_isr.hpp"
#include "debug.hpp"
#include "dwt.h"
#include "event_center.hpp"
#include "imu.hpp"
#include "message_center.hpp"
#include "motors.hpp"
#include "pc_comm.hpp"
#include "rc_comm.hpp"
#include "referee_ui.hpp"
#include "robot_config.hpp"
#include "stdio.h"
#include "stm32f407xx.h"
#include "uart_isr.hpp"

#include "Chassis_App.h"
#include "Comm_App.h"
#include "Gimbal_App.h"
#include "IMU_App.h"
#include "Omni_Drive.h"
#include "PC_UART_App.h"
#include "RC_App.hpp"
#include "Referee_App.h"
#include "Shoot_App.h"
#include "Swerve_Drive.h"
#include "Timer_App.h"
#include "WatchDog_App.h"

// Function signature so main.c can find main_cpp().
extern "C" {
void main_cpp(void);
}

// TODO Remove
uint8_t ref_rx_frame[256] = {0};  //referee temp frame buffer

static MessageCenter& message_center = MessageCenter::get_instance();
static EventCenter event_center;
static Debug debug;
static CanComm can_comm;
static Motors motors;
static RefereeUI ref_ui;
static Motors no_init_motors;
static Imu imu(1000 / IMUApp::LOOP_PERIOD_MS, 0.4,
               robot_config::gimbal_params::IMU_ORIENTATION);
static AmmoLid ammo_lid;
static RCComm rc_comm;
static PCComm pc_comm;

// TODO Make all parameters injectable via struct instead of apps including robot_config.hpp
#ifdef SWERVE_CHASSIS
static constexpr float swerve_chassis_width = 0.352728f;
static constexpr float swerve_dt =
    ChassisApp<SwerveDrive>::LOOP_PERIOD_MS * 0.001;
static SwerveDrive swerve_drive(message_center, no_init_motors,
                                swerve_chassis_width, swerve_dt);
static ChassisApp<SwerveDrive> chassis_app(swerve_drive, message_center, debug);
#else

#ifdef OMNI_CHASSIS
static constexpr float omni_chassis_width = 0.40f;
static OmniDrive omni_drive(message_center, no_init_motors, omni_chassis_width,
                            omni_chassis_width, 80,
                            ChassisApp<OmniDrive>::LOOP_PERIOD_MS * 0.001);
#else
static constexpr float mecanum_chassis_width = 0.41f;
static constexpr float mecanum_chassis_length = 0.35f;
static OmniDrive omni_drive(message_center, no_init_motors,
                            mecanum_chassis_width, mecanum_chassis_length, 50,
                            ChassisApp<OmniDrive>::LOOP_PERIOD_MS * 0.001);
#endif

static ChassisApp<OmniDrive> chassis_app(omni_drive, message_center, debug);
#endif

static RCApp rc_app(message_center, rc_comm);
static CommApp comm_app(message_center, debug, can_comm);
static TimerApp timer_app(motors, message_center, debug);
static PCUARTApp pc_uart_app(message_center, no_init_motors, pc_comm);
static IMUApp imu_app(message_center, event_center, imu, debug);
static RefereeApp referee_app(message_center, event_center, debug, ref_ui);
static GimbalApp gimbal_app(message_center, event_center, debug,
                            no_init_motors);
static ShootApp shoot_app(
    message_center, ammo_lid, no_init_motors,
    robot_config::shoot_params::LOADER_ACTIVE_RPM,
    robot_config::shoot_params::FLYWHEEL_ACTIVE_TARGET_RPM,
    robot_config::shoot_params::MAX_FLYWHEEL_ACCEL);

void main_cpp(void) {
    message_center.init();
    can_comm.init();
    event_center.init();

    HAL_GPIO_WritePin(LED_Green_GPIO_Port, LED_Green_Pin,
                      GPIO_PIN_RESET);  // turn off the green led
    if (firmware_and_system_init() != HAL_OK) {
        Error_Handler();
    } else {
        HAL_GPIO_WritePin(LED_Green_GPIO_Port, LED_Green_Pin,
                          GPIO_PIN_SET);  // turn on the green led
    }

    BoardStatus_t board_status = debug.get_board_status();

    osThreadDef(
        TimerTask, [](const void* arg) { timer_app.run(arg); }, osPriorityHigh,
        0, 256);
    osThreadCreate(osThread(TimerTask), NULL);

    osThreadDef(
        CommTask, [](const void* arg) { comm_app.run(arg); }, osPriorityHigh, 0,
        256);
    osThreadCreate(osThread(CommTask), NULL);

    if (board_status == CHASSIS_BOARD) {
        osThreadDef(
            ChassisTask, [](const void* arg) { chassis_app.run(arg); },
            osPriorityHigh, 0, 256);
        osThreadCreate(osThread(ChassisTask), NULL);

        osThreadDef(
            RCTask, [](const void* arg) { rc_app.run(arg); }, osPriorityHigh, 0,
            384);
        osThreadCreate(osThread(RCTask), NULL);

        osThreadDef(
            RefTask, [](const void* arg) { referee_app.run(arg); },
            osPriorityHigh, 0, 384);
        osThreadCreate(osThread(RefTask), NULL);

    } else if (board_status == GIMBAL_BOARD) {
        osThreadDef(
            GimbalTask, [](const void* arg) { gimbal_app.run(arg); },
            osPriorityRealtime, 0, 512);
        osThreadCreate(osThread(GimbalTask), NULL);

        osThreadDef(
            ShootTask, [](const void* arg) { shoot_app.run(arg); },
            osPriorityHigh, 0, 256);
        osThreadCreate(osThread(ShootTask), NULL);

        osThreadDef(
            IMUTask, [](const void* arg) { imu_app.run(arg); },
            osPriorityRealtime, 0, 256);
        osThreadCreate(osThread(IMUTask), NULL);

        osThreadDef(
            PCUARTTask, [](const void* arg) { pc_uart_app.run(arg); },
            osPriorityHigh, 0, 256);
        osThreadCreate(osThread(PCUARTTask), NULL);
    }
}

HAL_StatusTypeDef firmware_and_system_init(void) {
    /* CAN1 & CAN2 Init */
    if (HAL_CAN_Start(&hcan1) != HAL_OK) {
        return HAL_ERROR;
    }
    if (HAL_CAN_Start(&hcan2) != HAL_OK) {
        return HAL_ERROR;
    }
    /* CAN1 & CAN2 filter Init */
    can_filter_enable(&hcan1);
    can_filter_enable(&hcan2);

    /* Timer 13 IT Init */
    if (HAL_TIM_Base_Start_IT(&htim13) != HAL_OK) {
        return HAL_ERROR;
    }
    /* Heat PWM signal Init */
    if (HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1) != HAL_OK) {
        return HAL_ERROR;
    }
    // referee_init(&referee);
    dwt_init();

    UART_Config_t config;
    if (debug.get_board_status() == CHASSIS_BOARD) {
        config = CHASSIS;
    } else {
        config = GIMBAL;
    }
    init_uart_isr(config);

#ifdef SWERVE_CHASSIS
    constexpr CAN_ISR_Config can_config = CAN_ISR_Config::SWERVE;
#else
    constexpr CAN_ISR_Config can_config = CAN_ISR_Config::NORMAL;
#endif
    init_can_isr(can_config);

    return HAL_OK;
}
/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM5 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
    /* USER CODE BEGIN Callback 0 */
    /* USER CODE END Callback 0 */
    if (htim->Instance == TIM5) {
        HAL_IncTick();
    }
    /* USER CODE BEGIN Callback 1 */

    /* USER CODE END Callback 1 */
}
