// TODO: Rename file to avoid confusion and rework project structure to hide auto-generated code.
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
#include "can_isr.hpp"
#include "dwt.h"
#include "robot_config.hpp"
#include "stdio.h"
#include "stm32f407xx.h"
#include "subsystems_classes.hpp"
#include "uart_isr.hpp"

#include "apps_types.hpp"

// Function signature so main.c can find main_cpp().
extern "C" {
void main_cpp(void);
}

static MessageCenter& message_center = MessageCenter::get_instance();
static EventCenter event_center;
static Debug debug;
static CanComm can_comm;
static Motors motors;
static RefereeUI ref_ui;
static Motors no_init_motors;
static Imu imu(1000 / IMUApp::loop_period_ms, 0.4,
               robot_config::gimbal_params::IMU_ORIENTATION);
static AmmoLid ammo_lid;
static RCComm rc_comm;
static PCComm pc_comm;

static CAN_ISR::CAN_ISR can_isr(message_center);

// TODO Make all parameters injectable via struct instead of apps including robot_config.hpp
#ifdef SWERVE_CHASSIS
static constexpr float swerve_chassis_width = 0.352728f;
static constexpr float swerve_dt = ChassisApp<SwerveDrive>::get_loop_period();
static SwerveDrive swerve_drive(message_center, no_init_motors,
                                swerve_chassis_width, swerve_dt);
static ChassisApp<SwerveDrive> chassis_app(swerve_drive, message_center, debug);
#else

#ifdef OMNI_CHASSIS
static constexpr float omni_chassis_width = 0.40f;
static OmniDrive omni_drive(message_center, no_init_motors, omni_chassis_width,
                            omni_chassis_width, 80,
                            ChassisApp<OmniDrive>::get_loop_period());
#else
static constexpr float mecanum_chassis_width = 0.41f;
static constexpr float mecanum_chassis_length = 0.35f;
static OmniDrive omni_drive(message_center, no_init_motors,
                            mecanum_chassis_width, mecanum_chassis_length, 50,
                            ChassisApp<OmniDrive>::get_loop_period());
#endif

static ChassisApp<OmniDrive> chassis_app(omni_drive, message_center, debug);
#endif

static RCApp rc_app(message_center, rc_comm);

#ifdef AUTO_AIM_RIG
static CommApp::Config comm_config = {.op_mode =
                                          CommApp::OperationMode::Loopback};
#else
static CommApp::Config comm_config = {.op_mode =
                                          CommApp::OperationMode::Normal};
#endif

static CommApp::CommApp comm_app(message_center, debug, can_comm, comm_config);
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

void init_robot_apps() {
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

void init_auto_aim_apps() {
    osThreadDef(
        GimbalTask, [](const void* arg) { gimbal_app.run(arg); },
        osPriorityRealtime, 0, 512);
    osThreadCreate(osThread(GimbalTask), NULL);

    osThreadDef(
        IMUTask, [](const void* arg) { imu_app.run(arg); }, osPriorityRealtime,
        0, 256);
    osThreadCreate(osThread(IMUTask), NULL);

    osThreadDef(
        PCUARTTask, [](const void* arg) { pc_uart_app.run(arg); },
        osPriorityHigh, 0, 256);
    osThreadCreate(osThread(PCUARTTask), NULL);

    osThreadDef(
        TimerTask, [](const void* arg) { timer_app.run(arg); }, osPriorityHigh,
        0, 256);
    osThreadCreate(osThread(TimerTask), NULL);

    osThreadDef(
        RCTask, [](const void* arg) { rc_app.run(arg); }, osPriorityHigh, 0,
        384);
    osThreadCreate(osThread(RCTask), NULL);

    osThreadDef(
        CommTask, [](const void* arg) { comm_app.run(arg); }, osPriorityHigh, 0,
        256);
    osThreadCreate(osThread(CommTask), NULL);
}

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

    switch (robot_config::config_type) {
        case robot_config::ConfigType::Infantry:
        case robot_config::ConfigType::Hero:
        case robot_config::ConfigType::Sentry:
            init_robot_apps();
            break;
        case robot_config::ConfigType::AutoAim:
            init_auto_aim_apps();
            break;
        default:
            ASSERT(false, "Unrecognized config_type.");
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

    UART_Config_t uart_config;
    CAN_ISR::Config can_isr_config;
    if (debug.get_board_status() == CHASSIS_BOARD) {
        uart_config = CHASSIS;

        if constexpr (robot_config::config_type ==
                      robot_config::ConfigType::Sentry) {
            can_isr_config = CAN_ISR::Config::SENTRY_CHASSIS;
        } else {
            can_isr_config = CAN_ISR::Config::NORMAL;
        }
    } else {
        if constexpr (robot_config::config_type ==
                      robot_config::ConfigType::AutoAim) {
            // Add mode for enabling PC UART and RC for one board.
            uart_config = UART_AUTO_AIM;
        } else {
            uart_config = GIMBAL;
        }
        can_isr_config = CAN_ISR::Config::NORMAL;
    }
    init_uart_isr(uart_config);
    can_isr.init(can_isr_config);

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

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan) {
    can_isr.message_pending(hcan);
}
