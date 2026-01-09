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
#include "apps_classes.hpp"
#include "apps_types.hpp"
#include "can_isr.hpp"
#include "debug.hpp"
#include "dji_typec_middleware.cpp"
#include "dwt.h"
#include "message_center.hpp"
#include "middleware_classes.hpp"
#include "robot_config.hpp"
#include "stdio.h"
#include "stm32f407xx.h"
#include "subsystems_classes.hpp"
#include "subsystems_modules.hpp"
#include "uart_isr.hpp"

// TODO: Remove
#define Servo_Motor_Pin_Pin GPIO_PIN_6
#define Servo_Motor_Pin_GPIO_Port GPIOI
#define Board_Status_Pin GPIO_PIN_1
#define Board_Status_GPIO_Port GPIOF
#define RSTN_IST8310_Pin GPIO_PIN_6
#define RSTN_IST8310_GPIO_Port GPIOG
#define IMU_Heat_Pin_Pin GPIO_PIN_6
#define IMU_Heat_Pin_GPIO_Port GPIOF
#define LED_Red_Pin GPIO_PIN_12
#define LED_Red_GPIO_Port GPIOH
#define DRDY_IST8310_Pin GPIO_PIN_3
#define DRDY_IST8310_GPIO_Port GPIOG
#define LED_Green_Pin GPIO_PIN_11
#define LED_Green_GPIO_Port GPIOH
#define LED_Blue_Pin GPIO_PIN_10
#define LED_Blue_GPIO_Port GPIOH
#define Buzzer_Pin_Pin GPIO_PIN_14
#define Buzzer_Pin_GPIO_Port GPIOD
#define CS1_ACCEL_Pin GPIO_PIN_4
#define CS1_ACCEL_GPIO_Port GPIOA
#define INT1_ACCEL_Pin GPIO_PIN_4
#define INT1_ACCEL_GPIO_Port GPIOC
#define INT1_ACCEL_EXTI_IRQn EXTI4_IRQn
#define SOFTWARE_EXTI_Pin GPIO_PIN_0
#define SOFTWARE_EXTI_GPIO_Port GPIOG
#define SOFTWARE_EXTI_EXTI_IRQn EXTI0_IRQn
#define INT1_GYRO_Pin GPIO_PIN_5
#define INT1_GYRO_GPIO_Port GPIOC
#define INT1_GYRO_EXTI_IRQn EXTI9_5_IRQn
#define CS1_GYRO_Pin GPIO_PIN_0
#define CS1_GYRO_GPIO_Port GPIOB
#define HIGH_VOLT_Pin GPIO_PIN_15
#define HIGH_VOLT_GPIO_Port GPIOB

static MW_RTOS::RTOS rtos;
static MW_TIM::PWM pwm;
static MW_UART::UART uart;
static MW_GPIO::GPIO gpio;

static MW_CAN::CAN can;
static mc2::RobotMC mc(rtos);
static EventCenter event_center;
static modules::debug::Debug debug(gpio, uart);
static Motors motors;
static RefereeUI ref_ui(uart);
static Motors
    no_init_motors;  // TODO: Refactor? -> remove or split responsibilities into another module?
static Imu imu(1000 / IMUApp::loop_period_ms, 0.4,
               robot_config::gimbal_params::IMU_ORIENTATION);
static ammo_lid::AmmoLid ammo_lid_(pwm);
static RCComm rc_comm;
static PCComm pc_comm;

static isr::can::CAN_ISR can_isr(can);
static isr::uart::UART_ISR uart_isr(uart);

// TODO Make all parameters injectable via struct instead of apps including robot_config.hpp
#ifdef SWERVE_CHASSIS
static constexpr float swerve_chassis_width = 0.352728f;
static constexpr float swerve_dt = ChassisApp<SwerveDrive>::get_loop_period();
static SwerveDrive swerve_drive(mc, no_init_motors, swerve_chassis_width,
                                swerve_dt);
static ChassisApp<SwerveDrive> chassis_app(rtos, swerve_drive, mc, debug);
#else

#ifdef OMNI_CHASSIS
static constexpr float omni_chassis_width = 0.40f;
static OmniDrive omni_drive(mc, no_init_motors, omni_chassis_width,
                            omni_chassis_width, 80,
                            ChassisApp<OmniDrive>::get_loop_period());
#else
static constexpr float mecanum_chassis_width = 0.41f;
static constexpr float mecanum_chassis_length = 0.35f;
static OmniDrive omni_drive(mc, no_init_motors, mecanum_chassis_width,
                            mecanum_chassis_length, 50,
                            ChassisApp<OmniDrive>::get_loop_period());
#endif

static ChassisApp<OmniDrive> chassis_app(rtos, omni_drive, mc, debug);
#endif

static RCApp rc_app(rtos, mc, rc_comm, uart_isr);

// TODO: Add loopback mode for auto-aim jig.
#ifdef AUTO_AIM_RIG
static CommApp::Config comm_config = {CommApp::OperationMode::Loopback};
#else
static CommApp::Config comm_config = {CommApp::OperationMode::Normal};
#endif

#ifdef OLD_COMM_APP
static CommApp::CommApp comm_app(rtos, mc, debug, can, comm_config, can_isr);
#else
static simple_comm::SimpleComm<CommApp::v3::MAX_SIMPLE_COMM_FX_FIFO_SIZE>
    _simple_comm;
static CommApp::v3::CommApp comm_app(rtos, can_isr, uart_isr, _simple_comm, can,
                                     uart, mc, debug);
#endif

static TimerApp timer_app(rtos, motors, mc, debug, can_isr);
static PCUARTApp pc_uart_app(rtos, mc, no_init_motors, pc_comm, uart_isr);
static IMUApp imu_app(rtos, mc, event_center, imu, debug);
static RefereeApp referee_app(rtos, mc, event_center, debug, ref_ui, uart_isr);
static GimbalApp gimbal_app(rtos, mc, event_center, no_init_motors, debug);
static ShootApp shoot_app(
    rtos, mc, ammo_lid_, no_init_motors,
    robot_config::shoot_params::LOADER_ACTIVE_RPM,
    robot_config::shoot_params::FLYWHEEL_ACTIVE_TARGET_RPM,
    robot_config::shoot_params::MAX_FLYWHEEL_ACCEL);

void init_robot_apps() {
    modules::debug::BoardConfig board_status = debug.get_board_config();

    osThreadDef(
        TimerTask, [](const void* arg) { timer_app.run(arg); }, osPriorityHigh,
        0, 256);
    osThreadCreate(osThread(TimerTask), NULL);

    osThreadDef(
        CommTask, [](const void* arg) { comm_app.run(arg); }, osPriorityHigh, 0,
        256);
    osThreadCreate(osThread(CommTask), NULL);

    if (board_status == modules::debug::BoardConfig::CHASSIS) {
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

    } else if (board_status == modules::debug::BoardConfig::GIMBAL) {
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

void can_filter_enable(CAN_HandleTypeDef* hcan) {
    CAN_FilterTypeDef CAN_FilterConfigStructure;

    CAN_FilterConfigStructure.FilterIdHigh = 0x0000;
    CAN_FilterConfigStructure.FilterIdLow = 0x0000;
    CAN_FilterConfigStructure.FilterMaskIdHigh = 0x0000;
    CAN_FilterConfigStructure.FilterMaskIdLow = 0x0000;
    CAN_FilterConfigStructure.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    CAN_FilterConfigStructure.FilterMode = CAN_FILTERMODE_IDMASK;
    CAN_FilterConfigStructure.FilterScale = CAN_FILTERSCALE_16BIT;
    CAN_FilterConfigStructure.FilterActivation = ENABLE;
    if (hcan == &hcan1) {
        CAN_FilterConfigStructure.FilterBank = 0;
    } else if (hcan == &hcan2) {
        CAN_FilterConfigStructure.SlaveStartFilterBank = 14;
        CAN_FilterConfigStructure.FilterBank = 14;
    }

    HAL_CAN_ConfigFilter(hcan, &CAN_FilterConfigStructure);
    // activate the canx msg callback interrupt
    HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
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

    ASSERT(uart_isr.init(), "UART ISR init failed.");
    ASSERT(can_isr.init(), "CAN ISR init failed.");

    return HAL_OK;
}

// Function signature so main.c can find main_cpp().
extern "C" {
void main_cpp(void);
}

void main_cpp(void) {
    mc.init();
    event_center.init();

    // TODO: Remove
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

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM5 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
    if (htim->Instance == TIM5) {
        HAL_IncTick();
    }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan) {
    MW_CAN::BUS bus;

    // TODO: Refactor in cause of using FIFO1 in the future.
    uint32_t frame_ide =
        CAN_RI0R_IDE & hcan->Instance->sFIFOMailBox[CAN_RX_FIFO0].RIR;
    bool is_extended_id = frame_ide == CAN_ID_EXT;
    if (hcan == &hcan1) {
        bus = is_extended_id ? MW_CAN::BUS::CAN_1B : MW_CAN::BUS::CAN_1;
    } else if (hcan == &hcan2) {
        bus = is_extended_id ? MW_CAN::BUS::CAN_2B : MW_CAN::BUS::CAN_2;
    } else {
        ASSERT(false, "Received message on unknown hcan.");
    }
    can_isr.run_isr_routines(isr::can::ECallbacks::MESSAGE_PENDING, bus);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart) {
    MW_UART::Peripheral peripheral;
    if (huart == &huart1) {
        peripheral = MW_UART::Peripheral::UART1;
    } else if (huart == &huart3) {
        peripheral = MW_UART::Peripheral::UART3;
    } else {
        ASSERT(false, "Receive complete on unknown huart.");
    }

    uart_isr.run_isr_routines(isr::uart::ECallbacks::RECEIVE_COMPLETE,
                              peripheral);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef* huart) {
    MW_UART::Peripheral peripheral;
    if (huart == &huart1) {
        peripheral = MW_UART::Peripheral::UART1;
    } else if (huart == &huart3) {
        peripheral = MW_UART::Peripheral::UART3;
    } else {
        ASSERT(false, "Receive complete on unknown huart.");
    }

    uart_isr.run_isr_routines(isr::uart::ECallbacks::ON_ERROR, peripheral);
}