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
 * @attention
 * Code Frame Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 * */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"
#include "apps_classes.hpp"
#include "apps_types.hpp"
#include "can_isr.hpp"
#include "communication.hpp"
#include "debug.hpp"
#include "dji_typec_middleware.cpp"
#include "message_center.hpp"
#include "middleware_classes.hpp"
#include "robot_config.hpp"
#include "subsystems_classes.hpp"
#include "subsystems_modules.hpp"
#include "uart_isr.hpp"

static MW_RTOS::RTOS rtos;
static MW_TIM::TIM tim;
static MW_TIM::PWM pwm;
static MW_UART::UART uart;
static MW_GPIO::GPIO gpio;
static MW_SPI::SPI spi;
static MW_I2C::I2C i2c;

static MW_CAN::CAN can;
static mc2::RobotMC mc(rtos);
static EventCenter event_center;
static modules::debug::Debug debug(gpio, uart);
static bmi088_driver::BMI088 bmi088(spi, rtos, gpio, pwm);
static ist8310_driver::IST8310 ist8310(i2c, rtos, gpio);
static Motors motors(can);
static RefereeUI ref_ui(uart);
static Motors no_init_motors(
    can);  // TODO: Refactor? -> remove or split responsibilities into another module?
static Imu imu(bmi088, ist8310, 1000 / IMUApp::loop_period_ms, 0.4,
               robot_config::gimbal_params::IMU_ORIENTATION);
static ammo_lid::AmmoLid ammo_lid_(pwm);
static RCComm rc_comm;

static isr::can::CAN_ISR can_isr(can);
static isr::uart::UART_ISR uart_isr(uart);

static simple_comm::SimpleCommCodec simple_comm_codec;
static comm::Communication<mc2::RobotMC, mc2::RobotMC::Topics> communication(
    mc, simple_comm_codec, can, uart);

// TODO Make all parameters injectable via struct instead of apps including robot_config.hpp
#ifdef SWERVE_CHASSIS
static constexpr float swerve_chassis_width = 0.352728f;
static constexpr float swerve_dt = ChassisApp<SwerveDrive>::get_loop_period();
static SwerveDrive swerve_drive(mc, no_init_motors, swerve_chassis_width,
                                swerve_dt);
static ChassisApp<SwerveDrive> chassis_app(rtos, swerve_drive, mc,
                                           communication, debug);
#else
static constexpr float omni_chassis_width = 0.40f;
static OmniDrive omni_drive(mc, no_init_motors, omni_chassis_width,
                            omni_chassis_width, 80,
                            ChassisApp<OmniDrive>::get_loop_period());

static ChassisApp<OmniDrive> chassis_app(rtos, omni_drive, mc, communication,
                                         debug);
#endif

static RCApp rc_app(rtos, mc, communication, rc_comm, uart_isr);

static TimerApp timer_app(rtos, motors, mc, communication, debug, can_isr);
static IMUApp imu_app(rtos, mc, communication, event_center, imu, debug);
static RefereeApp referee_app(rtos, mc, communication, event_center, debug,
                              ref_ui, uart_isr);
static GimbalApp gimbal_app(rtos, mc, communication, event_center,
                            no_init_motors, debug);
static ShootApp shoot_app(
    rtos, mc, communication, ammo_lid_, no_init_motors,
    robot_config::shoot_params::LOADER_ACTIVE_RPM,
    robot_config::shoot_params::FLYWHEEL_ACTIVE_TARGET_RPM,
    robot_config::shoot_params::MAX_FLYWHEEL_ACCEL);

void init_robot_apps() {
    modules::debug::BoardConfig board_status = debug.get_board_config();

    osThreadDef(
        TimerTask, [](const void* arg) { timer_app.run(arg); }, osPriorityHigh,
        0, 256);
    osThreadCreate(osThread(TimerTask), NULL);

    // osThreadDef(
    //     CommTask, [](const void* arg) { comm_app.run(arg); }, osPriorityHigh, 0,
    //     256);
    // osThreadCreate(osThread(CommTask), NULL);

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
    }
}

void init_auto_aim_apps() {
    // Set communication node ID for auto aim rig (gimbal board)
    communication.set_node_id(simple_comm::NodeID::Gimbal);

    osThreadDef(
        GimbalTask, [](const void* arg) { gimbal_app.run(arg); },
        osPriorityRealtime, 0, 512);
    osThreadCreate(osThread(GimbalTask), NULL);

    osThreadDef(
        IMUTask, [](const void* arg) { imu_app.run(arg); }, osPriorityRealtime,
        0, 256);
    osThreadCreate(osThread(IMUTask), NULL);

    osThreadDef(
        TimerTask, [](const void* arg) { timer_app.run(arg); }, osPriorityHigh,
        0, 256);
    osThreadCreate(osThread(TimerTask), NULL);

    osThreadDef(
        RCTask, [](const void* arg) { rc_app.run(arg); }, osPriorityHigh, 0,
        384);
    osThreadCreate(osThread(RCTask), NULL);
}

void can_filter_enable(MW_CAN::BUS bus) {
    MW_CAN::Filter filter = {
        .id_high = 0x0000,
        .id_low = 0x0000,
        .mask_id_high = 0x0000,
        .mask_id_low = 0x0000,
        .fifo_assignment = MW_CAN::FIFO::FIFO_0,
        .mode = MW_CAN::FilterMode::IDMask,
        .is_activated = true,
        .filter_bank = 0,
        .slave_start_filter_bank = 14,
    };

    if (bus == MW_CAN::BUS::CAN_1 || bus == MW_CAN::BUS::CAN_1B) {
        filter.filter_bank = 0;
    } else if (bus == MW_CAN::BUS::CAN_2 || bus == MW_CAN::BUS::CAN_2B) {
        filter.filter_bank = 14;
    } else {
        ASSERT(false, "Trying to configure unknown CAN bus.");
        return;
    }

    ASSERT(can.configure_filter(bus, filter), "CAN filter config failed.");
    ASSERT(can.activate_notification(
               bus, MW_CAN::Notification::RX_FIFO0_MSG_PENDING),
           "CAN notification activation failed.");
}

bool firmware_and_system_init(void) {
    /* CAN1 & CAN2 Init */
    ASSERT(can.start(MW_CAN::BUS::CAN_1), "Failed to start CAN bus 1.");
    ASSERT(can.start(MW_CAN::BUS::CAN_2), "Failed to start CAN bus 2.");
    /* CAN1 & CAN2 filter Init */
    can_filter_enable(MW_CAN::BUS::CAN_1);
    can_filter_enable(MW_CAN::BUS::CAN_2);

    ASSERT(
        tim.base_start(MW_TIM::Timer::TIM_13, MW_TIM::BaseStartMode::Interrupt),
        "Failed to start timer base interrupt on TIM13.");

    ASSERT(pwm.start(MW_TIM::Timer::TIM_10, MW_TIM::Channel::CHANNEL_1),
           "Failed to start PWM on TIM10 channel 1.");
    // referee_init(&referee);

    ASSERT(uart_isr.init(), "UART ISR init failed.");
    ASSERT(isr::uart::install_isr(uart_isr), "UART ISR installation failed.");
    ASSERT(can_isr.init(), "CAN ISR init failed.");
    ASSERT(isr::can::install_isr(can_isr), "CAN ISR installation failed.");
    ASSERT(mc.init(), "MC init failed.");
    ASSERT(debug.init(), "Debug init failed.");
    event_center.init();

    // Set communication node ID based on board configuration
    modules::debug::BoardConfig board_status = debug.get_board_config();
    if (board_status == modules::debug::BoardConfig::CHASSIS) {
        communication.set_node_id(simple_comm::NodeID::Chassis);
    } else if (board_status == modules::debug::BoardConfig::GIMBAL) {
        communication.set_node_id(simple_comm::NodeID::Gimbal);
    } else {
        communication.set_node_id(simple_comm::NodeID::UnknownNode);
    }

    // Initialize communication submodule
    ASSERT(communication.init(), "Communication init failed.");
    can_isr.register_routine(
        isr::can::ECallbacks::MESSAGE_PENDING,
        [](MW_CAN::BUS bus, isr::can::CANFrame frame) {
            MW_CAN::CANFrame mw_frame;
            mw_frame.sid = frame.stdid;
            mw_frame.eid = frame.extid;
            mw_frame.is_extended_id =
                (bus == MW_CAN::BUS::CAN_1B || bus == MW_CAN::BUS::CAN_2B);

            ASSERT(frame.payload_length <= mw_frame.payload.size(),
                   "Invalid CAN ISR frame length.");
            mw_frame.dlc = static_cast<uint8_t>(frame.payload_length);
            for (size_t i = 0; i < frame.payload_length; ++i) {
                mw_frame.payload[i] = std::byte {frame.payload[i]};
            }

            communication.can_isr_message_pending(bus, mw_frame);
        });

    return true;
}

// Function signature so main.c can find main_cpp().
extern "C" {
void main_cpp(void);
}

void main_cpp(void) {
    ASSERT(firmware_and_system_init(),
           "Firmware and system initialization failed.");

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