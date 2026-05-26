#include "apps_classes.hpp"
#include "apps_types.hpp"
#include "can_isr.hpp"
#include "communication.hpp"
#include "debug.hpp"
#include "message_center.hpp"
#include "middleware_classes.hpp"
#include "robot_config.hpp"
#include "subsystems_classes.hpp"
#include "subsystems_modules.hpp"
#include "uart_isr.hpp"

#if defined(SWERVE_CHASSIS)
constexpr bool is_swerve = true;
#else
constexpr bool is_swerve = false;
#endif

static MW_RTOS::RTOS rtos;
static MW_TIM::TIM tim;
static MW_TIM::PWM pwm;
static MW_UART::UART uart;
static MW_GPIO::GPIO gpio;
static MW_SPI::SPI spi;
static MW_I2C::I2C i2c;

static MW_CAN::CAN can;
static mc2::RobotMC mc(rtos);
static EventCenter event_center(rtos);
static modules::debug::Debug debug(gpio, uart);
static bmi088_driver::BMI088 bmi088(spi, rtos, gpio, pwm);
static ist8310_driver::IST8310 ist8310(i2c, rtos, gpio);
static Motors motors(can);
static RefereeUI ref_ui(uart);
static Motors no_init_motors(
    can);  // TODO: Refactor? -> remove or split responsibilities into another module?
static Imu imu(bmi088, ist8310, rtos, 1000 / IMUApp::loop_period_ms, 0.4,
               robot_config::gimbal_config.imu_orientation);
static ammo_lid::AmmoLid ammo_lid_(pwm);
static RCComm rc_comm;

static isr::can::CAN_ISR can_isr(can);
static isr::uart::UART_ISR uart_isr(uart);

static simple_comm::SimpleCommCodec simple_comm_codec;
static comm::Communication<mc2::RobotMC, mc2::RobotMC::Topics> communication(
    mc, simple_comm_codec, can, uart);

static const apps::chassis::SwerveDriveConfig swerve_drive_config = {
    .drive_wheel_pid_config = robot_config::chassis_config.wheel_pid_config,
    .max_wheel_ramp_accel = robot_config::chassis_config.max_wheel_ramp_accel,
    .chassis_width = uarm::strong_types::Meter(0.352728f),
    .chassis_length = uarm::strong_types::Meter(0.352728f),
    .chassis_app_dt = uarm::strong_types::Second(
        ChassisApp<SwerveDrive>::get_loop_period() / 1000.0f),
};

static SwerveDrive swerve_drive(swerve_drive_config, mc, no_init_motors);
static ChassisApp<SwerveDrive> swerve_chassis_app(rtos,
                                                  robot_config::chassis_config,
                                                  swerve_drive, mc,
                                                  communication, debug);
static constexpr float omni_chassis_width = 0.40f;
static const apps::chassis::OmniDriveConfig omni_drive_config = {
    .wheel_pid_config = robot_config::chassis_config.wheel_pid_config,
    .max_wheel_ramp_accel = robot_config::chassis_config.max_wheel_ramp_accel,
    .chassis_width = uarm::strong_types::Meter(0.40f),
    .chassis_length = uarm::strong_types::Meter(0.40f),
    .chassis_app_dt = uarm::strong_types::Second(
        ChassisApp<OmniDrive>::get_loop_period() / 1000.0f),
    .a = 0,
    .k1 = 0,
    .k2 = 0};
static OmniDrive omni_drive(omni_drive_config, mc, no_init_motors);

static ChassisApp<OmniDrive> omni_chassis_app(rtos,
                                              robot_config::chassis_config,
                                              omni_drive, mc, communication,
                                              debug);

static const apps::rc::RCConfig rc_config = {
    .chassis_translation_speed =
        robot_config::chassis_config.max_translation_speed,
    .chassis_rotation_speed = robot_config::chassis_config.max_rotation_speed,
};
static RCApp rc_app(rtos, rc_config, mc, communication, rc_comm, uart_isr);

static TimerApp timer_app(rtos, motors, mc, communication, debug, can_isr);
static IMUApp imu_app(rtos, mc, communication, event_center, imu, debug);
static RefereeApp referee_app(rtos, mc, communication, event_center, debug,
                              ref_ui, uart_isr);
static GimbalApp gimbal_app(rtos, robot_config::gimbal_config, mc,
                            communication, event_center, no_init_motors, debug);

static ShootApp::ShootApp shoot_app(rtos, robot_config::shoot_config, mc,
                                    communication, ammo_lid_, no_init_motors);

static MW_RTOS::TaskHandle timer_task_handle = nullptr;
static MW_RTOS::TaskHandle omni_chassis_task_handle = nullptr;
static MW_RTOS::TaskHandle swerve_chassis_task_handle = nullptr;
static MW_RTOS::TaskHandle rc_task_handle = nullptr;
static MW_RTOS::TaskHandle ref_task_handle = nullptr;
static MW_RTOS::TaskHandle gimbal_task_handle = nullptr;
static MW_RTOS::TaskHandle shoot_task_handle = nullptr;
static MW_RTOS::TaskHandle imu_task_handle = nullptr;
static MW_RTOS::TaskHandle default_task_handle = nullptr;

bool firmware_and_system_init(void) {
    ASSERT(rtos.init(), "Failed to init RTOS middleware.");
    ASSERT(can.init(), "Failed to init CAN middleware.");
    ASSERT(i2c.init(), "Failed to init I2C middleware.");
    ASSERT(spi.init(), "Failed to init SPI middleware.");

    ASSERT(
        tim.base_start(MW_TIM::Timer::TIM_13, MW_TIM::BaseStartMode::Interrupt),
        "Failed to start timer base interrupt on TIM13.");

    ASSERT(pwm.start(MW_TIM::Timer::TIM_10, MW_TIM::Channel::CHANNEL_1),
           "Failed to start PWM on TIM10 channel 1.");
    // referee_init(&referee);

    ASSERT(uart_isr.init(), "UART ISR init failed.");
    ASSERT(isr::uart::install_isr(&uart_isr), "UART ISR installation failed.");
    ASSERT(can_isr.init(), "CAN ISR init failed.");
    ASSERT(isr::can::install_isr(&can_isr), "CAN ISR installation failed.");
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
    ASSERT(can_isr.register_routine(
               isr::can::ECallbacks::MESSAGE_PENDING,
               [](MW_CAN::BUS bus, isr::can::CANFrame frame) {
                   MW_CAN::CANFrame mw_frame;
                   mw_frame.sid = frame.stdid;
                   mw_frame.eid = frame.extid;
                   mw_frame.is_extended_id = (bus == MW_CAN::BUS::CAN_1B ||
                                              bus == MW_CAN::BUS::CAN_2B);

                   ASSERT(frame.payload_length <= mw_frame.payload.size(),
                          "Invalid CAN ISR frame length.");
                   mw_frame.dlc = static_cast<uint8_t>(frame.payload_length);
                   for (size_t i = 0; i < frame.payload_length; ++i) {
                       mw_frame.payload[i] = std::byte {frame.payload[i]};
                   }

                   communication.can_isr_message_pending(bus, mw_frame);
               }),
           "Failed to register CAN ISR routine for message pending.");

    return true;
}

void init_robot_apps() {
    modules::debug::BoardConfig board_status = debug.get_board_config();

    ASSERT(rtos.task_create(
               default_task_handle, const_cast<char*>("DefaultTask"),
               [](void* arg) {
                   (void) arg;
                   while (true) {
                       rtos.delay_ms(1);
                   }
               },
               nullptr, 256, MW_RTOS::TaskPriority::Low),
           "Failed to create DefaultTask.");

    ASSERT(rtos.task_create(
               timer_task_handle, const_cast<char*>("TimerTask"),
               [](void* arg) { timer_app.run(static_cast<const void*>(arg)); },
               nullptr, 256, MW_RTOS::TaskPriority::High),
           "Failed to create TimerTask.");

    ASSERT(rtos.task_create(
               imu_task_handle, const_cast<char*>("IMUTask"),
               [](void* arg) { imu_app.run(static_cast<const void*>(arg)); },
               nullptr, 256, MW_RTOS::TaskPriority::Realtime),
           "Failed to create IMUTask.");

    if (board_status == modules::debug::BoardConfig::CHASSIS) {
        if constexpr (is_swerve) {
            ASSERT(
                rtos.task_create(
                    swerve_chassis_task_handle,
                    const_cast<char*>("SwerveChassisTask"),
                    [](void* arg) {
                        swerve_chassis_app.run(static_cast<const void*>(arg));
                    },
                    nullptr, 256, MW_RTOS::TaskPriority::High),
                "Failed to create SwerveChassisTask.");
        } else {
            ASSERT(rtos.task_create(
                       omni_chassis_task_handle,
                       const_cast<char*>("OmniChassisTask"),
                       [](void* arg) {
                           omni_chassis_app.run(static_cast<const void*>(arg));
                       },
                       nullptr, 256, MW_RTOS::TaskPriority::High),
                   "Failed to create OmniChassisTask.");
        }

        ASSERT(rtos.task_create(
                   rc_task_handle, const_cast<char*>("RCTask"),
                   [](void* arg) { rc_app.run(static_cast<const void*>(arg)); },
                   nullptr, 384, MW_RTOS::TaskPriority::High),
               "Failed to create RCTask.");

        ASSERT(rtos.task_create(
                   ref_task_handle, const_cast<char*>("RefTask"),
                   [](void* arg) {
                       referee_app.run(static_cast<const void*>(arg));
                   },
                   nullptr, 384, MW_RTOS::TaskPriority::High),
               "Failed to create RefTask.");

    } else if (board_status == modules::debug::BoardConfig::GIMBAL) {
        ASSERT(rtos.task_create(
                   gimbal_task_handle, const_cast<char*>("GimbalTask"),
                   [](void* arg) {
                       gimbal_app.run(static_cast<const void*>(arg));
                   },
                   nullptr, 512, MW_RTOS::TaskPriority::Realtime),
               "Failed to create GimbalTask.");

        ASSERT(
            rtos.task_create(
                shoot_task_handle, const_cast<char*>("ShootTask"),
                [](void* arg) { shoot_app.run(static_cast<const void*>(arg)); },
                nullptr, 256, MW_RTOS::TaskPriority::High),
            "Failed to create ShootTask.");
    }
}

void main_cpp() {
    ASSERT(firmware_and_system_init(),
           "Firmware and system initialization failed.");
    switch (robot_config::config_type) {
        case robot_config::ConfigType::Infantry:
        case robot_config::ConfigType::Hero:
        case robot_config::ConfigType::Sentry:
            init_robot_apps();
            break;
        default:
            ASSERT(false, "Unrecognized config_type.");
    }
}
