#ifndef __APPS_CLASSES_HPP
#define __APPS_CLASSES_HPP

#include <array>
#include "apps_interfaces.hpp"
#include "apps_types.hpp"
#include "communication.hpp"
#include "debug.hpp"
#include "message_center.hpp"
#include "messages.hpp"
#include "simple_comm.hpp"
#include "subsystems_interfaces.hpp"
#include "can_isr.hpp"
#include "uart_isr.hpp"

template <class DriveTrain>
class ChassisApp : public RTOSApp<ChassisApp<DriveTrain>,
                                  apps_defines::chassis_task_loop_period_ms> {
   private:
    ChassisDrive<DriveTrain>& drive_train;
    mc2::RobotMC& mc;
    comm::Communication<mc2::RobotMC, mc2::RobotMC::Topics>& communication;
    modules::debug::Debug& debug;

    Chassis_t chassis;

   public:
    static constexpr float MAX_TRANSLATION = 2;  // in m/s
    static constexpr float MAX_ROTATION = PI;    // rad/s
    static constexpr float GYRO_SPEED = PI;

    explicit ChassisApp(MW_RTOS::IRTOS& _rtos, DriveTrain& drive_train_ref,
                        mc2::RobotMC& mc_ref,
                        comm::Communication<mc2::RobotMC,
                                            mc2::RobotMC::Topics>&
                            communication_ref,
                        modules::debug::Debug& _debug);
    void init();
    void set_initial_state();

    void loop();

    void calc_movement_vectors();

    void process_commands();
    void chassis_get_gimbal_rel_angles();

    void set_board_mode(BoardMode_t new_board_mode);
    void set_act_mode(BoardActMode_t new_act_mode);
};

// TODO: Move DriveTrains into policies or some other kind of subdirectory.
class OmniDrive : public ChassisDrive<OmniDrive> {
   private:
    std::array<Chassis_Wheel_Control_t, 4> motor_controls;
    std::array<float, 4> motor_angular_vel;
    std::array<float, 4> wheel_power_consumption;
    mc2::RobotMC& mc;
    IMotors& motors;
    float width, length, power_limit, chassis_dt;
    const float a = 0;
    const float k1 = 0;
    const float k2 = 0;

   public:
    explicit OmniDrive(mc2::RobotMC& mc2_ref, IMotors& motors,
                       float chassis_width, float chassis_length,
                       float power_limit_, float chassis_dt_);

    void init_impl();

    void get_motor_feedback();

    void calc_target_motor_speeds(float vx, float vy, float wz);
    void calc_motor_volts();
    void calc_motor_outputs(float vx, float vy, float wz);
    void calc_power_limits();

    void calc_wheel_power_consumption();
    float calc_power_consumption();
    void set_max_power_impl(float new_max_power);

    void send_motor_messages();
};

class SwerveDrive : public ChassisDrive<SwerveDrive> {
   private:
    static constexpr size_t NUM_STEER_MOTORS = 4;
    static constexpr size_t NUM_DRIVE_MOTORS = 4;

    mc2::RobotMC& mc;
    IMotors& motors;
    const float width;
    const float dt;

    std::array<Swerve_Drive_Control_t, NUM_DRIVE_MOTORS> drive_motors;
    std::array<Swerve_Steer_Control_t, NUM_STEER_MOTORS> steer_motors;

    std::array<float, NUM_STEER_MOTORS> steer_curr_angle;
    std::array<int16_t, NUM_STEER_MOTORS> steer_curr_speed;
    std::array<float, NUM_STEER_MOTORS> steer_cw_mag;
    std::array<float, NUM_STEER_MOTORS> steer_ccw_mag;
    std::array<float, NUM_STEER_MOTORS> steer_target_angle;
    std::array<float, NUM_DRIVE_MOTORS> drive_target_speed;

    std::array<uint16_t, NUM_STEER_MOTORS> steer_max_speed;
    std::array<bool, NUM_STEER_MOTORS> steer_ccw;
    std::array<float, NUM_STEER_MOTORS> steer_output_angle;
    std::array<int32_t, NUM_DRIVE_MOTORS> drive_output;

   public:
    static int32_t pack_lk_motor_message(bool spin_ccw, uint16_t max_speed,
                                         uint32_t angle);

    explicit SwerveDrive(mc2::RobotMC& mc2_ref, IMotors& motors_ref,
                         float width_, float dt_);

    void init_impl();
    void get_motor_feedback();
    void calc_motor_outputs(float vx, float vy, float wz);
    void send_motor_messages();
    float calc_power_consumption();
};

/*
 * @attention:
 * 		We transit each angle mentioned here to radian because:
 * 		1. Unit and normalize the calculation of PID (spec for angular loop)
 * 		2. for safety consideration, sometimes swap the mode between gyro and ecd
 * 		   keep radians helping unit the input value
 * 		3. easier for calculating cos/sin functions
 *
 * */
class GimbalApp
    : public ExtendedRTOSApp<GimbalApp,
                             apps_defines::gimbal_task_loop_period_ms> {
   private:
    Gimbal_t gimbal;
    modules::debug::UARTAccessToken
        debug_uart_access_token;  // Token to indicate ownership of debug UART
    Gimbal_Imu_Calibration_t imu_calibration;
    Gimbal_Motor_Control_t motor_controls[GIMBAL_MOTOR_COUNT];
    int16_t gimbal_channels[2];
    float command_deltas[2];

    mc2::RobotMC& mc;
    comm::Communication<mc2::RobotMC, mc2::RobotMC::Topics>& communication;
    IEventCenter& event_center;
    IMotors& motors;
    modules::debug::Debug& debug;

   public:
    // Software limits on pitch targets to prevent pitch from hitting mechanical hard stops.
    static constexpr float PITCH_LOWER_LIMIT = -0.1;
    static constexpr float PITCH_UPPER_LIMIT = 0.4;
    static constexpr uint32_t IMU_CENTER_TARGET_SAMPLES = 100;

    static float calc_rel_angle(float angle1, float angle2);
    static int16_t calc_ecd_rel_angle(int16_t raw_ecd, int16_t center_offset);

    explicit GimbalApp(MW_RTOS::IRTOS& _rtos, mc2::RobotMC& mc_ref,
                       comm::Communication<mc2::RobotMC,
                                           mc2::RobotMC::Topics>&
                           communication_ref,
                       IEventCenter& event_center, IMotors& motors_ref,
                       modules::debug::Debug& _debug);
    void init();
    void set_initial_state();
    bool calibrate_start_precondition();
    void wait_for_motors();

    bool exit_calibrate_cond();
    void calibrate();

    void loop();

    bool is_imu_calibrated();

    void set_modes(uint8_t modes[3]);
    void set_board_mode(BoardMode_t mode);
    void set_act_mode(BoardActMode_t mode);
    void set_motor_mode(GimbalMotorMode_t mode);
    void safe_mode_switch();

    void get_motor_feedback();
    void get_imu_headings();

    void process_commands();

    void calc_imu_center();

    void update_imu_angle(float yaw, float pitch);
    void update_ecd_angles();
    void update_headings();
    void update_targets();

    void limit_pitch_target();

    void cmd_exec();
    void calc_control_signals();

    void send_motor_volts();
    void send_rel_angles();
};

class ShootApp
    : public RTOSApp<ShootApp, apps_defines::shoot_task_loop_period_ms> {
   private:
    mc2::RobotMC& mc;
    comm::Communication<mc2::RobotMC, mc2::RobotMC::Topics>& communication;
    IAmmoLid& ammo_lid;
    IMotors& motors;

    Shoot shoot;
    LoaderSpeedControl speed_loader_control;
    LoaderPositionControl position_loader_control;
    Motor_Feedback_t loader_feedback;
    FlyWheelControl flywheel_controls[3];
    Motor_Feedback_t left_flywheel_feedback, right_flywheel_feedback, third_flywheel_feedback;
    const float LOADER_ACTIVE_RPM;
    const float FLYWHEEL_ACTIVE_TARGET_RPM;
    const float MAX_FLYWHEEL_ACCEL;
    bool is_hero;

   public:
    explicit ShootApp(MW_RTOS::IRTOS& _rtos, mc2::RobotMC& mc2_ref,
                      comm::Communication<mc2::RobotMC,
                                          mc2::RobotMC::Topics>&
                          communication_ref,
                      IAmmoLid& ammo_lid_ref, IMotors& motors_ref,
                      float loader_active_rpm_, float flywheel_target_rpm_,
                      float max_flywheel_accel, bool is_hero);

    void init();
    void loop();

    void get_motor_feedback();
    void detect_loader_stall();

    void process_commands();

    void calc_targets();
    void calc_motor_outputs();
    void send_motor_outputs();

    void set_shoot_mode(ShootActMode_t new_mode);
    void set_loader_target(float new_target);
    void set_flywheel_target(float new_target);
};

class IMUApp
    : public ExtendedRTOSApp<IMUApp, apps_defines::imu_task_loop_period_ms> {
   private:
    mc2::RobotMC& mc;
    comm::Communication<mc2::RobotMC, mc2::RobotMC::Topics>& communication;
    IEventCenter& event_center;
    IImu& imu;
    modules::debug::Debug& debug;

    IMU_t imu_app_state;
    IMU_Heat_t imu_heating_control;
    Attitude_t attitude;
    AhrsSensor_t sensor_data;

   public:
    static constexpr float TARGET_IMU_TEMP = 40.0f;
    static constexpr float NORMAL_TEMP_THRESHOLD = 1.0f;
    static constexpr float IMU_RESET_THRESHOLD = 7.0f;

    explicit IMUApp(MW_RTOS::IRTOS& _rtos, mc2::RobotMC& mc2_ref,
                    comm::Communication<mc2::RobotMC,
                                        mc2::RobotMC::Topics>&
                        communication_ref,
                    IEventCenter& event_center_ref, IImu& imu_ref,
                    modules::debug::Debug& _debug);
    void init();
    void calibrate();
    bool exit_calibrate_cond();
    void loop();

    int32_t imu_temp_pid_control();

    void set_imu_temp_status(IMU_temp_status status);
};

class RefereeApp
    : public RTOSApp<RefereeApp, apps_defines::referee_task_loop_period_ms> {
   private:
    mc2::RobotMC& mc;
    comm::Communication<mc2::RobotMC, mc2::RobotMC::Topics>& communication;
    IEventCenter& event_center;
    modules::debug::Debug& debug;
    IRefUI& ref_ui;  // Referee UI interface
    isr::uart::UART_ISR& uart_isr;

    mc2::RefereeIn uart_referee_in;
    Referee_t ref;
    uint16_t non_recv_count =
        0;  // Count the number of times referee data is not received
    uint8_t ui_sendig_count = 0;  // Count the number of times UI data is sent

   public:
    explicit RefereeApp(MW_RTOS::IRTOS& _rtos, mc2::RobotMC& mc2_ref,
                        comm::Communication<mc2::RobotMC,
                                            mc2::RobotMC::Topics>&
                            communication_ref,
                        IEventCenter& evt_center, modules::debug::Debug& _debug,
                        IRefUI& ref_ui, isr::uart::UART_ISR& _uart_isr);
    void init();
    void loop();
    bool uart_isr_init(MW_UART::IUART& uart);
    void uart_isr_receive_complete(MW_UART::IUART& uart,
                                   MW_UART::Peripheral peripheral);
    void uart_isr_on_error(MW_UART::IUART& uart,
                           MW_UART::Peripheral peripheral);

    void read_ref_data();
    void draw_all_ui();
    void reset();
};

class RCApp : public RTOSApp<RCApp, apps_defines::rc_task_loop_period_ms> {
   private:
    mc2::RobotMC& mc;
    comm::Communication<mc2::RobotMC, mc2::RobotMC::Topics>& communication;
    IRCComm& rc_comm;
    isr::uart::UART_ISR& uart_isr;

    // TODO: remove and replace with rc_rx_buffer.
    mc2::RCRaw rc_raw;
    Buffer rc_rx_buffer;
    mc2::RCRaw uart_rx;
    RemoteControl_t rc;
    uint32_t rc_idle_count = 0;

    BoardMode_t pc_board_mode;
    BoardActMode_t pc_act_mode;
    ShootActMode_t pc_shoot_mode;
    ammo_lid::LidStatus pc_ammo_status;

   public:
    explicit RCApp(MW_RTOS::IRTOS& _rtos, mc2::RobotMC& mc2_ref,
                   comm::Communication<mc2::RobotMC,
                                       mc2::RobotMC::Topics>&
                       communication_ref,
                   IRCComm& rc_comm_ref, isr::uart::UART_ISR& uart_isr);

    void init();
    void loop();
    bool uart_isr_init(MW_UART::IUART& uart);
    void uart_isr_receive_complete(MW_UART::IUART& uart,
                                   MW_UART::Peripheral peripheral);

    void parse_raw_rc();
    void map_switches_to_modes(BoardMode_t& board_mode,
                               BoardActMode_t& act_mode,
                               ShootActMode_t& shoot_mode);
    void detect_rc_loss();
    void send_gimbal_command(float yaw, float pitch, BoardMode_t board_mode,
                             BoardActMode_t act_mode);
    void send_chassis_command(float v_parallel, float v_perp, float wz,
                              BoardMode_t board_mode, BoardActMode_t act_mode);
    void send_shoot_command(ShootActMode_t shoot_mode,
                            ammo_lid::LidStatus ammo_lid_status);

    void pub_command_messages();
};

class TimerApp
    : public RTOSApp<TimerApp, apps_defines::timer_task_loop_period_ms> {
   private:
    IMotors& system_motors;
    mc2::RobotMC& mc;
    comm::Communication<mc2::RobotMC, mc2::RobotMC::Topics>& communication;
    modules::debug::Debug& debug;
    isr::can::CAN_ISR& can_isr;

    const std::array<Motor_CAN_ID_t, 4> swerve_ids = {
        SWERVE_STEER_MOTOR1, SWERVE_STEER_MOTOR2, SWERVE_STEER_MOTOR3,
        SWERVE_STEER_MOTOR4};
    mc2::MotorSet motor_set;
    BoardStatus_t board_status;
    mc2::MotorRead motor_read;
    CANISRConfig can_isr_config;

   public:
    explicit TimerApp(MW_RTOS::IRTOS& _rtos, IMotors& system_motors_ref,
                      mc2::RobotMC& mc2_ref,
                      comm::Communication<mc2::RobotMC,
                                          mc2::RobotMC::Topics>&
                          communication_ref,
                      modules::debug::Debug& _debug,
                      isr::can::CAN_ISR& can_isr);
    void init();
    void loop();
    void parse_motor_feedback(isr::can::CANFrame frame);
    bool can_isr_init(MW_CAN::ICAN& can);
    void can_isr_message_receive(MW_CAN::BUS bus, isr::can::CANFrame frame);
};

class PCUARTApp
    : public RTOSApp<PCUARTApp, apps_defines::pc_uart_task_loop_period_ms> {
   private:
    mc2::RobotMC& mc;
    comm::Communication<mc2::RobotMC, mc2::RobotMC::Topics>& communication;
    IMotors& motors;
    IPCComm& pc_comm;
    isr::uart::UART_ISR& uart_isr;

    uint32_t idle_count = 0;
    mc2::UCPackIn uc_pack_in;
    uint8_t new_send_buffer[196];
    float recent_deltas[2];
    mc2::UCPackIn uart_pack_in;

   public:
    explicit PCUARTApp(MW_RTOS::IRTOS& _rtos, mc2::RobotMC& mc2_ref,
                       comm::Communication<mc2::RobotMC,
                                           mc2::RobotMC::Topics>&
                           communication_ref,
                       IMotors& motors_, IPCComm& pc_comm_,
                       isr::uart::UART_ISR& _uart_isr);
    void init();
    void loop();
    bool uart_isr_init(MW_UART::IUART& uart);
    void uart_isr_receive_complete(MW_UART::IUART& uart,
                                   MW_UART::Peripheral peripheral);

    void send_swerve_data();
};

namespace CommApp {
    // inline namespace v3 {
    //     constexpr size_t MAX_SIMPLE_COMM_FX_FIFO_SIZE = 10;
    //     constexpr size_t INTERNAL_FIFO_SIZE = MAX_SIMPLE_COMM_FX_FIFO_SIZE * 2;
    //     constexpr size_t TO_PUBLISH_BUFFER_SIZE = 128;
    //     class CommApp
    //         : public RTOSApp<CommApp, apps_defines::comm_task_loop_period_ms> {
    //        private:
    //         isr::can::CAN_ISR& can_isr;
    //         isr::uart::UART_ISR& uart_isr;
    //         simple_comm::SimpleComm<MAX_SIMPLE_COMM_FX_FIFO_SIZE>& simple_comm;
    //         mc2::RobotMC& mc;
    //         MW_CAN::ICAN& can;
    //         MW_UART::IUART& uart;
    //         modules::debug::Debug& debug;
    //         mc2::MessageNode current_node;
    //         std::array<std::byte, 32> deserialize_message_buffer;
    //         std::array<simple_comm::utils::IndexableDeserializer, 256>
    //             deserializers;
    //         std::array<simple_comm::utils::IndexableSerializer, 256>
    //             serializers;

    //         static_assert(TO_PUBLISH_BUFFER_SIZE >
    //                           simple_comm::UART_MAX_MESSAGE_SIZE,
    //                       "TO_PUBLISH_BUFFER_SIZE must be larger than "
    //                       "UART_MAX_MESSAGE_SIZE.");
    //         std::array<std::byte, TO_PUBLISH_BUFFER_SIZE> to_publish_buffer;
    //         std::array<std::byte, simple_comm::UART_MAX_MESSAGE_SIZE>
    //             uart_out_buffer;
    //         dsa::StrictRingBuffer<simple_comm::SimpleMessage,
    //                               INTERNAL_FIFO_SIZE>
    //             messages_to_process_buffer;

    //        public:
    //         explicit CommApp(
    //             MW_RTOS::IRTOS& _rtos, isr::can::CAN_ISR& _can_isr,
    //             isr::uart::UART_ISR& _uart_isr,
    //             simple_comm::SimpleComm<MAX_SIMPLE_COMM_FX_FIFO_SIZE>&
    //                 _simple_comm,
    //             MW_CAN::ICAN& _can, MW_UART::IUART& _uart,
    //             mc2::RobotMC& mc2_ref, modules::debug::Debug& _debug);

    //         bool init();
    //         void loop();
    //         bool send_message_via_can(const simple_comm::SimpleMessage& msg);
    //         bool send_message_via_uart(const simple_comm::SimpleMessage& msg);
    //         void enqueue_interboard_messages();
    //     };
    // }  // namespace v3
    inline namespace v4 {
        class CommApp : public RTOSApp<CommApp, apps_defines::comm_task_loop_period_ms> {
           private:
           public:
            explicit CommApp();

            bool init();
            void loop();
        };
    }
}  // namespace CommApp

#endif