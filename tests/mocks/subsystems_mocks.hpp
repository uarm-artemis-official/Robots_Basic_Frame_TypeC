#ifndef __SUBSYSTEMS_MOCKS_HPP
#define __SUBSYSTEMS_MOCKS_HPP

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include "subsystems_interfaces.hpp"

class MockMotors : public IMotors {
   public:
    MOCK_METHOD(void, init, (Motor_Config_t config), (override));
    MOCK_METHOD(bool, is_valid_output, (size_t motor_index, int32_t new_output),
                (override));
    MOCK_METHOD(void, set_motor_voltage, (uint32_t can_id, int32_t output),
                (override));
    MOCK_METHOD(void, send_motor_voltage, (), (override));
    MOCK_METHOD(void, request_feedback, (Motor_CAN_ID_t can_id), (override));
    MOCK_METHOD(void, get_raw_feedback,
                (uint32_t stdid, uint8_t data[8], void* feedback), (override));
    MOCK_METHOD(Motor_Brand_t, get_motor_brand, (uint32_t stdid));
};

class MockImu : public IImu {
   public:
    MOCK_METHOD(void, init, (), (override));
    MOCK_METHOD(float, get_temp, (), (override));
    MOCK_METHOD(void, get_attitude, (Attitude_t & attitude), (override));
    MOCK_METHOD(void, get_sensor_data, (AhrsSensor_t & sensor), (override));
    MOCK_METHOD(void, set_heat_pwm, (uint16_t duty_cycle), (override));
    MOCK_METHOD(void, gather_sensor_data,
                (AhrsSensor_t & sensor, bool read_mag), (override));
};

class MockEventCenter : public IEventCenter {
   public:
    MOCK_METHOD(void, init, (), (override));
    MOCK_METHOD(UARM_Events_t, wait_events,
                (UARM_Events_t wait_events, uint32_t timeout), (override));
    MOCK_METHOD(void, emit_events, (UARM_Events_t new_events), (override));
    MOCK_METHOD(void, clear_events, (UARM_Events_t clear_events), (override));
    MOCK_METHOD(bool, sync_tasks,
                (Sync_Event_t sync_event, UARM_Events_t set_task,
                 uint32_t timeout),
                (override));
};

class MockAmmoLid : public IAmmoLid {
   public:
    MOCK_METHOD(void, init, (), (override));
    MOCK_METHOD(void, set_lid_status, (ammo_lid::LidStatus new_status),
                (override));
};

class MockRefUI : public IRefUI {
   public:
    MOCK_METHOD(void, init, (), (override));
    MOCK_METHOD(void, set_ui_data,
                (referee_ui_type_t ui_type, uint8_t robot_id,
                 ref_ui_info_t ref_ui_info),
                (override));
    MOCK_METHOD(void, send_ui_data,
                (uint16_t cmd_id, uint16_t len, referee_ui_type_t ui_type),
                (override));
    MOCK_METHOD(void, draw_marks, (), (override));
    MOCK_METHOD(void, draw_vaild_info, (uint32_t act_mode, uint32_t level),
                (override));
};

class MockRCComm : public IRCComm {
   public:
    MOCK_METHOD(void, buffer_init, (Buffer & buffer), (override));
    MOCK_METHOD(void, key_object_init, (KeyObject & key), (override));
    MOCK_METHOD(void, keyboard_init, (Keyboard & keyboard), (override));
    MOCK_METHOD(void, mouse_init, (Mouse & mouse), (override));
    MOCK_METHOD(void, pc_init, (PC & pc), (override));
    MOCK_METHOD(void, controller_init, (Controller & controller), (override));
    MOCK_METHOD(void, parse_switches,
                (Buffer & buffer, ESwitchState& s1, ESwitchState& s2),
                (override));
    MOCK_METHOD(void, parse_controller,
                (Buffer & buffer, Controller& controller), (override));
    MOCK_METHOD(void, parse_pc, (Buffer & buffer, PC& pc), (override));
    MOCK_METHOD(void, key_scan,
                (KeyObject & key, uint16_t key_buffer,
                 EKeyBitIndex key_bit_index),
                (override));
};

class MockPCComm : public IPCComm {
   public:
    MOCK_METHOD(uint8_t, uc_check_pack_integrity,
                (uint8_t* pack_bytes, uint8_t pack_size), (override));
    MOCK_METHOD(void, send_bytes, (uint8_t* bytes, uint32_t size), (override));
    MOCK_METHOD(uint8_t, get_data_size, (uint8_t header_id), (override));
    MOCK_METHOD(void, start_receive, (uint8_t* pack_buffer), (override));
    MOCK_METHOD(void, restart_receive, (uint8_t* pack_buffer), (override));
    MOCK_METHOD(UC_Checksum_t, calc_checksum,
                (void* buffer_ptr, size_t buffer_size), (override));
    MOCK_METHOD(uint8_t, is_valid_header, (uint8_t* input_buffer), (override));
};

#endif