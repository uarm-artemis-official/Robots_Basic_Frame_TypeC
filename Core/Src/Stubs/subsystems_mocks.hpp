#ifndef __SUBSYSTEMS_MOCKS_HPP
#define __SUBSYSTEMS_MOCKS_HPP

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include "subsystems_interfaces.h"

class MockMessageCenter : public IMessageCenter {
   public:
    MOCK_METHOD(void, init, (), (override));
    MOCK_METHOD(uint8_t, get_message,
                (Topic_Name_t topic, void* data_ptr, int ticks_to_wait),
                (override));
    MOCK_METHOD(uint8_t, peek_message,
                (Topic_Name_t topic, void* data_ptr, int ticks_to_wait),
                (override));
    MOCK_METHOD(uint8_t, pub_message, (Topic_Name_t topic, void* data_ptr),
                (override));
    MOCK_METHOD(uint8_t, pub_message_from_isr,
                (Topic_Name_t topic, void* data_ptr,
                 uint8_t* will_context_switch),
                (override));
    MOCK_METHOD(Topic_Handle_t&, get_topic_handle, (Topic_Name_t name),
                (override));
};

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

class MockDebug : public IDebug {
   public:
    MOCK_METHOD(BoardStatus_t, get_board_status, (), (override));
    MOCK_METHOD(void, set_led_state, (Board_LED_t led, Board_LED_State_t state),
                (override));
};

class MockCanComm : public ICanComm {
   public:
    MOCK_METHOD(void, init, (), (override));
    MOCK_METHOD(void, can_transmit_comm_message,
                (uint8_t send_data[8], uint32_t comm_id), (override));
};

class MockAmmoLid : public IAmmoLid {
   public:
    MOCK_METHOD(void, init, (), (override));
    MOCK_METHOD(void, set_lid_status, (EAmmoLidStatus new_status), (override));
};

#endif