#include "message_center.h"


static Topic_Handle_t topic_handles[] = {
		{
				.name = MOTOR_SET,
				//
				.item_size = sizeof(MotorSetMessage_t),
				.queue_length = 1,
				.queue_handle = NULL
		},
		{
				.name = MOTOR_READ,
				//
				.item_size = sizeof(uint8_t) * 8 * 8,
				.queue_length = 1,
				.queue_handle = NULL
		},
//		{
//				.name = MOTOR_IN,
//				.item_size = sizeof()
//		},
		{
				.name = RC_INFO,
				.item_size = sizeof(RCInfoMessage_t),
				.queue_length = 1,
				.queue_handle = NULL
		},
		{
				.name = COMM_OUT,
				// TODO: Make struct for standardizing communication over CAN.
				.item_size = sizeof(CANCommMessage_t),
				.queue_length = 5,
				.queue_handle = NULL
		},
		{
				.name = COMM_IN,
				.item_size = sizeof(CANCommMessage_t),
				.queue_length = 5,
				.queue_handle = NULL
		},
		{
				.name = IMU_READINGS,
				// Pitch and yaw readings.
				.item_size = sizeof(float) * 2,
				.queue_length = 1,
				.queue_handle = NULL
		},
		{
				/*
				 * float[0] - gimbal yaw relative angle.
				 * float[1] - gimbal pitch relative angle.
				 */
				.name = GIMBAL_REL_ANGLES,
				.item_size = sizeof(float) * 2,
				.queue_length = 1,
				.queue_handle = NULL
		},
		{
				.name = REF_INFO,
				// TODO: Make struct with necessary info from ref system.
				.item_size = 0,
				.queue_length = 1,
				.queue_handle = NULL
		},
		{
				.name = PLAYER_COMMANDS,
				// TODO: Make struct with necessary info for commands from player inputs.
				.item_size = 0,
				.queue_length = 1,
				.queue_handle = NULL
		},
		{
				.name = REFEREE_INFO,
				.item_size = sizeof(uint8_t) * 2,
				.queue_length = 1,
				.queue_handle = NULL
		}
};


static QueueHandle_t get_queue_handle_by_topic(Topic_Name_t name) {
	for (int i = 0; i < sizeof(topic_handles) / sizeof(Topic_Handle_t); i++) {
		if (topic_handles[i].name == name) {
			return topic_handles[i].queue_handle;
		}
	}
}


void message_center_init() {
	for (int i = 0; i < sizeof(topic_handles) / sizeof(Topic_Handle_t); i++) {
		topic_handles[i].queue_handle = xQueueCreate(topic_handles[i].queue_length, topic_handles[i].item_size);
	}
}


BaseType_t get_message(Topic_Name_t topic, void *data_ptr, int ticks_to_wait) {
	QueueHandle_t queue_handle = get_queue_handle_by_topic(topic);
	return xQueueReceive(queue_handle, data_ptr, ticks_to_wait);
}


BaseType_t peek_message(Topic_Name_t topic, void *data_ptr, int ticks_to_wait) {
	QueueHandle_t queue_handle = get_queue_handle_by_topic(topic);
	return xQueuePeek(queue_handle, data_ptr, ticks_to_wait);
}


BaseType_t pub_message(Topic_Name_t topic, void *data_ptr) {
	QueueHandle_t queue_handle = get_queue_handle_by_topic(topic);
	return xQueueOverwrite(queue_handle, data_ptr);
}
