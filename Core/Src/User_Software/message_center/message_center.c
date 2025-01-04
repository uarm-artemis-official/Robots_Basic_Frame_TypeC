#include "message_center.h"


static Topic_Handle_t topic_handles[] = {
		{
				.name = MOTOR_SET,
				.item_size = sizeof(MotorSetMessage_t),
				.queue_length = 5,
				.queue_handle = NULL
		},
		{
				.name = MOTOR_READ,
				.item_size = sizeof(Motor_Feedback_t) * 8,
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
		},
		{
				.name = RC_RAW,
				.item_size = sizeof(uint8_t) * 18,
				.queue_length = 5,
				.queue_handle = NULL
		}
};


static Topic_Handle_t* get_topic_handle(Topic_Name_t name) {
	for (int i = 0; i < sizeof(topic_handles) / sizeof(Topic_Handle_t); i++) {
		if (topic_handles[i].name == name) {
			return &(topic_handles[i]);
		}
	}
// TODO: Add error handling for this case.
//	Error_Handler();
	return NULL;
}


void message_center_init() {
	for (int i = 0; i < sizeof(topic_handles) / sizeof(Topic_Handle_t); i++) {
		topic_handles[i].queue_handle = xQueueCreate(topic_handles[i].queue_length, topic_handles[i].item_size);
	}
}


BaseType_t get_message(Topic_Name_t topic, void *data_ptr, int ticks_to_wait) {
	Topic_Handle_t* topic_handle = get_topic_handle(topic);
	return xQueueReceive(topic_handle->queue_handle, data_ptr, ticks_to_wait);
}


BaseType_t peek_message(Topic_Name_t topic, void *data_ptr, int ticks_to_wait) {
	Topic_Handle_t* topic_handle = get_topic_handle(topic);
	return xQueuePeek(topic_handle->queue_handle, data_ptr, ticks_to_wait);
}


BaseType_t pub_message(Topic_Name_t topic, void *data_ptr) {
	Topic_Handle_t* topic_handle = get_topic_handle(topic);
	if (topic_handle->queue_length == 1) {
		return xQueueOverwrite(topic_handle->queue_handle, data_ptr);
	} else {
		return xQueueSendToBack(topic_handle->queue_handle, data_ptr, 0);
	}
}

BaseType_t pub_message_from_isr(Topic_Name_t topic, void *data_ptr, BaseType_t *will_context_switch) {
	Topic_Handle_t* topic_handle = get_topic_handle(topic);
	if (topic_handle->queue_length == 1) {
		return xQueueOverwriteFromISR(topic_handle->queue_handle, data_ptr, will_context_switch);
	} else {
		return xQueueSendToBackFromISR(topic_handle->queue_handle, data_ptr, will_context_switch);
	}
}
