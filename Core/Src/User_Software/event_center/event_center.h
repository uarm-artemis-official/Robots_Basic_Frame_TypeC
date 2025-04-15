/*
 * event_center.h
 *
 *  Created on: Mar 8, 2025
 *      Author: johnm
 */

#ifndef SRC_EVENT_CENTER_H_
#define SRC_EVENT_CENTER_H_


#include "cmsis_os.h"

#include "error_handler.h"


// Number of sync events that can occur at the same time.
// A sync event group has one event group that can allows up to 24 different tasks
// (listed in Task_Sync_t) to "sync" to one particular event (Sync_Event_t).
#define NUM_SYNC_GROUPS 5

// There is a maximum of 24 bits for our configuration for one event group.
// The other 8 bits are reserved for other use.
typedef enum Event_t {
	TEST_EVENT_0                 = 1 << 1,
	TEST_EVENT_1                 = 1 << 2,
	TEST_EVENT_2                 = 1 << 3,
	TEST_EVENT_3                 = 1 << 4,
	TEST_EVENT_4                 = 1 << 5,
} Event_t;


typedef enum Task_Sync_t {
	TS_CHASSIS = 0,
	TS_GIMBAL  = 1 << 0,
	TS_SHOOT   = 1 << 1,
	TS_IMU     = 1 << 2,
	TS_CONTROL = 1 << 3,
	TS_COMM    = 1 << 4,
	TS_REFEREE = 1 << 5,
	TS_TASK4   = 1 << 6,
	TS_TASK5   = 1 << 7,
} Task_Sync_t;


typedef enum Sync_Event_t {
	None,
	Sync_Test = TS_TASK4 | TS_TASK5,
} Sync_Event_t;


typedef struct {
	Sync_Event_t sync_event;
	EventGroupHandle_t event_group;
} Sync_group_t;


// Initialize event center.
void event_center_init();

// Block until events or timeout occur.
EventBits_t wait_events(EventBits_t wait_events, TickType_t timeout);

// Emit events to event groups in event center.
void emit_events(EventBits_t new_events);

// Clear events for event groups in event center.
void clear_events(EventBits_t clear_events);

// Sync tasks to a particular sync event.
BaseType_t sync_tasks(Sync_Event_t sync_event, EventBits_t set_task, TickType_t timeout);

#endif /* SRC_EVENT_CENTER_H_ */
