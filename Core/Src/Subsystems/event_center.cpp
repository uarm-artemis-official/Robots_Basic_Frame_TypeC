/*
 * event_center.c
 *
 *  Created on: Mar 8, 2025
 *      Author: johnm
 */

#include "event_center.h"
#include "uarm_lib.h"

static EventGroupHandle_t events_group;
static Sync_group_t sync_groups[NUM_SYNC_GROUPS];

void event_center_init() {
    events_group = xEventGroupCreate();

    for (int i = 0; i < NUM_SYNC_GROUPS; i++) {
        sync_groups[i] = Sync_group_t {None, xEventGroupCreate()};
        xEventGroupClearBits(sync_groups[i].event_group, 0xffffff);
    }
    xEventGroupClearBits(events_group, 0xffffff);
}

EventBits_t wait_events(EventBits_t wait_events, TickType_t timeout) {
    return xEventGroupWaitBits(events_group, wait_events, wait_events, pdTRUE,
                               timeout);
}

void emit_events(EventBits_t new_events) {
    xEventGroupSetBits(events_group, new_events);
}

void clear_events(EventBits_t clear_events) {
    xEventGroupClearBits(events_group, clear_events);
}

BaseType_t sync_tasks(Sync_Event_t sync_event, EventBits_t set_task,
                      TickType_t timeout) {
    ASSERT(sync_event != None, "sync_tasks cannot sync on None event.");

    int available_group_index = -1;
    int active_group_index = -1;
    for (int i = 0; i < NUM_SYNC_GROUPS; i++) {
        if (sync_groups[i].sync_event == sync_event)
            active_group_index = i;
        if (sync_groups[i].sync_event == None)
            available_group_index = i;
    }

    int sync_group_index =
        active_group_index != -1 ? active_group_index : available_group_index;
    if (sync_group_index != -1) {
        sync_groups[sync_group_index].sync_event = sync_event;
        EventBits_t rv =
            xEventGroupSync(sync_groups[sync_group_index].event_group, set_task,
                            (EventBits_t) sync_event, timeout);
        sync_groups[sync_group_index].sync_event = None;
        return rv == sync_event ? pdTRUE : pdFALSE;
    }
    return pdFALSE;
}
