/*
 * event_center.c
 *
 *  Created on: Mar 8, 2025
 *      Author: johnm
 */

#include "event_center.h"
#include "subsystems_defines.h"
#include "uarm_lib.h"

void EventCenter::init() {
    events_group = xEventGroupCreate();

    for (int i = 0; i < NUM_SYNC_GROUPS; i++) {
        sync_groups[i] = Sync_group_t {None, xEventGroupCreate()};
        xEventGroupClearBits(sync_groups[i].event_group, 0xffffff);
    }
    xEventGroupClearBits(events_group, 0xffffff);
}

// TODO: Make wait_events less esoteric to use (i.e. make rv bool which is true/false)
// if the wait_events happened.
UARM_Events_t EventCenter::wait_events(UARM_Events_t wait_events,
                                       uint32_t timeout) {
    return xEventGroupWaitBits(events_group, static_cast<uint32_t>(wait_events),
                               static_cast<uint32_t>(wait_events), pdTRUE,
                               timeout);
}

void EventCenter::emit_events(UARM_Events_t new_events) {
    xEventGroupSetBits(events_group, new_events);
}

void EventCenter::clear_events(UARM_Events_t clear_events) {
    xEventGroupClearBits(events_group, clear_events);
}

bool EventCenter::sync_tasks(Sync_Event_t sync_event, UARM_Events_t set_task,
                             uint32_t timeout) {
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
        UARM_Events_t rv =
            xEventGroupSync(sync_groups[sync_group_index].event_group, set_task,
                            (UARM_Events_t) sync_event, timeout);
        sync_groups[sync_group_index].sync_event = None;
        return rv == sync_event;
    }
    return false;
}
