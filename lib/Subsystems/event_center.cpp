#include "subsystems_classes.hpp"
#include "subsystems_defines.hpp"
#include "uarm_lib.hpp"

EventCenter::EventCenter(MW_RTOS::IRTOS& rtos_) : rtos(rtos_) {}

// TODO: Change to bool return type and check if groups initialized.
void EventCenter::init() {
    ASSERT(rtos.event_group_create(events_group),
           "Failed to create EventCenter event group.");

    for (int i = 0; i < NUM_SYNC_GROUPS; i++) {
        sync_groups[i] = Sync_group_t {None, nullptr};
        ASSERT(rtos.event_group_create(sync_groups[i].event_group),
               "Failed to create sync event group.");
        rtos.event_group_clear_bits(sync_groups[i].event_group, 0xffffff);
    }
    rtos.event_group_clear_bits(events_group, 0xffffff);
}

// TODO: Make wait_events less esoteric to use (i.e. make rv bool which is true/false)
// if the wait_events happened.
UARM_Events_t EventCenter::wait_events(UARM_Events_t wait_events,
                                       uint32_t timeout) {
    return static_cast<UARM_Events_t>(rtos.event_group_wait_bits(
        events_group, static_cast<uint32_t>(wait_events), true, true, timeout));
}

void EventCenter::emit_events(UARM_Events_t new_events) {
    (void) rtos.event_group_set_bits(events_group,
                                     static_cast<uint32_t>(new_events));
}

void EventCenter::clear_events(UARM_Events_t clear_events) {
    (void) rtos.event_group_clear_bits(events_group,
                                       static_cast<uint32_t>(clear_events));
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
        UARM_Events_t rv = static_cast<UARM_Events_t>(rtos.event_group_sync(
            sync_groups[sync_group_index].event_group,
            static_cast<uint32_t>(set_task),
            static_cast<uint32_t>(sync_event), timeout));
        sync_groups[sync_group_index].sync_event = None;
        return rv == sync_event;
    }
    return false;
}
