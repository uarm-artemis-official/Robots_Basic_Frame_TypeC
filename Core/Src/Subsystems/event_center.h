/*
 * event_center.h
 *
 *  Created on: Mar 8, 2025
 *      Author: johnm
 */

#ifndef SRC_EVENT_CENTER_H_
#define SRC_EVENT_CENTER_H_

#include "subsystems_interfaces.h"
#include "subsystems_types.h"

typedef struct {
    Sync_Event_t sync_event;
    EventGroupHandle_t event_group;
} Sync_group_t;

class EventCenter : public IEventCenter {
   private:
    EventGroupHandle_t events_group;
    Sync_group_t sync_groups[NUM_SYNC_GROUPS];

   public:
    void init() override;
    UARM_Events_t wait_events(UARM_Events_t wait_events,
                              uint32_t timeout) override;
    void emit_events(UARM_Events_t new_events) override;
    void clear_events(UARM_Events_t clear_events) override;
    bool sync_tasks(Sync_Event_t sync_event, UARM_Events_t set_task,
                    uint32_t timeout) override;
};

#endif /* SRC_EVENT_CENTER_H_ */
