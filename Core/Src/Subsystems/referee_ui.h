/*
 * referee_ui.h
 *
 *  Created on: Mar 8, 2025
 *      Author: johnm
 */

#ifndef SRC_REF_UI_H_
#define SRC_REF_UI_H_

#include "subsystems_interfaces.h"
#include "subsystems_types.h"

class RefereeUI : public IRefUI {
   private:
    Referee_UI_t ref_ui;
    uint8_t ref_tx_frame[256];

   public:
    void init() override;
    void set_ui_data(referee_ui_type_t ui_type, uint8_t robot_id) override;
    void send_ui_data(uint16_t cmd_id, uint16_t len) override;
    void draw_marks() override;
    void draw_vaild_info() override;
};

#endif /* SRC_REF_UI_H_ */