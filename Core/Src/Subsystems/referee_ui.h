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
    void set_ui_data(referee_ui_type_t ui_type, uint8_t robot_id,
                     ref_ui_info_t ref_ui_info) override;
    void send_ui_data(uint16_t cmd_id, uint16_t len,
                      referee_ui_type_t ui_type) override;
    void draw_marks() override;
    void draw_vaild_info(uint32_t act_mode, uint32_t level) override;
};

#endif /* SRC_REF_UI_H_ */