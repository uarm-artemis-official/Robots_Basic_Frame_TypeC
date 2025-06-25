#include "ammo_lid.hpp"
#include "timers_handler.h"
#include "uarm_lib.h"

void AmmoLid::init() {
    start_pwm(AMMO_LID, CHANNEL_1);

    set_lid_status(EAmmoLidStatus::CLOSED);
}

void AmmoLid::set_lid_status(EAmmoLidStatus new_status) {
    switch (new_status) {
        case EAmmoLidStatus::OPEN:
            set_pwm_compare_value(AMMO_LID, CHANNEL_1, OPEN_PWM_CMP);
            break;
        case EAmmoLidStatus::CLOSED:
            set_pwm_compare_value(AMMO_LID, CHANNEL_1, CLOSED_PWM_CMP);
            break;
        default:
            ASSERT(false, "Unknown new lid status.");
    }
    lid_status = new_status;
}