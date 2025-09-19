#include "subsystems_classes.hpp"
#include "uarm_lib.hpp"

namespace ammo_lid {
    AmmoLid::AmmoLid(MW_TIM::IPWM& pwm_ref) : pwm(pwm_ref) {}

    void AmmoLid::init() {
        pwm.start(MW_TIM::Timer::TIM_1, MW_TIM::Channel::CHANNEL_1);
        set_lid_status(LidStatus::CLOSED);
    }

    void AmmoLid::set_lid_status(LidStatus new_status) {
        switch (new_status) {
            case LidStatus::OPEN:
                pwm.set_duty_cycle(ammo_lid_timer, ammo_lid_channel,
                                   OPEN_PWM_CMP);
                break;
            case LidStatus::CLOSED:
                pwm.set_duty_cycle(ammo_lid_timer, ammo_lid_channel,
                                   CLOSED_PWM_CMP);
                break;
            default:
                ASSERT(false, "Unknown new lid status.");
        }
        lid_status = new_status;
    }
}  // namespace ammo_lid