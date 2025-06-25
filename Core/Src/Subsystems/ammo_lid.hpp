#ifndef __AMMO_LID_H
#define __AMMO_LID_H

#include "subsystems_interfaces.h"
#include "subsystems_types.h"

class AmmoLid : public IAmmoLid {
   private:
    static constexpr uint16_t CLOSED_PWM_CMP = 503;
    static constexpr uint16_t OPEN_PWM_CMP = 365;
    EAmmoLidStatus lid_status;

   public:
    void init() override;
    void set_lid_status(EAmmoLidStatus new_status) override;
};

#endif