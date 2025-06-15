/****************************************************************************
 *  Copyright (C) 2019 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/
#include "ramp.hpp"
#include <algorithm>
#include "uarm_math.h"

namespace {
    constexpr float NEW_TARGET_TOLERANCE = 0.000001;
}

void ramp_init(Ramp& ramp, float max_change_) {
    ramp.target = 0;
    ramp.max_change = max_change_;
    ramp.output = 0;
    ramp.cumsum_dt = 0;
    ramp.start = 0;
}

void ramp_set_target(Ramp& ramp, float start, float new_target) {
    if (abs(ramp.target - new_target) > NEW_TARGET_TOLERANCE) {
        ramp.target = new_target;
        ramp.cumsum_dt = 0;
        ramp.start = start;
    }
}

void ramp_calc_output(Ramp& ramp, float dt) {
    float diff = ramp.target - ramp.start;
    if (abs(ramp.max_change * ramp.cumsum_dt) < abs(diff))
        ramp.cumsum_dt += dt;

    float temp_output =
        ramp.start + sign(diff) * ramp.max_change * ramp.cumsum_dt;
    ramp.output = value_limit(temp_output, std::min(ramp.start, ramp.target),
                              std::max(ramp.start, ramp.target));
}