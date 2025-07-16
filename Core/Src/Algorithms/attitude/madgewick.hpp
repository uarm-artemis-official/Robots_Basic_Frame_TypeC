#ifndef __MADGEWICK_HPP
#define __MADGEWICK_HPP

#include "attitude_types.h"
#include "uarm_lib.hpp"
#include "uarm_math.hpp"

class Madgewick_Filter {
   private:
    uint32_t sampling_rate;
    float beta;
    float q0;
    float q1;
    float q2;
    float q3;

   public:
    Madgewick_Filter(uint32_t sampling_rate_, float beta_);
    void calc_imu(float ax, float ay, float az, float gx, float gy, float gz);
    void calc_marg(float ax, float ay, float az, float gx, float gy, float gz,
                   float mx, float my, float mz);
    void get_attitude(Attitude_t& atti);
};

#endif