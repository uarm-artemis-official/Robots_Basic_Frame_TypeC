#ifndef __DIGITAL_FILTERS_HPP
#define __DIGITAL_FILTERS_HPP

#include <array>

template <int TTaps>
class GenericFilter {
   private:
    std::array<float, TTaps> past_outputs;
    std::array<float, TTaps> a;

    std::array<float, TTaps> past_inputs;
    std::array<float, TTaps> b;

   public:
    GenericFilter(std::array<float, TTaps - 1> a_, std::array<float, TTaps> b_);
    float calc_output(float new_input);
};

#endif