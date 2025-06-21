#include "digital_filters.hpp"
#include "uarm_lib.h"

template <int TTaps>
GenericFilter<TTaps>::GenericFilter(std::array<float, TTaps - 1> a_,
                                    std::array<float, TTaps> b_)
    : a(a_), b(b_), past_inputs({0}), past_outputs({0}) {}

template <int TTaps>
float GenericFilter<TTaps>::calc_output(float new_input) {
    float next_output = 0;

    past_inputs.at(0) = new_input;
    for (size_t i = TTaps - 1; i >= 0; i--) {
        if (i != 0)
            next_output += a.at(i) * past_outputs.at(i);
        next_output += b.at(i) * past_inputs.at(i);
        if (i != TTaps - 1) {
            past_inputs[i - 1] = past_inputs[i];
            past_outputs[i - 1] = past_outputs[i];
        }
    }
    past_outputs.at(1) = next_output;
    return next_output;
}
