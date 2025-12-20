#include "quantize.hpp"
#include "uarm_math.hpp"

int16_t quantize_float(float x, float min_x, float max_x, int16_t min_y,
                       int16_t max_y) {
    return static_cast<int16_t>(
        std::roundf(in_out_map(x, min_x, max_x, min_y, max_y)));
}

float inv_quantize_float(int16_t x, int16_t min_x, int16_t max_x, float min_y,
                         float max_y) {
    return in_out_map(x, min_x, max_x, min_y, max_y);
}