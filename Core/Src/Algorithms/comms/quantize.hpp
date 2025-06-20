#ifndef __QUANTIZE_HPP
#define __QUANTIZE_HPP

#include <stdint.h>

// TODO: Remove functions and make in_out_map more generic
// (with input/output template parameters to determine types and ranges).
int16_t quantize_float(float x, float min_x, float max_x, int16_t min_y,
                       int16_t max_y);

float inv_quantize_float(int16_t x, int16_t min_x, int16_t max_x, float min_y,
                         float max_y);

#endif