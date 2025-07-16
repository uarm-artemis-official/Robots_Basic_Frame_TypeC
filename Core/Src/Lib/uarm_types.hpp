#ifndef __UARM_TYPES_HPP
#define __UARM_TYPES_HPP

#include <utility>
#include "stddef.h"
#include "stdint.h"
#include "stdlib.h"

#include "uarm_defines.hpp"

/* =========================================================================
 * UARM MATH TYPES
 * ====================================================================== */
typedef struct {
    float cur_data;
    float output_data;
    float last_output_data;

    float a;  //filter parameter
} first_order_low_pass_t;

typedef struct {
    float input_data;
    float output_data;
    float last_input_data;
    float last_output_data;

    float a;  //filter parameter
} first_order_high_pass_t;

typedef struct {
    float output_data;
    float last_output_data;

    float a;  //filter parameter
} ewma_filter_t;

typedef struct {
    float window[MAX_WINDOW_SIZE];  // buffer to the data window
    size_t window_size;             // size of the window
    size_t current_index;           // current position in the window
    float sum;                      // sum of the window's elements
} sliding_mean_filter_t;

/* =========================================================================
 * UARM STRONG TYPES
 * ====================================================================== */
template <typename T, typename Parameter>
class NamedType {
   public:
    constexpr explicit NamedType(T const& value) : value_(value) {}
    explicit NamedType(T&& value) : value_(std::move(value)) {}
    T& get() { return value_; }
    T const& get() const { return value_; }

   private:
    T value_;
};

struct MeterParameter {};
using Meter = NamedType<float, MeterParameter>;

struct MetersPerSecondParameter {};
using MetersPerSecond = NamedType<float, MetersPerSecondParameter>;

struct MetersPerSecondSecondParameter {};
using MetersPerSecondSecond = NamedType<float, MetersPerSecondSecondParameter>;

struct RadianParameter {};
using Radian = NamedType<float, RadianParameter>;

struct RadiansPerSecondParameter {};
using RadiansPerSecond = NamedType<float, RadiansPerSecondParameter>;

struct RadiansPerSecondSecondParameter {};
using RadiansPerSecondSecond =
    NamedType<float, RadiansPerSecondSecondParameter>;

struct WidthParameter {};
using Width = NamedType<Meter, WidthParameter>;

struct HeightParameter {};
using Height = NamedType<Meter, HeightParameter>;

struct LengthParameter {};
using Length = NamedType<Meter, LengthParameter>;

struct AngleParameter {};
using Angle = NamedType<Radian, AngleParameter>;

#endif