#pragma once

#include "config/scalar.h"
#include <limits>

struct PiParams {
    Scalar p_gain = 0;
    Scalar i_gain = 0;
    Scalar bias = 0;
};

struct PiContext {
    Scalar err = 0;
    Scalar integral = 0;
};

Scalar pi_control(const PiParams& params, PiContext* context, const Scalar dt,
                  const Scalar actual, const Scalar target);

inline Scalar pi_get_control(const PiParams& params, const PiContext& context) {
    return params.p_gain * context.err + params.i_gain * context.integral +
           params.bias;
}

// if controller will saturate, use back calculation of the saturation value to
// prevent windup
void pi_unwind(const PiParams& params, const Scalar saturation_value,
               PiContext* context);
