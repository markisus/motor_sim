#pragma once

#include "scalar.h"
#include <limits>

struct PiParams {
    Scalar p_gain = 0;
    Scalar i_gain = 0;
    Scalar max_integral = 0;
    Scalar bias = 0;
};

struct PiContext {
    Scalar err = 0;
    Scalar integral = 0;
};

Scalar pi_control(const PiParams& params, PiContext* context, const Scalar dt,
                  const Scalar actual, const Scalar target);
