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
    Scalar last_update_time = -std::numeric_limits<Scalar>::infinity();
    Scalar integral = 0;
};

Scalar pi_control(const PiParams& params, PiContext* context, const Scalar time,
                  const Scalar actual, const Scalar target);
