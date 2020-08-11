#pragma once

#include <limits>

using PiScalar = double;

struct PiParams {
    PiScalar p_gain = 0;
    PiScalar i_gain = 0;
    PiScalar max_integral = std::numeric_limits<PiScalar>::infinity();
    PiScalar bias = 0;
};

struct PiContext {
    PiScalar err = 0;
    PiScalar integral = 0;
};

PiScalar pi_control(const PiParams& params, PiContext* context,
                    const PiScalar dt, const PiScalar actual,
                    const PiScalar target);
