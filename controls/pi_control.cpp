#include "pi_control.h"
#include <algorithm>
#include <cmath>

PiScalar pi_control(const PiParams& params, PiContext* context,
                    const PiScalar dt, const PiScalar actual,
                    const PiScalar target) {
    context->err = target - actual;
    context->integral = std::clamp(context->integral + context->err * dt,
                                   -params.max_integral, params.max_integral);

    return params.p_gain * context->err + params.i_gain * context->integral +
           params.bias;
}
