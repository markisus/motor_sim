#include "pi_control.h"
#include <algorithm>
#include <cmath>

Scalar pi_control(const PiParams& params, PiContext* context, const Scalar time,
                  const Scalar actual, const Scalar target) {
    context->err = target - actual;

    if (std::isfinite(context->last_update_time)) {
        const Scalar dt = time - context->last_update_time;
        context->integral =
            std::clamp(context->integral + context->err * dt,
                       -params.max_integral, params.max_integral);
    }
    context->last_update_time = time;

    return params.p_gain * context->err + params.i_gain * context->integral +
           params.bias;
}
