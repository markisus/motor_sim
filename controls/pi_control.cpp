#include "pi_control.h"
#include <algorithm>
#include <cmath>

Scalar pi_control(const PiParams& params, PiContext* context, const Scalar dt,
                  const Scalar actual, const Scalar target) {
    context->err = target - actual;
    context->integral = context->integral + context->err * dt;
    return pi_get_control(params, *context);
}

void pi_unwind(const PiParams& pi_params, const Scalar saturation_value,
               PiContext* context) {
    // solve `i_gain * I + bias = saturation_value` for the variable I
    context->integral = (saturation_value - pi_params.bias) / pi_params.i_gain;
    context->err = 0;
}
