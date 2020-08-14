#pragma once

#include "config/scalar.h"
#include "foc_state.h"
#include "simulator/motor_state.h"
#include <complex>

std::complex<Scalar>
get_desired_current_qd_non_sinusoidal(const Scalar desired_torque,
                                      const MotorState& motor);

std::complex<Scalar> get_desired_current_qd(const Scalar desired_torque,
                                            const Scalar normed_bEmf0);

void step_foc_current_controller(const std::complex<Scalar>& desired_current_qd,
                                 const MotorState& motor, FocState* foc_state);
