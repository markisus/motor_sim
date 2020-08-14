#pragma once

#include "controls/pi_control.h"
#include "motor_state.h"
#include "scalar.h"
#include <Eigen/Dense>

PiParams make_motor_pi_params(Scalar bandwidth, Scalar resistance,
                              Scalar inductance);

Scalar get_back_emf(const Eigen::Matrix<Scalar, 5, 1>& normalized_bEmf_coeffs,
                    const Scalar electrical_angle);

void step_motor(const Scalar dt, const Scalar load_torque,
                const Eigen::Matrix<Scalar, 3, 1>& pole_voltages,
                MotorState* motor);
