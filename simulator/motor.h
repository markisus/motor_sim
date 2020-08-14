#pragma once

#include "config/scalar.h"
#include "controls/pi_control.h"
#include "motor_state.h"
#include <Eigen/Dense>

PiParams make_motor_pi_params(Scalar bandwidth, Scalar resistance,
                              Scalar inductance);

Scalar get_back_emf(const Eigen::Matrix<Scalar, 5, 1>& normalized_bEmf_coeffs,
                    const Scalar electrical_angle);

void step_motor(const Scalar dt, const Scalar load_torque,
                const Eigen::Matrix<Scalar, 3, 1>& pole_voltages,
                MotorState* motor);

inline Scalar
interp_cogging_torque(const Scalar encoder_position,
                      const std::array<Scalar, 3600>& cogging_torque_map) {
    const int integral_part = int(encoder_position);
    const Scalar fractional_part = encoder_position - integral_part;
    const Scalar t1 = cogging_torque_map[integral_part];
    const Scalar t2 =
        cogging_torque_map[(integral_part + 1) % cogging_torque_map.size()];
    return t1 * (1.0 - fractional_part) + t2 * fractional_part;
}
