#pragma once

#include "config/scalar.h"
#include "controls/pi_control.h"
#include "motor_state.h"
#include <Eigen/Dense>

PiParams make_motor_pi_params(Scalar bandwidth, Scalar resistance,
                              Scalar inductance);

Eigen::Matrix<Scalar, 3, 1>
get_phase_voltages(const Eigen::Matrix<Scalar, 3, 1>& pole_voltages,
                   const Eigen::Matrix<Scalar, 3, 1>& bEmfs);

void step_motor_electrical(const Scalar dt,
                           const Eigen::Matrix<Scalar, 3, 1>& pole_voltages,
                           const Scalar electrical_angle,
                           const Scalar electrical_angular_vel,
                           const MotorParams& motor_params,
                           MotorElectricalState* motor_electrical);

void step_motor(const Scalar dt, const Scalar load_torque,
                const Eigen::Matrix<Scalar, 3, 1>& pole_voltages,
                MotorState* motor);
