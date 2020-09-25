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

Eigen::Matrix<Scalar, 3, 1>
get_di_dt(const Scalar phase_resistance, const Scalar phase_inductance,
          const Eigen::Matrix<Scalar, 3, 1>& pole_voltages,
          const Eigen::Matrix<Scalar, 3, 1>& bEmfs,
          const Eigen::Matrix<Scalar, 3, 1>& phase_currents);

void step_motor_electrical(const Scalar dt, const Scalar phase_resistance,
                           const Scalar phase_inductance,
                           const Eigen::Matrix<Scalar, 3, 1>& pole_voltages,
                           MotorElectricalState* motor_electrical);

void step_motor(const Scalar dt, const Scalar load_torque,
                const Eigen::Matrix<Scalar, 3, 1>& pole_voltages,
                MotorState* motor);
