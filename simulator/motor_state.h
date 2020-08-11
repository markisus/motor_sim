#pragma once

#include "scalar.h"
#include <Eigen/Dense>

struct MotorState {
    Scalar rotor_angle = 0;
    Scalar rotor_angular_vel = 0;
    Scalar rotor_angular_accel = 0;
    Scalar electrical_angle = 0;
    Scalar torque = 0;
    Scalar neutral_voltage = 0;
    Eigen::Matrix<Scalar, 3, 1> pole_voltages =
        {}; // does this zero the entries?
    Eigen::Matrix<Scalar, 3, 1> phase_voltages = {};
    Eigen::Matrix<Scalar, 3, 1> phase_currents = {};
    Eigen::Matrix<Scalar, 3, 1> bEmfs = {};
    Eigen::Matrix<Scalar, 3, 1> normalized_bEmfs =
        {}; // units of V . s,
            // aka N . m / Amps
            // same thing as
            // - phase torque function
            // - derivative of rotor stator flux linkage wrt angle
};

inline void init_motor_state(MotorState* motor) {
    motor->pole_voltages.setZero();
    motor->phase_voltages.setZero();
    motor->phase_currents.setZero();
    motor->bEmfs.setZero();
}
