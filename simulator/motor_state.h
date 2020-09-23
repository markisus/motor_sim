#pragma once

#include "config/scalar.h"
#include "util/math_constants.h"
#include <Eigen/Dense>
#include <array>

constexpr Scalar kQAxisOffset = -kPI / 2;

struct MotorKinematicState {
    Scalar rotor_angle = 0;
    Scalar rotor_angular_vel = 0;
    Scalar rotor_angular_accel = 0;
    Scalar torque = 0;
};

struct MotorElectricalState {
    Eigen::Matrix<Scalar, 3, 1> phase_currents = {};
    Eigen::Matrix<Scalar, 3, 1> bEmfs = {};
    Eigen::Matrix<Scalar, 3, 1> normed_bEmfs =
        {}; // units of V . s,
            // aka N . m / Amps
            // same thing as
            // - phase torque function
            // - derivative of rotor stator flux linkage wrt angle
};

struct MotorParams {
    // motor characteristics
    int num_pole_pairs = 4;
    Scalar rotor_inertia = 0.1; // moment of inertia
    Scalar phase_inductance = 1e-3;
    Scalar phase_resistance = 1.0;
    // normalized bEmf aka torque/current curve
    // odd coefficients of sine fourier expansion
    Eigen::Matrix<Scalar, 5, 1> normed_bEmf_coeffs;
    std::array<Scalar, 3600> cogging_torque_map;
};

struct MotorState {
    MotorParams params;
    MotorElectricalState electrical;
    MotorKinematicState kinematic;
};

inline void init_motor_state(MotorState* motor) {
    motor->electrical.phase_currents.setZero();
    motor->electrical.bEmfs.setZero();
    motor->params.normed_bEmf_coeffs << 0.01, 0, 0, 0, 0;
}

Scalar
get_normed_bEmf(const Eigen::Matrix<Scalar, 5, 1>& normalized_bEmf_coeffs,
                const Scalar electrical_angle);

// returns normed_bEmfs and bEmfs via output pointers
void get_bEmfs(const Eigen::Matrix<Scalar, 5, 1>& normed_bEmf_coeffs,
               const Scalar electrical_angle,
               const Scalar electrical_angular_vel,
               Eigen::Matrix<Scalar, 3, 1>* normed_bEmfs,
               Eigen::Matrix<Scalar, 3, 1>* bEmfs);

Scalar
interp_cogging_torque(const Scalar rotor_angle,
                      const std::array<Scalar, 3600>& cogging_torque_map);

// assumes rotor angle > 0
Scalar get_electrical_angle(const int num_pole_pairs, const Scalar rotor_angle);

Scalar get_q_axis_electrical_angle(const int num_pole_pairs,
                                   const Scalar rotor_angle);
