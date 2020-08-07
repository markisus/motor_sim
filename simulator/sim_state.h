#pragma once

#include "scalar.h"
#include <Eigen/Dense>

struct SimState {
    Scalar time = 0;

    Scalar angle_rotor;
    Scalar angular_vel_rotor;
    Scalar angular_accel_rotor;

    Scalar angle_electrical;

    Scalar torque;

    Scalar v_neutral;
    std::array<int, 3> switches;
    Eigen::Matrix<Scalar, 3, 1> v_poles;
    Eigen::Matrix<Scalar, 3, 1> v_phases;
    Eigen::Matrix<Scalar, 3, 1> i_coils;
    Eigen::Matrix<Scalar, 3, 1> bEmfs;
};

inline void init_sim_state(SimState* state) {
    state->switches = {};
    state->v_neutral = 0;
    state->v_poles.setZero();
    state->v_phases.setZero();
    state->i_coils.setZero();
    state->bEmfs.setZero();
    state->angle_rotor = 0;
    state->angular_vel_rotor = 0;
    state->angular_accel_rotor = 0;
    state->angle_electrical = 0;
    state->torque = 0;
}
