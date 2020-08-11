#pragma once

#include "pi_control.h"
#include "scalar.h"
#include <Eigen/Dense>

// literals for use with switch states
constexpr int LOW = 0;
constexpr int HIGH = 1;
constexpr int OFF = 2;

struct PwmState {
    std::array<Scalar, 3> duties = {};
    Scalar period = 1.0 / 1000; // sec, 1kHz
    Scalar progress = 0;        // 0 to 1
    Scalar level = 0;           // 0 to 1 to 0 (triangular)
};

inline void step_pwm_state(const Scalar dt, PwmState* state) {
    state->progress += dt / state->period;
    while (state->progress > 1.0) {
        state->progress -= 1.0;
    }
    state->level = (state->progress < 0.5) ? 2.0 * state->progress
                                           : 2.0 * (1.0 - state->progress);
}

inline std::array<bool, 3> get_pwm_gate_command(const PwmState& state) {
    std::array<bool, 3> commanded;
    for (int i = 0; i < 3; ++i) {
        commanded[i] = state.duties[i] > state.level;
    }
    return commanded;
}

struct GateState {
    std::array<bool, 3> commanded = {};
    std::array<int, 3> actual = {OFF, OFF, OFF};
    std::array<Scalar, 3> dead_time_remaining = {};
};

constexpr int kCommutationModeNone = 0;
constexpr int kCommutationModeSixStep = 1;
constexpr int kCommutationModeFOC = 2;

struct SimState {
    Scalar time = 0;

    Scalar rotor_angle;
    Scalar rotor_angular_vel;
    Scalar rotor_angular_accel;

    Scalar electrical_angle;

    Scalar torque;

    Scalar neutral_voltage;

    int commutation_mode = kCommutationModeNone;

    Scalar six_step_phase_advance = 0; // proportion of a cycle (0 to 1)

    bool gate_controlled_by_pwm = false;

    GateState gate_state;
    PwmState pwm_state;
    PiContext current_q_pi_context;
    PiContext current_d_pi_context;

    Scalar foc_dt = 1.0 / 100; // sec, 100Hz
    Scalar foc_timer = foc_dt;

    Eigen::Matrix<Scalar, 3, 1> pole_voltages;
    Eigen::Matrix<Scalar, 3, 1> phase_voltages;
    Eigen::Matrix<Scalar, 3, 1> coil_currents;
    Eigen::Matrix<Scalar, 3, 1> bEmfs;
    Eigen::Matrix<Scalar, 3, 1>
        normalized_bEmfs; // units of V . s,
                          // aka N . m / Amps
                          // same thing as
                          // - phase torque function
                          // - derivative of rotor stator flux linkage wrt angle
};

inline void init_sim_state(SimState* state) {
    state->neutral_voltage = 0;
    state->pole_voltages.setZero();
    state->phase_voltages.setZero();
    state->coil_currents.setZero();
    state->bEmfs.setZero();
    state->rotor_angle = 0;
    state->rotor_angular_vel = 0;
    state->rotor_angular_accel = 0;
    state->electrical_angle = 0;
    state->torque = 0;
}
