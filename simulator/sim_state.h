#pragma once

#include "gate_state.h"
#include "motor_state.h"
#include "pi_control.h"
#include "pwm_state.h"
#include "scalar.h"
#include <Eigen/Dense>

constexpr int kCommutationModeNone = 0;
constexpr int kCommutationModeSixStep = 1;
constexpr int kCommutationModeFOC = 2;

struct FocState {
    Scalar period = 1.0 / 100; // sec, 100Hz
    Scalar timer = 0;

    PiParams i_controller_params;
    PiContext iq_controller;
    PiContext id_controller;
};

struct SimState {
    Scalar time = 0;
    MotorState motor;
    PwmState pwm;
    int commutation_mode = kCommutationModeNone;
    Scalar six_step_phase_advance = 0; // proportion of a cycle (0 to 1)
    FocState foc;
    GateState gate;
};

inline void init_sim_state(SimState* state) {
    init_motor_state(&state->motor); // needed?
}
