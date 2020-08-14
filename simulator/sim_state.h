#pragma once

#include "board/board_state.h"
#include "config/scalar.h"
#include "controls/foc_state.h"
#include "controls/pi_control.h"
#include "motor_state.h"
#include <Eigen/Dense>

constexpr int kCommutationModeNone = 0;
constexpr int kCommutationModeSixStep = 1;
constexpr int kCommutationModeFOC = 2;

struct SimState {
    Scalar time = 0;
    bool paused = false;
    Scalar dt = 1.0 / 1000000; // sec, 1MHz
    int step_multiplier = 100; // sec

    Scalar load_torque = 0;

    BoardState board;
    MotorState motor;

    int commutation_mode = kCommutationModeNone;

    // six step state
    Scalar six_step_phase_advance = 0; // proportion of a cycle (0 to 1)

    // foc state
    Scalar foc_desired_torque = 0.0;
    bool foc_use_qd_decoupling = false;
    bool foc_use_cogging_compensation = false;
    bool foc_non_sinusoidal_drive_mode = false;
    FocState foc;
};

inline void init_sim_state(SimState* state) {
    init_motor_state(&state->motor);
    state->board.gate.dead_time = 2 * state->dt;
}
