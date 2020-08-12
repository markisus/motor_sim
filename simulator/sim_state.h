#pragma once

#include "controls/pi_control.h"
#include "gate_state.h"
#include "motor_state.h"
#include "pwm_state.h"
#include "scalar.h"
#include <Eigen/Dense>

constexpr int kCommutationModeNone = 0;
constexpr int kCommutationModeSixStep = 1;
constexpr int kCommutationModeFOC = 2;

struct FocState {
    Scalar period = 1.0 / 1500; // sec, 1.5kHz
    Scalar timer = 0;

    PiParams i_controller_params;
    PiContext iq_controller;
    PiContext id_controller;

    // output command
    std::complex<Scalar> voltage_qd;
};

struct SimState {
    Scalar time = 0;
    bool paused = false;
    Scalar dt = 1.0 / 1000000; // sec, 1MHz
    int step_multiplier = 5e3; // sec
    Scalar gate_dead_time = 2*dt; // sec
                               // time during commutation when gate is neither
                               // high nor low, to prevent shoot through current
    Scalar bus_voltage = 24;
    Scalar diode_active_voltage = 1;    // voltage drop,
                                        // which develops current flows across
                                        // flyback diode
    Scalar diode_active_current = 1e-3; // current,
                                        // above which diode develops the
                                        // v_diode_active voltage

    MotorState motor;
    PwmState pwm;
    int commutation_mode = kCommutationModeNone;
    Scalar six_step_phase_advance = 0; // proportion of a cycle (0 to 1)
    FocState foc;
    GateState gate;
};

inline void init_sim_state(SimState* state) { init_motor_state(&state->motor); }
