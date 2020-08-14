#pragma once

#include "config/scalar.h"
#include "gate_state.h"
#include "pwm_state.h"

struct BoardState {
    Scalar bus_voltage = 24;

    // time between commutations when gate is neither
    // high nor low, to prevent shoot-through current
    Scalar gate_dead_time = 0; // sec

    Scalar diode_active_voltage = 1;    // voltage drop,
                                        // which develops current flows across
                                        // flyback diode
    Scalar diode_active_current = 1e-3; // current,
                                        // above which diode develops the
                                        // v_diode_active voltage

    PwmState pwm;
    GateState gate;
};
