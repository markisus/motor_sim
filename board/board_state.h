#pragma once

#include "config/scalar.h"
#include "gate_state.h"
#include "pwm_state.h"

struct BoardState {
    Scalar bus_voltage = 24;

    PwmState pwm;
    GateState gate;
};
