#pragma once

#include "scalar.h"
#include <array>

// literals for use with switch states
constexpr int LOW = 0;
constexpr int HIGH = 1;
constexpr int OFF = 2;

struct GateState {
    std::array<bool, 3> commanded = {};
    std::array<int, 3> actual = {OFF, OFF, OFF};
    std::array<Scalar, 3> dead_time_remaining = {};
};

inline void update_gate_state(const Scalar dead_time, const Scalar dt,
                              GateState* gate_state) {
    for (int i = 0; i < 3; ++i) {
        const int command = gate_state->commanded[i] ? HIGH : LOW;

        if (gate_state->actual[i] == command) {
            // nothing to do here
            continue;
        }

        if (gate_state->actual[i] == OFF) {
            gate_state->dead_time_remaining[i] -= dt;

            // see if sufficient dead time has been acheived
            if (gate_state->dead_time_remaining[i] <= 0) {
                gate_state->actual[i] = command;
            }
        } else {
            // gate is actually on - need to enter dead time
            gate_state->actual[i] = OFF;
            gate_state->dead_time_remaining[i] = dead_time - dt;

            // see if sufficient dead time has been acheived
            // simulation speed is too coarse to see the dead time
            if (gate_state->dead_time_remaining[i] <= 0) {
                gate_state->actual[i] = command;
            }
        }
    }
}
