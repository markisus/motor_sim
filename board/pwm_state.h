#pragma once

#include "config/scalar.h"
#include "util/time.h"
#include <array>

struct PwmState {
    std::array<Scalar, 3> duties = {};
    Scalar period = 1.0 / 15000; // sec, 15kHz
    Scalar timer = 0;            // 0 to 1
    Scalar level = 0; // triangle wave from 0 to 1, lasting one period
};

inline bool step_pwm_state(const Scalar dt, PwmState* state) {
    const bool ticked = periodic_timer(state->period, dt, &state->timer);
    const Scalar progress = state->timer / state->period;
    state->level = progress < 0.5 ? 2.0 * progress : 2.0 * (1.0 - progress);
    return ticked;
}

inline std::array<bool, 3> get_pwm_gate_command(const PwmState& state) {
    std::array<bool, 3> commanded;
    for (int i = 0; i < 3; ++i) {
        commanded[i] = state.duties[i] > state.level;
    }
    return commanded;
}
