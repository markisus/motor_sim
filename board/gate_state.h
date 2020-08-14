#pragma once

#include "config/scalar.h"
#include <Eigen/Dense>
#include <array>

// literals for use with switch states
constexpr int LOW = 0;
constexpr int HIGH = 1;
constexpr int OFF = 2;

struct GateState {
    // time between commutations when gate is neither
    // high nor low, to prevent shoot-through current
    Scalar dead_time = 0; // sec

    Scalar diode_active_voltage = 1;    // voltage drop,
                                        // which develops current flows across
                                        // flyback diode
    Scalar diode_active_current = 1e-3; // current,
                                        // above which diode develops the
                                        // v_diode_active voltage

    std::array<bool, 3> commanded = {};
    std::array<int, 3> actual = {OFF, OFF, OFF};
    std::array<Scalar, 3> dead_time_remaining = {};
};

inline void update_gate_state(const Scalar dt, GateState* gate_state) {
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
            gate_state->dead_time_remaining[i] = gate_state->dead_time - dt;

            // see if sufficient dead time has been acheived
            // simulation speed is too coarse to see the dead time
            if (gate_state->dead_time_remaining[i] <= 0) {
                gate_state->actual[i] = command;
            }
        }
    }
}

inline Eigen::Matrix<Scalar, 3, 1>
get_pole_voltages(const Scalar bus_voltage,
                  const Eigen::Matrix<Scalar, 3, 1>& phase_currents,
                  const GateState& gate) {
    Eigen::Matrix<Scalar, 3, 1> pole_voltages;
    for (int i = 0; i < 3; ++i) {
        Scalar v_pole = 0;
        switch (gate.actual[i]) {
        case OFF:
            // todo: derivation
            if (phase_currents(i) > 0) {
                pole_voltages(i) = 0;
            } else {
                pole_voltages(i) = bus_voltage;
            }
            if (std::abs(phase_currents(i)) > gate.diode_active_current) {
                pole_voltages(i) -= gate.diode_active_voltage;
            }
            break;
        case HIGH:
            pole_voltages(i) = bus_voltage;
            break;
        case LOW:
            pole_voltages(i) = 0;
            break;
        default:
            printf("Unhandled switch case!");
            exit(-1);
        }
    }
    return pole_voltages;
}
