#pragma once

#include "scalar.h"
#include <Eigen/Dense>
#include <array>

struct SimParams {
    static constexpr int kNumBEmfCoeffs = 5;

    bool paused = false;
    Scalar dt = 1.0 / 100000; // sec, 100kHz

    int step_multiplier = 5e3; // sec
    Scalar gate_dead_time =
        dt * 5; // sec
                // time during commutation when gate is neither high nor low, to
                // prevent shoot through current

    int num_pole_pairs;
    Scalar rotor_inertia; // moment of inertia
    Scalar mechanical_to_electrical_offset;

    Scalar phase_inductance;
    Scalar phase_resistance;
    Scalar bus_voltage;

    Scalar diode_active_voltage; // voltage drop,
                                 // which develops current flows across flyback
                                 // diode
    Scalar diode_active_current; // current,
                                 // above which diode develops the
                                 // v_diode_active voltage

    // normalized bEmf aka torque/current curve
    // odd coefficients of sine fourier expansion
    Eigen::Matrix<Scalar, kNumBEmfCoeffs, 1> normalized_bEmf_coeffs;

    std::array<Scalar, 3600> cogging_torque_map;
    std::array<Scalar, 3600> stiction_torque_map;
};

inline void init_sim_params(SimParams* sim_params) {
    sim_params->num_pole_pairs = 4;
    sim_params->phase_inductance = 1;
    sim_params->phase_resistance = 1;
    sim_params->bus_voltage = 5;
    sim_params->diode_active_voltage = 1;
    sim_params->diode_active_current = 1e-3;
    sim_params->rotor_inertia = 1;
    sim_params->normalized_bEmf_coeffs << 1, 0, 0, 0, 0;
}
