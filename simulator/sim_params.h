#pragma once

#include "scalar.h"
#include <Eigen/Dense>
#include <array>

struct SimParams {
    Scalar dt = 1e-6; // 1 us
    int step_multiplier = 5e3;

    int num_pole_pairs;
    Scalar inertia_rotor; // moment of inertia
    Scalar mechanical_to_electrical_offset;

    Scalar phase_inductance;
    Scalar phase_resistance;
    Scalar v_bus;

    Scalar v_diode_active; // voltage drop,
                           // which develops current flows across flyback diode
    Scalar i_diode_active; // current,
                           // above which diode develops the  v_diode_active
                           // voltage

    // normalized bEmf aka torque/current curve
    // odd coefficients of sine fourier expansion
    Eigen::Matrix<Scalar, 5, 1> normalized_bEmf_coeffs;

    std::array<Scalar, 3600> cogging_torque_map;
    std::array<Scalar, 3600> stiction_torque_map;
};

inline void init_sim_params(SimParams* sim_params) {
    sim_params->num_pole_pairs = 4;
    sim_params->phase_inductance = 1;
    sim_params->phase_resistance = 1;
    sim_params->v_bus = 5;
    sim_params->v_diode_active = 1;
    sim_params->i_diode_active = 1e-3;
    sim_params->inertia_rotor = 1;
    sim_params->normalized_bEmf_coeffs << 1, 0, 0, 0, 0;
}
