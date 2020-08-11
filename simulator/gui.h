#pragma once

#include "scalar.h"
#include "sim_state.h"
#include <array>

inline void rolling_buffer_advance_idx(const size_t rolling_buffer_capacity,
                                       int* next_idx, bool* wrap_around) {
    ++(*next_idx);
    if ((*next_idx) >= rolling_buffer_capacity) {
        (*next_idx) = 0;
        (*wrap_around) = true;
    }
}

inline int get_rolling_buffer_count(const size_t rolling_buffer_capacity,
                                    const int next_idx,
                                    const bool wrap_around) {
    if (wrap_around) {
        return rolling_buffer_capacity;
    }
    return next_idx;
}

inline int get_rolling_buffer_offset(const size_t rolling_buffer_capacity,
                                     const int next_idx,
                                     const bool wrap_around) {
    if (wrap_around) {
        return next_idx;
    }
    return 0;
}

constexpr int kNumRollingPts = 200;

struct RollingBuffers {
    int next_idx = 0;
    bool wrap_around = false;

    std::array<Scalar, kNumRollingPts> timestamps;

    std::array<std::array<Scalar, kNumRollingPts>, 3> phase_vs;
    std::array<std::array<Scalar, kNumRollingPts>, 3> phase_currents;
    std::array<std::array<Scalar, kNumRollingPts>, 3> bEmfs;
    std::array<std::array<Scalar, kNumRollingPts>, 3> normalized_bEmfs;
    std::array<Scalar, kNumRollingPts> torque;
    std::array<Scalar, kNumRollingPts> rotor_angular_vel;
    std::array<Scalar, kNumRollingPts> pwm_level;
    std::array<std::array<Scalar, kNumRollingPts>, 3> pwm_duties;
    std::array<Scalar, kNumRollingPts> current_q_err;
    std::array<Scalar, kNumRollingPts> current_q_integral;
    std::array<Scalar, kNumRollingPts> current_d_err;
    std::array<Scalar, kNumRollingPts> current_d_integral;
};

struct VizOptions {
    bool show_bEmfs = false;
    bool show_normalized_bEmfs = true;
    bool show_phase_currents = true;
    bool show_phase_voltages = true;
    bool use_rotor_frame = true; // space vector viz
    float rolling_history = 1;   // sec
    std::array<bool, 3> coil_visible = {true, false, false};
};

struct VizData {
    std::array<Scalar, 50> circle_xs;
    std::array<Scalar, 50> circle_ys;
    std::array<uint32_t, 3> coil_colors;

    RollingBuffers rolling_buffers;
};

void init_viz_data(VizData* viz_data);

void update_rolling_buffers(const Scalar time, const MotorState& motor,
                            const PwmState& pwm, const FocState& foc,
                            RollingBuffers* buffers);

void run_gui(const VizData& viz_data, VizOptions* viz_options,
             SimState* sim_state);
