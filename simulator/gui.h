#pragma once

#include "scalar.h"
#include "sim_state.h"
#include "util/rolling_buffer.h"
#include <array>

constexpr int kNumRollingPts = 200;

struct RollingBuffers {

    RollingBufferContext ctx{kNumRollingPts};

    std::array<Scalar, kNumRollingPts> timestamps;

    std::array<std::array<Scalar, kNumRollingPts>, 3> phase_vs;
    std::array<std::array<Scalar, kNumRollingPts>, 3> phase_currents;
    std::array<std::array<Scalar, kNumRollingPts>, 3> bEmfs;
    std::array<std::array<Scalar, kNumRollingPts>, 3> normalized_bEmfs;
    std::array<Scalar, kNumRollingPts> torque;
    std::array<Scalar, kNumRollingPts> rotor_angular_vel;
    std::array<Scalar, kNumRollingPts> pwm_level;
    std::array<std::array<Scalar, kNumRollingPts>, 3> pwm_duties;
    std::array<std::array<Scalar, kNumRollingPts>, 3> gate_states;
    std::array<Scalar, kNumRollingPts> current_q;
    std::array<Scalar, kNumRollingPts> current_d;
    std::array<Scalar, kNumRollingPts> current_q_err;
    std::array<Scalar, kNumRollingPts> current_q_integral;
    std::array<Scalar, kNumRollingPts> current_d_err;
    std::array<Scalar, kNumRollingPts> current_d_integral;
};

struct VizOptions {
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
                            const PwmState& pwm, const GateState& gate_state,
                            const FocState& foc, RollingBuffers* buffers);

void run_gui(const VizData& viz_data, VizOptions* viz_options,
             SimState* sim_state);
