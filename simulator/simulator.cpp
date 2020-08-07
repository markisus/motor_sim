#include "implot.h"
#include "sine_series.h"
#include "wrappers/sdl_context.h"
#include "wrappers/sdl_imgui.h"
#include "wrappers/sdl_imgui_context.h"
#include <Eigen/Dense>
#include <absl/strings/str_format.h>
#include <array>
#include <glad/glad.h>
#include <iostream>

using Scalar = double;
constexpr Scalar kPI = 3.14159265358979323846264338327950288;

// literals for use with switch states
constexpr int LOW = 0;
constexpr int HIGH = 1;
constexpr int OFF = 2;

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

void init_sim_params(SimParams* sim_params) {
    sim_params->num_pole_pairs = 4;
    sim_params->phase_inductance = 1;
    sim_params->phase_resistance = 1;
    sim_params->v_bus = 5;
    sim_params->v_diode_active = 1;
    sim_params->i_diode_active = 1e-3;
    sim_params->inertia_rotor = 1;
    sim_params->normalized_bEmf_coeffs << 1, 0, 0, 0, 0;
}

struct SimState {
    Scalar time = 0;

    Scalar angle_rotor;
    Scalar angular_vel_rotor;
    Scalar angular_accel_rotor;

    Scalar angle_electrical;

    Scalar torque;

    Scalar v_neutral;
    std::array<int, 3> switches;
    Eigen::Matrix<Scalar, 3, 1> v_poles;
    Eigen::Matrix<Scalar, 3, 1> v_phases;
    Eigen::Matrix<Scalar, 3, 1> i_coils;
    Eigen::Matrix<Scalar, 3, 1> bEmfs;
};

void init_sim_state(SimState* state) {
    state->switches = {};
    state->v_neutral = 0;
    state->v_poles.setZero();
    state->v_phases.setZero();
    state->i_coils.setZero();
    state->bEmfs.setZero();
    state->angle_rotor = 0;
    state->angular_vel_rotor = 0;
    state->angular_accel_rotor = 0;
    state->angle_electrical = 0;
    state->torque = 0;
}

Scalar get_back_emf(const Eigen::Matrix<Scalar, 5, 1>& normalized_bEmf_coeffs,
                    const Scalar angle_electrical) {
    Eigen::Matrix<Scalar, 5, 1> sines;
    generate_odd_sine_series(/*num_terms=*/sines.rows(), angle_electrical,
                             sines.data());
    return sines.dot(normalized_bEmf_coeffs);
}

void step(const SimParams& params, SimState* state) {
    state->time += params.dt;

    // apply pole voltages
    for (int n = 0; n < 3; ++n) {
        Scalar v_pole = 0;
        switch (state->switches[n]) {
        case OFF:
            // todo: derivation
            if (state->i_coils(n) > 0) {
                state->v_poles(n) = 0;
            } else {
                state->v_poles(n) = params.v_bus;
            }
            if (std::abs(state->i_coils(n)) > params.i_diode_active) {
                state->v_poles(n) -= params.v_diode_active;
            }
            break;
        case HIGH:
            state->v_poles(n) = params.v_bus;
            break;
        case LOW:
            state->v_poles(n) = 0;
            break;
        default:
            printf("Unhandled switch case!");
            exit(-1);
        }
    }

    // compute normalized back emfs
    Eigen::Matrix<Scalar, 3, 1> normalized_bEmfs;
    normalized_bEmfs << // clang-format off
        get_back_emf(params.normalized_bEmf_coeffs, state->angle_electrical),
        get_back_emf(params.normalized_bEmf_coeffs,
                     state->angle_electrical + 2 * kPI / 3),
        get_back_emf(params.normalized_bEmf_coeffs,
                     state->angle_electrical - 2 * kPI / 3); // clang-format on

    state->bEmfs = normalized_bEmfs * state->angular_vel_rotor;

    // compute neutral point voltage
    // todo: derivation
    state->v_neutral = (state->v_poles.sum() - state->bEmfs.sum()) / 3;

    for (int n = 0; n < 3; ++n) {
        state->v_phases(n) = state->v_poles(n) - state->v_neutral;
    }

    Eigen::Matrix<Scalar, 3, 1> di_dt;
    for (int n = 0; n < 3; ++n) {
        di_dt(n) = (state->v_phases(n) - state->bEmfs(n) -
                    state->i_coils(n) * params.phase_resistance) /
                   params.phase_inductance;
    }

    state->i_coils += di_dt * params.dt;

    state->torque = state->i_coils.dot(normalized_bEmfs);

    state->angular_accel_rotor = state->torque / params.inertia_rotor;
    state->angular_vel_rotor += state->angular_accel_rotor * params.dt;
    state->angle_rotor += state->angular_vel_rotor * params.dt;
    state->angle_rotor = std::fmod(state->angle_rotor, 2 * kPI);

    state->angle_electrical = state->angle_rotor * params.num_pole_pairs;
    state->angle_electrical = std::fmod(state->angle_electrical, 2 * kPI);
}

using namespace biro;

void rolling_buffer_advance_idx(const size_t rolling_buffer_capacity,
                                int* next_idx, bool* wrap_around) {
    ++(*next_idx);
    if ((*next_idx) >= rolling_buffer_capacity) {
        (*next_idx) = 0;
        (*wrap_around) = true;
    }
}

int get_rolling_buffer_count(const size_t rolling_buffer_capacity,
                             const int next_idx, const bool wrap_around) {
    if (wrap_around) {
        return rolling_buffer_capacity;
    }
    return next_idx;
}

int get_rolling_buffer_offset(const size_t rolling_buffer_capacity,
                              const int next_idx, const bool wrap_around) {
    if (wrap_around) {
        return next_idx;
    }
    return 0;
}

constexpr int kNumRollingPts = 200;

struct VizData {
    std::array<Scalar, 50> circle_xs;
    std::array<Scalar, 50> circle_ys;

    std::array<Scalar, 2> angle_xs_rotor;
    std::array<Scalar, 2> angle_ys_rotor;

    std::array<Scalar, 2> bEmf_xs;
    std::array<Scalar, 2> bEmf_ys;

    int rolling_buffers_next_idx = 0;
    bool rolling_buffers_wrap_around = false;
    std::array<Scalar, kNumRollingPts> rolling_timestamps;
    std::array<std::array<Scalar, kNumRollingPts>, 3> rolling_phase_vs;
    std::array<std::array<Scalar, kNumRollingPts>, 3> rolling_is;
    std::array<std::array<Scalar, kNumRollingPts>, 3> rolling_bEmfs;
};

void init_viz_data(VizData* viz_data) {
    const int num_pts = viz_data->circle_xs.size();
    for (int i = 0; i < num_pts; ++i) {
        viz_data->circle_xs[i] = std::cos(Scalar(i) / (num_pts - 1) * 2 * kPI);
        viz_data->circle_ys[i] = std::sin(Scalar(i) / (num_pts - 1) * 2 * kPI);
    }
}

void run_gui(SimParams* sim_params, SimState* sim_state, bool* should_step,
             VizData* viz_data) {

    ImGui::Begin("Viz");

    // Add data points to plots
    if (*should_step) {
        viz_data->rolling_timestamps[viz_data->rolling_buffers_next_idx] =
            sim_state->time;
        for (int i = 0; i < 3; ++i) {
            viz_data->rolling_phase_vs[i][viz_data->rolling_buffers_next_idx] =
                sim_state->v_phases(i);
            viz_data->rolling_is[i][viz_data->rolling_buffers_next_idx] =
                sim_state->i_coils(i);
            viz_data->rolling_bEmfs[i][viz_data->rolling_buffers_next_idx] =
                sim_state->bEmfs(i);
        }
        rolling_buffer_advance_idx(viz_data->rolling_timestamps.size(),
                                   &viz_data->rolling_buffers_next_idx,
                                   &viz_data->rolling_buffers_wrap_around);
    }

    Scalar history = 1;
    ImPlot::SetNextPlotLimitsX(sim_state->time - history, sim_state->time,
                               ImGuiCond_Always);
    ImPlot::SetNextPlotLimitsY(-sim_params->v_bus, sim_params->v_bus,
                               ImGuiCond_Once);

    const int rolling_buffer_count = get_rolling_buffer_count(
        viz_data->rolling_timestamps.size(), viz_data->rolling_buffers_next_idx,
        viz_data->rolling_buffers_wrap_around);
    const int rolling_buffer_offset = get_rolling_buffer_offset(
        viz_data->rolling_timestamps.size(), viz_data->rolling_buffers_next_idx,
        viz_data->rolling_buffers_wrap_around);

    if (ImPlot::BeginPlot("Phase bEMFs", "Time (seconds)", "Volts",
                          ImVec2(-1, 350))) {
        for (int i = 0; i < 3; ++i) {
            ImPlot::PlotLine(absl::StrFormat("Coil %d", i).c_str(),
                             viz_data->rolling_timestamps.data(),
                             viz_data->rolling_bEmfs[i].data(),
                             rolling_buffer_count, rolling_buffer_offset,
                             sizeof(Scalar));
        }
        ImPlot::EndPlot();
    }

    ImPlot::SetNextPlotLimits(/*x_min=*/-1.5,
                              /*x_max=*/1.5,
                              /*y_min=*/-1.5,
                              /*y_max=*/1.5);
    if (ImPlot::BeginPlot("Rotor Angle", nullptr, nullptr, ImVec2{300, 300},
                          ImPlotFlags_Default & !ImPlotFlags_Legend,
                          ImPlotAxisFlags_Default & ~ImPlotAxisFlags_TickLabels,
                          ImPlotAxisFlags_Default &
                              ~ImPlotAxisFlags_TickLabels)) {
        const Scalar cos_angle = std::cos(sim_state->angle_rotor);
        const Scalar sin_angle = std::sin(sim_state->angle_rotor);
        viz_data->angle_xs_rotor = {0.5f * cos_angle, 1.2f * cos_angle};
        viz_data->angle_ys_rotor = {0.5f * sin_angle, 1.2f * sin_angle};

        ImPlot::PushStyleVar(ImPlotStyleVar_LineWeight, /*weight=*/1.0f);
        ImPlot::PlotLine("Rotor Circle", viz_data->circle_xs.data(),
                         viz_data->circle_ys.data(),
                         viz_data->circle_xs.size());
        ImPlot::PlotLine("Rotor Angle", viz_data->angle_xs_rotor.data(),
                         viz_data->angle_ys_rotor.data(),
                         viz_data->angle_xs_rotor.size());
        ImPlot::PopStyleVar(1);
        ImPlot::EndPlot();
    }
    ImGui::End();

    ImGui::Begin("Simulation");
    ImGui::Checkbox("Should Step", should_step);

    ImGui::Text("Bus Voltage: %f", sim_params->v_bus);
    ImGui::Text("Phase Inductance: %f", sim_params->phase_inductance);
    ImGui::Text("Phase Resistance: %f", sim_params->phase_resistance);
    ImGui::Text("Num Pole Pairs: %d", sim_params->num_pole_pairs);
    ImGui::Text("Rotor Moment of Inertia: %f", sim_params->inertia_rotor);

    // Commutation state
    ImGui::Text("Commutation State");
    for (int n = 0; n < 3; ++n) {
        ImGui::Text("Gate %d:", n);
        ImGui::SameLine();
        ImGui::PushID(n);
        ImGui::RadioButton("Low", &sim_state->switches[n], 0);
        ImGui::SameLine();
        ImGui::RadioButton("High", &sim_state->switches[n], 1);
        ImGui::SameLine();
        ImGui::RadioButton("Off", &sim_state->switches[n], 2);
        ImGui::PopID();
    }

    ImGui::Text("Electrical State");
    ImGui::Text("Neutral Voltage: %f", sim_state->v_neutral);
    for (int n = 0; n < 3; ++n) {
        ImGui::Text("Pole %d voltage: %f", n, sim_state->v_poles(n));
    }

    for (int n = 0; n < 3; ++n) {
        ImGui::Text("Coil %d Current: %f", n, sim_state->i_coils(n));
        ImGui::Text("Coil %d Phase Voltage: %f", n, sim_state->v_phases(n));
        ImGui::Text("Coil %d Back Emf: %f", n, sim_state->bEmfs(n));
    }

    ImGui::Text("Torque: %f", sim_state->torque);
    ImGui::Text("Rotor Angle: %f", sim_state->angle_rotor);
    ImGui::Text("Rotor Angular Velocity: %f", sim_state->angular_vel_rotor);
    ImGui::Text("Rotor Angular Acceleration: %f",
                sim_state->angular_accel_rotor);

    ImGui::End();
}

int main(int argc, char* argv[]) {
    SimParams params;
    init_sim_params(&params);

    SimState state;
    init_sim_state(&state);

    VizData viz_data;
    init_viz_data(&viz_data);

    wrappers::SdlContext sdl_context("Motor Simulator",
                                     /*width=*/1920 / 2,
                                     /*height=*/1080 / 2);

    if (sdl_context.status_ != 0) {
        return -1;
    }

    if (gladLoadGL() == 0) {
        printf("Failed to initialize OpenGL loader!\n");
        exit(-1);
    }

    wrappers::SdlImguiContext imgui_context(sdl_context.window_,
                                            sdl_context.gl_context_);

    bool quit_flag = false;
    bool should_step = false;
    while (!quit_flag) {
        quit_flag = wrappers::process_sdl_imgui_events(sdl_context.window_);
        if (quit_flag) {
            break;
        }

        wrappers::sdl_imgui_newframe(sdl_context.window_);

        run_gui(&params, &state, &should_step, &viz_data);
        if (should_step) {
            for (int i = 0; i < params.step_multiplier; ++i) {
                step(params, &state);
            }
        }

        ImGui::ShowDemoWindow();
        ImPlot::ShowDemoWindow();

        ImGui::Render();
        int display_w, display_h;
        SDL_GetWindowSize(sdl_context.window_, &display_w, &display_h);
        glViewport(0, 0, display_w, display_h);
        glClearColor(0.0, 0.0, 0.0, 0.0);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glEnable(GL_DEPTH_TEST);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        SDL_GL_SwapWindow(sdl_context.window_);
    }

    step(params, &state);
    return 0;
}
