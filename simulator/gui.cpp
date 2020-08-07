#include "gui.h"
#include "pi.h"
#include "sim_params.h"
#include <absl/strings/str_format.h>
#include <imgui.h>
#include <implot.h>

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
        viz_data->rolling_torques[viz_data->rolling_buffers_next_idx] =
            sim_state->torque;

        rolling_buffer_advance_idx(viz_data->rolling_timestamps.size(),
                                   &viz_data->rolling_buffers_next_idx,
                                   &viz_data->rolling_buffers_wrap_around);
    }

    const int rolling_buffer_count = get_rolling_buffer_count(
        viz_data->rolling_timestamps.size(), viz_data->rolling_buffers_next_idx,
        viz_data->rolling_buffers_wrap_around);
    const int rolling_buffer_offset = get_rolling_buffer_offset(
        viz_data->rolling_timestamps.size(), viz_data->rolling_buffers_next_idx,
        viz_data->rolling_buffers_wrap_around);

    Scalar history = 1;
    ImPlot::SetNextPlotLimitsX(sim_state->time - history, sim_state->time,
                               ImGuiCond_Always);
    ImPlot::SetNextPlotLimitsY(-sim_params->v_bus, sim_params->v_bus,
                               ImGuiCond_Once);
    if (ImPlot::BeginPlot("Phase bEMFs", "Seconds", "Volts", ImVec2(-1, 350))) {
        for (int i = 0; i < 3; ++i) {
            ImPlot::PlotLine(absl::StrFormat("Coil %d", i).c_str(),
                             viz_data->rolling_timestamps.data(),
                             viz_data->rolling_bEmfs[i].data(),
                             rolling_buffer_count, rolling_buffer_offset,
                             sizeof(Scalar));
        }
        ImPlot::EndPlot();
    }

    ImPlot::SetNextPlotLimitsX(sim_state->time - history, sim_state->time,
                               ImGuiCond_Always);
    ImPlot::SetNextPlotLimitsY(
        -sim_params->v_bus / sim_params->phase_resistance,
        sim_params->v_bus / sim_params->phase_resistance, ImGuiCond_Once);
    if (ImPlot::BeginPlot("Phase Currents", "Seconds", "Amperes",
                          ImVec2(-1, 350))) {
        for (int i = 0; i < 3; ++i) {
            ImPlot::PlotLine(absl::StrFormat("Coil %d", i).c_str(),
                             viz_data->rolling_timestamps.data(),
                             viz_data->rolling_is[i].data(),
                             rolling_buffer_count, rolling_buffer_offset,
                             sizeof(Scalar));
        }
        ImPlot::EndPlot();
    }

    ImPlot::SetNextPlotLimitsX(sim_state->time - history, sim_state->time,
                               ImGuiCond_Always);
    ImPlot::SetNextPlotLimitsY(-10, 10, ImGuiCond_Once);
    if (ImPlot::BeginPlot("Torque", "Time (seconds)", "N m", ImVec2(-1, 350))) {
        ImPlot::PlotLine("", viz_data->rolling_timestamps.data(),
                         viz_data->rolling_torques.data(), rolling_buffer_count,
                         rolling_buffer_offset, sizeof(Scalar));
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
