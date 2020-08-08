#include "gui.h"
#include "pi.h"
#include "scalar.h"
#include "sim_params.h"
#include <absl/strings/str_format.h>
#include <imgui.h>
#include <implot.h>

constexpr Scalar kRollingHistory = 1; // sec
constexpr int kPlotHeight = 250;      // sec
constexpr int kPlotWidth = 350;       // sec

void init_viz_data(VizData* viz_data) {
    const int num_pts = viz_data->circle_xs.size();
    for (int i = 0; i < num_pts; ++i) {
        viz_data->circle_xs[i] = std::cos(Scalar(i) / (num_pts - 1) * 2 * kPI);
        viz_data->circle_ys[i] = std::sin(Scalar(i) / (num_pts - 1) * 2 * kPI);
    }
}

uint32_t get_coil_color(int coil, float alpha) {
    switch (coil) {
    case 0:
        return ImColor(0.0f, 0.7490196228f, 1.0f, alpha); // DeepSkyBlue
    case 1:
        return ImColor(1.0f, 0.0f, 0.0f, alpha); // Red
    case 2:
        return ImColor(0.4980392158f, 1.0f, 0.0f, alpha); // Greens
    default:
        printf("Unhandled coil color %d\n", coil);
        exit(-1);
        return -1;
    }
}

void draw_electrical_plot(const int count, const int offset,
                          SimParams* sim_params, SimState* sim_state,
                          VizData* viz_data) {

    ImGui::Text("Left Axis: Volts");
    ImGui::Checkbox("Show Phase Voltages", &viz_data->show_phase_voltages);
    ImGui::SameLine();
    ImGui::Checkbox("Show bEmfs", &viz_data->show_bEmfs);
    ImGui::Text("Right Axis: Amperes");
    ImGui::Checkbox("Show Phase Currents", &viz_data->show_phase_currents);

    if (viz_data->show_bEmfs || viz_data->show_phase_currents ||
        viz_data->show_phase_voltages) {
        ImGui::Checkbox("Show Coil 0", &viz_data->coil_visible[0]);
        ImGui::SameLine();
        ImGui::Checkbox("Show Coil 1", &viz_data->coil_visible[1]);
        ImGui::SameLine();
        ImGui::Checkbox("Show Coil 2", &viz_data->coil_visible[2]);
    }

    ImPlot::SetNextPlotLimitsX(sim_state->time - kRollingHistory,
                               sim_state->time, ImGuiCond_Always);

    const bool have_voltage_axis =
        viz_data->show_bEmfs || viz_data->show_phase_voltages;
    const bool have_current_axis = viz_data->show_phase_currents;

    ImPlot::SetNextPlotLimitsY(-sim_params->bus_voltage,
                               sim_params->bus_voltage, ImGuiCond_Once, 0);

    ImPlot::SetNextPlotLimitsY(
        -0.5 * sim_params->bus_voltage / sim_params->phase_resistance,
        0.5 * sim_params->bus_voltage / sim_params->phase_resistance,
        ImGuiCond_Once, 1);

    if (ImPlot::BeginPlot("Electrical Plots", "Seconds", nullptr,
                          ImVec2(kPlotWidth, kPlotHeight),
                          ImPlotFlags_Default | ImPlotFlags_YAxis2)) {
        for (int i = 0; i < 3; ++i) {
            if (!viz_data->coil_visible[i]) {
                continue;
            }

            if (viz_data->show_phase_voltages) {
                ImPlot::SetPlotYAxis(0);
                ImPlot::PushStyleColor(ImPlotCol_Line, get_coil_color(i, 1.0f));
                ImPlot::PushStyleVar(ImPlotStyleVar_LineWeight, 1.0f);
                ImPlot::PlotLine(absl::StrFormat("Coil %d Voltage", i).c_str(),
                                 viz_data->rolling_timestamps.data(),
                                 viz_data->rolling_phase_vs[i].data(), count,
                                 offset, sizeof(Scalar));
                ImPlot::PopStyleVar();
                ImPlot::PopStyleColor();
            }

            if (viz_data->show_bEmfs) {
                ImPlot::SetPlotYAxis(0);
                ImPlot::PushStyleColor(ImPlotCol_Line, get_coil_color(i, 0.5f));
                ImPlot::PlotLine(absl::StrFormat("Coil %d bEmf", i).c_str(),
                                 viz_data->rolling_timestamps.data(),
                                 viz_data->rolling_bEmfs[i].data(), count,
                                 offset, sizeof(Scalar));
                ImPlot::PopStyleColor();
            }

            if (viz_data->show_phase_currents) {
                ImPlot::SetPlotYAxis(1);
                ImPlot::PushStyleColor(ImPlotCol_Line,
                                       get_coil_color(i, 0.25f));
                ImPlot::PushStyleVar(ImPlotStyleVar_LineWeight, 4.0f);
                ImPlot::PlotLine(absl::StrFormat("Coil %d Current", i).c_str(),
                                 viz_data->rolling_timestamps.data(),
                                 viz_data->rolling_is[i].data(), count, offset,
                                 sizeof(Scalar));
                ImPlot::PopStyleVar();
                ImPlot::PopStyleColor();
            }
        }
        ImPlot::EndPlot();
    }
}

void draw_phase_currents_plot(const int count, const int offset,
                              SimParams* sim_params, SimState* sim_state,
                              VizData* viz_data) {
    ImPlot::SetNextPlotLimitsX(sim_state->time - kRollingHistory,
                               sim_state->time, ImGuiCond_Always);
    ImPlot::SetNextPlotLimitsY(
        -sim_params->bus_voltage / sim_params->phase_resistance,
        sim_params->bus_voltage / sim_params->phase_resistance, ImGuiCond_Once);
    if (ImPlot::BeginPlot("Phase Currents", "Seconds", "Amperes",
                          ImVec2(kPlotWidth, kPlotHeight))) {
        for (int i = 0; i < 3; ++i) {
            ImPlot::PlotLine(absl::StrFormat("Coil %d", i).c_str(),
                             viz_data->rolling_timestamps.data(),
                             viz_data->rolling_is[i].data(), count, offset,
                             sizeof(Scalar));
        }
        ImPlot::EndPlot();
    }
}

void draw_torque_plot(const int count, const int offset, SimParams* sim_params,
                      SimState* sim_state, VizData* viz_data) {
    ImPlot::SetNextPlotLimitsX(sim_state->time - kRollingHistory,
                               sim_state->time, ImGuiCond_Always);
    ImPlot::SetNextPlotLimitsY(-10, 10, ImGuiCond_Once);
    if (ImPlot::BeginPlot("Torque", "Seconds", "N . m",
                          ImVec2(kPlotWidth, kPlotHeight))) {
        ImPlot::PlotLine("", viz_data->rolling_timestamps.data(),
                         viz_data->rolling_torques.data(), count, offset,
                         sizeof(Scalar));
        ImPlot::EndPlot();
    }
}

void draw_rotor_angular_vel_plot(const int count, const int offset,
                                 SimParams* sim_params, SimState* sim_state,
                                 VizData* viz_data) {
    ImPlot::SetNextPlotLimitsX(sim_state->time - kRollingHistory,
                               sim_state->time, ImGuiCond_Always);
    ImPlot::SetNextPlotLimitsY(-10, 10, ImGuiCond_Once);
    if (ImPlot::BeginPlot("Rotor Angular Vel", "Seconds", "Radians / Sec",
                          ImVec2(kPlotWidth, kPlotHeight))) {
        ImPlot::PlotLine("", viz_data->rolling_timestamps.data(),
                         viz_data->rolling_rotor_angular_vel.data(), count,
                         offset, sizeof(Scalar));
        ImPlot::EndPlot();
    }
}

void update_rolling_buffers(SimState* sim_state, VizData* viz_data) {
    viz_data->rolling_timestamps[viz_data->rolling_buffers_next_idx] =
        sim_state->time;
    for (int i = 0; i < 3; ++i) {
        viz_data->rolling_phase_vs[i][viz_data->rolling_buffers_next_idx] =
            sim_state->phase_voltages(i);
        viz_data->rolling_is[i][viz_data->rolling_buffers_next_idx] =
            sim_state->coil_currents(i);
        viz_data->rolling_bEmfs[i][viz_data->rolling_buffers_next_idx] =
            sim_state->bEmfs(i);
    }

    viz_data->rolling_rotor_angular_vel[viz_data->rolling_buffers_next_idx] =
        sim_state->rotor_angular_vel;

    viz_data->rolling_torques[viz_data->rolling_buffers_next_idx] =
        sim_state->torque;

    rolling_buffer_advance_idx(viz_data->rolling_timestamps.size(),
                               &viz_data->rolling_buffers_next_idx,
                               &viz_data->rolling_buffers_wrap_around);
}

void draw_rotor_plot(SimState* sim_state, VizData* viz_data) {
    ImPlot::SetNextPlotLimits(/*x_min=*/-1.5,
                              /*x_max=*/1.5,
                              /*y_min=*/-1.5,
                              /*y_max=*/1.5);
    if (ImPlot::BeginPlot("Rotor Angle", nullptr, nullptr, ImVec2{300, 300},
                          ImPlotFlags_Default & !ImPlotFlags_Legend,
                          ImPlotAxisFlags_Default & ~ImPlotAxisFlags_TickLabels,
                          ImPlotAxisFlags_Default &
                              ~ImPlotAxisFlags_TickLabels)) {
        const Scalar cos_angle = std::cos(sim_state->rotor_angle);
        const Scalar sin_angle = std::sin(sim_state->rotor_angle);
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
}

bool order_of_magnitude_control(const char* label, Scalar* controllee,
                                const int order_of_magnitude = 4) {
    bool interacted = false;
    Scalar l10 = std::log10(*controllee);

    int integral_part = int(l10);
    float fractional_part = float(l10 - integral_part);

    while (fractional_part < 0) {
        --integral_part;
        ++fractional_part;
    }

    // 10^(integral_part + fractional_part) =
    // 10^integral_part * base
    float base = std::pow(10, fractional_part);

    ImGui::Text(label);
    ImGui::SameLine();
    ImGui::Text("%fe%d", base, integral_part);
    ImGui::PushID(label);
    interacted = ImGui::SliderFloat("mantissa", &base, 1.0f, 9.99f);
    interacted = ImGui::SliderInt("exponent (base 10)", &integral_part,
                                  -order_of_magnitude, order_of_magnitude) ||
                 interacted;
    ImGui::PopID();

    *controllee = base * std::pow(10, integral_part);

    return interacted;
}

// imgui doesn't have SliderDouble
bool Slider(const char* label, Scalar* scalar, Scalar low, Scalar high) {
    float wrapped = *scalar;
    const bool interacted =
        ImGui::SliderFloat(label, &wrapped, float(low), float(high));
    *scalar = Scalar(wrapped);
    return interacted;
}

void run_gui(SimParams* sim_params, SimState* sim_state, bool* should_step,
             VizData* viz_data) {

    ImGui::Begin("Simulation Params");

    Slider("Rotor Moment of Inertia (kg m^2)", &sim_params->rotor_inertia, 0.1,
           10);

    Slider("Bus Voltage", &sim_params->bus_voltage, 1.0, 120);
    Slider("Diode Active Voltage", &sim_params->diode_active_voltage, 0.0, 5);

    order_of_magnitude_control("Phase Inductance",
                               &sim_params->phase_inductance);
    order_of_magnitude_control("Phase Resistance",
                               &sim_params->phase_resistance);

    ImGui::End();

    ImGui::Begin("Rotor Viz");
    draw_rotor_plot(sim_state, viz_data);
    ImGui::End();

    ImGui::Begin("Plots");

    if (*should_step) {
        update_rolling_buffers(sim_state, viz_data);
    }

    const int rolling_buffer_count = get_rolling_buffer_count(
        viz_data->rolling_timestamps.size(), viz_data->rolling_buffers_next_idx,
        viz_data->rolling_buffers_wrap_around);
    const int rolling_buffer_offset = get_rolling_buffer_offset(
        viz_data->rolling_timestamps.size(), viz_data->rolling_buffers_next_idx,
        viz_data->rolling_buffers_wrap_around);

    if (viz_data->show_bEmfs || viz_data->show_phase_currents) {
        draw_electrical_plot(rolling_buffer_count, rolling_buffer_offset,
                             sim_params, sim_state, viz_data);
    }

    // dynamics
    draw_rotor_angular_vel_plot(rolling_buffer_count, rolling_buffer_offset,
                                sim_params, sim_state, viz_data);
    ImGui::SameLine();
    draw_torque_plot(rolling_buffer_count, rolling_buffer_offset, sim_params,
                     sim_state, viz_data);

    ImGui::End();

    ImGui::Begin("Simulation");
    ImGui::Checkbox("Should Step", should_step);

    ImGui::Text("Bus Voltage: %f", sim_params->bus_voltage);
    ImGui::Text("Phase Inductance: %f", sim_params->phase_inductance);
    ImGui::Text("Phase Resistance: %f", sim_params->phase_resistance);
    ImGui::Text("Num Pole Pairs: %d", sim_params->num_pole_pairs);
    ImGui::Text("Rotor Moment of Inertia: %f", sim_params->rotor_inertia);

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
    ImGui::Text("Neutral Voltage: %f", sim_state->neutral_voltage);
    for (int n = 0; n < 3; ++n) {
        ImGui::Text("Pole %d voltage: %f", n, sim_state->pole_voltages(n));
    }

    for (int n = 0; n < 3; ++n) {
        ImGui::Text("Coil %d Current: %f", n, sim_state->coil_currents(n));
        ImGui::Text("Coil %d Phase Voltage: %f", n,
                    sim_state->phase_voltages(n));
        ImGui::Text("Coil %d Back Emf: %f", n, sim_state->bEmfs(n));
    }

    ImGui::Text("Torque: %f", sim_state->torque);
    ImGui::Text("Rotor Angle: %f", sim_state->rotor_angle);
    ImGui::Text("Rotor Angular Velocity: %f", sim_state->rotor_angular_vel);
    ImGui::Text("Rotor Angular Acceleration: %f",
                sim_state->rotor_angular_accel);

    ImGui::End();
}
