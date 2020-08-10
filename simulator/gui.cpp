#include "gui.h"
#include "clarke_transform.h"
#include "conversions.h"
#include "pi.h"
#include "scalar.h"
#include "sim_params.h"
#include <absl/strings/str_format.h>
#include <imgui.h>
#include <implot.h>

constexpr int kPlotHeight = 250; // sec
constexpr int kPlotWidth = -1;   // sec

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

    ImGui::Text("Left Axis (Volts):");
    ImGui::Checkbox("Phase Voltages", &viz_data->show_phase_voltages);
    ImGui::SameLine();
    ImGui::Checkbox("Normed bEmfs (Volt . seconds)",
                    &viz_data->show_normalized_bEmfs);
    ImGui::SameLine();
    ImGui::Checkbox("bEmfs", &viz_data->show_bEmfs);
    ImGui::Text("Right Axis (Amperes):");
    ImGui::SameLine();
    ImGui::Checkbox("Phase Currents", &viz_data->show_phase_currents);

    ImGui::Text("Coil Visibility:");
    ImGui::SameLine();
    if (viz_data->show_bEmfs || viz_data->show_phase_currents ||
        viz_data->show_phase_voltages) {
        ImGui::Checkbox("Coil 0", &viz_data->coil_visible[0]);
        ImGui::SameLine();
        ImGui::Checkbox("Coil 1", &viz_data->coil_visible[1]);
        ImGui::SameLine();
        ImGui::Checkbox("Coil 2", &viz_data->coil_visible[2]);
    }

    ImPlot::SetNextPlotLimitsX(sim_state->time - viz_data->rolling_history,
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

    if (ImPlot::BeginPlot("##Electrical Plots", "Seconds", nullptr,
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

            if (viz_data->show_normalized_bEmfs) {
                ImPlot::SetPlotYAxis(0);
                ImPlot::PushStyleColor(ImPlotCol_Line, get_coil_color(i, 0.8f));
                ImPlot::PlotLine(
                    absl::StrFormat("Coil %d Normed bEmf", i).c_str(),
                    viz_data->rolling_timestamps.data(),
                    viz_data->rolling_normalized_bEmfs[i].data(), count, offset,
                    sizeof(Scalar));
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
                ImPlot::PushStyleColor(ImPlotCol_Line, get_coil_color(i, 0.3f));
                ImPlot::PushStyleVar(ImPlotStyleVar_LineWeight, 4.0f);
                ImPlot::PlotLine(absl::StrFormat("Coil %d Current", i).c_str(),
                                 viz_data->rolling_timestamps.data(),
                                 viz_data->rolling_phase_currents[i].data(),
                                 count, offset, sizeof(Scalar));
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
    ImPlot::SetNextPlotLimitsX(sim_state->time - viz_data->rolling_history,
                               sim_state->time, ImGuiCond_Always);
    ImPlot::SetNextPlotLimitsY(
        -sim_params->bus_voltage / sim_params->phase_resistance,
        sim_params->bus_voltage / sim_params->phase_resistance, ImGuiCond_Once);
    if (ImPlot::BeginPlot("Phase Currents", "Seconds", "Amperes",
                          ImVec2(kPlotWidth, kPlotHeight))) {
        for (int i = 0; i < 3; ++i) {
            ImPlot::PlotLine(absl::StrFormat("Coil %d", i).c_str(),
                             viz_data->rolling_timestamps.data(),
                             viz_data->rolling_phase_currents[i].data(), count,
                             offset, sizeof(Scalar));
        }
        ImPlot::EndPlot();
    }
}

void draw_torque_plot(const int count, const int offset, SimParams* sim_params,
                      SimState* sim_state, VizData* viz_data) {
    ImPlot::SetNextPlotLimitsX(sim_state->time - viz_data->rolling_history,
                               sim_state->time, ImGuiCond_Always);
    ImPlot::SetNextPlotLimitsY(-10, 10, ImGuiCond_Once);
    if (ImPlot::BeginPlot("Torque", "Seconds", "N . m",
                          ImVec2(kPlotWidth, kPlotHeight))) {
        ImPlot::PlotLine("", viz_data->rolling_timestamps.data(),
                         viz_data->rolling_torque.data(), count, offset,
                         sizeof(Scalar));
        ImPlot::EndPlot();
    }
}

void draw_rotor_angular_vel_plot(const int count, const int offset,
                                 SimParams* sim_params, SimState* sim_state,
                                 VizData* viz_data) {
    ImPlot::SetNextPlotLimitsX(sim_state->time - viz_data->rolling_history,
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
        viz_data
            ->rolling_phase_currents[i][viz_data->rolling_buffers_next_idx] =
            sim_state->coil_currents(i);
        viz_data->rolling_bEmfs[i][viz_data->rolling_buffers_next_idx] =
            sim_state->bEmfs(i);
        viz_data
            ->rolling_normalized_bEmfs[i][viz_data->rolling_buffers_next_idx] =
            sim_state->normalized_bEmfs(i);
    }

    viz_data->rolling_rotor_angular_vel[viz_data->rolling_buffers_next_idx] =
        sim_state->rotor_angular_vel;

    viz_data->rolling_torque[viz_data->rolling_buffers_next_idx] =
        sim_state->torque;

    rolling_buffer_advance_idx(viz_data->rolling_timestamps.size(),
                               &viz_data->rolling_buffers_next_idx,
                               &viz_data->rolling_buffers_wrap_around);
}

void implot_radial_line(const char* name, float inner_radius,
                        float outer_radius, float angle) {
    const float cos_angle = (float)std::cos(angle);
    const float sin_angle = (float)std::sin(angle);
    std::array<float, 2> xs = {inner_radius * cos_angle,
                               outer_radius * cos_angle};
    std::array<float, 2> ys = {inner_radius * sin_angle,
                               outer_radius * sin_angle};
    ImPlot::PlotLine(name, xs.data(), ys.data(), 2);
}

void implot_central_line(const char* name, float x, float y) {
    std::array<float, 2> xs = {0, x};
    std::array<float, 2> ys = {0, y};
    ImPlot::PlotLine(name, xs.data(), ys.data(), 2);
}

void draw_rotor_plot(SimState* sim_state, VizData* viz_data) {
    ImPlot::SetNextPlotLimits(/*x_min=*/-1.5,
                              /*x_max=*/1.5,
                              /*y_min=*/-1.5,
                              /*y_max=*/1.5);
    if (ImPlot::BeginPlot("##Rotor Angle", nullptr, nullptr, ImVec2{150, 150},
                          ImPlotFlags_Default & !ImPlotFlags_Legend,
                          ImPlotAxisFlags_Default & ~ImPlotAxisFlags_TickLabels,
                          ImPlotAxisFlags_Default &
                              ~ImPlotAxisFlags_TickLabels)) {

        ImPlot::PushStyleVar(ImPlotStyleVar_LineWeight, /*weight=*/1.0f);
        ImPlot::PlotLine("Rotor Circle", viz_data->circle_xs.data(),
                         viz_data->circle_ys.data(),
                         viz_data->circle_xs.size());
        implot_radial_line("Rotor Angle", 0.5f, 0.9f, sim_state->rotor_angle);
        ImPlot::PopStyleVar(1);
        ImPlot::EndPlot();
    }
}

void draw_space_vector_plot(SimState* sim_state, VizData* viz_data) {
    ImGui::Checkbox("Use Rotor Frame", &viz_data->use_rotor_frame);

    ImPlot::SetNextPlotLimits(/*x_min=*/-1.5,
                              /*x_max=*/1.5,
                              /*y_min=*/-1.5,
                              /*y_max=*/1.5);
    if (ImPlot::BeginPlot("##Space Vector Plot", nullptr, nullptr,
                          ImVec2{300, 300}, ImPlotFlags_Default,
                          ImPlotAxisFlags_Default & ~ImPlotAxisFlags_TickLabels,
                          ImPlotAxisFlags_Default &
                              ~ImPlotAxisFlags_TickLabels)) {

        ImPlot::PushStyleVar(ImPlotStyleVar_LineWeight, /*weight=*/1.0f);
        ImPlot::PlotLine("##Electrical Circle", viz_data->circle_xs.data(),
                         viz_data->circle_ys.data(),
                         viz_data->circle_xs.size());

        implot_radial_line(
            "Rotor Angle", 0.5f, 0.9f,
            viz_data->use_rotor_frame ? 0 : sim_state->electrical_angle);

        const std::complex<Scalar> park_transform{
            std::cos(-sim_state->electrical_angle),
            std::sin(-sim_state->electrical_angle)};

        std::complex<Scalar> phase_voltage_sv =
            to_complex<Scalar>(kClarkeTransform2x3 * sim_state->phase_voltages);
        if (viz_data->use_rotor_frame) {
            phase_voltage_sv *= park_transform;
        }

        implot_central_line("Phase Voltage Space Vector",
                            phase_voltage_sv.real(), phase_voltage_sv.imag());

        std::complex<Scalar> current_sv =
            to_complex<Scalar>(kClarkeTransform2x3 * sim_state->coil_currents);
        if (viz_data->use_rotor_frame) {
            current_sv *= park_transform;
        }

        implot_central_line("Current Space Vector", current_sv.real(),
                            current_sv.imag());

        std::complex<Scalar> normed_bEmf_sv = to_complex<Scalar>(
            kClarkeTransform2x3 * sim_state->normalized_bEmfs);
        if (viz_data->use_rotor_frame) {
            normed_bEmf_sv *= park_transform;
        }

        implot_central_line("Normed bEmf Space Vector", normed_bEmf_sv.real(),
                            normed_bEmf_sv.imag());

        ImPlot::PopStyleVar(1);
        ImPlot::EndPlot();
    }
}

bool order_of_magnitude_control(const char* label, Scalar* controllee,
                                const int exp_min = -4, const int exp_max = 4) {
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
    interacted = ImGui::SliderInt("exponent (base 10)", &integral_part, exp_min,
                                  exp_max) ||
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

void run_gui(SimParams* sim_params, SimState* sim_state, VizData* viz_data) {
    if (!sim_params->paused) {
        update_rolling_buffers(sim_state, viz_data);
    }

    ImGui::Begin("Simulation Control");

    ImGui::Text("Simulation Time: %f", sim_state->time);
    if (sim_params->paused) {
        sim_params->paused = !ImGui::Button("Resume");
    } else {
        sim_params->paused = ImGui::Button("Pause");
    }

    ImGui::SliderInt("Step Multiplier", &sim_params->step_multiplier, 1, 5000);

    ImGui::SliderFloat("Rolling History (sec)", &viz_data->rolling_history,
                       0.001f, 1.0f);

    ImGui::SliderInt("Num Pole Pairs", &sim_params->num_pole_pairs, 1, 8);

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

    ImGui::Begin("Space Vector Viz");
    draw_space_vector_plot(sim_state, viz_data);
    ImGui::End();

    ImGui::Begin("Electrical Plots");

    const int rolling_buffer_count = get_rolling_buffer_count(
        viz_data->rolling_timestamps.size(), viz_data->rolling_buffers_next_idx,
        viz_data->rolling_buffers_wrap_around);
    const int rolling_buffer_offset = get_rolling_buffer_offset(
        viz_data->rolling_timestamps.size(), viz_data->rolling_buffers_next_idx,
        viz_data->rolling_buffers_wrap_around);

    draw_electrical_plot(rolling_buffer_count, rolling_buffer_offset,
                         sim_params, sim_state, viz_data);

    ImGui::End();

    ImGui::Begin("Dynamics Plots");
    draw_rotor_angular_vel_plot(rolling_buffer_count, rolling_buffer_offset,
                                sim_params, sim_state, viz_data);

    draw_torque_plot(rolling_buffer_count, rolling_buffer_offset, sim_params,
                     sim_state, viz_data);

    ImGui::End();

    ImGui::Begin("Commutation");

    // Commutation state
    ImGui::Text("Commutation State");
    Slider("Phase Advance", &sim_state->six_step_phase_advance, -0.5, 0.5);
    for (int n = 0; n < 3; ++n) {
        ImGui::Text("Gate %d:", n);
        ImGui::SameLine();
        ImGui::PushID(n);
        ImGui::RadioButton("Low", &sim_state->gate_state.actual[n], 0);
        ImGui::SameLine();
        ImGui::RadioButton("High", &sim_state->gate_state.actual[n], 1);
        ImGui::SameLine();
        ImGui::RadioButton("Off", &sim_state->gate_state.actual[n], 2);
        ImGui::PopID();
    }

    ImGui::End();
}
