#include "gui.h"
#include "clarke_transform.h"
#include "conversions.h"
#include "scalar.h"
#include "util/math_constants.h"
#include "util/rotation.h"
#include <absl/strings/str_format.h>
#include <imgui.h>
#include <implot.h>

constexpr int kPlotHeight = 250; // sec
constexpr int kPlotWidth = -1;   // sec

struct RollingPlotParams {
    int count;
    int begin;
    Scalar begin_time;
    Scalar end_time;
};

RollingPlotParams get_rolling_plot_params(const RollingBuffers& buffers,
                                          const Scalar rolling_history) {

    RollingPlotParams params;

    params.count = get_rolling_buffer_count(buffers.ctx);
    params.begin = get_rolling_buffer_begin(buffers.ctx);
    if (params.count != 0) {
        params.begin_time = buffers.timestamps[params.begin];
        params.end_time =
            buffers.timestamps[get_rolling_buffer_back(buffers.ctx)];
    } else {
        // no data to display
        params.begin_time = 0;
        params.end_time = 0;
    }

    params.begin_time =
        std::max(params.begin_time, params.end_time - rolling_history);

    return params;
}

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

void draw_electrical_plot(const RollingPlotParams& params,
                          const RollingBuffers& buffers, VizOptions* options) {

    ImGui::Text("Left Axis (Volts):");
    ImGui::Checkbox("Phase Voltages", &options->show_phase_voltages);
    ImGui::SameLine();
    ImGui::Checkbox("Normed bEmfs (Volt . seconds)",
                    &options->show_normalized_bEmfs);
    ImGui::SameLine();
    ImGui::Checkbox("bEmfs", &options->show_bEmfs);
    ImGui::Text("Right Axis (Amperes):");
    ImGui::SameLine();
    ImGui::Checkbox("Phase Currents", &options->show_phase_currents);

    ImGui::Text("Coil Visibility:");
    ImGui::SameLine();
    if (options->show_bEmfs || options->show_phase_currents ||
        options->show_phase_voltages) {
        ImGui::Checkbox("Coil 0", &options->coil_visible[0]);
        ImGui::SameLine();
        ImGui::Checkbox("Coil 1", &options->coil_visible[1]);
        ImGui::SameLine();
        ImGui::Checkbox("Coil 2", &options->coil_visible[2]);
    }

    ImPlot::SetNextPlotLimitsX(params.begin_time, params.end_time,
                               ImGuiCond_Always);

    const bool have_voltage_axis =
        options->show_bEmfs || options->show_phase_voltages;
    const bool have_current_axis = options->show_phase_currents;

    ImPlot::SetNextPlotLimitsY(-10, 10, ImGuiCond_Once, 0);

    ImPlot::SetNextPlotLimitsY(-10, 10, ImGuiCond_Once, 1);

    if (ImPlot::BeginPlot("##Electrical Plots", "Seconds", nullptr,
                          ImVec2(kPlotWidth, kPlotHeight),
                          ImPlotFlags_Default | ImPlotFlags_YAxis2)) {
        for (int i = 0; i < 3; ++i) {
            if (!options->coil_visible[i]) {
                continue;
            }

            if (options->show_phase_voltages) {
                ImPlot::SetPlotYAxis(0);
                ImPlot::PushStyleColor(ImPlotCol_Line, get_coil_color(i, 1.0f));
                ImPlot::PushStyleVar(ImPlotStyleVar_LineWeight, 1.0f);
                ImPlot::PlotLine(absl::StrFormat("Coil %d Voltage", i).c_str(),
                                 buffers.timestamps.data(),
                                 buffers.phase_vs[i].data(), params.count,
                                 params.begin, sizeof(Scalar));
                ImPlot::PopStyleVar();
                ImPlot::PopStyleColor();
            }

            if (options->show_normalized_bEmfs) {
                ImPlot::SetPlotYAxis(0);
                ImPlot::PushStyleColor(ImPlotCol_Line, get_coil_color(i, 0.8f));
                ImPlot::PlotLine(
                    absl::StrFormat("Coil %d Normed bEmf", i).c_str(),
                    buffers.timestamps.data(),
                    buffers.normalized_bEmfs[i].data(), params.count,
                    params.begin, sizeof(Scalar));
                ImPlot::PopStyleColor();
            }

            if (options->show_bEmfs) {
                ImPlot::SetPlotYAxis(0);
                ImPlot::PushStyleColor(ImPlotCol_Line, get_coil_color(i, 0.5f));
                ImPlot::PlotLine(absl::StrFormat("Coil %d bEmf", i).c_str(),
                                 buffers.timestamps.data(),
                                 buffers.bEmfs[i].data(), params.count,
                                 params.begin, sizeof(Scalar));
                ImPlot::PopStyleColor();
            }

            if (options->show_phase_currents) {
                ImPlot::SetPlotYAxis(1);
                ImPlot::PushStyleColor(ImPlotCol_Line, get_coil_color(i, 0.3f));
                ImPlot::PushStyleVar(ImPlotStyleVar_LineWeight, 4.0f);
                ImPlot::PlotLine(absl::StrFormat("Coil %d Current", i).c_str(),
                                 buffers.timestamps.data(),
                                 buffers.phase_currents[i].data(), params.count,
                                 params.begin, sizeof(Scalar));
                ImPlot::PopStyleVar();
                ImPlot::PopStyleColor();
            }
        }
        ImPlot::EndPlot();
    }
}

void draw_torque_plot(const RollingPlotParams& params,
                      const RollingBuffers& buffers) {
    ImPlot::SetNextPlotLimitsX(params.begin_time, params.end_time,
                               ImGuiCond_Always);
    ImPlot::SetNextPlotLimitsY(-10, 10, ImGuiCond_Once);
    if (ImPlot::BeginPlot("Torque", "Seconds", "N . m",
                          ImVec2(kPlotWidth, kPlotHeight))) {
        ImPlot::PlotLine("", buffers.timestamps.data(), buffers.torque.data(),
                         params.count, params.begin, sizeof(Scalar));
        ImPlot::EndPlot();
    }
}

void draw_rotor_angular_vel_plot(const RollingPlotParams& params,
                                 const RollingBuffers& buffers) {
    ImPlot::SetNextPlotLimitsX(params.begin_time, params.end_time,
                               ImGuiCond_Always);

    ImPlot::SetNextPlotLimitsY(-10, 10, ImGuiCond_Once);

    // This auto scroll implementation is janky
    // it breaks the zooming functionality so it only activates half the time
    // (see comment on can_trigger_auto_scroll)

    // state variables for auto scroll
    static float last_y_min = -10;
    static float last_y_max = 10;
    static float last_y_range = last_y_max - last_y_min;
    static bool can_trigger_auto_scroll =
        false; // hack: flips true and false to allow ImPlot library to win half
               // the time when it's trying to update the zoom

    // auto scroll
    if (get_rolling_buffer_count(buffers.ctx) > 0 && can_trigger_auto_scroll) {
        const int last_idx = get_rolling_buffer_back(buffers.ctx);
        const Scalar last_angular_vel = buffers.rotor_angular_vel[last_idx];
        if (last_angular_vel < last_y_min) {
            ImPlot::SetNextPlotLimitsY(last_angular_vel,
                                       last_angular_vel + last_y_range,
                                       ImGuiCond_Always);
        } else if (last_angular_vel > last_y_max) {
            ImPlot::SetNextPlotLimitsY(last_angular_vel - last_y_range,
                                       last_angular_vel, ImGuiCond_Always);
        }
    }
    can_trigger_auto_scroll = !can_trigger_auto_scroll;

    if (ImPlot::BeginPlot("Rotor Angular Vel", "Seconds", "Radians / Sec",
                          ImVec2(kPlotWidth, kPlotHeight))) {

        ImPlot::PlotLine("", buffers.timestamps.data(),
                         buffers.rotor_angular_vel.data(), params.count,
                         params.begin, sizeof(Scalar));

        const auto plot_limits = ImPlot::GetPlotLimits(0);
        last_y_min = plot_limits.Y.Min;
        last_y_max = plot_limits.Y.Max;
        last_y_range = last_y_max - last_y_min;
        ImPlot::EndPlot();
    }
}

void update_rolling_buffers(const Scalar time, const MotorState& motor,
                            const PwmState& pwm, const FocState& foc,
                            RollingBuffers* buffers) {
    const int next_idx = buffers->ctx.next_idx;

    buffers->timestamps[next_idx] = time;

    for (int i = 0; i < 3; ++i) {
        buffers->phase_vs[i][next_idx] = motor.phase_voltages(i);

        buffers->phase_currents[i][next_idx] = motor.phase_currents(i);

        buffers->bEmfs[i][next_idx] = motor.bEmfs(i);

        buffers->normalized_bEmfs[i][next_idx] = motor.normalized_bEmfs(i);

        buffers->pwm_duties[i][next_idx] = pwm.duties[i];

        buffers->pwm_level[next_idx] = pwm.level;

        // Project current onto qd axes
        {
            const std::complex<Scalar> park_transform =
                get_rotation(-motor.q_axis_electrical_angle);
            const std::complex<Scalar> current_qd =
                park_transform *
                to_complex<Scalar>(
                    (kClarkeTransform2x3 * motor.phase_currents).head<2>());
            buffers->current_q[next_idx] = current_qd.real();
            buffers->current_d[next_idx] = current_qd.imag();
        }

        buffers->current_q_err[next_idx] = foc.iq_controller.err;

        buffers->current_q_integral[next_idx] = foc.iq_controller.integral;

        buffers->current_d_err[next_idx] = foc.id_controller.err;

        buffers->current_d_integral[next_idx] = foc.id_controller.integral;
    }

    buffers->rotor_angular_vel[next_idx] = motor.rotor_angular_vel;
    buffers->torque[next_idx] = motor.torque;

    rolling_buffer_advance_idx(&buffers->ctx);
}

void draw_control_plots(const RollingPlotParams& params,
                        const RollingBuffers& buffers) {

    ImPlot::SetNextPlotLimitsX(params.begin_time, params.end_time,
                               ImGuiCond_Always);
    ImPlot::SetNextPlotLimitsY(-0.1, 1.1, ImGuiCond_Once);
    if (ImPlot::BeginPlot("PWM", "Seconds", "",
                          ImVec2(kPlotWidth, kPlotHeight))) {
        for (int i = 0; i < 3; ++i) {
            ImPlot::PlotLine(absl::StrFormat("Gate %d", i).c_str(),
                             buffers.timestamps.data(),
                             buffers.pwm_duties[i].data(), params.count,
                             params.begin, sizeof(Scalar));
        }
        ImPlot::PushStyleColor(ImPlotCol_Line,
                               (uint32_t)ImColor(1.0f, 1.0f, 1.0f, 0.2));
        ImPlot::PlotLine("Level", buffers.timestamps.data(),
                         buffers.pwm_level.data(), params.count, params.begin,
                         sizeof(Scalar));
        ImPlot::PopStyleColor();

        ImPlot::EndPlot();
    }

    ImPlot::SetNextPlotLimitsX(params.begin_time, params.end_time,
                               ImGuiCond_Always);
    ImPlot::SetNextPlotLimitsY(-1, 1, ImGuiCond_Once);
    if (ImPlot::BeginPlot("Current Controller Errors", "Seconds", nullptr,
                          ImVec2(kPlotWidth, kPlotHeight))) {
        ImPlot::PlotLine("iq error", buffers.timestamps.data(),
                         buffers.current_q_err.data(), params.count,
                         params.begin, sizeof(Scalar));

        ImPlot::PlotLine("id error", buffers.timestamps.data(),
                         buffers.current_d_err.data(), params.count,
                         params.begin, sizeof(Scalar));
        ImPlot::EndPlot();
    }

    ImPlot::SetNextPlotLimitsX(params.begin_time, params.end_time,
                               ImGuiCond_Always);

    if (ImPlot::BeginPlot("Current qd", "Seconds", nullptr,
                          ImVec2(kPlotWidth, kPlotHeight))) {
        ImPlot::PlotLine("iq", buffers.timestamps.data(),
                         buffers.current_q.data(), params.count, params.begin,
                         sizeof(Scalar));

        ImPlot::PlotLine("id", buffers.timestamps.data(),
                         buffers.current_d.data(), params.count, params.begin,
                         sizeof(Scalar));
        ImPlot::EndPlot();
    }

    ImPlot::SetNextPlotLimitsX(params.begin_time, params.end_time,
                               ImGuiCond_Always);

    if (ImPlot::BeginPlot("Current Controller Integrals", "Seconds", nullptr,
                          ImVec2(kPlotWidth, kPlotHeight))) {
        ImPlot::PlotLine("iq int", buffers.timestamps.data(),
                         buffers.current_q_integral.data(), params.count,
                         params.begin, sizeof(Scalar));

        ImPlot::PlotLine("id int", buffers.timestamps.data(),
                         buffers.current_d_integral.data(), params.count,
                         params.begin, sizeof(Scalar));
        ImPlot::EndPlot();
    }
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

void draw_rotor_plot(const VizData& viz_data, const Scalar rotor_angle) {
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
        ImPlot::PlotLine("Rotor Circle", viz_data.circle_xs.data(),
                         viz_data.circle_ys.data(), viz_data.circle_xs.size());
        implot_radial_line("Rotor Angle", 0.5f, 0.9f, rotor_angle);
        ImPlot::PopStyleVar(1);
        ImPlot::EndPlot();
    }
}

void draw_space_vector_plot(const VizData& viz_data, const SimState& state,
                            VizOptions* options) {
    ImGui::Checkbox("Use Rotor Frame", &options->use_rotor_frame);

    ImPlot::SetNextPlotLimits(/*x_min=*/-10.5,
                              /*x_max=*/10.5,
                              /*y_min=*/-10.5,
                              /*y_max=*/10.5);
    if (ImPlot::BeginPlot("##Space Vector Plot", nullptr, nullptr,
                          ImVec2{300, 300}, ImPlotFlags_Default,
                          ImPlotAxisFlags_Default & ~ImPlotAxisFlags_TickLabels,
                          ImPlotAxisFlags_Default &
                              ~ImPlotAxisFlags_TickLabels)) {

        ImPlot::PushStyleVar(ImPlotStyleVar_LineWeight, /*weight=*/1.0f);
        ImPlot::PlotLine("##Electrical Circle", viz_data.circle_xs.data(),
                         viz_data.circle_ys.data(), viz_data.circle_xs.size());

        implot_radial_line(
            "Rotor Angle", 0.0f, 1.0f,
            options->use_rotor_frame ? 0 : state.motor.electrical_angle);

        const std::complex<Scalar> park_transform{
            std::cos(-state.motor.electrical_angle),
            std::sin(-state.motor.electrical_angle)};

        std::complex<Scalar> phase_voltage_sv = to_complex<Scalar>(
            kClarkeTransform2x3 * state.motor.phase_voltages);

        if (options->use_rotor_frame) {
            phase_voltage_sv *= park_transform;
        }

        implot_central_line("Phase Voltage Space Vector",
                            phase_voltage_sv.real(), phase_voltage_sv.imag());

        std::complex<Scalar> current_sv = to_complex<Scalar>(
            kClarkeTransform2x3 * state.motor.phase_currents);
        if (options->use_rotor_frame) {
            current_sv *= park_transform;
        }

        implot_central_line("Current Space Vector", current_sv.real(),
                            current_sv.imag());

        std::complex<Scalar> normed_bEmf_sv = to_complex<Scalar>(
            kClarkeTransform2x3 * state.motor.normalized_bEmfs);
        if (options->use_rotor_frame) {
            normed_bEmf_sv *= park_transform;
        }

        implot_central_line("Normed bEmf Space Vector", normed_bEmf_sv.real(),
                            normed_bEmf_sv.imag());

        if (state.commutation_mode == kCommutationModeFOC) {
            auto voltage_sv = state.foc.voltage_qd;
            const std::complex<Scalar> rot90{0, -1};
            voltage_sv *= rot90; // put into rotor frame

            if (!options->use_rotor_frame) {
                voltage_sv *= std::conj(park_transform);
            }

            implot_central_line("FOC Voltage Command", voltage_sv.real(),
                                voltage_sv.imag());
        }

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

void run_gui(const VizData& viz_data, VizOptions* options,
             SimState* sim_state) {

    ImGui::Begin("Simulation Control");

    ImGui::Text("Simulation Time: %f", sim_state->time);
    if (sim_state->paused) {
        sim_state->paused = !ImGui::Button("Resume");
    } else {
        sim_state->paused = ImGui::Button("Pause");
    }

    ImGui::SliderInt("Step Multiplier", &sim_state->step_multiplier, 1, 5000);

    ImGui::SliderFloat("Rolling History (sec)", &options->rolling_history,
                       sim_state->dt * 10, 1.0f);

    ImGui::SliderInt("Num Pole Pairs", &sim_state->motor.num_pole_pairs, 1, 8);

    Slider("Rotor Moment of Inertia (kg m^2)", &sim_state->motor.rotor_inertia,
           0.1, 10);

    Slider("Bus Voltage", &sim_state->bus_voltage, 1.0, 120);
    Slider("Diode Active Voltage", &sim_state->diode_active_voltage, 0.0, 5);

    order_of_magnitude_control("Phase Inductance",
                               &sim_state->motor.phase_inductance);
    order_of_magnitude_control("Phase Resistance",
                               &sim_state->motor.phase_resistance);

    ImGui::End();

    ImGui::Begin("Rotor Viz");
    draw_rotor_plot(viz_data, sim_state->motor.rotor_angle);
    ImGui::End();

    ImGui::Begin("Space Vector Viz");
    draw_space_vector_plot(viz_data, *sim_state, options);
    ImGui::End();

    ImGui::Begin("Electrical Plots");

    const RollingPlotParams rolling_plot_params = get_rolling_plot_params(
        viz_data.rolling_buffers, options->rolling_history);

    draw_electrical_plot(rolling_plot_params, viz_data.rolling_buffers,
                         options);

    ImGui::End();

    ImGui::Begin("Dynamics Plots");
    draw_rotor_angular_vel_plot(rolling_plot_params, viz_data.rolling_buffers);

    draw_torque_plot(rolling_plot_params, viz_data.rolling_buffers);

    ImGui::End();

    ImGui::Begin("Controls");
    draw_control_plots(rolling_plot_params, viz_data.rolling_buffers);

    ImGui::End();

    ImGui::Begin("Commutation");

    // Commutation state
    ImGui::Text("Commutation State");

    ImGui::Text("Gate States");
    for (int i = 0; i < 3; ++i) {
        std::array<const char*, 3> gate_state_texts{"LOW", "HIGH", "OFF"};
        ImGui::Text("Gate %d %s", i,
                    gate_state_texts[sim_state->gate.actual[i]]);
    }

    ImGui::NewLine();

    ImGui::RadioButton("Manual", &sim_state->commutation_mode,
                       kCommutationModeNone);
    ImGui::SameLine();
    ImGui::RadioButton("Six Step", &sim_state->commutation_mode,
                       kCommutationModeSixStep);
    ImGui::SameLine();
    ImGui::RadioButton("FOC", &sim_state->commutation_mode,
                       kCommutationModeFOC);

    if (sim_state->commutation_mode == kCommutationModeSixStep) {
        Slider("Phase Advance", &sim_state->six_step_phase_advance, -0.5, 0.5);
    }

    if (sim_state->commutation_mode == kCommutationModeNone) {
        // manual controls
        ImGui::Text("Manual Command");
        for (int i = 0; i < 3; ++i) {
            ImGui::Text("Gate %d", i);
            ImGui::SameLine();
            ImGui::PushID(i);

            int current_command = (int)sim_state->gate.commanded[i];
            ImGui::RadioButton(absl::StrFormat("HIGH", i).c_str(),
                               &current_command, 1);
            ImGui::SameLine();
            ImGui::RadioButton(absl::StrFormat("LOW", i).c_str(),
                               &current_command, 0);

            sim_state->gate.commanded[i] = (bool)current_command;
            ImGui::PopID();
        }
    }

    ImGui::End();
}
