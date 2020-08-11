#include "clarke_transform.h"
#include "conversions.h"
#include "gui.h"
#include "pi_control.h"
#include "scalar.h"
#include "sine_series.h"
#include "space_vector_modulation.h"
#include "util/math_constants.h"
#include "util/time.h"
#include "wrappers/sdl_context.h"
#include "wrappers/sdl_imgui.h"
#include "wrappers/sdl_imgui_context.h"
#include <Eigen/Dense>
#include <absl/strings/str_format.h>
#include <array>
#include <glad/glad.h>
#include <implot.h>
#include <iostream>

PiParams make_motor_pi_params(Scalar bandwidth, Scalar resistance,
                              Scalar inductance) {
    PiParams params;
    params.p_gain = inductance * bandwidth;
    params.i_gain = resistance * bandwidth;
    params.max_integral = std::numeric_limits<Scalar>::infinity();
    return params;
}

int to_gate_enum(bool command) { return command ? HIGH : LOW; }

Scalar get_back_emf(const Eigen::Matrix<Scalar, 5, 1>& normalized_bEmf_coeffs,
                    const Scalar electrical_angle) {
    Eigen::Matrix<Scalar, 5, 1> sines;
    generate_odd_sine_series(/*num_terms=*/sines.rows(), electrical_angle,
                             sines.data());
    return sines.dot(normalized_bEmf_coeffs);
}

bool get_commutation_state(Scalar progress) {
    while (progress <= 0) {
        progress += 1;
    }
    while (progress > 1) {
        progress -= 1;
    }
    return (progress >= 0.5);
}

std::array<bool, 3> six_step_commutate(const Scalar electrical_angle,
                                       const Scalar phase_advance) {
    const Scalar progress = std::fmod(electrical_angle, 2 * kPI) / (2 * kPI);

    return {get_commutation_state(progress + phase_advance),
            get_commutation_state(progress + phase_advance - 1.0 / 3),
            get_commutation_state(progress + phase_advance - 2.0 / 3)};
}

void apply_pole_voltages(const Scalar dt,
                         const Eigen::Matrix<Scalar, 3, 1>& pole_voltages,
                         MotorState* motor) {

    motor->pole_voltages = pole_voltages;
    motor->normalized_bEmfs << // clang-format off
        get_back_emf(motor->normalized_bEmf_coeffs, motor->electrical_angle),
        get_back_emf(motor->normalized_bEmf_coeffs,
                     motor->electrical_angle - 2 * kPI / 3),
        get_back_emf(motor->normalized_bEmf_coeffs,
                     motor->electrical_angle - 4 * kPI / 3); // clang-format on

    motor->bEmfs = motor->normalized_bEmfs * motor->rotor_angular_vel;

    // compute neutral point voltage
    // todo: derivation
    motor->neutral_voltage =
        (motor->pole_voltages.sum() - motor->bEmfs.sum()) / 3;

    for (int i = 0; i < 3; ++i) {
        motor->phase_voltages(i) =
            motor->pole_voltages(i) - motor->neutral_voltage;
    }

    Eigen::Matrix<Scalar, 3, 1> di_dt;
    for (int i = 0; i < 3; ++i) {
        di_dt(i) = (motor->phase_voltages(i) - motor->bEmfs(i) -
                    motor->phase_currents(i) * motor->phase_resistance) /
                   motor->phase_inductance;
    }

    motor->phase_currents += di_dt * dt;

    motor->torque = motor->phase_currents.dot(motor->normalized_bEmfs);

    motor->rotor_angular_accel = motor->torque / motor->rotor_inertia;
    motor->rotor_angular_vel += motor->rotor_angular_accel * dt;
    motor->rotor_angle += motor->rotor_angular_vel * dt;
    motor->rotor_angle = std::fmod(motor->rotor_angle, 2 * kPI);
    if (motor->rotor_angle < 0) {
        motor->rotor_angle += 2 * kPI;
    }

    motor->electrical_angle = motor->rotor_angle * motor->num_pole_pairs;
    motor->electrical_angle = std::fmod(motor->electrical_angle, 2 * kPI);
}

Eigen::Matrix<Scalar, 3, 1>
get_pole_voltages(const Scalar bus_voltage, const Scalar diode_active_voltage,
                  const Scalar diode_active_current,
                  const std::array<int, 3>& gates,
                  const Eigen::Matrix<Scalar, 3, 1>& phase_currents) {
    Eigen::Matrix<Scalar, 3, 1> pole_voltages;
    for (int i = 0; i < 3; ++i) {
        Scalar v_pole = 0;
        switch (gates[i]) {
        case OFF:
            // todo: derivation
            if (phase_currents(i) > 0) {
                pole_voltages(i) = 0;
            } else {
                pole_voltages(i) = bus_voltage;
            }
            if (std::abs(phase_currents(i)) > diode_active_current) {
                pole_voltages(i) -= diode_active_voltage;
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

void step_foc(const MotorState& motor, FocState* foc_state) {
    const Scalar desired_current_q = 1.0;

    const Scalar q_axis_angle = motor.electrical_angle - kPI / 2;
    const Scalar cs = std::cos(-q_axis_angle);
    const Scalar sn = std::sin(-q_axis_angle);
    const std::complex<Scalar> park_transform{cs, sn};

    const std::complex<Scalar> current_qd =
        park_transform *
        to_complex<Scalar>(
            (kClarkeTransform2x3 * motor.phase_currents).head<2>());

    const Scalar voltage_q =
        pi_control(foc_state->i_controller_params, &foc_state->iq_controller,
                   foc_state->period, current_qd.real(), desired_current_q);
    const Scalar voltage_d =
        pi_control(foc_state->i_controller_params, &foc_state->id_controller,
                   foc_state->period, current_qd.imag(), 0);

    foc_state->voltage_qd = {voltage_q, voltage_d};

    // const std::complex<Scalar> decoupling_qd =
    //     park_transform *
    //     to_complex<Scalar>(
    //         (kClarkeTransform2x3 * motor.normalized_bEmfs).head<2>() *
    //         motor.rotor_angular_vel);

    // return voltage_qd * std::conj(park_transform);

    // motor.pwm_state.duties = get_pwm_duties(bus_voltage, voltage_ab);

    // printf("Setting duties to %f %f %f\n", motor.pwm_state.duties[0],
    //        motor.pwm_state.duties[1], motor.pwm_state.duties[2]);
}

using namespace biro;

int main(int argc, char* argv[]) {
    SimState state;
    init_sim_state(&state);

    VizData viz_data;
    init_viz_data(&viz_data);

    VizOptions viz_options;

    // FOC stuff
    const Scalar desired_torque = 1.0; // N.m

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
    while (!quit_flag) {
        quit_flag = wrappers::process_sdl_imgui_events(sdl_context.window_);
        if (quit_flag) {
            break;
        }

        wrappers::sdl_imgui_newframe(sdl_context.window_);

        if (!state.paused) {
            update_rolling_buffers(state.time, state.motor, state.pwm,
                                   state.foc, &viz_data.rolling_buffers);
        }
        run_gui(viz_data, &viz_options, &state);

        state.foc.i_controller_params =
            make_motor_pi_params(/*bandwidth=*/10,
                                 /*resistance=*/state.motor.phase_resistance,
                                 /*inductance=*/state.motor.phase_inductance);

        if (!state.paused) {
            for (int i = 0; i < state.step_multiplier; ++i) {
                // update relevant commutation modes
                if (state.commutation_mode == kCommutationModeSixStep) {
                    state.gate.commanded =
                        six_step_commutate(state.motor.electrical_angle,
                                           state.six_step_phase_advance);
                }

                if (state.commutation_mode == kCommutationModeFOC) {
                    if (periodic_timer(state.foc.period, state.dt,
                                       &state.foc.timer)) {
                        step_foc(state.motor, &state.foc);
                    }

                    if (step_pwm_state(state.dt, &state.pwm)) {
                        // establish the requested qd voltage (fast loop)
                        const Scalar q_axis_angle =
                            state.motor.electrical_angle - kPI / 2;
                        const Scalar cs = std::cos(q_axis_angle);
                        const Scalar sn = std::sin(q_axis_angle);
                        const std::complex<Scalar> inv_park_transform{cs, sn};
                        std::complex<Scalar> voltage_ab =
                            inv_park_transform * state.foc.voltage_qd;
                        // need to decouple (todo: investigate sign)
                        // voltage_ab += to_complex<Scalar>(
                        //     (kClarkeTransform2x3 * state.motor.normalized_bEmfs)
                        //         .head<2>() *
                        //     state.motor.rotor_angular_vel);
                        state.pwm.duties =
                            get_pwm_duties(state.bus_voltage, voltage_ab);
                    }

                    state.gate.commanded = get_pwm_gate_command(state.pwm);
                }

                // step motor
                update_gate_state(state.gate_dead_time, state.dt, &state.gate);

                const auto pole_voltages = get_pole_voltages(
                    state.bus_voltage, state.diode_active_voltage,
                    state.diode_active_current, state.gate.actual,
                    state.motor.phase_currents);

                apply_pole_voltages(state.dt, pole_voltages, &state.motor);

                state.time += state.dt;
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

    return 0;
}
