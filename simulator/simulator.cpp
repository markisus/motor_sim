#include "clarke_transform.h"
#include "conversions.h"
#include "gui.h"
#include "pi_control.h"
#include "scalar.h"
#include "sim_params.h"
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

void step(const SimParams& params, SimState* state) {
    update_gate_state(params.gate_dead_time, params.dt, &state->gate);

    // apply pole voltages
    for (int i = 0; i < 3; ++i) {
        Scalar v_pole = 0;
        switch (state->gate.actual[i]) {
        case OFF:
            // todo: derivation
            if (state->motor.phase_currents(i) > 0) {
                state->motor.pole_voltages(i) = 0;
            } else {
                state->motor.pole_voltages(i) = params.bus_voltage;
            }
            if (std::abs(state->motor.phase_currents(i)) >
                params.diode_active_current) {
                state->motor.pole_voltages(i) -= params.diode_active_voltage;
            }
            break;
        case HIGH:
            state->motor.pole_voltages(i) = params.bus_voltage;
            break;
        case LOW:
            state->motor.pole_voltages(i) = 0;
            break;
        default:
            printf("Unhandled switch case!");
            exit(-1);
        }
    }

    state->motor.normalized_bEmfs << // clang-format off
        get_back_emf(params.normalized_bEmf_coeffs, state->motor.electrical_angle),
        get_back_emf(params.normalized_bEmf_coeffs,
                     state->motor.electrical_angle - 2 * kPI / 3),
        get_back_emf(params.normalized_bEmf_coeffs,
                     state->motor.electrical_angle - 4 * kPI / 3); // clang-format on

    state->motor.bEmfs =
        state->motor.normalized_bEmfs * state->motor.rotor_angular_vel;

    // compute neutral point voltage
    // todo: derivation
    state->motor.neutral_voltage =
        (state->motor.pole_voltages.sum() - state->motor.bEmfs.sum()) / 3;

    for (int i = 0; i < 3; ++i) {
        state->motor.phase_voltages(i) =
            state->motor.pole_voltages(i) - state->motor.neutral_voltage;
    }

    Eigen::Matrix<Scalar, 3, 1> di_dt;
    for (int i = 0; i < 3; ++i) {
        di_dt(i) = (state->motor.phase_voltages(i) - state->motor.bEmfs(i) -
                    state->motor.phase_currents(i) * params.phase_resistance) /
                   params.phase_inductance;
    }

    state->motor.phase_currents += di_dt * params.dt;

    state->motor.torque =
        state->motor.phase_currents.dot(state->motor.normalized_bEmfs);

    state->motor.rotor_angular_accel =
        state->motor.torque / params.rotor_inertia;
    state->motor.rotor_angular_vel +=
        state->motor.rotor_angular_accel * params.dt;
    state->motor.rotor_angle += state->motor.rotor_angular_vel * params.dt;
    state->motor.rotor_angle = std::fmod(state->motor.rotor_angle, 2 * kPI);
    if (state->motor.rotor_angle < 0) {
        state->motor.rotor_angle += 2 * kPI;
    }

    state->motor.electrical_angle =
        state->motor.rotor_angle * params.num_pole_pairs;
    state->motor.electrical_angle =
        std::fmod(state->motor.electrical_angle, 2 * kPI);
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

    const Scalar voltage_q_coupled =
        pi_control(foc_state->i_controller_params, &foc_state->iq_controller,
                   foc_state->period, current_qd.real(), desired_current_q);
    const Scalar voltage_d_coupled =
        pi_control(foc_state->i_controller_params, &foc_state->id_controller,
                   foc_state->period, current_qd.imag(), 0);

    const std::complex<Scalar> voltage_qd_coupled = {voltage_q_coupled,
                                                     voltage_d_coupled};

    const std::complex<Scalar> decoupling_qd =
        park_transform *
        to_complex<Scalar>(
            (kClarkeTransform2x3 * motor.normalized_bEmfs).head<2>() *
            motor.rotor_angular_vel);

    const std::complex<Scalar> voltage_qd = voltage_qd_coupled - decoupling_qd;

    const std::complex<Scalar> voltage_ab =
        voltage_qd * std::conj(park_transform);

    // motor.pwm_state.duties = get_pwm_duties(bus_voltage, voltage_ab);

    // printf("Setting duties to %f %f %f\n", motor.pwm_state.duties[0],
    //        motor.pwm_state.duties[1], motor.pwm_state.duties[2]);
}

using namespace biro;

int main(int argc, char* argv[]) {
    SimParams params;
    init_sim_params(&params);

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

	
        run_gui(&viz_data, &params, &state, &viz_options);

        state.foc.i_controller_params =
            make_motor_pi_params(/*bandwidth=*/1,
                                 /*resistance=*/params.phase_resistance,
                                 /*inductance=*/params.phase_inductance);

        if (!params.paused) {
            for (int i = 0; i < params.step_multiplier; ++i) {
                if (state.commutation_mode == kCommutationModeSixStep) {
                    state.gate.commanded =
                        six_step_commutate(state.motor.electrical_angle,
                                           state.six_step_phase_advance);
                }

                if (state.commutation_mode == kCommutationModeFOC) {
                    if (periodic_timer(state.foc.period, params.dt,
                                       &state.foc.timer)) {
                        // step_foc
                    }

                    // current control loop (fast)
                }

                step(params, &state);
                state.time += params.dt;
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
