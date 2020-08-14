#include "config/scalar.h"
#include "controls/foc.h"
#include "controls/pi_control.h"
#include "controls/six_step.h"
#include "controls/space_vector_modulation.h"
#include "gui.h"
#include "motor.h"
#include "util/clarke_transform.h"
#include "util/conversions.h"
#include "util/math_constants.h"
#include "util/rotation.h"
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

using namespace biro;

int main(int argc, char* argv[]) {
    SimState state;
    init_sim_state(&state);

    VizData viz_data;
    init_viz_data(&viz_data);

    VizOptions viz_options;

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
            update_rolling_buffers(state.time, state.motor, state.board.pwm,
                                   state.board.gate, state.foc,
                                   &viz_data.rolling_buffers);
        }
        run_gui(viz_data, &viz_options, &state);

        state.foc.i_controller_params =
            make_motor_pi_params(/*bandwidth=*/10000,
                                 /*resistance=*/state.motor.phase_resistance,
                                 /*inductance=*/state.motor.phase_inductance);

        if (!state.paused) {
            for (int i = 0; i < state.step_multiplier; ++i) {
                const bool new_pwm_cycle =
                    step_pwm_state(state.dt, &state.board.pwm);

                std::array<bool, 3> gate_command = {};

                // update relevant commutation modes
                if (state.commutation_mode == kCommutationModeManual) {
                    // maintain the existing the gate command
                    // which has probably been set from the GUI
                    gate_command = state.board.gate.commanded;
                }

                if (state.commutation_mode == kCommutationModeSixStep) {
                    gate_command =
                        six_step_commutate(state.motor.electrical_angle,
                                           state.six_step_phase_advance);
                }

                if (state.commutation_mode == kCommutationModeFOC) {
                    if (periodic_timer(state.foc.period, state.dt,
                                       &state.foc.timer)) {
                        Scalar desired_torque = state.foc_desired_torque;
                        if (state.foc_use_cogging_compensation) {
                            desired_torque -= interp_cogging_torque(
                                state.motor.encoder_position,
                                state.motor.cogging_torque_map);
                        }

                        std::complex<Scalar> desired_current_qd;
                        if (state.foc_non_sinusoidal_drive_mode) {
                            desired_current_qd =
                                get_desired_current_qd_non_sinusoidal(
                                    desired_torque, state.motor);
                        } else {
                            desired_current_qd = get_desired_current_qd(
                                desired_torque,
                                state.motor.normed_bEmf_coeffs(0));
                        }

                        step_foc_current_controller(desired_current_qd,
                                                    state.motor, &state.foc);
                    }

                    // assert the requested qd voltage with PWM
                    if (new_pwm_cycle) {
                        const std::complex<Scalar> inv_park_transform =
                            get_rotation(state.motor.q_axis_electrical_angle);

                        std::complex<Scalar> voltage_ab =
                            inv_park_transform * state.foc.voltage_qd;

                        if (state.foc_use_qd_decoupling) {
                            const std::complex<Scalar> existing_back_emf_ab =
                                clarke_transform(state.motor.normed_bEmfs) *
                                state.motor.rotor_angular_vel;
                            voltage_ab += existing_back_emf_ab;
                        }

                        state.board.pwm.duties =
                            get_pwm_duties(state.board.bus_voltage, voltage_ab);
                    }

                    gate_command = get_pwm_gate_command(state.board.pwm);
                }

                state.board.gate.commanded = gate_command;
                update_gate_state(state.dt, &state.board.gate);

                const auto pole_voltages = get_pole_voltages(
                    state.board.bus_voltage, state.motor.phase_currents,
                    state.board.gate);

                step_motor(state.dt, state.load_torque, pole_voltages,
                           &state.motor);

                state.time += state.dt;
            }
        }

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
