#include "controls/pi_control.h"
#include "util/rolling_buffer.h"
#include "util/time.h"
#include "wrappers/sdl_context.h"
#include "wrappers/sdl_imgui.h"
#include "wrappers/sdl_imgui_context.h"
#include <glad/glad.h>
#include <implot.h>
#include <iostream>

using Scalar = double;
using namespace biro;

/*

The circuit in question is the following

     --> i

  +  +--R-L-E-+
 Vin          |
  -  +--------+

Vin is the control voltage
R is the resistor
L is the inductor (motor coil)
E is the back EMF (assumed constant)
i is the phase current

The goal is set-point control of i via a PI controller.

 */

void step(const Scalar dt, const Scalar v_in, const Scalar R, const Scalar L,
          const Scalar E, Scalar* i) {
    const Scalar di_dt = (v_in - R * (*i) - E) / L;
    *i += di_dt * dt;
}

int main(int argc, char* argv[]) {
    std::cout << "This is the single phase model"
              << "\n";

    Scalar time = 0;
    Scalar v_in = 0;
    Scalar v_in_desired = 0;
    Scalar R = 1e-3;
    Scalar L = 1e-4;
    Scalar E = 0.0;
    Scalar i = 0;
    Scalar dt = 1e-6;

    // const Scalar pi_period = 1e-4;

    int pi_period_exp = -3;
    Scalar pi_period = 1e-3; // ms
    Scalar pi_timer = 0;
    float pi_bandwidth = 100;
    PiParams pi_params;
    // pi_params.p_gain = L * bandwidth;
    // pi_params.i_gain = R * bandwidth;
    PiContext pi_context;

    float target_current = 1.0;

    int step_multiplier = 1;

    constexpr int kNumRollingPts = 200;
    RollingBufferContext rolling_buffer_ctx(kNumRollingPts);
    std::array<Scalar, kNumRollingPts> roll_timestamp;
    std::array<Scalar, kNumRollingPts> roll_i;
    std::array<Scalar, kNumRollingPts> roll_v_in;
    std::array<Scalar, kNumRollingPts> roll_i_err;
    std::array<Scalar, kNumRollingPts> roll_i_int;

    wrappers::SdlContext sdl_context("Single Phase Sim",
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

        pi_params.p_gain = L * pi_bandwidth;
        pi_params.i_gain = R * pi_bandwidth;

        for (int j = 0; j < step_multiplier; ++j) {
            if (periodic_timer<Scalar>(pi_period, dt, &pi_timer)) {
                v_in_desired = pi_control(pi_params, &pi_context, pi_period, i,
                                  target_current);
            }
	    v_in = v_in_desired + E;
            step(dt, v_in, R, L, E, &i);
            time += dt;
        }

        // save rolling buffer
        const int roll_idx = rolling_buffer_ctx.next_idx;
        roll_timestamp[roll_idx] = time;
        roll_v_in[roll_idx] = v_in;
        roll_i[roll_idx] = i;
        roll_i_err[roll_idx] = pi_context.err;
        roll_i_int[roll_idx] = pi_context.integral;
        rolling_buffer_advance_idx(&rolling_buffer_ctx);

        wrappers::sdl_imgui_newframe(sdl_context.window_);

        ImGui::Begin("Sim");
        ImGui::SliderInt("Step Multiplier", &step_multiplier, 1, 1000);
        ImGui::SliderFloat("Target Current", &target_current, -10, 10);
        ImGui::SliderFloat("PI Bandwidth", &pi_bandwidth, 1.0, 1000);
        if (ImGui::SliderInt("PI Period Exp", &pi_period_exp, -6, -1)) {
            pi_period = std::pow(10, pi_period_exp);
        };

        float Ef = (float)E;
        ImGui::SliderFloat("E", &Ef, -5.0, 5.0);
        E = Ef;

        ImGui::Text("time %f", time);
        ImGui::Text("v_in %f", v_in);
        ImGui::Text("i %f", i);
        ImGui::Text("R %f", R);
        ImGui::Text("L %f", L);
        ImGui::Text("E %f", E);
        ImGui::Text("i err %f", pi_context.err);
        ImGui::Text("i int %f", pi_context.integral);
        ImGui::End();

        ImGui::Begin("Plots");
        // Begin real time plot
        const int roll_count = get_rolling_buffer_count(rolling_buffer_ctx);
        const int roll_first = get_rolling_buffer_begin(rolling_buffer_ctx);
        if (roll_count > 0) {
            const int roll_last = get_rolling_buffer_back(rolling_buffer_ctx);
            const Scalar begin_time = roll_timestamp[roll_first];
            const Scalar last_time = roll_timestamp[roll_last];
            ImPlot::SetNextPlotLimitsX(begin_time, last_time, ImGuiCond_Always);
        }
        ImPlot::SetNextPlotLimitsY(-1, 1, ImGuiCond_Once);
        if (ImPlot::BeginPlot("V_in", "Seconds", nullptr, ImVec2(-1, 350))) {
            ImPlot::PlotLine("", roll_timestamp.data(), roll_v_in.data(),
                             roll_count, roll_first, sizeof(Scalar));
            ImPlot::EndPlot();
        }

        if (roll_count > 0) {
            const int roll_last = get_rolling_buffer_back(rolling_buffer_ctx);
            const Scalar begin_time = roll_timestamp[roll_first];
            const Scalar last_time = roll_timestamp[roll_last];
            ImPlot::SetNextPlotLimitsX(begin_time, last_time, ImGuiCond_Always);
        }
        ImPlot::SetNextPlotLimitsY(-10, 10, ImGuiCond_Once);
        if (ImPlot::BeginPlot("Current", "Seconds", nullptr, ImVec2(-1, 350))) {
            ImPlot::PlotLine("i", roll_timestamp.data(), roll_i.data(),
                             roll_count, roll_first, sizeof(Scalar));
            ImPlot::PlotLine("i err", roll_timestamp.data(), roll_i_err.data(),
                             roll_count, roll_first, sizeof(Scalar));
            ImPlot::PlotLine("i int", roll_timestamp.data(), roll_i_int.data(),
                             roll_count, roll_first, sizeof(Scalar));
            ImPlot::EndPlot();
        }
        ImGui::End();

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
