#include "config/scalar.h"
#include "controls/foc.h"
#include "controls/pi_control.h"
#include "controls/six_step.h"
#include "controls/space_vector_modulation.h"
#include "motor.h"
#include "util/clarke_transform.h"
#include "util/conversions.h"
#include "util/math_constants.h"
#include "util/rolling_buffer.h"
#include "util/rotation.h"
#include "util/sine_series.h"
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

/*

Calibration setup

                     + V -
                  +         +
                  |         |
                  v         v
  +--E1---R---L---o---Rcal--+
  |                         |
  |                         |
  +--E2---R---L---o---Rcal--x---- GND
  |                         |
  |                         |
  +--E3---R---L---o---Rcal--+


E1, E2, E3: back EMF of coils
R: phase resistance
L: phase inductance
o: motor terminal
Rs: known resistance of calibration rig
V: measurement point

The goal of calibration is to deduce the motor paremeters E, R, and L
where E is a function of rotor angle and speed
 */

constexpr int kNumRollingPts = 800;

struct MotorModel {
    // fourier representation of back emf curve
    static constexpr int IDX_PHASE = 0;
    static constexpr int IDX_C1 = 1;
    static constexpr int IDX_C5 = 2;
    static constexpr int IDX_C7 = 3;
    static constexpr int IDX_C11 = 4;
    static constexpr int IDX_C13 = 5;

    // electrical characteristics
    static constexpr int IDX_RESISTANCE = 6;
    static constexpr int IDX_INDUCTANCE = 7;

    static constexpr int DIM = 8;

    Eigen::Matrix<Scalar, DIM, 1> mean;
    Eigen::Matrix<Scalar, DIM, DIM> covariance;
};

Scalar fourier_to_normed_bEmf(const Scalar angle, const Scalar phase,
                              const Scalar c1, const Scalar c5, const Scalar c7,
                              const Scalar c11, const Scalar c13) {

    const int max_fourier_coeff = 13;                  // c13
    const int num_terms = (max_fourier_coeff + 1) / 2; // c13

    Scalar sine_series_data[num_terms];
    generate_odd_sine_series(num_terms, phase + angle, sine_series_data);

    return c1 * sine_series_data[1 / 2] + c5 * sine_series_data[5 / 2] +
           c7 * sine_series_data[7 / 2] + c11 * sine_series_data[11 / 2] +
           c13 * sine_series_data[13 / 2];
}

struct DynamicalState {
    static constexpr int IDX_THETA = 0;
    static constexpr int IDX_THETA_DOT = 1;
    static constexpr int IDX_VOLTAGE = 2;
    static constexpr int IDX_VOLTAGE_DOT = 3;
    static constexpr int DIM = 4;

    Eigen::Matrix<Scalar, DIM, 1> mean;
    Eigen::Matrix<Scalar, DIM, DIM> covariance;
};

struct Measurement {
    static constexpr int IDX_THETA = 0;
    static constexpr int IDX_THETA_DOT = 1;
    static constexpr int IDX_VOLTAGE = 2;
    static constexpr int DIM = 3;

    Eigen::Matrix<Scalar, DIM, 1> mean;
    Eigen::Matrix<Scalar, DIM, DIM> covariance;
};

template <int InDim, int OutDim> struct Stats {
    Eigen::Matrix<Scalar, OutDim, 1>
        mean; // the ekf mean e.g propagation of the center point

    // estimated measurement covariance
    Eigen::Matrix<Scalar, OutDim, OutDim> covariance;

    // cross covariance btwn input and measurement
    Eigen::Matrix<Scalar, InDim, OutDim> cross_covariance;

    // approximation of the jacobian of input->output function
    Eigen::Matrix<Scalar, OutDim, InDim> statistical_jacobian;
};

constexpr int num_sigma_points(int input_dim) { return 2 * input_dim; }

constexpr double scale = 1.73205080757; // any postive number here
constexpr double scale_squared = scale * scale;

inline std::pair<double, double> get_w0_w1(int input_dim) {
    const double kappa = scale_squared - input_dim;
    const double w0 = kappa / (input_dim + kappa);
    const double w1 = 1.0f / (2 * (input_dim + kappa));
    return {w0, w1};
}

// note the number of sigma points for the output depends on the dimension of
// the input. the output points are obtained by running the input through the
// nonlinearity / measurement function
template <int InDim, int OutDim>
Stats<InDim, OutDim> get_unscented_transform_stats(
    const Eigen::Matrix<Scalar, InDim, 1>& input_mean,
    const Eigen::Matrix<Scalar, InDim, InDim>& input_covariance,
    const Eigen::Matrix<Scalar, InDim, num_sigma_points(InDim)>&
        input_sigma_points,
    const Eigen::Matrix<Scalar, OutDim, 1>& output_mean,
    const Eigen::Matrix<Scalar, OutDim, num_sigma_points(InDim)>&
        output_sigma_points) {

    Stats<InDim, OutDim> stats;
    stats.mean = output_mean;
    stats.cross_covariance.setZero();
    stats.covariance.setZero();

    for (int i = 0; i < num_sigma_points(InDim); ++i) {
        const Eigen::Matrix<double, OutDim, 1> delta_out =
            (output_sigma_points.col(i) - output_mean);
        stats.covariance += delta_out * delta_out.transpose();

        const Eigen::Matrix<Scalar, InDim, 1> delta_in =
            input_sigma_points.col(i) - input_mean;
        stats.cross_covariance += delta_in * delta_out.transpose();
    }

    const auto [w0, w1] = get_w0_w1(/*input_dim=*/InDim);
    stats.cross_covariance *= w1;
    stats.covariance *= w1;
    stats.statistical_jacobian =
        // statistical differential from input to output
        // Pyx * Pxx^-1 = X
        // Pyx = X * Pxx
        // Pxy = Pxx X^t
        // numerical issues:
        // ut_stats.cross_covariance.transpose() *
        // prior.covariance.inverse();
        input_covariance.llt().solve(stats.cross_covariance).transpose();

    return stats;
};

// information space correction
// can sum them all and apply at once
template <int InDim> struct InfoCorrection {
    Eigen::Matrix<Scalar, InDim, 1> info_vector_correction =
        Eigen::Matrix<Scalar, InDim, 1>::Zero();
    Eigen::Matrix<Scalar, InDim, InDim> info_matrix_correction =
        Eigen::Matrix<Scalar, InDim, InDim>::Zero();

    InfoCorrection operator+(const InfoCorrection& other) {
        InfoCorrection result;
        result.info_vector_correction =
            this->info_vector_correction + other.info_vector_correction;
        result.info_matrix_correction =
            this->info_matrix_correction + other.info_matrix_correction;
        return result;
    }

    InfoCorrection& operator+=(const InfoCorrection& other) {
        this->info_vector_correction += other.info_vector_correction;
        this->info_matrix_correction += other.info_matrix_correction;
        return *this;
    }

    void clear() {
        this->info_vector_correction.setZero();
        this->info_matrix_correction.setZero();
    }
};

template <int InDim, int OutDim>
InfoCorrection<InDim> get_info_correction(
    const Stats<InDim, OutDim>& ut_stats,
    const Eigen::Matrix<Scalar, OutDim, 1>& detection,
    const Eigen::Matrix<Scalar, OutDim, OutDim>& detector_covariance_inverse) {
    const Eigen::Matrix<Scalar, OutDim, InDim>& D =
        ut_stats.statistical_jacobian;

    /* additional cost is
       || D dx - r ||^2_S =
       (dx.t Dt - r.t ) S^-1 ( D dx - r) =
        dx.t Dt Si D dx - dx.t Dt Si r - r Si D dx + r Si r =
        (x - m)t Dt Si D (x - m) - x Dt Si r - r Si D x + ... =
        xt Dt Si D x - xt Dt Si D m - mt Dt Si D x - xt Dt Si r - r Si D x +
        ...
        // multiply the above by -1/2 to get info parameterization
        // also m = 0
    */

    // info matrix change by
    // + Dt Si D

    // info vector change by
    // + Dt Si D m + Dt Si r = Dt Si r since m = 0

    const Eigen::Matrix<Scalar, OutDim, 1> residual = detection - ut_stats.mean;

    InfoCorrection<InDim> correction;
    correction.info_matrix_correction =
        D.transpose() * detector_covariance_inverse * D;
    correction.info_vector_correction =
        D.transpose() * detector_covariance_inverse * residual;

    return correction;
}

template <int DIM>
void apply_correction(const InfoCorrection<DIM>& correction,
                      Eigen::Matrix<Scalar, DIM, 1>* mean,
                      Eigen::Matrix<Scalar, DIM, DIM>* covariance) {

    Eigen::Matrix<Scalar, DIM, DIM> information_matrix =
        covariance->inverse() + correction.info_matrix_correction;

    Eigen::Matrix<Scalar, DIM, 1> information_vector =
        correction.info_vector_correction;

    *covariance = information_matrix.inverse();
    *mean += *covariance * information_vector;
}

struct InferenceState {
    Scalar dt; // time between dynamical samples
    Scalar rs; // resistor measurement
    std::vector<Measurement> measurements;

    MotorModel motor_model;
    Eigen::Matrix<Scalar, MotorModel::DIM, num_sigma_points(MotorModel::DIM)>
        motor_model_sigma_points;
    InfoCorrection<MotorModel::DIM> motor_model_info_correction;

    std::vector<DynamicalState> dynamical_states;
    std::vector<Eigen::Matrix<Scalar, DynamicalState::DIM,
                              num_sigma_points(DynamicalState::DIM)>>
        dynamical_state_sigma_points;

    std::vector<InfoCorrection<DynamicalState::DIM>>
        dynamical_state_info_corrections;

    // for every edge in the factor graph, we need a ut_stats
    std::vector<Stats<DynamicalState::DIM, DynamicalState::DIM>>
        dynamical_state__prev;
    std::vector<Stats<DynamicalState::DIM, DynamicalState::DIM>>
        dynamical_state__next;
    std::vector<Stats<DynamicalState::DIM, Measurement::DIM>>
        dynamical_state__meas;
    std::vector<Stats<DynamicalState::DIM, 1>> dynamical_state__motor_model;
    std::vector<Stats<MotorModel::DIM, 1>> motor_model__dynamical_state;
};

void reset_inference_state(InferenceState* inference) {
    inference->motor_model.mean.setZero();
    inference->motor_model.covariance.setIdentity();

    for (auto& ds : inference->dynamical_states) {
        ds.mean.setZero();
        ds.covariance.setIdentity();
    }

    // for (auto& meas : inference->measurements) {
    // meas.mean.setZero();
    // meas.covariance.setIdentity();
    // }
}

void init_inference_state(const int num_data_points, const Scalar dt,
                          const Scalar rs, InferenceState* inference) {
    inference->rs = rs;
    inference->dt = dt;
    inference->measurements.resize(num_data_points);
    inference->dynamical_states.resize(num_data_points);
    inference->dynamical_state_sigma_points.resize(num_data_points);
    inference->dynamical_state_info_corrections.resize(num_data_points);
    inference->dynamical_state__prev.resize(num_data_points);
    inference->dynamical_state__next.resize(num_data_points);
    inference->dynamical_state__meas.resize(num_data_points);
    inference->dynamical_state__motor_model.resize(num_data_points);
    inference->motor_model__dynamical_state.resize(num_data_points);

    reset_inference_state(inference);
}

template <int InDim>
Eigen::Matrix<Scalar, InDim, num_sigma_points(InDim)>
get_sigma_points(const Eigen::Matrix<Scalar, InDim, 1>& mean,
                 const Eigen::Matrix<Scalar, InDim, InDim>& covariance) {

    Eigen::Matrix<Scalar, InDim, num_sigma_points(InDim)> sigma_points;

    const Eigen::Matrix<Scalar, InDim, InDim> cov_llt_l =
        covariance.llt().matrixL();

    for (int i = 0; i < num_sigma_points(InDim); ++i) {
        const Scalar posneg = i % 2 == 0 ? 1.0f : -1.0f;
        sigma_points.col(i) = posneg * scale * cov_llt_l.col(i / 2) + mean;
    }

    return sigma_points;
};

Scalar motor_dynamics_factor(
    const Scalar rs,
    const Eigen::Matrix<Scalar, MotorModel::DIM, 1> motor_model,
    const Eigen::Matrix<Scalar, DynamicalState::DIM, 1> dynamical_state) {

    const Scalar bEmf = fourier_to_normed_bEmf(
        dynamical_state(DynamicalState::IDX_THETA),
        motor_model(MotorModel::IDX_PHASE), motor_model(MotorModel::IDX_C1),
        motor_model(MotorModel::IDX_C5), motor_model(MotorModel::IDX_C7),
        motor_model(MotorModel::IDX_C11), motor_model(MotorModel::IDX_C13));

    const Scalar result =
        dynamical_state(DynamicalState::IDX_THETA_DOT) * bEmf +
        motor_model(MotorModel::IDX_INDUCTANCE) *
            (dynamical_state(DynamicalState::IDX_VOLTAGE_DOT) / rs) +
        motor_model(MotorModel::IDX_RESISTANCE) *
            (dynamical_state(DynamicalState::IDX_VOLTAGE_DOT) / rs) +
        dynamical_state(DynamicalState::IDX_VOLTAGE);

    return result;
}

void do_inference(InferenceState* inference) {
    // start by computing sigma points
    for (int i = 0; i < inference->dynamical_states.size(); ++i) {
        const auto& dynamical_state = inference->dynamical_states[i];
        inference->dynamical_state_sigma_points[i] =
            get_sigma_points<DynamicalState::DIM>(dynamical_state.mean,
                                                  dynamical_state.covariance);
    }
    inference->motor_model_sigma_points = get_sigma_points<MotorModel::DIM>(
        inference->motor_model.mean, inference->motor_model.covariance);

    const Scalar dt = inference->dt;

    // clang-format off
    Eigen::Matrix<Scalar, 4, 4> state_evolution;
    state_evolution <<
        1, dt,  0,  0,
        0,  1,  0,  0,
        0,  0,  1, dt,
        0,  0,  0,  1;
    // clang-format on

    Eigen::Matrix<Scalar, 3, 4> meas_func;
    // clang-format off
    meas_func <<
        1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0;
    // clang-format on

    // now compute ut stats into every edge
    for (int i = 1; i < inference->dynamical_states.size(); ++i) {
        const auto& curr = inference->dynamical_states[i];
        const auto& prev = inference->dynamical_states[i - 1];
        auto& stats = inference->dynamical_state__prev[i];
        stats.cross_covariance = -curr.covariance;
        stats.covariance = curr.covariance;
        stats.statistical_jacobian = -Eigen::Matrix<Scalar, 4, 4>::Identity();
        stats.mean = state_evolution * prev.mean - curr.mean;
    }
    for (int i = 0; i + 1 < inference->dynamical_states.size(); ++i) {
        const auto& curr = inference->dynamical_states[i];
        const auto& next = inference->dynamical_states[i + 1];
        auto& stats = inference->dynamical_state__next[i];
        stats.cross_covariance = curr.covariance * state_evolution.transpose();
        stats.covariance =
            state_evolution * curr.covariance * state_evolution.transpose();
        stats.statistical_jacobian = state_evolution;
        stats.mean = state_evolution * curr.mean - next.mean;
    }
    for (int i = 0; i < inference->dynamical_states.size(); ++i) {
        const auto& curr = inference->dynamical_states[i];
        const auto& meas = inference->measurements[i];
        auto& stats = inference->dynamical_state__meas[i];
        stats.cross_covariance = curr.covariance * meas_func.transpose();
        stats.covariance = meas_func * curr.covariance * meas_func.transpose();
        stats.statistical_jacobian = meas_func;
        stats.mean = meas_func * curr.mean;
    }
    for (int i = 0; i < inference->dynamical_states.size(); ++i) {
        const auto& motor_model = inference->motor_model;
        const auto& motor_model_sigma_points =
            inference->motor_model_sigma_points;
        const auto& dynamical_state = inference->dynamical_states[i];
        const auto& dynamical_state_sigma_points =
            inference->dynamical_state_sigma_points[i];

        const Scalar rs = inference->rs;
        Eigen::Matrix<Scalar, 1, 1> output_mean;
        output_mean << motor_dynamics_factor(rs, motor_model.mean,
                                             dynamical_state.mean);

        {
            // dynamical_state -> factor
            Eigen::Matrix<Scalar, 1, num_sigma_points(DynamicalState::DIM)>
                output_sigma_points;
            for (int s = 0; s < num_sigma_points(DynamicalState::DIM); ++s) {
                output_sigma_points.col(s) << motor_dynamics_factor(
                    rs, inference->motor_model.mean,
                    dynamical_state_sigma_points.col(s));
            }

            inference->dynamical_state__motor_model[i] =
                get_unscented_transform_stats<DynamicalState::DIM, 1>(
                    dynamical_state.mean, dynamical_state.covariance,
                    dynamical_state_sigma_points, output_mean,
                    output_sigma_points);
        }

        {
            // motor_model -> factor
            Eigen::Matrix<Scalar, 1, num_sigma_points(MotorModel::DIM)>
                output_sigma_points;
            for (int s = 0; s < num_sigma_points(MotorModel::DIM); ++s) {
                output_sigma_points.col(s) << motor_dynamics_factor(
                    rs, inference->motor_model_sigma_points.col(s),
                    dynamical_state.mean);
            }

            inference->motor_model__dynamical_state[i] =
                get_unscented_transform_stats<MotorModel::DIM, 1>(
                    motor_model.mean, motor_model.covariance,
                    inference->motor_model_sigma_points, output_mean,
                    output_sigma_points);
        }
    }

    // tweak these
    Eigen::Matrix<Scalar, DynamicalState::DIM, DynamicalState::DIM> process_cov;
    process_cov.setIdentity();
    process_cov(DynamicalState::IDX_THETA) *= 1.0;
    process_cov(DynamicalState::IDX_THETA_DOT) *= 10.0;
    process_cov(DynamicalState::IDX_VOLTAGE) *= 1.0;
    process_cov(DynamicalState::IDX_VOLTAGE_DOT) *= 10.0;

    // Now collect information updates
    inference->motor_model_info_correction.clear();
    for (auto& ic : inference->dynamical_state_info_corrections) {
        ic.clear();
    }

    // updates from dynamical_state__prev factor
    for (int i = 1; i < inference->dynamical_states.size(); ++i) {
        const auto& stats = inference->dynamical_state__prev[i];
        const auto& from_prev_stats = inference->dynamical_state__next[i - 1];

        auto& info_correction = inference->dynamical_state_info_corrections[i];
        info_correction += get_info_correction<DynamicalState::DIM,
                                               DynamicalState::DIM>(
            stats,
            /*detection=*/Eigen::Matrix<Scalar, DynamicalState::DIM, 1>::Zero(),
            /*detector_covariance_inverse=*/
            (process_cov + from_prev_stats.covariance).inverse());
    }
    // updates from dynamical_state__next factor
    for (int i = 0; i + 1 < inference->dynamical_states.size(); ++i) {
        const auto& stats = inference->dynamical_state__next[i];
        const auto& from_next_stats = inference->dynamical_state__prev[i + 1];
        auto& info_correction = inference->dynamical_state_info_corrections[i];
        info_correction += get_info_correction<DynamicalState::DIM,
                                               DynamicalState::DIM>(
            stats,
            /*detection=*/Eigen::Matrix<Scalar, DynamicalState::DIM, 1>::Zero(),
            /*detector_covariance_inverse=*/
            (process_cov + from_next_stats.covariance).inverse());
    }
    // updates from dynamical_state__meas factor
    for (int i = 0; i < inference->dynamical_states.size(); ++i) {
        const auto& meas = inference->measurements[i];
        auto& stats = inference->dynamical_state__meas[i];
        auto& info_correction = inference->dynamical_state_info_corrections[i];
        info_correction +=
            get_info_correction<DynamicalState::DIM, Measurement::DIM>(
                stats, meas.mean, meas.covariance.inverse());
    }

    Eigen::Matrix<Scalar, 1, 1> eq_factor_cov;
    eq_factor_cov << 0.1;

    // updates from dynamical_state__motor_model factor
    for (int i = 0; i < inference->dynamical_states.size(); ++i) {
        const auto& stats = inference->dynamical_state__motor_model[i];
        const auto& motor_model_stats =
            inference->motor_model__dynamical_state[i];
        auto& info_correction = inference->dynamical_state_info_corrections[i];
        info_correction += get_info_correction<DynamicalState::DIM, 1>(
            stats,
            /*detection=*/Eigen::Matrix<Scalar, 1, 1>::Zero(),
            (motor_model_stats.covariance + eq_factor_cov).inverse());
    }
    // updates from motor_model__dynamical_state factor
    for (int i = 0; i < inference->dynamical_states.size(); ++i) {
        const auto& dynamical_state_stats =
            inference->dynamical_state__motor_model[i];
        const auto& stats = inference->motor_model__dynamical_state[i];
        auto& info_correction = inference->motor_model_info_correction;
        info_correction += get_info_correction<MotorModel::DIM, 1>(
            stats,
            /*detection=*/Eigen::Matrix<Scalar, 1, 1>::Zero(),
            (dynamical_state_stats.covariance + eq_factor_cov).inverse());
    }

    // finally apply updates
    apply_correction<MotorModel::DIM>(inference->motor_model_info_correction,
                                      &inference->motor_model.mean,
                                      &inference->motor_model.covariance);
    for (int i = 0; i < inference->dynamical_states.size(); ++i) {
        const auto& info_correction =
            inference->dynamical_state_info_corrections[i];
        auto& state = inference->dynamical_states[i];
        apply_correction<DynamicalState::DIM>(info_correction, &state.mean,
                                              &state.covariance);
    }
}

int main(int argc, char* argv[]) {
    wrappers::SdlContext sdl_context("Biro Calibration Simulator",
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

    Eigen::Matrix<Scalar, 5, 1> normed_bEmf_coeffs;
    normed_bEmf_coeffs << 0.01, 0, 0, 0, 0;
    const Scalar cal_resistance = 0.1;
    const Scalar phase_resistance = 1.0;
    const Scalar phase_inductance = 1e-3;
    const int num_pole_pairs = 4;
    const float jiggle_freq = 3;

    int step_multiplier = 200;
    Scalar time = 0;
    Scalar dt = 1e-4;
    Scalar rotor_angle = 0;
    Scalar rotor_angular_vel = 0;
    Scalar cal_voltage = 0;
    MotorElectricalState motor_electrical_state;

    Scalar data_collect_timer = 0;
    const Scalar data_collect_period = 0.01;
    constexpr int kNumDataPoints = 2;
    int num_data_pts = 0;
    std::array<Scalar, kNumDataPoints> time_data;
    std::array<Scalar, kNumDataPoints> electrical_angle_data;
    std::array<Scalar, kNumDataPoints> electrical_angular_vel_data;
    std::array<Scalar, kNumDataPoints> cal_voltage_data;

    RollingBufferContext rolling_buffer_ctx{kNumRollingPts};
    std::array<Scalar, kNumRollingPts> rolling_times;
    std::array<Scalar, kNumRollingPts> rolling_electrical_angles;
    std::array<Scalar, kNumRollingPts> rolling_electrical_angular_vels;
    std::array<Scalar, kNumRollingPts> rolling_cal_voltages;

    InferenceState inference;
    init_inference_state(kNumDataPoints, data_collect_period, cal_resistance,
                         &inference);

    bool quit_flag = false;
    while (!quit_flag) {
        quit_flag = wrappers::process_sdl_imgui_events(sdl_context.window_);
        if (quit_flag) {
            break;
        }

        for (int i = 0; i < step_multiplier; ++i) {
            if (num_data_pts == kNumDataPoints) {
                break;
            }

            rotor_angle = 2 * std::sin(time * jiggle_freq * 2 * kPI);
            rotor_angular_vel = 2 * jiggle_freq * 2 * kPI *
                                std::cos(time * jiggle_freq * 2 * kPI);

            const Scalar electrical_angle = rotor_angle * num_pole_pairs;
            const Scalar electrical_angular_vel =
                rotor_angular_vel * num_pole_pairs;

            get_bEmfs(normed_bEmf_coeffs, electrical_angle,
                      electrical_angular_vel,
                      &motor_electrical_state.normed_bEmfs,
                      &motor_electrical_state.bEmfs);

            // poles all connected to ground in the calibration rig
            const Eigen::Matrix<Scalar, 3, 1> pole_voltages =
                Eigen::Matrix<Scalar, 3, 1>::Zero();

            step_motor_electrical(dt, phase_resistance + cal_resistance,
                                  phase_inductance, pole_voltages,
                                  &motor_electrical_state);

            cal_voltage =
                -cal_resistance * motor_electrical_state.phase_currents(0);

            rolling_times[rolling_buffer_ctx.next_idx] = time;
            rolling_electrical_angles[rolling_buffer_ctx.next_idx] =
                electrical_angle;
            rolling_electrical_angular_vels[rolling_buffer_ctx.next_idx] =
                electrical_angular_vel;
            rolling_cal_voltages[rolling_buffer_ctx.next_idx] = cal_voltage;

            rolling_buffer_advance_idx(&rolling_buffer_ctx);

            if (periodic_timer(data_collect_period, dt, &data_collect_timer) &&
                num_data_pts < kNumDataPoints) {
                time_data[num_data_pts] = time;
                cal_voltage_data[num_data_pts] = cal_voltage;
                electrical_angle_data[num_data_pts] = electrical_angle;
                electrical_angular_vel_data[num_data_pts] =
                    electrical_angular_vel;
                ++num_data_pts;

                if (num_data_pts == kNumDataPoints) {
                    // Data collect complete!
                    // convert to Eigen data
                    for (int i = 0; i < kNumDataPoints; ++i) {
                        auto& meas = inference.measurements[i];
                        meas.mean(Measurement::IDX_THETA) =
                            electrical_angle_data[i];
                        meas.mean(Measurement::IDX_THETA_DOT) =
                            electrical_angular_vel_data[i];
                        meas.mean(Measurement::IDX_VOLTAGE) =
                            cal_voltage_data[i];
                    }
                }
            }

            time += dt;
        }

        wrappers::sdl_imgui_newframe(sdl_context.window_);

        ImGui::Begin("Calibration Control");
        ImGui::Text("Rotor Angle: %f", rotor_angle);
        ImGui::Text("Cal Voltage: %f", cal_voltage);
        ImGui::Text("bEmfs: %f %f %f", motor_electrical_state.bEmfs(0),
                    motor_electrical_state.bEmfs(1),
                    motor_electrical_state.bEmfs(2));

        ImGui::SliderInt("Step Multiplier", &step_multiplier, 1, 1000);
        ImGui::Text("time (sec) %f", time);
        ImGui::Text("dt (sec) %f", step_multiplier * dt);

        if (ImGui::Button("Recollect Data")) {
            num_data_pts = 0;
        }
        if (num_data_pts < kNumDataPoints) {
            ImGui::Text("Collecting data... %d/%d", num_data_pts,
                        kNumDataPoints);
        } else {
            ImGui::Text("Data collection complete");
        }

        ImGui::End();

        const int begin = get_rolling_buffer_begin(rolling_buffer_ctx);
        const Scalar begin_time = rolling_times[begin];
        const Scalar end_time =
            rolling_times[get_rolling_buffer_back(rolling_buffer_ctx)];
        const int count = get_rolling_buffer_count(rolling_buffer_ctx);

        ImGui::Begin("Ground Truth");
        ImPlot::SetNextPlotLimitsX(begin_time, end_time, ImGuiCond_Always);
        ImPlot::SetNextPlotLimitsY(-2, 2, ImGuiCond_Once);
        if (ImPlot::BeginPlot("Electrical Angle", "Seconds", "",
                              ImVec2(-1, 300))) {
            ImPlot::PlotLine("Electrical Angle", rolling_times.data(),
                             rolling_electrical_angles.data(), count, begin,
                             sizeof(Scalar));
            ImPlot::EndPlot();
        }
        ImPlot::SetNextPlotLimitsX(begin_time, end_time, ImGuiCond_Always);
        ImPlot::SetNextPlotLimitsY(-2, 2, ImGuiCond_Once);
        if (ImPlot::BeginPlot("Electrical Angular Vel", "Seconds", "",
                              ImVec2(-1, 300))) {
            ImPlot::PlotLine("Electrical Angular Vel", rolling_times.data(),
                             rolling_electrical_angular_vels.data(), count,
                             begin, sizeof(Scalar));

            ImPlot::EndPlot();
        }

        ImPlot::SetNextPlotLimitsX(begin_time, end_time, ImGuiCond_Always);
        ImPlot::SetNextPlotLimitsY(-2, 2, ImGuiCond_Once);
        if (ImPlot::BeginPlot("Cal Voltage", "Seconds", "", ImVec2(-1, 300))) {
            ImPlot::PlotLine("Cal Voltage", rolling_times.data(),
                             rolling_cal_voltages.data(), count, begin,
                             sizeof(Scalar));
            ImPlot::EndPlot();
        }
        ImGui::End();

        ImGui::Begin("Inference");

        static bool iterate = false;
        if (num_data_pts == kNumDataPoints) {
            if (ImGui::Button("Reset")) {
                reset_inference_state(&inference);
            }
            ImGui::Checkbox("Iterate", &iterate);
            if (iterate) {
                do_inference(&inference);
            }

            ImGui::Text("Phase %d",
                        inference.motor_model.mean(MotorModel::IDX_PHASE));
            ImGui::Text("C1 %d",
                        inference.motor_model.mean(MotorModel::IDX_C1));
            ImGui::Text("C5 %d",
                        inference.motor_model.mean(MotorModel::IDX_C5));
            ImGui::Text("C7 %d",
                        inference.motor_model.mean(MotorModel::IDX_C7));
            ImGui::Text("C11 %d",
                        inference.motor_model.mean(MotorModel::IDX_C11));
            ImGui::Text("C13 %d",
                        inference.motor_model.mean(MotorModel::IDX_C13));

            static int data_idx = 0;
            ImGui::SliderInt("Data idx", &data_idx, 0, kNumDataPoints - 1);

            const auto& meas = inference.measurements[data_idx];
            ImGui::Text("Meas: theta %f, theta_dot %f, voltage %f",
                        meas.mean(Measurement::IDX_THETA),
                        meas.mean(Measurement::IDX_THETA_DOT),
                        meas.mean(Measurement::IDX_VOLTAGE));

            const auto& dynamical_state = inference.dynamical_states[data_idx];
            ImGui::Text("Estd State: theta %f, theta_dot %f, voltage %f, "
                        "voltage_dot %f",
                        dynamical_state.mean(DynamicalState::IDX_THETA),
                        dynamical_state.mean(DynamicalState::IDX_THETA_DOT),
                        dynamical_state.mean(DynamicalState::IDX_VOLTAGE),
                        dynamical_state.mean(DynamicalState::IDX_VOLTAGE_DOT));

            for (int i = 0; i < num_sigma_points(DynamicalState::DIM); ++i) {
                const auto& sigma =
                    inference.dynamical_state_sigma_points[data_idx].col(i);
                ImGui::Text(
                    " - sigma %d: theta %f, theta_dot %f, voltage %f, "
                    "voltage_dot %f",
                    i, dynamical_state.mean(DynamicalState::IDX_THETA),
                    dynamical_state.mean(DynamicalState::IDX_THETA_DOT),
                    dynamical_state.mean(DynamicalState::IDX_VOLTAGE),
                    dynamical_state.mean(DynamicalState::IDX_VOLTAGE_DOT));
            }

        } else {
            ImGui::Text("Collecting data... %d/%d", num_data_pts,
                        kNumDataPoints);
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
