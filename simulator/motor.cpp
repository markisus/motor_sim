#include "motor.h"
#include "sine_series.h"

PiParams make_motor_pi_params(Scalar bandwidth, Scalar resistance,
                              Scalar inductance) {
    PiParams params;
    params.p_gain = inductance * bandwidth;
    params.i_gain = resistance * bandwidth;
    params.max_integral = std::numeric_limits<Scalar>::infinity();
    return params;
}

Scalar get_back_emf(const Eigen::Matrix<Scalar, 5, 1>& normed_bEmf_coeffs,
                    const Scalar electrical_angle) {
    Eigen::Matrix<Scalar, 5, 1> sines;
    generate_odd_sine_series(/*num_terms=*/sines.rows(), electrical_angle,
                             sines.data());
    return sines.dot(normed_bEmf_coeffs);
}

void step_motor(const Scalar dt, const Scalar load_torque,
                const Eigen::Matrix<Scalar, 3, 1>& pole_voltages,
                MotorState* motor) {

    motor->pole_voltages = pole_voltages;
    motor->normed_bEmfs << // clang-format off
        get_back_emf(motor->normed_bEmf_coeffs, motor->electrical_angle),
        get_back_emf(motor->normed_bEmf_coeffs,
                     motor->electrical_angle - 2 * kPI / 3),
        get_back_emf(motor->normed_bEmf_coeffs,
                     motor->electrical_angle - 4 * kPI / 3); // clang-format on

    motor->bEmfs = motor->normed_bEmfs * motor->rotor_angular_vel;

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

    motor->encoder_position =
        motor->cogging_torque_map.size() *
        std::clamp<Scalar>(motor->rotor_angle / (2 * kPI), 0.0, 1.0);

    const Scalar cogging_torque =
        motor->cogging_torque_map[int(motor->encoder_position)];

    motor->torque = motor->phase_currents.dot(motor->normed_bEmfs) +
                    cogging_torque + load_torque;

    motor->rotor_angular_accel = motor->torque / motor->rotor_inertia;
    motor->rotor_angular_vel += motor->rotor_angular_accel * dt;
    motor->rotor_angle += motor->rotor_angular_vel * dt;
    motor->rotor_angle = std::fmod(motor->rotor_angle, 2 * kPI);
    if (motor->rotor_angle < 0) {
        motor->rotor_angle += 2 * kPI;
    }

    motor->electrical_angle = motor->rotor_angle * motor->num_pole_pairs;
    motor->electrical_angle = std::fmod(motor->electrical_angle, 2 * kPI);
    motor->q_axis_electrical_angle =
        motor->electrical_angle + motor->q_axis_offset;
}
