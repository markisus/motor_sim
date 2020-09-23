#include "motor.h"
#include "util/sine_series.h"

PiParams make_motor_pi_params(Scalar bandwidth, Scalar resistance,
                              Scalar inductance) {
    PiParams params;
    params.p_gain = inductance * bandwidth;
    params.i_gain = resistance * bandwidth;
    return params;
}

Scalar get_back_emf(const Eigen::Matrix<Scalar, 5, 1>& normed_bEmf_coeffs,
                    const Scalar electrical_angle) {
    Eigen::Matrix<Scalar, 5, 1> sines;
    generate_odd_sine_series(/*num_terms=*/sines.rows(), electrical_angle,
                             sines.data());
    return sines.dot(normed_bEmf_coeffs);
}

Eigen::Matrix<Scalar, 3, 1>
get_normed_back_emfs(const Eigen::Matrix<Scalar, 5, 1>& normed_bEmf_coeffs,
                     const Scalar electrical_angle) {
    Eigen::Matrix<Scalar, 3, 1> normed_bEmfs;
    normed_bEmfs << // clang-format off
        get_back_emf(normed_bEmf_coeffs, electrical_angle),
        get_back_emf(normed_bEmf_coeffs, electrical_angle - 2 * kPI / 3),
        get_back_emf(normed_bEmf_coeffs, electrical_angle - 4 * kPI / 3); // clang-format on
    return normed_bEmfs;
}
/*
// calculates
// - bEmfs
// - phase_voltages
// - phase_currents
void step_motor_electrical(const Scalar dt,
                           const Eigen::Matrix<Scalar, 3, 1>& pole_voltages,
                           const MotorParams& params,
                           const Scalar rotor_angular_vel,
                           MotorElecticalState* e_state) {

    e_state->normed_bEmfs << // clang-format off
        get_back_emf(e_state->normed_bEmf_coeffs, e_state->electrical_angle),
        get_back_emf(e_state->normed_bEmf_coeffs,
                     e_state->electrical_angle - 2 * kPI / 3),
        get_back_emf(e_state->normed_bEmf_coeffs,
                     e_state->electrical_angle - 4 * kPI / 3); // clang-format
on

    const Scalar electrical_angular_vel =
        e_state->rotor_angular_vel * e_state->num_pole_pairs;

    // multiplying by electrical angular vel because normed bEmf map is
    // expressed in electrical angle.
    // This assumes the bEmf map was collected by dividing back emf by
    // electrical velocity.
    e_state->bEmfs = e_state->normed_bEmfs * electrical_angular_vel;

    // compute neutral point voltage
    // todo: derivation
    const Scalar neutral_voltage =
        (pole_voltages.sum() - e_state->bEmfs.sum()) / 3;

    for (int i = 0; i < 3; ++i) {
        e_state->phase_voltages(i) = pole_voltages(i) - neutral_voltage;
    }

    Eigen::Matrix<Scalar, 3, 1> di_dt;
    for (int i = 0; i < 3; ++i) {
        di_dt(i) = (e_state->phase_voltages(i) - e_state->bEmfs(i) -
                    e_state->phase_currents(i) * e_state->phase_resistance) /
                   e_state->phase_inductance;
    }

    e_state->phase_currents += di_dt * dt;
}
*/

Eigen::Matrix<Scalar, 3, 1>
get_phase_voltages(const Eigen::Matrix<Scalar, 3, 1>& pole_voltages,
                   const Eigen::Matrix<Scalar, 3, 1>& bEmfs) {
    // todo: derivation
    const Eigen::Matrix<Scalar, 3, 1> phase_voltages =
        pole_voltages -
        Eigen::Matrix<Scalar, 3, 1>::Ones() * pole_voltages.mean();
    return phase_voltages;
};

Eigen::Matrix<Scalar, 3, 1>
get_di_dt(const Scalar phase_resistance, const Scalar phase_inductance,
          const Eigen::Matrix<Scalar, 3, 1>& pole_voltages,
          const Eigen::Matrix<Scalar, 3, 1>& bEmfs,
          const Eigen::Matrix<Scalar, 3, 1>& phase_currents) {

    // zero series components
    const Eigen::Matrix<Scalar, 3, 1> pole_voltages_zs =
        pole_voltages.mean() * Eigen::Matrix<Scalar, 3, 1>::Ones();
    const Eigen::Matrix<Scalar, 3, 1> bEmfs_zs =
        bEmfs.mean() * Eigen::Matrix<Scalar, 3, 1>::Ones();

    Eigen::Matrix<Scalar, 3, 1> di_dt =
        ((pole_voltages - pole_voltages_zs) - (bEmfs - bEmfs_zs) -
         phase_currents * phase_resistance) /
        phase_inductance;
    return di_dt;
};

void step_motor(const Scalar dt, const Scalar load_torque,
                const Eigen::Matrix<Scalar, 3, 1>& pole_voltages,
                MotorState* motor) {
    motor->electrical.normed_bEmfs = get_normed_back_emfs(
        motor->params.normed_bEmf_coeffs, motor->kinematic.electrical_angle);

    const Scalar electrical_angular_vel =
        motor->kinematic.rotor_angular_vel * motor->params.num_pole_pairs;

    // multiplying by electrical angular vel because normed bEmf map is
    // expressed in electrical angle.
    // This assumes the bEmf map was collected by dividing back emf by
    // electrical velocity.
    motor->electrical.bEmfs =
        motor->electrical.normed_bEmfs * electrical_angular_vel;

    motor->electrical.phase_voltages =
        get_phase_voltages(pole_voltages, motor->electrical.bEmfs);

    // todo: handle the case where di_dt = infinity due to too small inductance
    const Eigen::Matrix<Scalar, 3, 1> di_dt =
        get_di_dt(motor->params.phase_resistance,
                  motor->params.phase_inductance, pole_voltages,
                  motor->electrical.bEmfs, motor->electrical.phase_currents);

    motor->electrical.phase_currents += di_dt * dt;

    motor->kinematic.encoder_position =
        motor->params.cogging_torque_map.size() *
        std::clamp<Scalar>(motor->kinematic.rotor_angle / (2 * kPI), 0.0, 1.0);

    const Scalar cogging_torque = interp_cogging_torque(
        motor->kinematic.encoder_position, motor->params.cogging_torque_map);

    motor->kinematic.torque =
        motor->electrical.phase_currents.dot(motor->electrical.normed_bEmfs) +
        cogging_torque + load_torque;

    motor->kinematic.rotor_angular_accel =
        motor->kinematic.torque / motor->params.rotor_inertia;
    motor->kinematic.rotor_angular_vel +=
        motor->kinematic.rotor_angular_accel * dt;
    motor->kinematic.rotor_angle += motor->kinematic.rotor_angular_vel * dt;
    motor->kinematic.rotor_angle =
        std::fmod(motor->kinematic.rotor_angle, 2 * kPI);
    if (motor->kinematic.rotor_angle < 0) {
        motor->kinematic.rotor_angle += 2 * kPI;
    }

    motor->kinematic.electrical_angle =
        motor->kinematic.rotor_angle * motor->params.num_pole_pairs;
    motor->kinematic.electrical_angle =
        std::fmod(motor->kinematic.electrical_angle, 2 * kPI);
    motor->kinematic.q_axis_electrical_angle =
        motor->kinematic.electrical_angle + motor->params.q_axis_offset;
}
