#include "motor.h"

PiParams make_motor_pi_params(Scalar bandwidth, Scalar resistance,
                              Scalar inductance) {
    PiParams params;
    params.p_gain = inductance * bandwidth;
    params.i_gain = resistance * bandwidth;
    return params;
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
        pole_voltages - Eigen::Matrix<Scalar, 3, 1>::Ones() *
                            (pole_voltages.mean() - bEmfs.mean());
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

void step_motor_electrical(const Scalar dt,
                           const Eigen::Matrix<Scalar, 3, 1>& pole_voltages,
                           const Scalar electrical_angle,
                           const Scalar electrical_angular_vel,
                           const MotorParams& motor_params,
                           MotorElectricalState* motor_electrical) {
    get_bEmfs(motor_params.normed_bEmf_coeffs, electrical_angle,
              electrical_angular_vel, &motor_electrical->normed_bEmfs,
              &motor_electrical->bEmfs);

    // todo: handle the case where di_dt = infinity due to too small inductance
    const Eigen::Matrix<Scalar, 3, 1> di_dt =
        get_di_dt(motor_params.phase_resistance, motor_params.phase_inductance,
                  pole_voltages, motor_electrical->bEmfs,
                  motor_electrical->phase_currents);

    motor_electrical->phase_currents += di_dt * dt;
}

void step_motor_kinematic(const Scalar dt, const Scalar load_torque,
                          const Eigen::Matrix<Scalar, 3, 1>& phase_currents,
                          const Eigen::Matrix<Scalar, 3, 1>& normed_bEmfs,
                          const MotorParams& motor_params,
                          MotorKinematicState* motor_kinematic) {
    const Scalar cogging_torque = interp_cogging_torque(
        motor_kinematic->rotor_angle, motor_params.cogging_torque_map);
    motor_kinematic->torque =
        phase_currents.dot(normed_bEmfs) + cogging_torque + load_torque;
    motor_kinematic->rotor_angular_accel =
        motor_kinematic->torque / motor_params.rotor_inertia;
    motor_kinematic->rotor_angular_vel +=
        motor_kinematic->rotor_angular_accel * dt;
    motor_kinematic->rotor_angle += motor_kinematic->rotor_angular_vel * dt;
    motor_kinematic->rotor_angle =
        std::fmod(motor_kinematic->rotor_angle, 2 * kPI);
    if (motor_kinematic->rotor_angle < 0) {
        motor_kinematic->rotor_angle += 2 * kPI;
    }
}

void step_motor(const Scalar dt, const Scalar load_torque,
                const Eigen::Matrix<Scalar, 3, 1>& pole_voltages,
                MotorState* motor) {

    const Scalar electrical_angle = get_electrical_angle(
        motor->params.num_pole_pairs, motor->kinematic.rotor_angle);
    const Scalar electrical_angular_vel =
        motor->kinematic.rotor_angular_vel * motor->params.num_pole_pairs;

    step_motor_electrical(dt, pole_voltages, electrical_angle,
                          electrical_angular_vel, motor->params,
                          &motor->electrical);
    step_motor_kinematic(dt, load_torque, motor->electrical.phase_currents,
                         motor->electrical.normed_bEmfs, motor->params,
                         &motor->kinematic);
}
