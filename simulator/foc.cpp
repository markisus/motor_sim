#include "foc.h"
#include "util/clarke_transform.h"
#include "util/rotation.h"

std::complex<Scalar>
get_desired_current_qd_non_sinusoidal(const Scalar desired_torque,
                                      const MotorState& motor) {
    const std::complex<Scalar> park_transform =
        get_rotation(-motor.q_axis_electrical_angle);

    // try to generate torque only along the q axis, even if there are
    // d-axis harmonics present
    const std::complex<Scalar> normed_bEmf_qd =
        park_transform * clarke_transform(motor.normed_bEmfs);
    // todo: handle when normed_bEmf_qd.real() == 0
    const Scalar desired_current_q = desired_torque / normed_bEmf_qd.real();

    return {desired_current_q, 0};
}

std::complex<Scalar> get_desired_current_qd(const Scalar desired_torque,
                                            const Scalar normed_bEmf0) {
    const Scalar desired_current_q =
        desired_torque / (kClarkeScale * normed_bEmf0 * 1.5);
    return {desired_current_q, 0};
}

void step_foc_current_controller(const std::complex<Scalar>& desired_current_qd,
                                 const MotorState& motor, FocState* foc_state) {
    const std::complex<Scalar> park_transform =
        get_rotation(-motor.q_axis_electrical_angle);
    const std::complex<Scalar> current_qd =
        park_transform * clarke_transform(motor.phase_currents);
    const Scalar voltage_q = pi_control(
        foc_state->i_controller_params, &foc_state->iq_controller,
        foc_state->period, current_qd.real(), desired_current_qd.real());
    const Scalar voltage_d = pi_control(
        foc_state->i_controller_params, &foc_state->id_controller,
        foc_state->period, current_qd.imag(), desired_current_qd.imag());
    foc_state->voltage_qd = {voltage_q, voltage_d};
}
