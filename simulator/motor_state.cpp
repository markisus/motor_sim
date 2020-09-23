#include "motor_state.h"
#include "util/sine_series.h"
#include <algorithm>

Scalar get_normed_bEmf(const Eigen::Matrix<Scalar, 5, 1>& normed_bEmf_coeffs,
                       const Scalar electrical_angle) {
    Eigen::Matrix<Scalar, 5, 1> sines;
    generate_odd_sine_series(/*num_terms=*/sines.rows(), electrical_angle,
                             sines.data());
    return sines.dot(normed_bEmf_coeffs);
}

Eigen::Matrix<Scalar, 3, 1>
get_normed_bEmfs(const Eigen::Matrix<Scalar, 5, 1>& normed_bEmf_coeffs,
                 const Scalar electrical_angle) {
    Eigen::Matrix<Scalar, 3, 1> normed_bEmfs;
    normed_bEmfs << // clang-format off
        get_normed_bEmf(normed_bEmf_coeffs, electrical_angle),
        get_normed_bEmf(normed_bEmf_coeffs, electrical_angle - 2 * kPI / 3),
        get_normed_bEmf(normed_bEmf_coeffs, electrical_angle - 4 * kPI / 3); // clang-format on
    return normed_bEmfs;
}

void get_bEmfs(const Eigen::Matrix<Scalar, 5, 1>& normed_bEmf_coeffs,
               const Scalar electrical_angle,
               const Scalar electrical_angular_vel,
               Eigen::Matrix<Scalar, 3, 1>* normed_bEmfs,
               Eigen::Matrix<Scalar, 3, 1>* bEmfs) {
    *normed_bEmfs = get_normed_bEmfs(normed_bEmf_coeffs, electrical_angle);

    // multiplying by electrical angular vel because normed bEmf map is
    // expressed in electrical angle.
    // This assumes the bEmf map was collected by dividing back emf by
    // electrical velocity.

    *bEmfs = *normed_bEmfs * electrical_angular_vel;
}

Scalar
interp_cogging_torque(const Scalar rotor_angle,
                      const std::array<Scalar, 3600>& cogging_torque_map) {
    const Scalar encoder_position =
        cogging_torque_map.size() *
        std::clamp<Scalar>(rotor_angle / (2 * kPI), 0.0, 1.0);
    const int integral_part = int(encoder_position);
    const Scalar fractional_part = encoder_position - integral_part;
    const Scalar t1 = cogging_torque_map[integral_part];
    const Scalar t2 =
        cogging_torque_map[(integral_part + 1) % cogging_torque_map.size()];
    return t1 * (1.0 - fractional_part) + t2 * fractional_part;
}

Scalar get_electrical_angle(const int num_pole_pairs,
                            const Scalar rotor_angle) {
    return std::fmod(rotor_angle * num_pole_pairs, 2 * kPI);
}

Scalar get_q_axis_electrical_angle(const int num_pole_pairs,
                                   const Scalar rotor_angle) {
    return get_electrical_angle(num_pole_pairs, rotor_angle) + kQAxisOffset;
}
