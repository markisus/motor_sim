#include "space_vector_modulation.h"
#include "global_debug.h"
#include "util/clarke_transform.h"
#include "util/conversions.h"
#include "util/math_constants.h"
#include <Eigen/Dense>

const std::array<std::complex<Scalar>, 6> kSvmVectors = []() {
    std::array<std::complex<Scalar>, 6> result;
    result[0] = {1, 0};
    for (int i = 0; i < 6; ++i) {
        result[i] = {std::cos(i * 2 * kPI / 6), std::sin(i * 2 * kPI / 6)};
    }
    return result;
}();

int get_sector(const std::complex<Scalar>& voltage_ab) {
    const std::complex<Scalar> rot_60_deg = std::conj(kSvmVectors[1]);

    std::complex<Scalar> curr = voltage_ab;
    std::complex<Scalar> next = voltage_ab * rot_60_deg;
    int sector;
    for (sector = 0; sector < 5; ++sector) {
        if (curr.imag() >= 0 && next.imag() < 0) {
            // found it
            break;
        }
        // rotate
        curr = next;
        next *= rot_60_deg;
    }

    return sector;
}

std::array<Scalar, 3> get_pwm_duties(const Scalar bus_voltage,
                                     const std::complex<Scalar>& voltage_ab) {
    const int sector_x = get_sector(voltage_ab);
    const int sector_y = (sector_x + 1) % 6;

    const std::complex<Scalar> boundary_x =
        kSvmVectors[sector_x] * kClarkeScale * bus_voltage;
    const std::complex<Scalar> boundary_y =
        kSvmVectors[sector_y] * kClarkeScale * bus_voltage;

    Eigen::Matrix<Scalar, 2, 2> boundaries;
    boundaries <<
        // clang-format off
	boundary_x.real(), boundary_y.real(),
	boundary_x.imag(), boundary_y.imag()
        // clang-format on
        ;
    Eigen::Matrix<Scalar, 2, 1> v_ab;
    v_ab << voltage_ab.real(), voltage_ab.imag();

    Eigen::Matrix<Scalar, 2, 1> coeffs = boundaries.inverse() * v_ab;
    Scalar cx = coeffs(0);
    Scalar cy = coeffs(1);
    Scalar cx_p_cy = cx + cy;
    Scalar c0 = 1.0 - cx_p_cy;
    if (cx_p_cy > 1) {
        c0 = 0;
        cx /= cx_p_cy;
        cy /= cx_p_cy;
    }

    const auto& gx = kSvmStates[sector_x];
    const auto& gy = kSvmStates[sector_y];

    std::array<Scalar, 3> duties;
    for (int i = 0; i < 3; ++i) {
        duties[i] = c0 / 2 + gx[i] * cx + gy[i] * cy;
    }
    return duties;
}

std::complex<Scalar> get_avg_voltage_ab(const Scalar bus_voltage,
                                        const std::array<Scalar, 3>& duties) {
    // optimal sort 3 elements
    int first_gate_on = 0;
    int second_gate_on = 1;
    int third_gate_on = 2;
    if (duties[second_gate_on] > duties[first_gate_on]) {
        std::swap(first_gate_on, second_gate_on);
    }
    if (duties[third_gate_on] > duties[second_gate_on]) {
        std::swap(third_gate_on, second_gate_on);
    }
    // after these two swaps, duties[third_gate_on] is minimal
    if (duties[second_gate_on] > duties[first_gate_on]) {
        std::swap(first_gate_on, second_gate_on);
    }
    // duties[first_gate_on] is maximum

    Eigen::Matrix<Scalar, 3, 1> pole_voltages_x =
        Eigen::Matrix<Scalar, 3, 1>::Zero();
    pole_voltages_x(first_gate_on) = bus_voltage;

    Eigen::Matrix<Scalar, 3, 1> pole_voltages_y = pole_voltages_x;
    pole_voltages_y(second_gate_on) = bus_voltage;

    Eigen::Matrix<Scalar, 3, 1> pole_voltages_avg =
        pole_voltages_x * (duties[first_gate_on] - duties[second_gate_on]) +
        pole_voltages_y * (duties[second_gate_on] - duties[third_gate_on]);

    return to_complex<Scalar>(kClarkeTransform2x3 * pole_voltages_avg);
}
