#pragma once

#include "util/conversions.h"
#include <Eigen/Dense>
#include <complex>

// these are the power invariant Clarke transforms
extern const Eigen::Matrix<double, 3, 3> kClarkeTransform3x3;
extern const Eigen::Matrix<double, 2, 3> kClarkeTransform2x3;

extern const double kClarkeScale; // magnitude scaling factor going from regular
                                  // voltages to space vector

extern const float kClarkeScalef; // float version of the above

inline std::complex<double>
clarke_transform(const Eigen::Matrix<double, 3, 1>& state) {
    return to_complex<double>((kClarkeTransform2x3 * state).head<2>());
}

inline std::complex<float>
clarke_transform(const Eigen::Matrix<float, 3, 1>& state) {
    return to_complex<float>(
        (kClarkeTransform2x3.cast<float>() * state).head<2>());
}
