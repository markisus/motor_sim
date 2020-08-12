#pragma once

#include "conversions.h"
#include "scalar.h"
#include <Eigen/Dense>
#include <complex>

extern const Eigen::Matrix<Scalar, 3, 3> kClarkeTransform3x3;
extern const Eigen::Matrix<Scalar, 2, 3> kClarkeTransform2x3;
extern const Scalar kClarkeScale; // magnitude scaling factor going from regular
                                  // voltages to space vector

inline std::complex<Scalar>
clarke_transform(const Eigen::Matrix<Scalar, 3, 1>& state) {
    return to_complex<Scalar>((kClarkeTransform2x3 * state).head<2>());
}
