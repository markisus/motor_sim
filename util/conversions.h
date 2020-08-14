#pragma once

#include <Eigen/Dense>
#include <complex>

template <typename TScalar>
inline std::complex<TScalar>
to_complex(const Eigen::Matrix<TScalar, 2, 1>& vect2) {
    return {vect2(0), vect2(1)};
}
