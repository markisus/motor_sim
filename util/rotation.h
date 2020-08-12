#pragma once

#include <cmath>
#include <complex>

template <typename TScalar>
std::complex<TScalar> get_rotation(const TScalar angle_radians) {
    return {std::cos(angle_radians), std::sin(angle_radians)};
}
