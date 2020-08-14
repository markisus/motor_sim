#pragma once

#include "config/scalar.h"
#include <array>
#include <complex>

constexpr std::array<std::array<bool, 3>, 6> kSvmStates = {
    // clang-format off
    1, 0, 0, // s1
    1, 1, 0, // s2
    0, 1, 0, // s3
    0, 1, 1, // s4
    0, 0, 1, // s5
    1, 0, 1, // s6
    // clang-format on

    // s0 = 0, 0, 0
    // s7 = 1, 1, 1
};

extern const std::array<std::complex<Scalar>, 6> kSvmVectors;

int get_sector(const std::complex<Scalar>& voltage_ab);

std::array<Scalar, 3> get_pwm_duties(const Scalar bus_voltage,
                                     const std::complex<Scalar>& voltage_ab);

std::complex<Scalar> get_avg_voltage_ab(const Scalar bus_voltage,
                                        const std::array<Scalar, 3>& duties);
