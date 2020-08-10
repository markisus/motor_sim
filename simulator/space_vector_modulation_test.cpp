#include "clarke_transform.h"
#include "global_debug.h"
#include "pi.h"
#include "scalar.h"
#include "space_vector_modulation.h"
#include <complex>
#include <gtest/gtest.h>
#include <iostream>

std::complex<Scalar> get_rotation(Scalar angle) {
    return {std::cos(angle), std::sin(angle)};
}

TEST(get_sector, 1) { EXPECT_EQ(get_sector(get_rotation(0.01)), 0); }

TEST(get_sector, 2) {
    EXPECT_EQ(get_sector(get_rotation(0.01 + 2 * kPI / 6)), 1);
}

TEST(get_sector, 3) {
    EXPECT_EQ(get_sector(get_rotation(0.01 + 2 * 2 * kPI / 6)), 2);
}

TEST(get_sector, 4) {
    EXPECT_EQ(get_sector(get_rotation(0.01 + 3 * 2 * kPI / 6)), 3);
}

TEST(get_sector, 5) {
    EXPECT_EQ(get_sector(get_rotation(0.01 + 4 * 2 * kPI / 6)), 4);
}

TEST(get_sector, 6) {
    EXPECT_EQ(get_sector(get_rotation(0.01 + 5 * 2 * kPI / 6)), 5);
}

TEST(kSvmStates, 2) {
    const std::array<bool, 3> expected{0, 1, 0};
    for (int i = 0; i < 3; ++i) {
        EXPECT_EQ(kSvmStates[2][i], expected[i]);
    }
}

TEST(get_avg_voltage_ab, zero_duties) {
    std::complex<Scalar> avg_voltage_ab =
        get_avg_voltage_ab(/*bus_voltage=*/5.0, {0, 0, 0});
    EXPECT_LT(std::norm(avg_voltage_ab), 1e-6);
}

TEST(get_avg_voltage_ab, gate_0_full) {
    const Scalar bus_voltage = 3.0;

    std::complex<Scalar> avg_voltage_ab =
        get_avg_voltage_ab(bus_voltage, {1, 0, 0});

    const std::complex<Scalar> expected =
        kSvmVectors[0] * kClarkeScale * bus_voltage;

    EXPECT_LT(std::norm(avg_voltage_ab - expected), 1e-6);
}

TEST(get_avg_voltage_ab, gate_0_half) {
    const Scalar bus_voltage = 3.0;

    std::complex<Scalar> avg_voltage_ab =
        get_avg_voltage_ab(bus_voltage, {0.5, 0, 0});

    const std::complex<Scalar> expected =
        kSvmVectors[0] * kClarkeScale * bus_voltage * 0.5;

    EXPECT_LT(std::norm(avg_voltage_ab - expected), 1e-6);
}

TEST(get_avg_voltage_ab, gate_1_full) {
    const Scalar bus_voltage = 3.0;

    std::complex<Scalar> avg_voltage_ab =
        get_avg_voltage_ab(bus_voltage, {0, 1, 0});

    const std::complex<Scalar> expected =
        kSvmVectors[2] * kClarkeScale * bus_voltage;

    EXPECT_LT(std::norm(avg_voltage_ab - expected), 1e-6);
}

TEST(get_avg_voltage_ab, gate_2_full) {
    const Scalar bus_voltage = 3.0;

    std::complex<Scalar> avg_voltage_ab =
        get_avg_voltage_ab(bus_voltage, {0, 0, 1});

    const std::complex<Scalar> expected =
        kSvmVectors[4] * kClarkeScale * bus_voltage;

    EXPECT_LT(std::norm(avg_voltage_ab - expected), 1e-6);
}

TEST(pwm_to_avg_voltage_round_trip, 0) {
    const Scalar bus_voltage = 10.0;

    const std::complex<Scalar> voltage_ab =
        (0.2 * kSvmVectors[2] + 0.6 * kSvmVectors[3]) * bus_voltage *
        kClarkeScale;

    const std::array<Scalar, 3> duties =
        get_pwm_duties(bus_voltage, voltage_ab);

    const std::complex<Scalar> voltage_ab_out =
        get_avg_voltage_ab(bus_voltage, duties);

    EXPECT_LT(std::norm(voltage_ab - voltage_ab_out), 1e-6);
}
