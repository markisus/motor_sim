#include "sine_series.h"
#include <array>
#include <gtest/gtest.h>

TEST(sine_series, generate_odd_sine_series) {
    std::array<double, 5> ref;
    std::array<double, 5> result;

    double angle = 1.23;

    generate_odd_sine_series_reference(5, angle, ref.data());
    generate_odd_sine_series(5, angle, result.data());

    for (int i = 0; i < result.size(); ++i) {
        EXPECT_NEAR(ref[i], result[i], 1e-5);
    }
}
