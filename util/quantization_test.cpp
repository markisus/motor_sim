#include "quantization.h"
#include <gtest/gtest.h>

TEST(quantization, half0) {
    const double result = quantize(/*resolution=*/0.5, 1.234);
    EXPECT_NEAR(result, 1.0, 1e-8);
}

TEST(quantization, half1) {
    const double result = quantize(/*resolution=*/0.5, 1.678);
    EXPECT_NEAR(result, 1.5, 1e-8);
}

TEST(quantization, third0) {
    const double result = quantize(/*resolution=*/1.0/3, 1.0/3 + 0.1);
    EXPECT_NEAR(result, 1.0/3, 1e-8);
}

TEST(quantization, third1) {
    const double result = quantize(/*resolution=*/1.0/3, 2.0/3 + 0.1);
    EXPECT_NEAR(result, 2.0/3, 1e-8);
}
