#include "config/scalar.h"
#include "pi_control.h"
#include <gtest/gtest.h>
#include <iostream>

TEST(pi_control, unwind) {
    PiParams params;
    params.p_gain = 10;
    params.i_gain = 123;
    params.bias = 5;

    PiContext context;
    context.integral = 20;
    context.err = 0.3;

    const Scalar saturation_value = 100;
    // Check that the test has set up saturation conditions
    ASSERT_LT(saturation_value, pi_get_control(params, context));

    pi_unwind(params, saturation_value, &context);

    EXPECT_NEAR(saturation_value, pi_get_control(params, context), 1e-5);
    EXPECT_EQ(context.err, 0);
}
