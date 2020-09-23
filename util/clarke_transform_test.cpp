#include "clarke_transform.h"
#include "config/scalar.h"
#include <Eigen/Dense>
#include <gtest/gtest.h>

TEST(kClarkeTransform2x3, zero_series_invariant) {
    const Eigen::Matrix<Scalar, 3, 1> ones =
        Eigen::Matrix<Scalar, 3, 1>::Ones();
    const Eigen::Matrix<Scalar, 2, 1> result = kClarkeTransform2x3 * ones;

    EXPECT_LT(result.norm(), 1e-7);
}

TEST(kClarkeTransform3x3, unitary) {
    const Eigen::Matrix<Scalar, 3, 3> delta =
        kClarkeTransform3x3 * kClarkeTransform3x3.transpose() -
        Eigen::Matrix<Scalar, 3, 3>::Identity();

    EXPECT_LT(delta.norm(), 1e-7);
}
