#pragma once

#include "scalar.h"
#include <Eigen/Dense>

extern const Eigen::Matrix<Scalar, 3, 3> kClarkeTransform3x3;
extern const Eigen::Matrix<Scalar, 2, 3> kClarkeTransform2x3;
extern const Scalar kClarkeScale; // magnitude scaling factor going from regular
                                  // voltages to space vector
