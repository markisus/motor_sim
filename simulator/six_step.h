#pragma once

#include "scalar.h"
#include <array>

std::array<bool, 3> six_step_commutate(const Scalar electrical_angle,
                                       const Scalar phase_advance);
