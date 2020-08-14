#pragma once

#include "controls/pi_control.h"
#include "scalar.h"
#include <complex>

struct FocState {
    Scalar period = 1.0 / 10000; // sec, 10kHz
    Scalar timer = 0;

    PiParams i_controller_params;
    PiContext iq_controller;
    PiContext id_controller;

    // output command
    std::complex<Scalar> voltage_qd;
};
