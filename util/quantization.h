#pragma once

template <typename TScalar>
TScalar quantize(const TScalar resolution, const TScalar input) {
    // round to level to the nearest integer * resolution mark
    // quantized(level) = i * resolution
    // quantized(level) / resolution = i
    const int i = int(input / resolution);
    return i * resolution;
}
