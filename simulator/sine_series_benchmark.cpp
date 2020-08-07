#include "sine_series.h"
#include <Eigen/Dense>
#include <benchmark/benchmark.h>
#include <cmath>

using Scalar = double;
constexpr Scalar angle = 1.2345;

static void BM_Sine_Series_Naive(benchmark::State& state) {
    // Perform setup here
    for (auto _ : state) {
        // This code gets timed
        Eigen::Matrix<Scalar, 5, 1> sines;
        generate_odd_sine_series_reference(5, angle, sines.data());
        benchmark::DoNotOptimize(sines.sum());
    }
}
BENCHMARK(BM_Sine_Series_Naive);

static void BM_Sine_Series_Custom(benchmark::State& state) {
    // Perform setup here
    for (auto _ : state) {
        // This code gets timed
        Eigen::Matrix<Scalar, 5, 1> sines;
        generate_odd_sine_series(5, angle, sines.data());
        benchmark::DoNotOptimize(sines.sum());
    }
}
BENCHMARK(BM_Sine_Series_Custom);

// Run the benchmark
BENCHMARK_MAIN();
