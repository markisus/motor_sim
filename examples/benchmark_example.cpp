#include <benchmark/benchmark.h>

#include <cmath>

static void BM_Cosine(benchmark::State& state) {
    // Perform setup here
    for (auto _ : state) {
        // This code gets timed
        std::cos(0.5);
    }
}
// Register the function as a benchmark
BENCHMARK(BM_Cosine);
// Run the benchmark
BENCHMARK_MAIN();
