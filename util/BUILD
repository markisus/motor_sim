package(default_visibility = ["//visibility:public"])

cc_library(
    name = "math_constants",
    hdrs = ["math_constants.h"])

cc_library(
    name = "time",
    hdrs = ["time.h"])

cc_library(
    name = "rolling_buffer",
    hdrs = ["rolling_buffer.h"])

cc_library(
    name = "rotation",
    hdrs = ["rotation.h"])

cc_library(
    name = "clarke_transform",
    hdrs = ["clarke_transform.h"],
    srcs = [
        "clarke_transform.h",
        "clarke_transform.cpp",
    ],
    deps = [
        ":conversions",
        "//third_party/eigen:eigen",
    ]
)

cc_library(
    name = "conversions",
    hdrs = ["conversions.h"])

cc_binary(
    name = "clarke_transform_test",
    srcs = ["clarke_transform_test.cpp"],
    deps = [
        ":clarke_transform",
        "@com_github_google_googletest//:gtest_main",
    ]
)

cc_library(
    name = "quantization",
    hdrs = ["quantization.h"]);

cc_binary(
    name = "quantization_test",
    srcs = ["quantization_test.cpp"],
    deps = [
        ":quantization",
        "@com_github_google_googletest//:gtest_main",
    ],
)

cc_binary(
    name = "sine_series_benchmark",
    srcs = ["sine_series_benchmark.cpp"],
    deps = [
        ":sine_series",
        "//third_party/eigen:eigen",
        "@com_github_google_benchmark//:benchmark_main",
    ],
)

cc_library(
    name = "sine_series",
    srcs = ["sine_series.h"],
    hdrs = ["sine_series.h"],
)

cc_binary(
    name = "sine_series_test",
    srcs = ["sine_series_test.cpp"],
    deps = [
        ":sine_series",
        "@com_github_google_googletest//:gtest_main",
    ],
)
