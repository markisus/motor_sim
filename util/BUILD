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

