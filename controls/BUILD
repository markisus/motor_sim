package(default_visibility = ["//visibility:public"])

COPTS = select({
    "@bazel_tools//src/conditions:windows": ["/std:c++17"],
    "//conditions:default": ["-std=c++17"],})

cc_library(
    name = "pi_control",
    hdrs = ["pi_control.h"],
    srcs = ["pi_control.cpp",
            "pi_control.h"],
    deps = ["//config:scalar"],
    copts = COPTS,
)

cc_library(
    name = "six_step",
    hdrs = ["six_step.h"],
    srcs = [
        "six_step.h",
        "six_step.cpp",

    ],
    deps = [
        "//config:scalar",
        "//util:math_constants",
    ]
)

cc_library(
    name = "foc_state",
    hdrs = ["foc_state.h"],
    deps = [
        "//config:scalar",
        ":pi_control",
    ]
)

cc_library(
    name = "foc",
    hdrs = ["foc.h"],
    srcs = [
        "foc.h",
        "foc.cpp",
    ],
    deps = [
        "//simulator:motor_state",
        "//util:clarke_transform",
        "//util:conversions",
        "//util:rotation",
        ":foc_state",
    ]
)

cc_library(
    name = "space_vector_modulation",
    hdrs = ["space_vector_modulation.h"],
    srcs = [
        "space_vector_modulation.cpp",
        "space_vector_modulation.h"],
    deps = [
        "//global_debug:global_debug",
        "//third_party/eigen:eigen",
        "//util:math_constants",
        "//util:clarke_transform",
        "//util:conversions",
        "//config:scalar",
    ]
)

cc_binary(
    name = "space_vector_modulation_test",
    srcs = ["space_vector_modulation_test.cpp"],
    deps = [
        "//config:scalar",
        "//global_debug:global_debug",
        "//util:conversions",
        "//util:math_constants",
        "//simulator:pwm_state",
        ":space_vector_modulation",
        "@com_github_google_googletest//:gtest_main",
    ])
