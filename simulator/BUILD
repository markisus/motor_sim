COPTS = select({
    "@bazel_tools//src/conditions:windows": ["/std:c++17"],
    "//conditions:default": ["-std=c++17"],})

LINKOPTS = select({
    "@bazel_tools//src/conditions:windows": [],
    "//conditions:default": ["-lstdc++fs"],})

cc_library(
    name = "gate_state",
    hdrs = ["gate_state.h"],
    deps = ["//config:scalar"])

cc_library(
    name = "pwm_state",
    hdrs = ["pwm_state.h"],
    deps = [
        "//config:scalar",
        "//util:time",
    ]
)

cc_library(
    name = "foc_state",
    hdrs = ["foc_state.h"],
    deps = [
        "//config:scalar",
        "//controls:pi_control",
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
        "//util:clarke_transform",
        "//util:rotation",
        "//util:conversions",
        ":foc_state",
        ":motor_state",
    ]
)

cc_library(
    name = "motor_state",
    hdrs = ["motor_state.h"],
    deps = [
        "//config:scalar",
        "//util:math_constants",
    ]
)

cc_library(
    name = "motor",
    hdrs = [
        "motor.h",
    ],
    srcs = [
        "motor.h",
        "motor.cpp",
    ],
    deps = [
        "//config:scalar",
        "//controls:pi_control",
        "//third_party/eigen:eigen",
        ":motor_state",
        ":sine_series",
    ],
    copts = COPTS,
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
        ":pwm_state",
        ":space_vector_modulation",
        "@com_github_google_googletest//:gtest_main",
    ])
    
cc_library(
    name = "sim_state",
    hdrs = ["sim_state.h"],
    deps = [
        ":gate_state",
        "//third_party/eigen:eigen"]
)

cc_library(
    name = "gui",
    srcs = ["gui.cpp"],
    hdrs = ["gui.h"],
    deps = [
        "//config:scalar",
        "//controls:pi_control",
        "//third_party/imgui:imgui_base",
        "//third_party/implot:implot",
        "//util:clarke_transform",
        "//util:conversions",
        "//util:math_constants",
        "//util:rolling_buffer",
        "//util:rotation",
        ":foc_state",
        ":motor_state",
        ":pwm_state",
        ":sim_state",
        ":sine_series",
        "@com_google_absl//absl/strings:str_format",
    ])


cc_library(
    name = "pi",
    hdrs = ["pi.h"],
    deps = ["//config:scalar"]
)

cc_binary(
    name = "simulator",
    srcs = ["simulator.cpp"],
    deps = [
        "//config:scalar",
        "//controls:pi_control",
        "//controls:six_step",
        "//third_party/eigen:eigen",
        "//third_party/glad:glad",
        "//third_party/imgui:imgui_sdl",
        "//third_party/implot:implot",
        "//util:clarke_transform",
        "//util:conversions",
        "//util:math_constants",
        "//util:rotation",
        "//util:time",
        "//wrappers:sdl_context",
        "//wrappers:sdl_imgui",
        ":foc",
        ":gui",
        ":motor",
        ":sine_series",
        ":space_vector_modulation",
        "@com_google_absl//absl/strings:str_format",
    ],
    copts = COPTS, # need cpp17 to avoid eigen weirdness
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
