COPTS = select({
    "@bazel_tools//src/conditions:windows": ["/std:c++17"],
    "//conditions:default": ["-std=c++17"],})

LINKOPTS = select({
    "@bazel_tools//src/conditions:windows": [],
    "//conditions:default": ["-lstdc++fs"],})

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "motor_state",
    hdrs = ["motor_state.h"],
    deps = [
        "//config:scalar",
        "//util:math_constants",
    ],
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
        "//util:sine_series",
        ":motor_state",
    ],
    copts = COPTS,
)

cc_library(
    name = "sim_state",
    hdrs = ["sim_state.h"],
    deps = [
        "//board:gate_state",
        "//third_party/eigen:eigen"]
)

cc_library(
    name = "gui",
    srcs = ["gui.cpp"],
    hdrs = ["gui.h"],
    deps = [
        "//board:pwm_state",
        "//board:board_state",
        "//config:scalar",
        "//controls:foc_state",
        "//controls:pi_control",
        "//third_party/imgui:imgui_base",
        "//third_party/implot:implot",
        "//util:clarke_transform",
        "//util:conversions",
        "//util:math_constants",
        "//util:rolling_buffer",
        "//util:rotation",
        "//util:sine_series",
        ":motor",
        ":motor_state",
        ":sim_state",
        "@com_google_absl//absl/strings:str_format",
    ])


cc_binary(
    name = "simulator",
    srcs = ["simulator.cpp"],
    deps = [
        "//board:board_state",
        "//config:scalar",
        "//controls:foc",
        "//controls:pi_control",
        "//controls:six_step",
        "//controls:space_vector_modulation",
        "//third_party/eigen:eigen",
        "//third_party/glad:glad",
        "//third_party/imgui:imgui_sdl",
        "//third_party/implot:implot",
        "//util:clarke_transform",
        "//util:conversions",
        "//util:math_constants",
        "//util:rotation",
        "//util:sine_series",
        "//util:time",
        "//wrappers:sdl_context",
        "//wrappers:sdl_imgui",
        ":gui",
        ":motor",
        "@com_google_absl//absl/strings:str_format",
    ],
    copts = COPTS, # need cpp17 to avoid eigen weirdness
)

