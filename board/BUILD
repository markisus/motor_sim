COPTS = select({
    "@bazel_tools//src/conditions:windows": ["/std:c++17"],
    "//conditions:default": ["-std=c++17"],})

LINKOPTS = select({
    "@bazel_tools//src/conditions:windows": [],
    "//conditions:default": ["-lstdc++fs"],})

package(default_visibility = ["//visibility:public"])

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
    name = "board_state",
    hdrs = ["board_state.h"],
    deps = [
        ":gate_state",
        ":pwm_state",
        "//config:scalar"]
)
