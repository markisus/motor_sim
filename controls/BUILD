package(default_visibility = ["//visibility:public"])

COPTS = select({
    "@bazel_tools//src/conditions:windows": ["/std:c++17"],
    "//conditions:default": ["-std=c++17"],})

cc_library(
    name = "pi_control",
    hdrs = ["pi_control.h"],
    srcs = ["pi_control.cpp",
            "pi_control.h"],
    copts = COPTS,
)
