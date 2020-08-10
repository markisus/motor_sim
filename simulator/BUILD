COPTS = select({
    "@bazel_tools//src/conditions:windows": ["/std:c++17"],
    "//conditions:default": ["-std=c++17"],})

LINKOPTS = select({
    "@bazel_tools//src/conditions:windows": [],
    "//conditions:default": ["-lstdc++fs"],})

cc_library(
    name = "conversions",
    hdrs = ["conversions.h"])

cc_library(
    name = "clarke_transform",
    hdrs = ["clarke_transform.h"],
    srcs = [
        "clarke_transform.h",
        "clarke_transform.cpp"],
    deps = [
        "//third_party/eigen:eigen",
        ":scalar",
    ]
)

cc_binary(
    name = "clarke_transform_test",
    srcs = ["clarke_transform_test.cpp"],
    deps = [
        ":clarke_transform",
        "@com_github_google_googletest//:gtest_main",
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
        "//third_party/eigen:eigen"
        ":clarke_transform",
        ":conversions",
        ":pi",
        ":scalar",
    ]
)

cc_binary(
    name = "space_vector_modulation_test",
    srcs = ["space_vector_modulation_test.cpp"],
    deps = [
        "//global_debug:global_debug",
        ":pi",
        ":scalar",
        ":space_vector_modulation",
        "@com_github_google_googletest//:gtest_main",
    ])
    

cc_library(
    name = "pi_control",
    hdrs = ["pi_control.h"],
    srcs = ["pi_control.cpp",
            "pi_control.h"],
    deps = [
        ":scalar"],
    copts = COPTS,
)

cc_library(
    name = "sim_state",
    hdrs = ["sim_state.h"],
    deps = ["//third_party/eigen:eigen"]
)

cc_library(
    name = "gui",
    srcs = ["gui.cpp"],
    hdrs = ["gui.h"],
    deps = [
        ":clarke_transform",
        ":pi",
        ":scalar",
        ":sim_params",
        ":sim_state",
        "//third_party/imgui:imgui_base",
        "//third_party/implot:implot",
        "@com_google_absl//absl/strings:str_format",
    ])

cc_library(
    name = "scalar",
    hdrs = ["scalar.h"])

cc_library(
    name = "pi",
    hdrs = ["pi.h"],
    deps = [":scalar"]
)

cc_library(
    name = "sim_params",
    hdrs = ["sim_params.h"],
    deps = [
        ":pi",
        ":scalar",
        "//third_party/eigen:eigen",
    ]
)
cc_binary(
    name = "simulator",
    srcs = ["simulator.cpp"],
    deps = [
        ":clarke_transform",
        ":conversions",
        ":gui",
        ":scalar",
        ":sim_params",
        ":sine_series",
        "//third_party/eigen:eigen",
        "//third_party/glad:glad",
        "//third_party/imgui:imgui_sdl",
        "//third_party/implot:implot",
        "//wrappers:sdl_context",
        "//wrappers:sdl_imgui",
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
