COPTS = select({
    "@bazel_tools//src/conditions:windows": ["/std:c++17"],
    "//conditions:default": ["-std=c++17"],})

LINKOPTS = select({
    "@bazel_tools//src/conditions:windows": [],
    "//conditions:default": ["-lstdc++fs"],})

cc_binary(
    name = "abseil_example",
    srcs = ["abseil_example.cpp"],
    deps = [
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/container:flat_hash_map",
    ])

cc_binary(
    name = "gtest_example",
    srcs = ["gtest_example.cpp"],
    deps = [
        "@com_github_google_googletest//:gtest_main",
    ],
)

cc_binary(
    name = "benchmark_example",
    srcs = ["benchmark_example.cpp"],
    deps = [
        "@com_github_google_benchmark//:benchmark_main",
    ],
)

cc_binary(
    name = "imgui_sdl",
    srcs = ["imgui_sdl.cpp"],
    deps = [
        "//third_party/glad:glad",
        "//third_party/sdl:sdl",
        "//third_party/imgui:imgui_sdl",
    ]
)

cc_binary(
    name = "gflags_example",
    srcs = ["gflags_example.cpp"],
    deps = [
        "@com_github_gflags_gflags//:gflags"
    ]
)

cc_binary(
    name = "eigen_example",
    srcs = ["eigen_example.cpp"],
    deps = [
        "//third_party/eigen:eigen",
    ]
)
