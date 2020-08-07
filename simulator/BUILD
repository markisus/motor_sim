COPTS = select({
    "@bazel_tools//src/conditions:windows": ["/std:c++17"],
    "//conditions:default": ["-std=c++17"],})

LINKOPTS = select({
    "@bazel_tools//src/conditions:windows": [],
    "//conditions:default": ["-lstdc++fs"],})

cc_binary(
    name = "simulator",
    srcs = ["simulator.cpp"],
    deps = [
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
