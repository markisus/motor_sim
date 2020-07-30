package(default_visibility = ["//visibility:public"])

# Users should depend on "imgui_glfw" or "imgui_sdl"
# depending on which window library you want to use

cc_library(
    name = "implot",
    srcs = [
        "implot.cpp",
        "implot_demo.cpp",
        "implot.h"
    ],
    hdrs = [
        "implot.h",
    ],
    deps = [
        "//third_party/imgui:imgui_base",
    ],
    strip_include_prefix = "/third_party/implot",
)
